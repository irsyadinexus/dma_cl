# Design Specification: Milestone M3 Scatter-Gather Linked-List & Out-of-Order Reorder Core

**Date**: 2026-10-23  
**Status**: Draft for Review  
**Target Milestone**: Milestone M3 (Scatter-Gather Linked-List Descriptor Engine, Completion Reorder Buffer, 8-Channel Scaling, WRR Arbiter)

---

## 1. Executive Summary & Scope

This specification details the Milestone M3 micro-architecture for the vendor-neutral, synthesizable PCIe Gen4 HDMA-compatible DMA core. Milestone M3 upgrades the Milestone M2 register-mode core to support full hardware scatter-gather linked-list walking, dynamic PCIe extended tag allocation, out-of-order completion reassembly, multi-channel scaling (4 Read + 4 Write channels), and weighted round-robin arbitration.

### Core Objectives for M3
1. **Scatter-Gather Linked-List Walking:** Hardware autonomously walks in-memory descriptor chains (`dma_pkg::hdma_elem_t`, 24 bytes / 6 DWORDs) over host PCIe Memory Reads (`MRd`), supporting:
   - Lockless ownership handshakes via Cycle-Bit (`CB`).
   - Non-contiguous lists via Link Element pointers (`LLP=1`).
   - Cyclic ring buffers via Toggle-CB markers (`TCB=1`).
   - Local interrupt assertions on designated elements (`LIE=1`).
2. **PCIe Extended Tag Allocation & Out-of-Order Reorder Buffer:**
   - 10-bit tag slicing: `tag[9:8]` encodes Channel ID (0..3 for Read channels); `tag[7:0]` encodes in-flight transaction tags.
   - Private dual-port reassembly memory and scoreboard per Read channel to handle out-of-order `CplD` responses and 64-byte Read Completion Boundary (RCB) fragmentation.
   - Strictly in-order commitment to the device AXI4-MM write datapath (`AWADDR`, sequential `WDATA`, `WLAST`).
   - Complete channel isolation: a stalled or faulting channel cannot starve or head-of-line block other channels, satisfying **NFR-1** and the M5 PRI architecture requirements.
3. **Multi-Channel Scaling:**
   - Full concurrent operation across `NUM_RD_CH = 4` Read (H2C) and `NUM_WR_CH = 4` Write (C2H) channels (8 total channels).
4. **Weighted Round-Robin (WRR) Channel Arbiter:**
   - Programmable per-channel QoS weights (`DMA_READ_CH_ARB_WEIGHT` / `DMA_WRITE_CH_ARB_WEIGHT`).
   - Multi-beat Write burst grant locking to preserve TLP atomicity.
   - Parallel combinatorial completion demultiplexing.
5. **Directed System Verification (`sim/tb_m3.sv`):**
   - Out-of-order & RCB-split completion responder in Root Complex BFM.
   - 7 test scenarios verifying multi-element lists, link pointers, cyclic mode, reorder reassembly, C2H mover, concurrent 8-channel operation, and WRR QoS.

---

## 2. Top-Level Interface Architecture (`rtl/dma_top.sv`)

The IP-agnostic interface boundary established in M2 remains fully preserved. No proprietary controller signals cross the top-level boundary.

```
                 Host PCIe Controller / Bridge (Any Vendor)
                             │ host_req (Out)     ▲ host_cpl (In)
                             │ host_req_ready     │ host_cpl_ready
┌────────────────────────────▼────────────────────┴──────────────────────────┐
│ dma_top                                                                    │
│                                                                            │
│   s_apb_* (BAR0 APB Slave)         ats_pri_subsys (Transparent in M3)      │
│        │                                        ▲                          │
│        ▼                                        │                          │
│  ┌───────────┐                            ┌─────┴─────┐                    │
│  │ csr_block │                            │chan_arbiter│                   │
│  │           │                            │ (WRR/QoS) │                    │
│  └─────┬─────┘                            └─────▲─────┘                    │
│        │ ch_ctrl[0..7] / ch_stat[0..7]          │ req/cpl per channel      │
│        ▼                                        ▼                          │
│  ┌────────────────────────────────────────────────────────┐                │
│  │ dma_channel[0..3] (Read / H2C with Private Reorder RAM) │                │
│  │ dma_channel[4..7] (Write / C2H with Read Skid Buffer)   │                │
│  └─────────────┬────────────────────────────┬─────────────┘                │
└────────────────┼────────────────────────────┼──────────────────────────────┘
                 ▼                            ▼
          dev_mm_m2s[0..3]             dev_mm_m2s[4..7]
          dev_mm_s2m[0..3]             dev_mm_s2m[4..7]
          (AXI4-MM Masters)            (AXI4-MM Masters)
                 │                            │
                 ▼                            ▼
         Device Read Sinks            Device Write Sources
```

### Module Ports (`rtl/dma_top.sv`)
- **Clocks & Resets:** `core_clk` (~500 MHz), `core_rst_n`, `axi_clk` (~400 MHz), `axi_rst_n`, `csr_clk` (~100 MHz), `csr_rst_n`.
- **PCIe Requester (Master Out):** `host_req` (`dma_pkg::pcie_req_bus_t`), `host_req_ready`.
- **PCIe Completion (Slave In):** `host_cpl` (`dma_pkg::pcie_cpl_bus_t`), `host_cpl_ready`.
- **CSR APB Slave:** `s_apb_paddr[19:0]`, `s_apb_psel`, `s_apb_penable`, `s_apb_pwrite`, `s_apb_pwdata[31:0]`, `s_apb_pstrb[3:0]`, `s_apb_prdata[31:0]`, `s_apb_pready`, `s_apb_pslverr`.
- **Device AXI4-MM Ports:** `dev_mm_m2s[8]`, `dev_mm_s2m[8]`.
- **Device AXI4-Stream Ports:** `dev_st_o[8]`, `dev_st_o_ready[8]`, `dev_st_i[8]`, `dev_st_i_ready[8]`.
- **Interrupts & MSI:** `ch_irq[8]`, `ch_msi[8]`.

---

## 3. Scatter-Gather Linked-List Engine (`rtl/blocks/dma_channel.sv`)

### 3.1 Descriptor ABI Format
Descriptors follow `dma_pkg::hdma_elem_t` (24 bytes / 6 DWORDs):
- **DWORD 0 (`control`):**
  - `control[0]` (`CTRL_CB`): Cycle bit indicating software vs hardware ownership.
  - `control[1]` (`CTRL_TCB`): Toggle cycle bit marker (for cyclic rings).
  - `control[2]` (`CTRL_LLP`): 1 = Link element; 0 = Data element.
  - `control[3]` (`CTRL_LIE`): Local interrupt enable; asserts `done_irq` upon element completion.
  - `control[4]` (`CTRL_RIE`): Remote interrupt enable.
- **DWORD 1 (`size`):** Transfer length in bytes (up to 4 GB).
- **DWORD 2 (`sar_lo`):** Source address [31:0] (or link pointer low [31:0] if `LLP=1`).
- **DWORD 3 (`sar_hi`):** Source address [63:32] (or link pointer high [63:32] if `LLP=1`).
- **DWORD 4 (`dar_lo`):** Destination address [31:0].
- **DWORD 5 (`dar_hi`):** Destination address [63:32].

### 3.2 Descriptor Fetch & Execution State Machine
```
                       ┌─────────────────────────┐
                       │         ST_IDLE         │◄────────────────┐
                       └────────────┬────────────┘                 │
                                    │ Doorbell + lle=1             │
                                    ▼                              │
                       ┌─────────────────────────┐                 │
                       │        F_REQ_DESC       │                 │
                       │ Issue MRd for 24B elem  │                 │
                       └────────────┬────────────┘                 │
                                    │ CplD received                │
                                    ▼                              │
                       ┌─────────────────────────┐                 │
                       │       F_PARSE_DESC      │                 │
                       │ Check CB == expected_cb │                 │
                       └─────┬──────────────┬────┘                 │
           CB == expected_cb │              │ CB != expected_cb    │
                             ▼              └──────────────────────┤
                 ┌───────────────────────┐   (Wait for software /   │
                 │   Check control.LLP   │    pause transfer)      │
                 └───────┬───────┬───────┘                         │
                LLP == 1 │       │ LLP == 0                        │
                         │       ▼                                 │
                         │ ┌───────────────────────────┐           │
                         │ │          ST_RUN           │           │
                         │ │ Execute Data Mover        │           │
                         │ │ (Segment to 4KB/MPS/MRRS) │           │
                         │ └─────────────┬─────────────┘           │
                         │               │ Transfer done           │
                         │               ▼                         │
                         │ ┌───────────────────────────┐           │
                         │ │        F_NEXT_ELEM        │           │
                         │ │ llp_ptr <= llp_ptr + 24   │           │
                         │ │ If LIE: pulse done_irq    │           │
                         │ └─────────────┬─────────────┘           │
                         │               │                         │
                         ▼               ▼                         │
           ┌───────────────────────────────────────────┐           │
           │ llp_ptr <= {elem.sar_hi, elem.sar_lo}     │           │
           │ If TCB: expected_cb <= ~expected_cb       │───────────┘
           └───────────────────────────────────────────┘
```

---

## 4. Tag Allocator & Out-of-Order Completion Reorder Subsystem

### 4.1 Tag Slicing
Extended 10-bit PCIe tags (`TAG_W=10`) are strictly partitioned:
- `tag[9:8]`: Read Channel Index (`2'b00` = Ch0, `2'b01` = Ch1, `2'b10` = Ch2, `2'b11` = Ch3).
- `tag[7:0]`: Per-Channel Transaction Tag (`0 .. TAGS_PER_CH-1`), where `TAGS_PER_CH = 16` by default.

### 4.2 Scoreboard Table
Each Read channel contains a scoreboard table indexed by `local_tag` (16 entries):
- `valid`: Entry currently tracks an outstanding MRd request.
- `expected_bytes`: Total byte length of the MRd segment (≤ 512 bytes / 8 beats).
- `bytes_received`: Running count of bytes received across multiple `CplD` packets.
- `ram_slot_offset`: Base beat index in the reorder RAM (each slot = 8 beats / 512 bytes).
- `status`:
  - `TAG_FREE`: Tag available for allocation.
  - `TAG_WAIT_CPL`: Awaiting completion beats.
  - `TAG_COMMITTED`: All bytes received; ready for in-order drain.

### 4.3 Reorder Dual-Port RAM & Assembly Logic
- **RAM Sizing:** Width = 512-bit (`DATA_WIDTH`), Depth = `TAGS_PER_CH * 8 beats` = 128 beats (8 KB per channel).
- **Intake Logic:**
  - Upon completion beat arrival:
    - `local_tag = cpl.meta.tag[7:0]`.
    - `slot_offset = scoreboard[local_tag].ram_slot_offset`.
    - `beat_offset = scoreboard[local_tag].bytes_received >> 6`.
    - RAM write address: `slot_offset + beat_offset`.
    - Write `cpl.rdata` into RAM.
    - Increment `bytes_received += cpl.meta.byte_count` (or beat size).
    - When `bytes_received == expected_bytes` or `cpl.meta.last`: mark `TAG_COMMITTED`.

### 4.4 In-Order Commit Engine (AXI-MM Master Write Pusher)
- Pointer `commit_tag_ptr` tracks the oldest uncommitted transaction.
- When `scoreboard[commit_tag_ptr].status == TAG_COMMITTED`:
  1. Issues AXI4 `AW` channel burst (`awaddr = dar_current`, `awlen = (expected_bytes >> 6) - 1`).
  2. Sequentially reads beats from `ram_slot_offset` and streams onto AXI `W` channel.
  3. Asserts `wlast` on the final beat of the segment.
  4. Waits for AXI `BVALID`/`BREADY`.
  5. Advances `dar_current += expected_bytes`, updates `bytes_xferred`, frees `commit_tag_ptr` to `TAG_FREE`, and advances `commit_tag_ptr = (commit_tag_ptr + 1) % TAGS_PER_CH`.

### 4.5 Error Handling
- **Watchdog Timer:** If a tag remains in `TAG_WAIT_CPL` for > 65,536 cycles, `ERR_CPL_TIMEOUT` is raised.
- **Completer Abort / UR:** If `cpl.meta.status != CPL_SC`, channel aborts immediately with `ERR_CPL_UR` or `ERR_CPL_CA`.

---

## 5. Multi-Channel Weighted Round-Robin (WRR) Arbiter (`rtl/blocks/chan_arbiter.sv`)

### 5.1 Architecture
The arbiter multiplexes 8 channel requester buses (`ch_req[0..7]`) to the top-level master port `m_req`:
- **Deficit / Credit Mechanism:**
  - Each channel `i` has an internal 8-bit credit counter initialized to `qos[i]` (from `ch_ctrl[i].qos[3:0]`). If `qos[i] == 0`, default weight is 1.
  - The arbiter services requesting channels with non-zero credit in round-robin order.
  - Each granted request beat decrements `credit[i]`. When `credit[i] == 0`, the arbiter moves to the next channel and reloads `credit[i] <= qos[i]`.
  - If all requesting channels exhaust their credits simultaneously, credits are refreshed in one cycle.
- **Multi-Beat Burst Protection:**
  - For Write channels (`!ch_req[gnt].req.is_read`):
    - When `ch_req[gnt].valid && m_req_ready`, arbiter latches `in_burst <= 1` and `burst_ch <= gnt`.
    - Grant remains locked to `burst_ch` until `ch_req[burst_ch].wlast && m_req_ready`.
- **Completion Demultiplexer:**
  - Pure parallel routing: `ch_cpl[i].valid = m_cpl.valid && (m_cpl.meta.tag[9:8] == i[1:0])`.
  - Completions for channels 0..3 never block or interlock with one another.

---

## 6. Verification & Test Plan (`sim/tb_m3.sv`)

### 6.1 Testbench Environment
- `sim/tb_m3.sv` instantiates `dma_top`, 8 instances of `sim/axi_ram.sv` (one per channel), and a host PCIe BFM with:
  - APB CSR master driver.
  - Host memory array (2 MB).
  - Out-of-order & RCB-split completion responder.

### 6.2 Test Cases & Pass Criteria

| Test ID | Scenario | Pass Criteria |
| :--- | :--- | :--- |
| **Test 1** | Multi-element H2C linked-list transfer (4 contiguous elements, 512B each) | All 2048 bytes match in device AXI RAM; `done_irq` asserted on element with `LIE=1`. |
| **Test 2** | Non-contiguous linked list with Link Element (`CTRL_LLP=1`) | Engine jumps to new host memory pointer; completes all elements across chunks. |
| **Test 3** | Cyclic mode linked list (`CTRL_TCB=1`) | Ring executes 3 iterations; cycle-bit flips automatically without software intervention. |
| **Test 4** | Out-of-Order & RCB-Split Completion Reassembly (H2C) | BFM returns completions out of order and split into 64B beats; data in AXI RAM matches source byte-for-byte. |
| **Test 5** | Multi-element C2H linked-list transfer (4 elements, Device to Host) | Data read from AXI RAM packetized into `MWr` and written to host memory correctly. |
| **Test 6** | Concurrent 8-channel operation (4 H2C + 4 C2H simultaneously active) | All 8 channels complete; zero data corruption; independent interrupts received. |
| **Test 7** | Weighted Round-Robin (WRR) QoS verification | Channel with weight 8 receives significantly more grants than channel with weight 1 under congestion. |

---

## 7. Deliverables & Acceptance Checklist
- [ ] `rtl/dma_pkg.sv`: updated parameters and helper functions if needed.
- [ ] `rtl/blocks/dma_channel.sv`: full implementation of linked-list fetch FSM, tag scoreboard, and reorder RAM reassembly.
- [ ] `rtl/blocks/chan_arbiter.sv`: upgraded to WRR deficit arbiter with QoS weights and multi-beat lock.
- [ ] `rtl/dma_top.sv`: updated wiring for all 8 channels and WRR weights.
- [ ] `sim/tb_m3.sv`: complete verification testbench executing Tests 1–7.
- [ ] `Makefile`: `make lint`, `make lint-if`, `make sim-m3` all pass cleanly with 0 errors.
