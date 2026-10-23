# Milestone M3 Scatter-Gather Linked-List & Out-of-Order Reorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the Milestone M3 Scatter-Gather Linked-List Descriptor Engine (`hdma_elem_t`), Dynamic 10-bit Tag Allocator, Completion Reorder Buffer with RCB split support, Multi-Channel scaling (`NUM_RD_CH=4`, `NUM_WR_CH=4`), and Weighted Round-Robin (WRR) Arbiter, verified by a directed testbench (`make sim-m3`) with all 7 tests passing.

**Architecture:** Each Read channel mover (`dma_channel.sv`) is autonomous with its own private 10-bit tag partition (`tag[9:8]` = channel ID, `tag[7:0]` = local tag) and private dual-port reorder RAM/scoreboard to eliminate inter-channel head-of-line blocking (spec NFR-1 & M5 PRI requirement). Descriptors are fetched via PCIe MRds and parsed locklessly using cycle-bit (`CB`/`TCB`) and link element (`LLP`) semantics. The central arbiter (`chan_arbiter.sv`) provides deficit Weighted Round-Robin (WRR) scheduling with atomic multi-beat burst protection.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Verilator 5.032+ (lint and simulation), standard GNU Make, bash.

**Spec:** [`docs/superpowers/specs/2026-10-23-milestone-m3-linked-list-reorder-design.md`](file:///home/user/RTL-VIBECODE/DMA/dma_cl/docs/superpowers/specs/2026-10-23-milestone-m3-linked-list-reorder-design.md)

## Global Constraints

- Native WSL2 / Linux toolchain only: Use standard Linux binaries (`/usr/bin/verilator`, `make`, `git`), never Windows `.exe` tools or `/mnt/c/`.
- IP-Agnostic Core: Zero proprietary IP dependencies (no Synopsys DWC shims in synthesizable RTL).
- Data Width: Primary `DATA_WIDTH = 512` bits (64 bytes/beat) at 500 MHz datapath clock.
- Strict In-Order Commit: Device AXI4-MM write bursts must receive strictly sequential beats with valid `WLAST` and proper `AW` handshakes.
- Complete Channel Isolation: Stalled or faulting channels must not block or starve active sibling channels.
- Verification Rule: Evidence before assertions always (`make lint`, `make lint-if`, `make sim-m2`, `make sim-m3`).

## Review Focus

1. **Cycle-Bit Ownership Race on List End:** Software may populate descriptors concurrently. If `CB != expected_cb`, channel must pause without asserting false error and resume cleanly when software rings doorbell.
2. **Out-of-Order Completion Reassembly with RCB Splits:** Completions fragmented into 64-byte chunks arriving in reverse or interleaved order must reassemble into exact contiguous memory without dropping or duplicating bytes.
3. **Multi-Beat Burst Interleaving Bug in WRR Arbiter:** A Write channel sending an 8-beat `MWr` burst must not lose arbitration mid-burst to a higher-weight channel, which would corrupt PCIe packet framing.
4. **Link Element (`LLP=1`) at 4KB Boundary:** Link descriptors residing at page boundaries must fetch cleanly without violating the 4KB PCIe crossing rule.
5. **Zero-Byte or Misaligned Descriptor Handling:** Transfers with unaligned addresses or invalid descriptor sizes must gracefully flag `ERR_LL_FORMAT` and halt safely.

---

### Task 1: Package Parameters & Scoreboard Types (`rtl/dma_pkg.sv`)

**Files:**
- Modify: `rtl/dma_pkg.sv:14-38, 77-98`
- Test: `sim/lint_top.sv`, `Makefile`

**Interfaces:**
- Consumes: Existing `hdma_elem_t`, `pcie_req_t`, `pcie_cpl_meta_t` definitions.
- Produces: `TAGS_PER_CH`, `REORDER_BEATS_PER_CH`, `MAX_BURST_BEATS`, and tag status enums for channel reorder logic.

- [x] **Step 1: Inspect `rtl/dma_pkg.sv` and write the failing lint check for new types**

Run: `make lint` to verify current clean baseline.

- [x] **Step 2: Add parameters and tag tracking types to `rtl/dma_pkg.sv`**

In `rtl/dma_pkg.sv`, define:
```systemverilog
  localparam int unsigned TAGS_PER_CH       = 16;   // in-flight MRd tags per channel
  localparam int unsigned MAX_BURST_BEATS   = 8;    // 512B / 64B = 8 beats max per MRd/MWr
  localparam int unsigned REORDER_BEATS     = TAGS_PER_CH * MAX_BURST_BEATS; // 128 beats (8KB)
  localparam int unsigned CPL_TIMEOUT_LIMIT = 65535;// watchdog cycles before ERR_CPL_TIMEOUT

  typedef enum logic [1:0] {
    TAG_FREE      = 2'b00,
    TAG_WAIT_CPL  = 2'b01,
    TAG_COMMITTED = 2'b10
  } tag_state_e;
```

- [x] **Step 3: Run lint to verify cleanliness**

Run: `make lint && make lint-if`
Expected: 0 errors, 0 warnings.

- [x] **Step 4: Commit**

```bash
git add rtl/dma_pkg.sv
git commit -m "feat(pkg): add M3 reorder buffer and tag scoreboard definitions"
```

---

### Task 2: Completion Reorder Buffer & In-Order Commit Engine in `dma_channel.sv`

**Files:**
- Modify: `rtl/blocks/dma_channel.sv:53-168`
- Test: `sim/tb_m2.sv`

**Interfaces:**
- Consumes: `host_cpl` bus, `dma_pkg::TAGS_PER_CH`, `dma_pkg::REORDER_BEATS`, `dma_pkg::MAX_BURST_BEATS`.
- Produces: Tag-scorecard tracking, dual-port reorder RAM, and in-order AXI-MM write bursts with out-of-order `CplD` reassembly.

- [x] **Step 1: Write scoreboard and reorder RAM data structures in `dma_channel.sv`**

In `rtl/blocks/dma_channel.sv` under `g_h2c`:
```systemverilog
  // Tag scoreboard entry
  typedef struct packed {
    tag_state_e        state;
    logic [12:0]       expected_bytes;
    logic [12:0]       bytes_received;
    logic [6:0]        ram_slot_offset; // base beat offset in reorder RAM
    logic [63:0]       dar;
  } tag_entry_t;

  tag_entry_t scoreboard [TAGS_PER_CH];
  logic [DATA_WIDTH-1:0] reorder_ram [REORDER_BEATS];

  // Pointers for in-order commit
  logic [$clog2(TAGS_PER_CH)-1:0] alloc_tag_ptr;
  logic [$clog2(TAGS_PER_CH)-1:0] commit_tag_ptr;
```

- [x] **Step 2: Implement completion intake with RCB offset mapping**

Map incoming completion `cpl`:
```systemverilog
  wire [1:0] cpl_ch_id = cpl.meta.tag[9:8];
  wire [$clog2(TAGS_PER_CH)-1:0] cpl_local_tag = cpl.meta.tag[$clog2(TAGS_PER_CH)-1:0];

  // Write into reorder RAM when completion arrives
  always_ff @(posedge core_clk) begin
    if (cpl.valid && cpl_ready) begin
      int unsigned write_idx;
      write_idx = int'(scoreboard[cpl_local_tag].ram_slot_offset) +
                  (int'(scoreboard[cpl_local_tag].bytes_received) >> 6);
      reorder_ram[write_idx] <= cpl.rdata;
    end
  end
```

- [x] **Step 3: Implement in-order commit engine driving AXI AW and W channels**

When `scoreboard[commit_tag_ptr].state == TAG_COMMITTED`:
1. Drive `mm_m2s.awaddr <= scoreboard[commit_tag_ptr].dar` and `awvalid <= 1'b1`.
2. Sequentially read `reorder_ram[ram_slot_offset + beat]` and drive `mm_m2s.wdata`, `wvalid`, `wlast`.
3. Wait for `bvalid && bready`, advance `commit_tag_ptr`, and mark tag `TAG_FREE`.

- [x] **Step 4: Verify with `make lint` and run `make sim-m2`**

Run: `make lint && make sim-m2`
Expected: PASS (all 7 M2 tests continue to pass with 0 errors).

- [x] **Step 5: Commit**

```bash
git add rtl/blocks/dma_channel.sv
git commit -m "feat(channel): add out-of-order completion reorder buffer and commit engine"
```

---

### Task 3: Scatter-Gather Linked-List Fetch & Parser Engine in `dma_channel.sv`

**Files:**
- Modify: `rtl/blocks/dma_channel.sv`
- Test: `sim/tb_m2.sv`

**Interfaces:**
- Consumes: `ctrl.lle`, `ctrl.llp_base`, `ctrl.doorbell`, `hdma_elem_t`.
- Produces: Autonomous descriptor walking, cycle-bit toggling, link element jump, and `done_irq` generation on `CTRL_LIE`.

- [x] **Step 1: Define descriptor fetch FSM states and registers**

```systemverilog
  typedef enum logic [3:0] {
    ST_IDLE_M      = 4'd0,
    ST_FETCH_REQ   = 4'd1,
    ST_FETCH_WAIT  = 4'd2,
    ST_PARSE       = 4'd3,
    ST_EXEC_DATA   = 4'd4,
    ST_ELEM_DONE   = 4'd5,
    ST_ERROR_M     = 4'd6
  } ll_state_e;

  logic [63:0] llp_ptr;
  logic        expected_cb;
  hdma_elem_t  cur_elem;
```

- [x] **Step 2: Implement descriptor fetch MRd request and completion capture**

When `ctrl.enable && ctrl.doorbell && ctrl.lle`:
1. `llp_ptr <= ctrl.llp_base`, `expected_cb <= 1'b1`.
2. Issue MRd for 24 bytes (or 64B beat) at `addr = llp_ptr`.
3. When `cpl.valid` arrives for the descriptor, unpack DWORDs into `cur_elem`:
   ```systemverilog
   cur_elem.control = cpl.rdata[31:0];
   cur_elem.size    = cpl.rdata[63:32];
   cur_elem.sar_lo  = cpl.rdata[95:64];
   cur_elem.sar_hi  = cpl.rdata[127:96];
   cur_elem.dar_lo  = cpl.rdata[159:128];
   cur_elem.dar_hi  = cpl.rdata[191:160];
   ```

- [x] **Step 3: Implement Cycle-Bit (`CB`), Link (`LLP`), and Toggle (`TCB`) Logic**

1. If `cur_elem.control[CTRL_CB] != expected_cb`:
   - Software has not written more descriptors. Return to `ST_IDLE_M` gracefully.
2. If `cur_elem.control[CTRL_LLP] == 1'b1`:
   - Jump `llp_ptr <= {cur_elem.sar_hi, cur_elem.sar_lo}`.
   - If `cur_elem.control[CTRL_TCB] == 1'b1`, flip `expected_cb <= ~expected_cb`.
   - Transition back to `ST_FETCH_REQ`.
3. If `cur_elem.control[CTRL_LLP] == 1'b0`:
   - Hand off to data mover (`cur_sar <= {cur_elem.sar_hi, cur_elem.sar_lo}`, `cur_dar <= {cur_elem.dar_hi, cur_elem.dar_lo}`, `rem_bytes <= cur_elem.size`).
   - On completion of this element, advance `llp_ptr <= llp_ptr + 64'd24`.
   - If `cur_elem.control[CTRL_LIE] == 1'b1`, assert `done_irq_r <= 1'b1`.
   - Return to `ST_FETCH_REQ`.

- [x] **Step 4: Implement equivalent linked-list fetch for C2H (Write Channel)**

Apply the same linked-list fetch and parsing logic to the C2H generator block in `dma_channel.sv`.

- [x] **Step 5: Run lint and verify regression**

Run: `make lint && make sim-m2`
Expected: 0 lint errors, all 7 M2 tests pass.

- [x] **Step 6: Commit**

```bash
git add rtl/blocks/dma_channel.sv
git commit -m "feat(channel): add scatter-gather linked-list fetch and parse FSM"
```

---

### Task 4: Weighted Round-Robin (WRR) Arbiter (`rtl/blocks/chan_arbiter.sv`)

**Files:**
- Modify: `rtl/blocks/chan_arbiter.sv`
- Test: `sim/tb_m2.sv`

**Interfaces:**
- Consumes: `ch_req[NUM_CH]`, `ch_ctrl[NUM_CH].qos[3:0]`.
- Produces: Weighted round-robin grant with multi-beat burst protection and parallel completion demux.

- [x] **Step 1: Add per-channel credit tracking to `chan_arbiter.sv`**

```systemverilog
  logic [7:0] credit [NUM_CH];
  logic [NUM_CH-1:0] req_active;

  always_comb begin
    for (int i = 0; i < NUM_CH; i++) begin
      req_active[i] = ch_req[i].valid;
    end
  end
```

- [x] **Step 2: Implement WRR deficit selection logic**

1. Scan channels starting from `rr_ptr`.
2. Grant first channel with `req_active[i] && credit[i] > 0`.
3. When `m_req_ready && ch_req[gnt].valid`:
   - Decrement `credit[gnt] <= credit[gnt] - 1'b1`.
   - If `credit[gnt] == 1'b1` (or becomes 0), reload `credit[gnt] <= (ch_ctrl_qos[gnt] == 0) ? 8'd1 : {4'b0, ch_ctrl_qos[gnt]}` and advance `rr_ptr <= (gnt + 1) % NUM_CH`.
4. If all requesting channels have `credit == 0`, immediately refresh all credits from `ch_ctrl_qos` in the same cycle.

- [x] **Step 3: Preserve atomic multi-beat burst lock**

Hold grant when `!ch_req[gnt].req.is_read && !ch_req[gnt].wlast`.

- [x] **Step 4: Upgrade completion demux to sliced channel ID**

Route completion `m_cpl` to Channel `i`:
```systemverilog
  wire [1:0] cpl_dest_ch = m_cpl.meta.tag[9:8];
  always_comb begin
    for (int i = 0; i < NUM_CH; i++) begin
      ch_cpl[i]       = m_cpl;
      ch_cpl[i].valid = m_cpl.valid && (cpl_dest_ch == i[1:0]);
    end
  end
```

- [x] **Step 5: Run lint and verify regression**

Run: `make lint && make sim-m2`
Expected: PASS with 0 warnings.

- [x] **Step 6: Commit**

```bash
git add rtl/blocks/chan_arbiter.sv
git commit -m "feat(arbiter): implement weighted round-robin QoS and tag-sliced completion demux"
```

---

### Task 5: Top-Level Multi-Channel Scaling (`rtl/dma_top.sv`)

**Files:**
- Modify: `rtl/dma_top.sv`
- Test: `sim/lint_top.sv`

**Interfaces:**
- Consumes: All 8 channels (`NUM_RD_CH=4`, `NUM_WR_CH=4`).
- Produces: Integrated top-level with WRR weight wiring and full 8-channel AXI-MM ports.

- [x] **Step 1: Wire per-channel QoS weights into `chan_arbiter`**

In `rtl/dma_top.sv`, pass `ch_ctrl[i].qos` to `u_arb`.

- [x] **Step 2: Ensure all 8 channels are connected to top-level ports and CSR arrays**

Verify `g_chan` generate loop instantiates 8 channels and connects `ch_ctrl[0..7]`, `ch_stat[0..7]`, `ch_irq[0..7]`, `dev_mm_m2s[0..7]`, and `dev_mm_s2m[0..7]`.

- [x] **Step 3: Run full lint check**

Run: `make lint && make lint-if`
Expected: 0 errors, 0 warnings.

- [x] **Step 4: Commit**

```bash
git add rtl/dma_top.sv
git commit -m "feat(top): integrate 8-channel multi-channel scaling with WRR arbiter"
```

---

### Task 6: Milestone M3 Verification Testbench (`sim/tb_m3.sv` & `Makefile`)

**Files:**
- Create: `sim/tb_m3.sv`
- Modify: `Makefile`
- Test: `make sim-m3`

**Interfaces:**
- Consumes: `dma_top`, `sim/axi_ram.sv`.
- Produces: Directed verification suite with out-of-order & RCB-split completion responder, executing Tests 1 through 7.

- [x] **Step 1: Create `sim/tb_m3.sv` scaffolding and 8-channel RAM instances**

Instantiate `dma_top` with 8 `axi_ram` instances for `dev_mm_m2s[0..7]`.

- [x] **Step 2: Implement Out-of-Order & Split Completion Engine in Root Complex BFM**

```systemverilog
  // Completion queue structure for shuffling
  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [63:0]      addr;
    logic [12:0]      len_bytes;
  } pending_cpl_t;

  pending_cpl_t cpl_fifo [32];
```
Add support for:
- Shuffling pending completions out of order.
- Fragmenting requests into 64-byte chunks with separate completion beats.

- [x] **Step 3: Implement linked-list helper task `build_sg_elem`**

```systemverilog
  task automatic build_elem(
    input logic [63:0] addr,
    input logic [31:0] ctrl,
    input logic [31:0] size,
    input logic [63:0] sar,
    input logic [63:0] dar
  );
    // Write 24 bytes (6 DWORDs) into host_mem
    host_mem_write32(addr + 0,  ctrl);
    host_mem_write32(addr + 4,  size);
    host_mem_write32(addr + 8,  sar[31:0]);
    host_mem_write32(addr + 12, sar[63:32]);
    host_mem_write32(addr + 16, dar[31:0]);
    host_mem_write32(addr + 20, dar[63:32]);
  endtask
```

- [x] **Step 4: Implement Tests 1 through 7 in `tb_m3.sv`**

1. **Test 1:** Contiguous 4-element H2C linked-list transfer.
2. **Test 2:** Non-contiguous linked list with Link pointer (`LLP=1`).
3. **Test 3:** Cyclic linked list with cycle-bit toggling (`TCB=1`).
4. **Test 4:** Out-of-order & RCB-split completion reassembly test.
5. **Test 5:** Contiguous 4-element C2H linked-list transfer.
6. **Test 6:** Concurrent 8-channel operation (all 4 Read + 4 Write channels active).
7. **Test 7:** Weighted Round-Robin (WRR) QoS verification (Ch0 weight 8 vs Ch1 weight 1).

- [x] **Step 5: Add `sim-m3` target to `Makefile`**

```makefile
sim-m3:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m3.sv --top-module tb_m3 -o tb_m3
	@./obj_dir/tb_m3
```

- [x] **Step 6: Run `make sim-m3` and verify all tests pass**

Run: `make sim-m3`
Expected: `[tb_m3] ALL TESTS PASSED (0 errors)`.

- [x] **Step 7: Run complete regression suite**

Run: `make lint && make lint-if && make sim-csr && make sim-m2 && make sim-m3`
Expected: All targets succeed with 0 errors and 0 warnings.

- [x] **Step 8: Commit**

```bash
git add sim/tb_m3.sv Makefile
git commit -m "test(m3): add comprehensive Milestone M3 verification suite"
```
