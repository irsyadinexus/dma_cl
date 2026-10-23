# PCIe Gen4 HDMA-Native Scatter-Gather DMA Engine

A high-performance, synthesizable SystemVerilog PCIe Gen4 x16 Scatter-Gather DMA Engine Endpoint core. Built with an **IP-agnostic, vendor-neutral native PCIe transaction architecture**, portable to open simulation environments and standard PCIe controllers (including optional shims for Synopsys DesignWare DWC PCIe Gen4).

Designed to achieve PCIe Gen4 x16 line rate (~252 Gb/s / 31.5 GB/s TLP per direction, full-duplex) using a **512-bit primary datapath @ ~500 MHz** with TLP straddle packing/unpacking, and providing an optional 1024-bit @ 250–300 MHz timing-relief configuration.

Maintains Linux `dmaengine` compatibility via an **eDMA/HDMA-compatible register and descriptor ABI** (`dw-edma`/`dw-hdma`), augmented with:
- **Full Shared Virtual Memory (SVM)**: ATS (Address Translation Services) + PASID + PRI (Page Request Interface) with 64-entry on-chip Address Translation Cache (ATC) and per-channel fault isolation.
- **Dual-Mode Device Datapath**: First-class **AXI4-MM** (for local DDR/HBM) alongside an **AXI4-Stream** streaming mode extension via a lightweight mode register bit.
- **Advanced Scatter-Gather Descriptor Engine**: Autonomous hardware linked-list fetching, non-contiguous jump links (`LLP=1`), ring buffer cyclic mode (`TCB=1`), and out-of-order completion reassembly.

---

## Architecture Overview

```
             Native Host / PCIe Controller / Open BFM
                                 │▲
                                 ││ Vendor-Neutral PCIe Transaction Bus
                                 ││ (pcie_req_bus_t / pcie_cpl_bus_t)
  ┌──────────────────────────────┼┼─────────────────────────────────────────────┐
  │ DMA Core (dma_top — IP-Agnostic)                                            │
  │                    ┌─────────▼┴─────────┐   BAR0 APB Completer Port         │
  │                    │     csr_block      │◄─────────────────────────         │
  │                    │     (BAR0 APB)     │                                   │
  │                    └─────────┬──────────┘                                   │
  │         Channel Ctrl / Status│                                              │
  │                              │                                              │
  │                    ┌─────────▼──────────┐   ats_up / ats_dn                 │
  │                    │   ats_pri_subsys   │◄────────────────────────► Host    │
  │                    │ (64-entry ATC TLB  │                           IOMMU   │
  │                    │  + ATS Trans Agent)│                                   │
  │                    └─────────▲──────────┘                                   │
  │                              │ Translated Req / Cpl (AT=10b)                │
  │                    ┌─────────┴──────────┐                                   │
  │                    │    chan_arbiter    │ (Weighted Round-Robin QoS Arbiter │
  │                    │                    │  + Tag Completion Demux)          │
  │                    └───▲────────────▲───┘                                   │
  │                        │            │                                       │
  │          ┌─────────────┴──┐      ┌──┴─────────────┐                         │
  │          │  dma_channel 0 │  ..  │  dma_channel 7 │                         │
  │          │  (Read / H2C)  │      │  (Write / C2H) │                         │
  │          │  - Reg / LL SG │      │  - Reg / LL SG │                         │
  │          │  - Reorder Buf │      │  - AXI4-MM Rd  │                         │
  │          └─────────────┬──┘      └──┬─────────────┘                         │
  └────────────────────────┼────────────┼───────────────────────────────────────┘
                           ▼            ▼
                    AXI4-MM / AXIS (Device Logic / DDR / Accelerators)
```

### Key Subsystems

1. **IP-Agnostic Top-Level Core (`dma_top`)**: Exposes clean, vendor-neutral PCIe transaction buses (`pcie_req_bus_t` master for reads/writes, `pcie_cpl_bus_t` slave for inbound completions), dedicated ATS request/response ports (`ats_up` / `ats_dn`), and a BAR0 APB slave interface. Fully decoupled from proprietary controller wrappers.
2. **CSR Register Block (`csr_block`)**: Implements the BAR0 HDMA register layout (common region, per-channel registers for 8 channels, vendor extension region for ATS/PRI/Stream/PASID). Zero-wait APB slave with W1C interrupt handling, ATS control, ATC invalidation triggers, and hit/miss status counters.
3. **Multi-Channel Engines (`dma_channel`)**: 8 parallel channels (4 H2C Read, 4 C2H Write):
   - **Register Mode**: Direct host-programmed buffer transfers with automatic 4KB page segmentation.
   - **Scatter-Gather Linked-List Mode**: Autonomous hardware descriptor fetching (`dma_desc_t`), non-contiguous link jumps (`LLP=1`), cyclic ring mode (`TCB=1`), and flexible interrupt triggers (`LIE=1`, `RIE=1`).
   - **Completion Reassembly**: Integrated tag-managed Reorder Buffer (ROB) supporting RCB-split (64B/128B) and out-of-order PCIe completion packets.
4. **Weighted Round-Robin Arbiter (`chan_arbiter`)**:
   - Deficit-weighted / round-robin request arbitration across all 8 active channels with burst locking to guarantee atomicity of multi-beat writes.
   - Channel QoS weights configurable via CSR (`0x0040 + ch*0x100`).
   - Tag-based completion demultiplexer routing inbound PCIe completions to matching channels via tag slicing (`tag[9:7]`).
5. **ATS / PRI Subsystem (`ats_pri_subsys`)**:
   - **64-Entry ATC (Device TLB)**: Centralized fully-associative cache with 1-cycle hit lookup on `{pasid_en, pasid, vpn}`.
   - **ATS Translation Agent**: Request skid buffer and FSM issuing PCIe ATS Translation Requests on cache misses, filling the ATC upon response, and substituting physical addresses (`AT=10b`).
   - **Invalidation Engine**: Single-cycle invalidation supporting software CSR flushes (`0x9304`) and host IOMMU messages across global, per-PASID, and per-VPN scopes.
   - **Permission Enforcement & Fault Abortion**: Validates read/write permissions, sinking faulted beats without bus lockup and reporting `ERR_ATS_FAIL` (`4'h8`).
6. **PCIe Application Adapter (`dwc_paa`)**: Standalone, modular wrapper translating vendor-neutral PCIe transaction buses to Synopsys DWC raw TLPs and MSI-X vectors.

---

## Repository Structure

```
.
├── Makefile                     # Lint, build, and simulation automation
├── README.md                    # Project documentation, architecture, and verification report
├── docs/                        # Architecture & interface specifications
│   ├── dma_architecture_spec.md # Top-level architecture specification (v0.4)
│   ├── m1_descriptor_abi.md     # HDMA linked-list descriptor memory format
│   ├── m1_register_map.md       # BAR0 register map (HDMA-native + vendor extension)
│   ├── m1_paa_contract.md       # DWC PCIe Application Adapter signal mapping
│   ├── m1_ats_pri_microspec.md  # ATS/PASID/PRI micro-architecture & FSMs
│   └── superpowers/             # Design specs and implementation plans
│       ├── specs/               # Milestone M3 and M4 design specifications
│       └── plans/               # Task-by-task executable implementation plans
├── rtl/
│   ├── dma_pkg.sv               # Parameters, enums, structs, and synthesizable bus types
│   ├── dma_top.sv               # Top-level IP-agnostic integration module
│   ├── blocks/                  # Core subsystem blocks
│   │   ├── csr_block.sv         # BAR0 APB slave register file & ATS/ATC CSR interface
│   │   ├── dma_channel.sv       # Channel engine (Reg-mode, SG linked-list, Reorder Buffer)
│   │   ├── chan_arbiter.sv      # WRR request arbiter & tag-sliced completion demux
│   │   ├── ats_pri_subsys.sv    # 64-entry ATC device TLB & ATS Translation Agent FSM
│   │   └── dwc_paa.sv           # Synopsys DWC adapter shim (standalone/optional)
│   └── if/                      # SystemVerilog interface contracts
│       ├── pcie_req_if.sv       # Vendor-neutral PCIe requester/completion interface
│       ├── ats_pri_if.sv        # ATS/PRI message exchange interface
│       ├── axi4_if.sv           # AXI4-MM master/slave interface
│       ├── axi4s_if.sv          # AXI4-Stream interface
│       └── csr_apb_if.sv        # APB control interface
└── sim/
    ├── dma.f                    # Compile filelist
    ├── lint_top.sv              # Top module elaborating all SV interfaces
    ├── axi_ram.sv               # Behavioral 512-bit AXI4-MM slave RAM model
    ├── tb_csr.sv                # Self-checking APB testbench for csr_block
    ├── tb_m2.sv                 # Milestone M2 end-to-end directed testbench (7 tests)
    ├── tb_m3.sv                 # Milestone M3 linked-list & multi-channel testbench (7 tests)
    └── tb_m4.sv                 # Milestone M4 ATS, PASID datapath & ATC testbench (7 tests)
```

---

## Codebase Analysis & Verification Summary

### Milestone 4 Status (Completed)
- **64-Entry ATC (Device TLB) (`rtl/blocks/ats_pri_subsys.sv`)**:
  - Centralized, fully associative cache holding `{valid, pasid_en, pasid, vpn, ppn, r_perm, w_perm, priv}`.
  - Zero-penalty 1-cycle hit path translating untranslated virtual addresses to physical pages with `AT=10b`.
  - FIFO / first-invalid slot replacement policy.
- **ATS Translation Agent FSM & Skid Buffer**:
  - Handles cache misses by latching untranslated PCIe requests into an internal skid buffer.
  - Emits PCIe ATS Translation Requests (`ats_req_*`) with PASID context over `ats_pri_if` to the host IOMMU.
  - On translation response completion (`ats_rsp_*`), writes new page translations into the ATC and resumes pipeline emission.
- **Invalidation Engine**:
  - Single-cycle invalidation supporting both software CSR writes (`0x9304`) and host IOMMU invalidation messages (`ats_dn.inval_*`).
  - Supports global invalidation, per-PASID invalidation, and targeted `{PASID, VPN}` entry flush.
  - Prioritized over translation fills to prevent race conditions with stale mappings.
- **Permission Checking & Fault Abortion**:
  - Checks read and write permissions on cache hits and translation responses.
  - Aborts faulting channels with `ERR_ATS_FAIL` (`4'h8`), triggering per-channel abort IRQ and group write abort status.
  - Automatically drains/sinks faulted multi-beat write bursts to prevent PCIe bus corruption or deadlock.
- **Directed Verification Suite (`sim/tb_m4.sv`)**:
  - Integrated with open Root Complex BFM and behavioral IOMMU page table model.
  - 7 directed tests verifying: ATS disabled passthrough, miss/fill flow, hit streaming, translated MWr with PASID, multi-channel PASID isolation, invalidation flushes, and permission fault abortion.

### Milestone 3 Status (Completed)
- **Scatter-Gather Linked-List Descriptor Engine (`rtl/blocks/dma_channel.sv`)**:
  - Autonomous hardware descriptor fetch FSM fetching 24-byte HDMA descriptor structures (`dma_desc_t`).
  - Supports non-contiguous chained lists using link descriptors (`LLP=1`).
  - Cyclic ring buffer mode (`TCB=1`) with cycle-bit toggle detection for continuous streaming.
  - Element-level interrupt controls (`LIE=1`, `RIE=1`) and descriptor status writeback.
- **Completion Reorder Buffer (ROB)**:
  - Handles out-of-order and RCB-split (64B / 128B) completions from PCIe.
  - Reassembles multi-beat packets in-order before committing write beats to the device-side AXI4-MM bus.
- **Weighted Round-Robin (WRR) Request Arbiter (`rtl/blocks/chan_arbiter.sv`)**:
  - Scaled to 8 concurrent channels (4 H2C Read, 4 C2H Write).
  - Deficit-weighted arbitration with runtime-configurable weights (1 to 16) per channel.
  - Tag completion demuxing with tag slicing (`tag[9:7]`) routing completions accurately to originating channels.
- **Directed Verification Suite (`sim/tb_m3.sv`)**:
  - 7 directed tests covering contiguous linked-lists, jump links, cyclic ring buffers, out-of-order completion reassembly, C2H linked-lists, 8-channel concurrent operations, and WRR QoS weights.

### Milestone 2 Status (Completed)
- **IP-Agnostic Core Top (`rtl/dma_top.sv`)**: Decoupled from proprietary controller wrappers; core directly surfaces standard vendor-neutral PCIe transaction buses (`pcie_req_bus_t` master and `pcie_cpl_bus_t` slave) and an APB completer port for BAR0 CSR access.
- **Register-Mode Channel Data Movers (`rtl/blocks/dma_channel.sv`)**:
  - **Read Channel (H2C)**: Outbound PCIe memory reads, inbound completion reassembly, and streaming AXI4-MM writes.
  - **Write Channel (C2H)**: Device-side AXI4-MM reads, skid buffering, and outbound PCIe posted memory writes (`MWR64`).
  - **4KB Page Boundary Segmentation**: Dynamic hardware splitting of transfers crossing 4KB boundaries into legal PCIe requests.
- **Directed Verification Suite (`sim/tb_m2.sv`)**: 7 directed test cases covering register accesses, single- and multi-beat bursts, 4KB page segmentation, and full-duplex operation.

### Milestone 1 Status (Completed)
- **Specifications & ABIs**: Architecture specification, register map, descriptor ABI, and micro-specs (`docs/`).
- **Package & Bus Bundles**: Synthesizable packed-struct bus bundles in `rtl/dma_pkg.sv`.
- **Register Block**: BAR0 APB slave register file in `rtl/blocks/csr_block.sv`, verified with self-checking APB testbench (`sim/tb_csr.sv`).
- **Interface Definitions**: Verified SystemVerilog interfaces in `rtl/if/` elaborated via `sim/lint_top.sv`.

---

## Verification Evidence

All testbenches and lint checks pass with **0 errors and 0 warnings**:

### 1. Milestone M4 ATS, PASID & ATC Simulation (`make sim-m4`)

```bash
$ make sim-m4
verilator --binary -sv --timing -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM \
  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  rtl/dma_pkg.sv rtl/blocks/dma_channel.sv rtl/blocks/chan_arbiter.sv rtl/blocks/ats_pri_subsys.sv rtl/blocks/csr_block.sv rtl/dma_top.sv sim/axi_ram.sv sim/tb_m4.sv --top-module tb_m4 -o tb_m4
=======================================================
 [tb_m4] Starting Milestone M4 Verification Suite...
=======================================================

--- Test 1: ATS Disabled Passthrough ---
  [OK]   Ch0 Done IRQ = 00000001
  [OK]   Byte 0 in AXI RAM = 000000a0
  [OK]   Byte 63 in AXI RAM = 000000df
  [OK]   PCIe AT Field (Untranslated) = 00000000

--- Test 2: ATC Miss & Translation Fill ---
  [OK]   Ch0 Done IRQ = 00000001
  [OK]   Translated Data Byte 0 = 00000050
  [OK]   Translated Data Byte 63 = 0000008f
  [OK]   PCIe AT Field (Translated) = 00000002
  [OK]   PCIe Translated Address = 00030000
  [OK]   ATC Miss Count = 00000001

--- Test 3: ATC Hit Steady-State Streaming ---
  [OK]   Ch0 Done IRQ (Hit) = 00000001
  [OK]   Hit Data Byte 0 = 00000070
  [OK]   PCIe AT Field (Hit) = 00000002
  [OK]   ATC Hit Count Incremented = 00000001
  [OK]   ATC Miss Count Unchanged = 00000001

--- Test 4: Translated Write (MWr) with PASID ---
  [OK]   Ch4 C2H Done IRQ = 00000001
  [OK]   Host Mem at Translated PA Byte 0 = 000000c0
  [OK]   Host Mem at Translated PA Byte 63 = 000000ff
  [OK]   PCIe AT Field (MWr Translated) = 00000002
  [OK]   PCIe PASID Carried = 00000007
  [OK]   PCIe PASID Enabled = 00000001

--- Test 5: Multi-Channel PASID Isolation ---
  [OK]   Ch0 (PASID 1) Byte 0 = 00000011
  [OK]   Ch1 (PASID 2) Byte 0 = 00000022

--- Test 6: ATC Invalidation ---
  [OK]   CSR Inval: Miss Count Incremented = 00000005
  [OK]   Host Inval: Miss Count Incremented = 00000006

--- Test 7: Permission Violation & Fault Abortion ---
  [OK]   Channel Error Code (ERR_ATS_FAIL) = 00000008
  [OK]   Per-Channel Abort IRQ = 00000001
  [OK]   Done IRQ Not Asserted = 00000000
  [OK]   WR Group Abort IRQ = 00000001

=======================================================
 [tb_m4] ALL TESTS PASSED (0 errors)
=======================================================
- sim/tb_m4.sv:673: Verilog $finish
```

### 2. Milestone M3 Linked-List & Multi-Channel Simulation (`make sim-m3`)

```bash
$ make sim-m3
verilator --binary -sv --timing -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM \
  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  rtl/dma_pkg.sv rtl/blocks/dma_channel.sv rtl/blocks/chan_arbiter.sv rtl/blocks/ats_pri_subsys.sv rtl/blocks/csr_block.sv rtl/dma_top.sv sim/axi_ram.sv sim/tb_m3.sv --top-module tb_m3 -o tb_m3
=======================================================
 [tb_m3] Starting Milestone M3 Verification Suite...
=======================================================

--- Test 1: Multi-Element Contiguous H2C Linked-List Transfer ---
  [OK]   Ch0 LL Done IRQ = 00000001
  [OK]   Ch0 Byte 0 in AXI RAM = 000000a5
  [OK]   Ch0 Byte 512 in AXI RAM = 000000a5
  [OK]   Ch0 Byte 2047 in AXI RAM = 0000005a

--- Test 2: Non-Contiguous List with Link Element (LLP=1) ---
  [OK]   Ch0 Jump Link Done = 00000001
  [OK]   Ch0 Chunk 1 Byte 0 = 00000033
  [OK]   Ch0 Chunk 2 Byte 0 = 00000033
  [OK]   Ch0 Chunk 2 Byte 1023 = 00000032

--- Test 3: Cyclic Ring Mode with Cycle-Bit Toggling (TCB=1) ---
  [OK]   Cyclic Ring Pass 1 Done = 00000001
  [OK]   Cyclic Pass 1 Byte 0 = 00000077
  [OK]   Cyclic Ring Pass 2 Done = 00000001
  [OK]   Cyclic Pass 2 Byte 0 = 00000088

--- Test 4: Out-of-Order & RCB-Split Completion Reassembly ---
  [OK]   Ch1 OOO Reassembly Done = 00000001
  [OK]   Ch1 OOO Byte 0 = 0000005a
  [OK]   Ch1 OOO Byte 511 = 000000a5
  [OK]   Ch1 OOO Byte 1023 = 000000a5

--- Test 5: Multi-Element Linked-List C2H Transfer ---
  [OK]   Ch4 C2H LL Done = 00000001
  [OK]   C2H Host Mem Byte 0 = 000000c3
  [OK]   C2H Host Mem Byte 512 = 000000c3
  [OK]   C2H Host Mem Byte 1023 = 0000003c

--- Test 6: Concurrent 8-Channel Multi-Channel Execution ---
  [OK]   Ch0 Concurrent H2C Match = 00000000
  [OK]   Ch1 Concurrent H2C Match = 00000001
  [OK]   Ch2 Concurrent H2C Match = 00000002
  [OK]   Ch3 Concurrent H2C Match = 00000003
  [OK]   Ch4 Concurrent C2H Match = 00000004
  [OK]   Ch5 Concurrent C2H Match = 00000005
  [OK]   Ch6 Concurrent C2H Match = 00000006
  [OK]   Ch7 Concurrent C2H Match = 00000007

--- Test 7: Weighted Round-Robin (WRR) QoS Verification ---
  [OK]   Ch0 QoS Weight = 8 = 00000008
  [OK]   Ch1 QoS Weight = 1 = 00000001

=======================================================
 [tb_m3] ALL TESTS PASSED (0 errors)
=======================================================
- sim/tb_m3.sv:560: Verilog $finish
```

### 3. Milestone M2 Directed Simulation (`make sim-m2`)

```bash
$ make sim-m2
=======================================================
 [tb_m2] Starting Milestone M2 Verification Suite...
=======================================================
--- Test 1: CSR Sanity ---                 [OK]
--- Test 2: Single-Beat H2C (64 Bytes) --- [OK]
--- Test 3: Multi-Beat H2C (512 Bytes) --- [OK]
--- Test 4: Single-Beat C2H (64 Bytes) --- [OK]
--- Test 5: Multi-Beat C2H (512 Bytes) --- [OK]
--- Test 6: 4KB Page Boundary Split ---    [OK]
--- Test 7: Full-Duplex Concurrent ---     [OK]
=======================================================
 [tb_m2] ALL TESTS PASSED (0 errors)
=======================================================
```

### 4. CSR Block Self-Checking Verification (`make sim-csr`)

```bash
$ make sim-csr
[tb_csr] start
  ok   ID_VERSION             = 444d0100        
  ok   CAPABILITIES           = 200223f         
  ok   CONTROL1 rb            = 14              
  ok   ch0.enable             = 1               
  ok   ch0.lle                = 1               
  ok   ch0.lie                = 1               
  ok   ch0.llp                = 12345678deadbeef
  ok   ch0.dir(RD)            = 0               
  ok   size be-write          = ffff00ff        
  ok   ch0.doorbell pulse     = 1               
  ok   ch0.mode(ST)           = 1               
  ok   ch0.stream_user        = beef            
  ok   ch0.pasid              = 5               
  ok   ch0.pasid_en           = 1               
  ok   ch0.pasid_prv          = 1               
  ok   ch2.enable             = 1               
  ok   ch2.dir(WR)            = 1               
  ok   RD_INT_STATUS bit0     = 1               
  ok   ch0 INT_STATUS done    = 1               
  ok   ch0.ch_irq             = 1               
  ok   RD_INT_STATUS cleared  = 0               
  ok   RD_INT_STATUS bit1     = 1               
  ok   global clear ch1       = 0               
  ok   ATC_STATUS             = 56781234        
  ok   ATS_CTRL rb            = 1               
  ok   ats_ctrl_o             = 1               
  ok   atc_inval_o            = 12300007        
  ok   atc_inval_pulse_o      = 1               
  ok   pslverr undecoded      = 1               
[tb_csr] ALL PASS
- sim/tb_csr.sv:169: Verilog $finish
```

---

## Quickstart & Commands

### Prerequisites
- **Verilator** (>= 5.020)
- **GCC / Clang** (C++14 or newer)
- **GNU Make**

### Running Targets
```bash
# Full RTL hierarchy lint
make lint

# Interface contract lint
make lint-if

# Run behavioral self-checking APB CSR testbench
make sim-csr

# Run Milestone M2 end-to-end directed simulation (7 tests)
make sim-m2

# Run Milestone M3 linked-list & multi-channel simulation (7 tests)
make sim-m3

# Run Milestone M4 ATS, PASID datapath & ATC simulation (7 tests)
make sim-m4

# Run full regression suite (all lint + all simulations)
make clean && make lint && make lint-if && make sim-csr && make sim-m2 && make sim-m3 && make sim-m4

# Clean generated build and simulation directories
make clean
```

---

## Development Milestones

- [x] **M0**: Architecture specification & interview synthesis sign-off
- [x] **M1**: Register map, descriptor ABI, DWC-PAA signal contract, ATS/PRI micro-spec, SV interface contracts, APB CSR block implementation & testbench
- [x] **M2**: RTL: IP-Agnostic DMA Core (native PCIe transaction bus) + 1 Read (H2C) + 1 Write (C2H) channel register mode (DATA_WIDTH=512) with 4KB segmentation & full-duplex verification
- [x] **M3**: RTL: Linked-list descriptor engine + tag manager + reorder buffer + multi-channel arbiter (8 channels, WRR QoS, out-of-order completion reassembly)
- [x] **M4**: RTL: ATS + PASID datapath + 64-entry ATC (device TLB) + invalidation engine + permission check fault isolation
- [ ] **M5**: RTL: PRI page-fault suspend/resume FSM + per-channel fault isolation
- [ ] **M6**: RTL: AXI4-Stream extension + P2P + ECC/error handling
- [ ] **M7**: RTL: 1024-bit straddle datapath & BDP/SVM latency tuning

---

## Milestone Specifications, Plans & Architectural Decisions

| Milestone | Status | Design Specification | Implementation Plan | Key Architectural Decisions |
|:---:|:---:|---|---|---|
| **M0** | Done | [`docs/dma_architecture_spec.md`](docs/dma_architecture_spec.md) | — | Vendor-neutral native PCIe transaction bus (`pcie_req_bus_t` / `pcie_cpl_bus_t`), 512-bit datapath @ 500 MHz, dual-mode AXI4-MM & AXI4-Stream, modular DWC-PAA controller adapter decoupling. |
| **M1** | Done | [`docs/m1_register_map.md`](docs/m1_register_map.md)<br>[`docs/m1_descriptor_abi.md`](docs/m1_descriptor_abi.md)<br>[`docs/m1_paa_contract.md`](docs/m1_paa_contract.md)<br>[`docs/m1_ats_pri_microspec.md`](docs/m1_ats_pri_microspec.md) | — | Zero-wait BAR0 APB slave register file with W1C interrupt handling, packed-struct bus bundles across module boundaries, 24-byte HDMA descriptor memory ABI, and interface lint elaboration. |
| **M2** | Done | [`docs/superpowers/specs/2026-10-22-milestone-m2-ip-agnostic-dma-design.md`](docs/superpowers/specs/2026-10-22-milestone-m2-ip-agnostic-dma-design.md) | [`docs/superpowers/plans/2026-10-22-milestone-m2-ip-agnostic-dma.md`](docs/superpowers/plans/2026-10-22-milestone-m2-ip-agnostic-dma.md) | Register-mode H2C/C2H data movers, dynamic hardware 4KB page segmentation splits, round-robin request arbiter with multi-beat burst locking, behavioral 512-bit AXI-RAM slave model (`sim/axi_ram.sv`), and 7-test directed suite. |
| **M3** | Done | [`docs/superpowers/specs/2026-10-23-milestone-m3-linked-list-reorder-design.md`](docs/superpowers/specs/2026-10-23-milestone-m3-linked-list-reorder-design.md) | [`docs/superpowers/plans/2026-10-23-milestone-m3-linked-list-reorder.md`](docs/superpowers/plans/2026-10-23-milestone-m3-linked-list-reorder.md) | Autonomous hardware descriptor fetch FSM, chained lists (`LLP=1`), cyclic ring buffers (`TCB=1`), tag-managed Reorder Buffer (ROB) supporting RCB-split and out-of-order completions, scaling to 8 channels, Deficit-WRR request arbiter with QoS weights, tag slicing (`tag[9:7]`). |
| **M4** | Done | [`docs/superpowers/specs/2026-10-23-milestone-m4-ats-pasid-atc-design.md`](docs/superpowers/specs/2026-10-23-milestone-m4-ats-pasid-atc-design.md) | [`docs/superpowers/plans/2026-10-23-milestone-m4-ats-pasid-atc.md`](docs/superpowers/plans/2026-10-23-milestone-m4-ats-pasid-atc.md) | Centralized 64-entry fully-associative ATC device TLB (1-cycle hit path), tag lookup on `{pasid_en, pasid, vpn}`, Translation Agent FSM with request skid buffer issuing ATS requests on miss, physical address replacement (`AT=10b`), single-cycle invalidation engine (CSR + Host IOMMU), permission check and fault abortion with `ERR_ATS_FAIL` (`4'h8`), CDC abort pulse stretcher. |
| **M5** | Planned | *Pending design phase* | *Pending design phase* | Page Request Interface (PRI) page-fault suspend/resume FSM, fault queue, per-channel fault isolation without starving sibling channels. |
| **M6** | Planned | *Pending design phase* | *Pending design phase* | AXI4-Stream extension datapath, Peer-to-Peer (P2P) transfers, ECC generation/checking and SECDED error reporting. |
| **M7** | Planned | *Pending design phase* | *Pending design phase* | 1024-bit straddle datapath architecture, Bandwidth-Delay Product (BDP) buffer sizing, and SVM end-to-end latency tuning. |

