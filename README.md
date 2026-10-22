# PCIe Gen4 HDMA-Native Scatter-Gather DMA Engine

A high-performance, synthesizable SystemVerilog PCIe Gen4 x16 Scatter-Gather DMA Engine Endpoint core. Built with an **IP-agnostic, vendor-neutral native PCIe transaction architecture**, portable to open simulation environments and standard PCIe controllers (including optional shims for Synopsys DesignWare DWC PCIe Gen4).

Designed to achieve PCIe Gen4 x16 line rate (~252 Gb/s / 31.5 GB/s TLP per direction, full-duplex) using a **512-bit primary datapath @ ~500 MHz** with TLP straddle packing/unpacking, and providing an optional 1024-bit @ 250–300 MHz timing-relief configuration.

Maintains Linux `dmaengine` compatibility via an **eDMA/HDMA-compatible register and descriptor ABI** (`dw-edma`/`dw-hdma`), augmented with:
- **Full Shared Virtual Memory (SVM)**: ATS (Address Translation Services) + PASID + PRI (Page Request Interface) with per-channel fault isolation.
- **Dual-Mode Device Datapath**: First-class **AXI4-MM** (for local DDR/HBM) alongside an **AXI4-Stream** streaming mode extension via a lightweight mode register bit.

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
  │                    ┌─────────▼──────────┐                                   │
  │                    │   ats_pri_subsys   │ (ATC TLB + ATS/PRI Fault Engine)  │
  │                    └─────────▲──────────┘                                   │
  │                              │ Translated Req / Cpl                         │
  │                    ┌─────────┴──────────┐                                   │
  │                    │    chan_arbiter    │ (Round-Robin Arb + Tag Cpl Demux) │
  │                    └───▲────────────▲───┘                                   │
  │                        │            │                                       │
  │          ┌─────────────┴──┐      ┌──┴─────────────┐                         │
  │          │  dma_channel 0 │  ..  │  dma_channel 1 │                         │
  │          │  (Read / H2C)  │      │  (Write / C2H) │                         │
  └──────────┼────────────────┼──────┼────────────────┼─────────────────────────┘
             ▼                ▼      ▼                ▼
      AXI4-MM / AXIS     AXI4-MM / AXIS (Device Logic / DDR / Accelerators)
```

### Key Subsystems

1. **IP-Agnostic Top-Level Core (`dma_top`)**: Exposes clean, vendor-neutral PCIe transaction buses (`pcie_req_bus_t` master for reads/writes, `pcie_cpl_bus_t` slave for inbound completions) and a BAR0 APB slave interface. Decoupled from proprietary controller wrappers.
2. **CSR Register Block (`csr_block`)**: Implements the BAR0 HDMA register layout (common region, per-channel registers, vendor extension region for ATS/PRI/Stream/PASID). Zero-wait APB slave with W1C interrupt handling.
3. **Channel Movers (`dma_channel`)**: H2C (Read) and C2H (Write) channels featuring register-mode data movers, dynamic 4KB address page segmentation, Skid buffering, AXI4-MM master engines, and completion tracking.
4. **Channel Arbiter (`chan_arbiter`)**: Channel request arbiter (round-robin with multi-beat burst lock) and tag-based completion demux routing inbound PCIe completions to their destination channel.
5. **ATS / PRI Subsystem (`ats_pri_subsys`)**: Full SVM management including a 1024-entry Address Translation Cache (ATC), ATS Translation Agent, and PRI page-fault suspend/resume FSM with per-channel fault isolation.
6. **PCIe Application Adapter (`dwc_paa`)**: Standalone, modular wrapper translating vendor-neutral PCIe transaction buses to Synopsys DWC raw TLPs and MSI-X vectors.

---

## Repository Structure

```
.
├── Makefile                     # Lint, build, and simulation automation
├── README.md                    # Project documentation, analysis, and fix report
├── docs/                        # Architecture & interface specifications (Milestone 1 & 2)
│   ├── dma_architecture_spec.md # Top-level architecture specification (v0.4)
│   ├── m1_descriptor_abi.md     # HDMA linked-list descriptor memory format
│   ├── m1_register_map.md       # BAR0 register map (HDMA-native + vendor extension)
│   ├── m1_paa_contract.md       # DWC PCIe Application Adapter signal mapping
│   └── m1_ats_pri_microspec.md  # ATS/PASID/PRI micro-architecture & FSMs
├── rtl/
│   ├── dma_pkg.sv               # Parameters, enums, structs, and synthesizable bus types
│   ├── dma_top.sv               # Top-level IP-agnostic integration module
│   ├── blocks/                  # Core subsystem blocks
│   │   ├── csr_block.sv         # BAR0 APB slave register file (functional)
│   │   ├── dma_channel.sv       # Channel engine (M2 register-mode mover, 4KB segmentation)
│   │   ├── chan_arbiter.sv      # Channel arbiter & completion demux (functional)
│   │   ├── ats_pri_subsys.sv    # ATS/PRI translation subsystem (M1 stub)
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
    └── tb_m2.sv                 # Milestone M2 end-to-end directed testbench (7 tests)
```

---

## Codebase Analysis & Verification Summary

### Milestone 2 Status (Completed)
- **IP-Agnostic Core Top (`rtl/dma_top.sv`)**: Decoupled from proprietary controller wrappers; core directly surfaces standard vendor-neutral PCIe transaction buses (`pcie_req_bus_t` master and `pcie_cpl_bus_t` slave) and an APB completer port for BAR0 CSR access.
- **Register-Mode Channel Data Movers (`rtl/blocks/dma_channel.sv`)**:
  - **Read Channel (H2C)**: Issues outbound PCIe memory read requests, receives inbound completions, and streams reassembled data directly into device-side AXI4-MM write transactions.
  - **Write Channel (C2H)**: Issues device-side AXI4-MM read transactions, buffers beats via a skid FIFO, and generates outbound PCIe posted memory writes (`MWR64`).
  - **4KB Page Boundary Segmentation**: Hardware split logic dynamically subdivides transfers crossing 4KB physical page boundaries into discrete, legal PCIe requests.
  - **Interrupt Handling**: Asserts channel interrupt (`ch_irq`) upon completion of programmed transfer lengths, clearing status via standard W1C registers.
- **Request Arbiter & Tag Demux (`rtl/blocks/chan_arbiter.sv`)**:
  - Round-robin request arbitration across active channels with burst locking to guarantee atomicity of multi-beat writes.
  - Tag-based completion demultiplexer routing inbound PCIe completions to the matching channel engine.
- **End-to-End Verification Suite (`sim/tb_m2.sv` & `sim/axi_ram.sv`)**:
  - Coupled with a 512-bit behavioral AXI4-MM slave RAM model (`sim/axi_ram.sv`) and an open PCIe Host BFM.
  - 7 directed test cases covering register accesses, single- and multi-beat bursts, 4KB page segmentation splits, and full-duplex concurrent operations.

### Milestone 1 Status (Completed)
- **Specifications & ABIs**: Complete (`docs/`).
- **Package & Bus Bundles**: Complete (`rtl/dma_pkg.sv`). Module boundaries use packed-struct bundles for broad tool compatibility and clean synthesis.
- **Register Block**: Complete (`rtl/blocks/csr_block.sv`). Verified with a 24-assertion self-checking APB testbench (`sim/tb_csr.sv`).
- **Interface Definitions**: Complete (`rtl/if/`). Verified via `sim/lint_top.sv`.
- **Structural Stubs**: Completed port hierarchies and tie-offs ready for Milestone 3–5 logic insertion.

---

## Verification Evidence

### Milestone M2 Full-Duplex Simulation (`make sim-m2`)

```bash
$ make sim-m2
verilator --binary -sv --timing -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM \
  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  rtl/dma_pkg.sv rtl/blocks/dma_channel.sv rtl/blocks/chan_arbiter.sv rtl/blocks/ats_pri_subsys.sv rtl/blocks/csr_block.sv rtl/dma_top.sv sim/axi_ram.sv sim/tb_m2.sv --top-module tb_m2 -o tb_m2
[tb_m2] Starting Milestone M2 Verification Suite...

--- Test 1: CSR Sanity ---
  [OK]   ID_VERSION = 444d0100
  [OK]   CAPABILITIES = 0000003f

--- Test 2: Single-Beat H2C (64 Bytes) ---
  [OK]   Ch0 IRQ Asserted = 00000001
  [OK]   H2C Byte 0 in RAM = 00000001
  [OK]   H2C Byte 63 in RAM = 00000040

--- Test 3: Multi-Beat H2C Burst (512 Bytes) ---
  [OK]   Ch0 Multi-Beat Done = 00000001
  [OK]   H2C Burst Byte 0 = 000000a5
  [OK]   H2C Burst Byte 511 = 0000005a

--- Test 4: Single-Beat C2H (64 Bytes) ---
  [OK]   Ch1 IRQ Asserted = 00000001
  [OK]   C2H Byte 0 in Host Mem = 00000050
  [OK]   C2H Byte 63 in Host Mem = 0000008f

--- Test 5: Multi-Beat C2H Burst (512 Bytes) ---
  [OK]   C2H Multi-Beat Done = 00000001
  [OK]   C2H Burst Byte 0 = 0000003c
  [OK]   C2H Burst Byte 511 = 000000c3

--- Test 6: 4KB Page Boundary Splitting Test ---
  [OK]   4KB Split Transfer Done = 00000001
  [OK]   4KB Split First Byte = 00000011
  [OK]   4KB Boundary Crossing Byte = 00000011
  [OK]   4KB Split Last Byte = 00000010

--- Test 7: Concurrent Full-Duplex Operation ---
  [OK]   Concurrent H2C Byte 0 = 000000aa
  [OK]   Concurrent C2H Byte 0 = 00000055

=======================================================
 [tb_m2] ALL TESTS PASSED (0 errors)
=======================================================
```

---

## Verilator Lint Analysis & Fixing Result

### Issue Description
Under Verilator 5.020 `-Wall`, `make lint` failed with 7 `%Warning-WIDTHEXPAND` errors in `rtl/blocks/csr_block.sv`:
1. **Lines 46–47 (`RD_END`, `WR_END`)**: Binary addition between 20-bit `RD_BASE` (`logic [APB_AW-1:0]`) and 32-bit `NUM_RD_CH * CH_STRIDE` (`int`) implicitly expanded `RD_BASE` to 32 bits before casting to `APB_AW'`.
2. **Lines 107–110 (`E_RD_STREAM_END`, `E_WR_STREAM_END`, `E_RD_PASID_END`, `E_WR_PASID_END`)**: Addition between 12-bit `E_RD_STREAM` and 32-bit `NUM_RD_CH * 4` inside `12'(...)` implicitly widened the 12-bit base.
3. **Line 181 (`gch`)**: In `wire [CHW-1:0] gch = in_wr ? CHW'(NUM_RD_CH + off_wr[APB_AW-1:9])`, adding 32-bit `int NUM_RD_CH` to the 11-bit slice `off_wr[19:9]` triggered implicit widening on the slice.

### Fix
Explicit operand width casting was introduced so that all operands in additions share matching bit widths:

```diff
--- a/rtl/blocks/csr_block.sv
+++ b/rtl/blocks/csr_block.sv
@@ -46,2 +46,2 @@
-  localparam logic [APB_AW-1:0] RD_END = APB_AW'(RD_BASE + NUM_RD_CH*CH_STRIDE);
-  localparam logic [APB_AW-1:0] WR_END = APB_AW'(WR_BASE + NUM_WR_CH*CH_STRIDE);
+  localparam logic [APB_AW-1:0] RD_END = RD_BASE + APB_AW'(NUM_RD_CH*CH_STRIDE);
+  localparam logic [APB_AW-1:0] WR_END = WR_BASE + APB_AW'(NUM_WR_CH*CH_STRIDE);
@@ -107,4 +107,4 @@
-  localparam logic [11:0] E_RD_STREAM_END = 12'(E_RD_STREAM + NUM_RD_CH*4);
-  localparam logic [11:0] E_WR_STREAM_END = 12'(E_WR_STREAM + NUM_WR_CH*4);
-  localparam logic [11:0] E_RD_PASID_END  = 12'(E_RD_PASID  + NUM_RD_CH*8);
-  localparam logic [11:0] E_WR_PASID_END  = 12'(E_WR_PASID  + NUM_WR_CH*8);
+  localparam logic [11:0] E_RD_STREAM_END = E_RD_STREAM + 12'(NUM_RD_CH*4);
+  localparam logic [11:0] E_WR_STREAM_END = E_WR_STREAM + 12'(NUM_WR_CH*4);
+  localparam logic [11:0] E_RD_PASID_END  = E_RD_PASID  + 12'(NUM_RD_CH*8);
+  localparam logic [11:0] E_WR_PASID_END  = E_WR_PASID  + 12'(NUM_WR_CH*8);
@@ -181,2 +181,2 @@
-  wire [CHW-1:0]  gch  = in_wr ? CHW'(NUM_RD_CH + off_wr[APB_AW-1:9])
+  wire [CHW-1:0]  gch  = in_wr ? (CHW'(NUM_RD_CH) + CHW'(off_wr[APB_AW-1:9]))
                                : CHW'(off_rd[APB_AW-1:9]);   // global channel idx
```

### Milestone M1 Verification Evidence

```bash
$ make clean && make lint && make lint-if && make sim-csr
rm -rf obj_dir build *.vvp
verilator --lint-only -sv -Wall -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM rtl/dma_pkg.sv rtl/blocks/dma_channel.sv rtl/blocks/chan_arbiter.sv rtl/blocks/ats_pri_subsys.sv rtl/blocks/csr_block.sv rtl/dma_top.sv --top-module dma_top
verilator --lint-only -sv -Wall -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM rtl/dma_pkg.sv rtl/if/pcie_req_if.sv rtl/if/ats_pri_if.sv rtl/if/axi4_if.sv rtl/if/axi4s_if.sv rtl/if/csr_apb_if.sv sim/lint_top.sv --top-module lint_top
verilator --binary -sv --timing -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-UNDRIVEN \
  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  rtl/dma_pkg.sv rtl/blocks/csr_block.sv sim/tb_csr.sv --top-module tb_csr -o tb_csr
make[1]: Entering directory '.../obj_dir'
...
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
  ok   pslverr undecoded      = 1               
[tb_csr] ALL PASS
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

# Run behavioral self-checking APB CSR testbench (24 assertions)
make sim-csr

# Run Milestone M2 end-to-end directed simulation (7 tests: H2C, C2H, 4KB split, duplex)
make sim-m2

# Run full regression suite
make clean && make lint && make lint-if && make sim-csr && make sim-m2

# Clean generated build and simulation directories
make clean
```

---

## Development Milestones

- [x] **M0**: Architecture specification & interview synthesis sign-off
- [x] **M1**: Register map, descriptor ABI, DWC-PAA signal contract, ATS/PRI micro-spec, SV interface contracts, APB CSR block implementation & testbench
- [x] **M2**: RTL: IP-Agnostic DMA Core (native PCIe transaction bus) + 1 Read (H2C) + 1 Write (C2H) channel register mode (DATA_WIDTH=512) with 4KB segmentation & full-duplex verification
- [ ] **M3**: RTL: Linked-list descriptor engine + tag manager + reorder buffer + multi-channel arbiter
- [ ] **M4**: RTL: ATS + PASID datapath + ATC (device TLB)
- [ ] **M5**: RTL: PRI page-fault suspend/resume FSM + per-channel fault isolation
- [ ] **M6**: RTL: AXI4-Stream extension + P2P + ECC/error handling
- [ ] **M7**: RTL: 1024-bit straddle datapath & BDP/SVM latency tuning
