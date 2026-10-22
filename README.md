# PCIe Gen4 HDMA-Native Scatter-Gather DMA Engine

A high-performance, synthesizable SystemVerilog PCIe Gen4 x16 Scatter-Gather DMA Engine Endpoint core targeting the **Synopsys DesignWare (DWC) PCIe Gen4 Controller**.

Designed to achieve PCIe Gen4 x16 line rate (~252 Gb/s / 31.5 GB/s TLP per direction, full-duplex) using a **512-bit primary datapath @ ~500 MHz** with TLP straddle packing/unpacking, and providing an optional 1024-bit @ 250–300 MHz timing-relief configuration.

Maintains Linux `dmaengine` compatibility via an **eDMA/HDMA-compatible register and descriptor ABI** (`dw-edma`/`dw-hdma`), augmented with:
- **Full Shared Virtual Memory (SVM)**: ATS (Address Translation Services) + PASID + PRI (Page Request Interface) with per-channel fault isolation.
- **Dual-Mode Device Datapath**: First-class **AXI4-MM** (for local DDR/HBM) alongside an **AXI4-Stream** streaming mode extension via a lightweight mode register bit.

---

## Architecture Overview

```
                      Synopsys DWC PCIe Gen4 Controller (Raw TLP)
                                          │▲
                                          ││ Native Application Bus (TLP)
                    ┌─────────────────────▼┴────────────────────┐
                    │ dwc_paa (PCIe Application Adapter)        │
                    │ - TLP Framing / Straddle Packing / Parsing │
                    │ - MSI-X Generator / APB Completer Master  │
                    └───────────┬───────────────▲───────────────┘
                     APB Master │               │ Merged PCIe Req/Cpl
                                │               │
 ┌──────────────────────────────┼───────────────┴──────────────────────────────┐
 │ DMA Core                     ▼                                              │
 │                    ┌──────────────────┐                                     │
 │                    │   csr_block      │                                     │
 │                    │   (BAR0 APB)     │                                     │
 │                    └────────┬─────────┘                                     │
 │        Channel Ctrl / Status│                                               │
 │                             │                                               │
 │                    ┌────────▼─────────┐                                     │
 │                    │  ats_pri_subsys  │ (ATC TLB + ATS/PRI Fault Engine)    │
 │                    └────────▲─────────┘                                     │
 │                             │ Translated Req / Cpl                          │
 │                    ┌────────┴─────────┐                                     │
 │                    │   chan_arbiter   │ (WRR Arbiter + Tag-to-Ch Demux)     │
 │                    └───▲───────────▲──┘                                     │
 │                        │           │                                        │
 │          ┌─────────────┴──┐     ┌──┴────────────┐                           │
 │          │  dma_channel 0 │ ... │ dma_channel N │                           │
 │          │  (Read / H2C)  │     │ (Write / C2H) │                           │
 └──────────┼────────────────┼─────┼───────────────┼───────────────────────────┘
            ▼                ▼     ▼               ▼
     AXI4-MM / AXIS    AXI4-MM / AXIS (Device Logic / DDR / Accelerators)
```

### Key Subsystems

1. **PCIe Application Adapter (`dwc_paa`)**: DWC-specific shim mapping vendor-neutral buses (`pcie_req_bus_t`, `pcie_cpl_bus_t`) to Synopsys DWC raw TLPs. Translates inbound BAR accesses into APB transactions.
2. **CSR Register Block (`csr_block`)**: Implements the BAR0 HDMA register layout (common region, per-channel registers, vendor extension region for ATS/PRI/Stream/PASID). Zero-wait APB slave with W1C interrupt handling.
3. **ATS / PRI Subsystem (`ats_pri_subsys`)**: Full SVM management including a 1024-entry Address Translation Cache (ATC), ATS Translation Agent, and PRI page-fault suspend/resume FSM with per-channel fault isolation.
4. **Channel Arbiter (`chan_arbiter`)**: Weighted Round-Robin (WRR) request issue arbiter and completion demux.
5. **Channel Movers (`dma_channel`)**: H2C (Read) and C2H (Write) channels walking linked-list descriptors (`hdma_elem_t`) with 4KB address segmentation and MPS/MRRS enforcement.

---

## Repository Structure

```
.
├── Makefile                     # Lint, build, and simulation automation
├── README.md                    # Project documentation, analysis, and fix report
├── docs/                        # Architecture & interface specifications (Milestone 1)
│   ├── dma_architecture_spec.md # Top-level architecture specification (v0.4)
│   ├── m1_descriptor_abi.md     # HDMA linked-list descriptor memory format
│   ├── m1_register_map.md       # BAR0 register map (HDMA-native + vendor extension)
│   ├── m1_paa_contract.md       # DWC PCIe Application Adapter signal mapping
│   └── m1_ats_pri_microspec.md  # ATS/PASID/PRI micro-architecture & FSMs
├── rtl/
│   ├── dma_pkg.sv               # Parameters, enums, structs, and synthesizable bus types
│   ├── dma_top.sv               # Top-level integration module
│   ├── blocks/                  # Core subsystem blocks
│   │   ├── csr_block.sv         # BAR0 APB slave register file (functional)
│   │   ├── dma_channel.sv       # Channel engine (M1 stub)
│   │   ├── chan_arbiter.sv      # Channel multiplexer / demux (M1 stub)
│   │   ├── ats_pri_subsys.sv    # ATS/PRI translation subsystem (M1 stub)
│   │   └── dwc_paa.sv           # Synopsys DWC adapter shim (M1 stub)
│   └── if/                      # SystemVerilog interface contracts
│       ├── pcie_req_if.sv       # Vendor-neutral PCIe requester/completion interface
│       ├── ats_pri_if.sv        # ATS/PRI message exchange interface
│       ├── axi4_if.sv           # AXI4-MM master/slave interface
│       ├── axi4s_if.sv          # AXI4-Stream interface
│       └── csr_apb_if.sv        # APB control interface
└── sim/
    ├── dma.f                    # Compile filelist
    ├── lint_top.sv              # Top module elaborating all SV interfaces
    └── tb_csr.sv                # Self-checking APB testbench for csr_block
```

---

## Codebase Analysis & Verification Summary

### Milestone 1 Status
- **Specifications & ABIs**: Complete (`docs/`).
- **Package & Bus Bundles**: Complete (`rtl/dma_pkg.sv`). Module boundaries use packed-struct bundles for broad tool compatibility and clean synthesis.
- **Register Block**: Complete (`rtl/blocks/csr_block.sv`). Verified with a 24-assertion self-checking APB testbench (`sim/tb_csr.sv`).
- **Interface Definitions**: Complete (`rtl/if/`). Verified via `sim/lint_top.sv`.
- **Structural Stubs**: `dma_channel.sv`, `chan_arbiter.sv`, `ats_pri_subsys.sv`, and `dwc_paa.sv` have completed port hierarchies and tie-offs ready for Milestone 2–5 logic insertion.

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

### Verification Evidence

```bash
$ make clean && make lint && make lint-if && make sim-csr
rm -rf obj_dir build *.vvp
verilator --lint-only -sv -Wall -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM rtl/dma_pkg.sv rtl/blocks/dma_channel.sv rtl/blocks/chan_arbiter.sv rtl/blocks/ats_pri_subsys.sv rtl/blocks/dwc_paa.sv rtl/blocks/csr_block.sv rtl/dma_top.sv --top-module dma_top
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

# Run behavioral self-checking APB testbench
make sim-csr

# Clean generated build and simulation directories
make clean
```

---

## Development Milestones

- [x] **M0**: Architecture specification & interview synthesis sign-off
- [x] **M1**: Register map, descriptor ABI, DWC-PAA signal contract, ATS/PRI micro-spec, SV interface contracts, APB CSR block implementation & testbench
- [ ] **M2**: RTL: DWC-PAA (raw TLP) + 1 Read + 1 Write channel register mode (DATA_WIDTH=512)
- [ ] **M3**: RTL: Linked-list descriptor engine + tag manager + reorder buffer + multi-channel arbiter
- [ ] **M4**: RTL: ATS + PASID datapath + ATC (device TLB)
- [ ] **M5**: RTL: PRI page-fault suspend/resume FSM + per-channel fault isolation
- [ ] **M6**: RTL: AXI4-Stream extension + P2P + ECC/error handling
- [ ] **M7**: RTL: 1024-bit straddle datapath & BDP/SVM latency tuning
