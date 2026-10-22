# Design Specification: Milestone M2 IP-Agnostic Scatter-Gather DMA Core

**Date**: 2026-10-22  
**Status**: Approved for Implementation Plan  
**Target Milestone**: Milestone M2 (1 Read + 1 Write Channel in Register Mode, DATA_WIDTH=512)

---

## 1. Executive Summary & Architectural Scope

This specification details the Milestone M2 implementation of the PCIe Gen4 HDMA-compatible DMA core.

### IP-Agnostic Core Mandate
The DMA core RTL is **100% vendor-agnostic and IP-independent**:
- The core contains **zero proprietary controller dependencies** (no Synopsys DWC, Cadence, or Xilinx proprietary shims in the synthesizable core hierarchy).
- `dma_top` communicates with the host PCIe subsystem via a clean, standardized, vendor-neutral transaction bus (`pcie_req_bus_t` / `pcie_cpl_bus_t`) and a standard APB slave interface for CSR programming.
- Any controller-specific PHY/TLP adaptation layers (e.g., Synopsys DWC raw-TLP framing, Xilinx AXI Bridge, Intel P-Tile) reside strictly as external integration wrappers outside `dma_top`.

### Milestone M2 Deliverables
1. **IP-Agnostic Top Level (`rtl/dma_top.sv`)**: Refactored to export native PCIe transaction request/completion ports and an APB slave port directly at the boundary.
2. **Channel Movers (`rtl/blocks/dma_channel.sv`)**: Full functional implementation of 1 Read (H2C) and 1 Write (C2H) channel in register mode (`SAR`, `DAR`, `TRANSFER_SIZE`, `DOORBELL`) with 4KB page boundary segmentation on a 512-bit datapath.
3. **Channel Arbiter (`rtl/blocks/chan_arbiter.sv`)**: Request multiplexing and completion demultiplexing between active channels.
4. **Device AXI4-MM RAM Model (`sim/axi_ram.sv`)**: Synthesizable/behavioral 512-bit AXI4 slave memory model.
5. **System Verification Testbench (`sim/tb_m2.sv`)**: Directed testbench with an open PCIe Root Complex BFM (host memory model, APB CSR driver, completion responder, write sink) executing 7 test scenarios.

---

## 2. Top-Level Interface Architecture (`rtl/dma_top.sv`)

### 2.1 Block Diagram

```
                 Host PCIe Controller / Bridge (Any Vendor)
                            │ host_req (Out)     ▲ host_cpl (In)
                            │ host_req_ready     │ host_cpl_ready
┌───────────────────────────▼────────────────────┴──────────────────────────┐
│ dma_top                                                                   │
│                                                                           │
│   s_apb_* (BAR0 APB Slave)         ats_pri_subsys (Transparent in M2)     │
│        │                                        ▲                         │
│        ▼                                        │                         │
│  ┌───────────┐                            ┌─────┴─────┐                   │
│  │ csr_block │                            │  arbiter  │                   │
│  │           │                            └─────▲─────┘                   │
│  └─────┬─────┘                                  │                         │
│        │ ch_ctrl / ch_stat                      │ req / cpl per channel   │
│        ▼                                        ▼                         │
│  ┌────────────────────────────────────────────────────────┐               │
│  │ dma_channel[0] (Read / H2C)   dma_channel[1] (Write/C2H)│               │
│  └─────────────┬────────────────────────────┬─────────────┘               │
└────────────────┼────────────────────────────┼─────────────────────────────┘
                 ▼                            ▼
          dev_mm_m2s[0]                dev_mm_m2s[1]
          dev_mm_s2m[0]                dev_mm_s2m[1]
         (AXI4-MM Master)             (AXI4-MM Master)
                 │                            │
                 ▼                            ▼
             Device On-Card Memory (SRAM / DDR / Accelerator)
```

### 2.2 Port Definitions

```systemverilog
module dma_top (
  // ---- Clocks & Resets ----
  input  logic core_clk,       // DMA datapath clock (~500 MHz)
  input  logic core_rst_n,
  input  logic axi_clk,        // Device-side AXI clock
  input  logic axi_rst_n,
  input  logic csr_clk,        // Register access clock
  input  logic csr_rst_n,

  // ---- Host PCIe Transaction Requester (Master Out) ----
  output dma_pkg::pcie_req_bus_t host_req,
  input  logic                   host_req_ready,

  // ---- Host PCIe Transaction Completion (Slave In) ----
  input  dma_pkg::pcie_cpl_bus_t host_cpl,
  output logic                   host_cpl_ready,

  // ---- Host CSR APB Slave Interface (BAR0) ----
  input  logic [dma_pkg::APB_AW-1:0] s_apb_paddr,
  input  logic                       s_apb_psel,
  input  logic                       s_apb_penable,
  input  logic                       s_apb_pwrite,
  input  logic [31:0]                s_apb_pwdata,
  input  logic [3:0]                 s_apb_pstrb,
  output logic [31:0]                s_apb_prdata,
  output logic                       s_apb_pready,
  output logic                       s_apb_pslverr,

  // ---- Device-side AXI4-MM, per channel ----
  output dma_pkg::axi_mm_m2s_t dev_mm_m2s     [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  dma_pkg::axi_mm_s2m_t dev_mm_s2m     [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],

  // ---- Device-side AXI4-Stream, per channel ----
  output dma_pkg::axis_data_t  dev_st_o       [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  logic                 dev_st_o_ready [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  dma_pkg::axis_data_t  dev_st_i       [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  output logic                 dev_st_i_ready [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],

  // ---- Interrupts & MSI Configuration ----
  output logic                 ch_irq         [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  output dma_pkg::ch_msi_t     ch_msi         [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH]
);
```

---

## 3. Channel Micro-Architecture (`rtl/blocks/dma_channel.sv`)

### 3.1 4KB Segmentation Engine
PCIe transactions must never cross a 4KB boundary in host memory space:
- **H2C**: Constrained by host source address `cur_sar[11:0]`.
- **C2H**: Constrained by host destination address `cur_dar[11:0]`.

```systemverilog
wire [12:0] bytes_to_4k = 13'h1000 - {1'b0, host_addr[11:0]};
wire [12:0] seg_len_calc = (rem_bytes < bytes_to_4k) ? rem_bytes[12:0] : bytes_to_4k;
wire [12:0] seg_len = (seg_len_calc > 13'd512) ? 13'd512 : seg_len_calc;
```

### 3.2 H2C (Read Channel) Datapath & FSM
- **FSM States**:
  - `H2C_IDLE`: Await `ctrl.enable && ctrl.doorbell`. Capture `cur_sar <= ctrl.sar`, `cur_dar <= ctrl.dar`, `rem_bytes <= ctrl.xfer_size`, `bytes_xferred <= 0`.
  - `H2C_REQ`: Assert `req.valid = 1`, `req.req.is_read = 1`, `req.req.addr = cur_sar`, `req.req.len_bytes = seg_len`, `req.req.tag = tag_alloc`. Handshake with `req_ready`.
  - `H2C_STREAM`: Wait for `cpl.valid` matching allocated tag:
    - Assert AXI `awvalid = 1` with `awaddr = cur_dar`, `awlen = ((seg_len + 63) >> 6) - 1`.
    - Forward `cpl.rdata` directly to `mm_m2s.wdata`, asserted with `wvalid` and `wlast` on the final beat.
  - `H2C_BRESP`: Wait for AXI `bvalid && bready`.
    - Advance pointers: `cur_sar <= cur_sar + seg_len`, `cur_dar <= cur_dar + seg_len`, `rem_bytes <= rem_bytes - seg_len`, `bytes_xferred <= bytes_xferred + seg_len`.
    - Transition to `H2C_REQ` if `rem_bytes > 0`, else `H2C_DONE`.
  - `H2C_DONE`: Assert single-cycle pulse `stat.done_irq = 1`, set `stat.state = ST_IDLE`.

### 3.3 C2H (Write Channel) Datapath & FSM
- **FSM States**:
  - `C2H_IDLE`: Await `ctrl.enable && ctrl.doorbell`. Capture `cur_sar <= ctrl.sar`, `cur_dar <= ctrl.dar`, `rem_bytes <= ctrl.xfer_size`.
  - `C2H_AR`: Issue device AXI read `arvalid = 1` with `araddr = cur_sar`, `arlen = ((seg_len + 63) >> 6) - 1`.
  - `C2H_STG`: Capture device AXI `rdata` beats into internal 4-entry $\times$ 512-bit skid buffer.
  - `C2H_MWR`: Issue PCIe write request `req.valid = 1`, `req.req.is_read = 0`, `req.req.addr = cur_dar`, `req.req.len_bytes = seg_len`. Stream buffered data beats over `req.wdata`, asserting `req.wlast` on final beat.
  - `C2H_NEXT`: Update `cur_sar`, `cur_dar`, `rem_bytes`, `bytes_xferred`. Loop to `C2H_AR` if `rem_bytes > 0`, else `C2H_DONE`.
  - `C2H_DONE`: Assert single-cycle pulse `stat.done_irq = 1`, set `stat.state = ST_IDLE`.

---

## 4. Arbiter Micro-Architecture (`rtl/blocks/chan_arbiter.sv`)

In Milestone M2 (1 Read + 1 Write channel active):
- **Request Arbitration (`m_req`)**: Round-robin priority between Channel 0 (H2C) and Channel 1 (C2H). Grant is held across multi-beat write bursts (`wlast` boundary).
- **Completion Demultiplexing (`ch_cpl`)**: Routes returning `m_cpl` to the requesting channel based on tag bit allocation (e.g. `tag[0] == 0` $\rightarrow$ Ch 0, `tag[0] == 1` $\rightarrow$ Ch 1).

---

## 5. Verification Architecture & Test Plan

### 5.1 Components
- **`sim/axi_ram.sv`**: 512-bit AXI4-MM slave RAM model with configurable latency and preload/verify helper methods.
- **`sim/tb_m2.sv`**: Directed verification testbench driving APB CSR configuration and modeling PCIe Root Complex memory.

### 5.2 Test Cases

```
Test 1: CSR Sanity & Access Decode
├── Read ID_VERSION (verify 32'h444D_0100)
├── Read CAPABILITIES
└── Check pslverr on out-of-range APB address

Test 2: Single-Beat H2C Transfer (64 Bytes)
├── Preload host memory at 0x1000_0000 with 64-byte incremental pattern
├── Configure Ch0: SAR=0x1000_0000, DAR=0x2000_0000, SIZE=64
├── Ring doorbell
├── BFM returns 64-byte completion
└── Verify device AXI RAM at 0x2000_0000 matches; verify ch_irq[0] asserted

Test 3: Multi-Beat H2C Burst Transfer (512 Bytes)
├── Preload host memory with 512 bytes (8 beats)
├── Configure Ch0: SIZE=512
├── Ring doorbell
└── Verify full 8-beat write burst in AXI RAM; verify bytes_xferred == 512

Test 4: Single-Beat C2H Transfer (64 Bytes)
├── Preload device AXI RAM at 0x3000_0000 with 64-byte pattern
├── Configure Ch1: SAR=0x3000_0000, DAR=0x4000_0000, SIZE=64
├── Ring doorbell
├── DMA reads AXI RAM and issues MWr transaction to host
└── Verify host memory at 0x4000_0000 matches; verify ch_irq[1] asserted

Test 5: Multi-Beat C2H Burst Transfer (512 Bytes)
├── Preload device AXI RAM with 512 bytes
├── Configure Ch1: SIZE=512
├── Ring doorbell
└── Verify 8-beat MWr stream in host memory; verify bytes_xferred == 512

Test 6: 4KB Page Boundary Crossing Test
├── Configure Ch0: SAR=0x1000_0F00 (256B before 4KB boundary), SIZE=512
├── Ring doorbell
├── Verify DMA splits transfer into two 256-byte transactions
└── Verify contiguous 512 bytes assembled accurately in device RAM

Test 7: Full-Duplex Concurrent Operation
├── Launch Ch0 (H2C) and Ch1 (C2H) simultaneously
├── Arbiter interleaves read requests and write streams
└── Both channels finish cleanly with done interrupts asserted
```

---

## 6. Exit Criteria for Milestone M2

1. **Lint Cleanliness**: `make lint` and `make lint-if` pass under Verilator 5.020 `-Wall` with **0 warnings**.
2. **Regression Cleanliness**: `make sim-csr` continues to pass with all 24 checks intact.
3. **M2 Simulation Verification**: `make sim-m2` runs all 7 test cases and outputs `[tb_m2] ALL TESTS PASSED`.
