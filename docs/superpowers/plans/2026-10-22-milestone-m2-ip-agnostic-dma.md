# Milestone M2 (IP-Agnostic DMA Core) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Milestone M2 of the PCIe Gen4 HDMA-compatible DMA engine: a 100% vendor-agnostic core (`dma_top`) featuring 1 Read (H2C) and 1 Write (C2H) channel in register mode with 4KB page boundary segmentation on a 512-bit datapath, verified against an open PCIe BFM and AXI4-MM RAM model.

**Architecture:** The core exposes a clean vendor-neutral PCIe transaction request bus (`pcie_req_bus_t`) and completion bus (`pcie_cpl_bus_t`) alongside a standard APB slave port for BAR0 CSR access. The channel movers implement direct streaming from PCIe completions to device AXI4-MM writes (H2C) and skid-buffered device AXI4-MM reads to PCIe writes (C2H).

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Verilator 5.020+, GNU Make, C++14.

**Spec:** [`docs/superpowers/specs/2026-10-22-milestone-m2-ip-agnostic-dma-design.md`](file:///home/user/RTL-VIBECODE/DMA/dma_cl/docs/superpowers/specs/2026-10-22-milestone-m2-ip-agnostic-dma-design.md)

---

## Global Constraints

- Native WSL2 / Linux tools only; no Windows binaries or wrappers.
- All RTL must be 100% vendor-agnostic and synthesizable SystemVerilog-2012; zero proprietary DWC or vendor-specific shims in `dma_top`.
- `make lint` and `make lint-if` must pass under Verilator `-Wall` with zero warnings.
- `make sim-csr` must maintain 100% backward compatibility (24/24 assertions passing).
- All byte offsets and register layouts must adhere to `csr_block.sv` and `docs/m1_register_map.md`.

## Review Focus

1. **4KB Boundary Overflow**: Transfers starting close to a 4KB boundary (e.g. `0x1000_0F00`) must be split and never emit an address crossing `0x1000_1000`.
2. **Backpressure Deadlock**: The H2C mover must not deadlock when device memory asserts `awready=0` or `wready=0` while completions arrive.
3. **Multi-Beat Burst Length Calculation**: The AXI `awlen` and `arlen` must be accurately calculated as `((seg_len + 63) >> 6) - 1` without off-by-one errors.
4. **Skid Buffer Overrun**: The C2H 4-entry FIFO must throttle device AXI `rready` when full to prevent data loss.
5. **Simultaneous Doorbell Assertion**: When both H2C and C2H doorbells ring in the same cycle, the arbiter must handle both channels without starvation or dropped requests.

---

## Task Decomposition

### Task 1: Device-Side AXI4-MM Behavioral RAM Model (`sim/axi_ram.sv`)

**Files:**
- Create: `sim/axi_ram.sv`
- Test: `sim/axi_ram.sv` (verified directly via compilation and helper checks in simulation)

**Interfaces:**
- Consumes: `dma_pkg::axi_mm_m2s_t`, `dma_pkg::axi_mm_s2m_t`, `DATA_WIDTH=512`, `AXI_ID_W=8`.
- Produces: `sim/axi_ram.sv` module responding to AXI read (`AR`/`R`) and write (`AW`/`W`/`B`) transactions with memory helper tasks `preload_mem(addr, data)` and `read_mem(addr)`.

- [ ] **Step 1: Write `sim/axi_ram.sv`**

```systemverilog
// ============================================================================
// sim/axi_ram.sv — Behavioral AXI4-MM slave RAM model (512-bit datapath).
// Used in directed simulation to model on-card DDR / local device SRAM.
// ============================================================================
module axi_ram #(
  parameter int DATA_WIDTH = dma_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH = 64,
  parameter int ID_WIDTH   = dma_pkg::AXI_ID_W,
  parameter int STRB_WIDTH = DATA_WIDTH / 8,
  parameter int MEM_SIZE   = 1024 * 1024 // 1 MB addressable model
) (
  input  logic                    clk,
  input  logic                    rst_n,
  input  dma_pkg::axi_mm_m2s_t    s_axi_m2s,
  output dma_pkg::axi_mm_s2m_t    s_axi_s2m
);
  import dma_pkg::*;

  // 1MB byte-addressable flat memory array
  logic [7:0] mem [MEM_SIZE];

  // Write channel state
  typedef enum logic [1:0] { W_IDLE, W_BURST, W_RESP } w_state_e;
  w_state_e w_state;
  logic [ADDR_WIDTH-1:0] w_addr;
  logic [7:0]            w_len;
  logic [ID_WIDTH-1:0]   w_id;

  // Read channel state
  typedef enum logic [1:0] { R_IDLE, R_BURST } r_state_e;
  r_state_e r_state;
  logic [ADDR_WIDTH-1:0] r_addr;
  logic [7:0]            r_len;
  logic [ID_WIDTH-1:0]   r_id;

  // Combinational default tie-offs
  assign s_axi_s2m.awready = (w_state == W_IDLE);
  assign s_axi_s2m.wready  = (w_state == W_BURST);
  assign s_axi_s2m.bvalid  = (w_state == W_RESP);
  assign s_axi_s2m.bresp   = 2'b00; // OKAY
  assign s_axi_s2m.bid     = w_id;

  assign s_axi_s2m.arready = (r_state == R_IDLE);
  assign s_axi_s2m.rvalid  = (r_state == R_BURST);
  assign s_axi_s2m.rresp   = 2'b00; // OKAY
  assign s_axi_s2m.rid     = r_id;
  assign s_axi_s2m.rlast   = (r_state == R_BURST) && (r_len == 0);

  // Read data output assembly
  always_comb begin
    s_axi_s2m.rdata = '0;
    if (r_state == R_BURST) begin
      for (int i = 0; i < STRB_WIDTH; i++) begin
        int unsigned offset = (int'(r_addr) + i) % MEM_SIZE;
        s_axi_s2m.rdata[8*i +: 8] = mem[offset];
      end
    end
  end

  // Sequential write engine
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w_state <= W_IDLE;
      w_addr  <= '0;
      w_len   <= '0;
      w_id    <= '0;
    end else begin
      case (w_state)
        W_IDLE: begin
          if (s_axi_m2s.awvalid && s_axi_s2m.awready) begin
            w_addr  <= s_axi_m2s.awaddr;
            w_len   <= s_axi_m2s.awlen;
            w_id    <= s_axi_m2s.awid;
            w_state <= W_BURST;
          end
        end
        W_BURST: begin
          if (s_axi_m2s.wvalid && s_axi_s2m.wready) begin
            for (int i = 0; i < STRB_WIDTH; i++) begin
              if (s_axi_m2s.wstrb[i]) begin
                int unsigned offset = (int'(w_addr) + i) % MEM_SIZE;
                mem[offset] <= s_axi_m2s.wdata[8*i +: 8];
              end
            end
            w_addr <= w_addr + STRB_WIDTH;
            if (s_axi_m2s.wlast || w_len == 0) begin
              w_state <= W_RESP;
            end else begin
              w_len <= w_len - 1'b1;
            end
          end
        end
        W_RESP: begin
          if (s_axi_m2s.bready && s_axi_s2m.bvalid) begin
            w_state <= W_IDLE;
          end
        end
        default: w_state <= W_IDLE;
      endcase
    end
  end

  // Sequential read engine
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_state <= R_IDLE;
      r_addr  <= '0;
      r_len   <= '0;
      r_id    <= '0;
    end else begin
      case (r_state)
        R_IDLE: begin
          if (s_axi_m2s.arvalid && s_axi_s2m.arready) begin
            r_addr  <= s_axi_m2s.araddr;
            r_len   <= s_axi_m2s.arlen;
            r_id    <= s_axi_m2s.arid;
            r_state <= R_BURST;
          end
        end
        R_BURST: begin
          if (s_axi_s2m.rvalid && s_axi_m2s.rready) begin
            r_addr <= r_addr + STRB_WIDTH;
            if (r_len == 0) begin
              r_state <= R_IDLE;
            end else begin
              r_len <= r_len - 1'b1;
            end
          end
        end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // Helper tasks for testbench verification
  task automatic write_byte(input logic [ADDR_WIDTH-1:0] a, input logic [7:0] d);
    mem[int'(a) % MEM_SIZE] = d;
  endtask

  function automatic logic [7:0] read_byte(input logic [ADDR_WIDTH-1:0] a);
    return mem[int'(a) % MEM_SIZE];
  endfunction

endmodule
```

- [ ] **Step 2: Lint verification for `sim/axi_ram.sv`**

Run: `verilator --lint-only -sv -Wall rtl/dma_pkg.sv sim/axi_ram.sv --top-module axi_ram`  
Expected: Clean exit code 0.

- [ ] **Step 3: Commit**

```bash
git add sim/axi_ram.sv
git commit -m "feat(sim): add 512-bit behavioral AXI4-MM slave RAM model"
```

---

### Task 2: Refactor Top-Level Core Architecture (`rtl/dma_top.sv`)

**Files:**
- Modify: `rtl/dma_top.sv`
- Test: `make lint` and `make lint-if`

**Interfaces:**
- Consumes: `rtl/dma_pkg.sv`, `rtl/blocks/csr_block.sv`, `rtl/blocks/dma_channel.sv`, `rtl/blocks/chan_arbiter.sv`, `rtl/blocks/ats_pri_subsys.sv`.
- Produces: `dma_top` with clean `host_req`, `host_req_ready`, `host_cpl`, `host_cpl_ready`, and `s_apb_*` interface ports. Eliminates `dwc_paa` dependency from synthesizable core.

- [ ] **Step 1: Update `rtl/dma_top.sv` to IP-Agnostic Top Level**

Replace `rtl/dma_top.sv` with the vendor-neutral top-level interface matching Section 2 of the design specification:

```systemverilog
// ============================================================================
// dma_top.sv — IP-Agnostic PCIe Gen4 HDMA-native Scatter-Gather DMA Top Level.
// Synthesizable, vendor-independent core.
// ============================================================================
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
  input  logic [19:0]            s_apb_paddr,
  input  logic                   s_apb_psel,
  input  logic                   s_apb_penable,
  input  logic                   s_apb_pwrite,
  input  logic [31:0]            s_apb_pwdata,
  input  logic [3:0]             s_apb_pstrb,
  output logic [31:0]            s_apb_prdata,
  output logic                   s_apb_pready,
  output logic                   s_apb_pslverr,

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
  import dma_pkg::*;

  localparam int NUM_CH = NUM_RD_CH + NUM_WR_CH;
  localparam int APB_AW = 20;

  // Internal per-channel request and completion buses
  pcie_req_bus_t ch_req       [NUM_CH];
  logic          ch_req_ready [NUM_CH];
  pcie_cpl_bus_t ch_cpl       [NUM_CH];
  logic          ch_cpl_ready [NUM_CH];

  // Merged arbiter request/completion buses
  pcie_req_bus_t arb_req;
  logic          arb_req_ready;
  pcie_cpl_bus_t arb_cpl;
  logic          arb_cpl_ready;

  // Unused ATS sideband buses in M2
  ats_pri_up_t   ats_up;
  ats_pri_dn_t   ats_dn;
  assign ats_dn = '0;

  ch_ctrl_t      ch_ctrl [NUM_CH];
  ch_stat_t      ch_stat [NUM_CH];
  logic          rd_en, wr_en, soft_reset;

  // ---- Per-Channel Movers ----
  genvar g;
  generate
    for (g = 0; g < NUM_CH; g++) begin : g_chan
      dma_channel #(
        .IS_WRITE   (g >= NUM_RD_CH)
      ) u_chan (
        .core_clk   (core_clk),
        .core_rst_n (core_rst_n),
        .axi_clk    (axi_clk),
        .axi_rst_n  (axi_rst_n),
        .ctrl       (ch_ctrl[g]),
        .stat       (ch_stat[g]),
        .mm_m2s     (dev_mm_m2s[g]),
        .mm_s2m     (dev_mm_s2m[g]),
        .st_o       (dev_st_o[g]),
        .st_o_ready (dev_st_o_ready[g]),
        .st_i       (dev_st_i[g]),
        .st_i_ready (dev_st_i_ready[g]),
        .req        (ch_req[g]),
        .req_ready  (ch_req_ready[g]),
        .cpl        (ch_cpl[g]),
        .cpl_ready  (ch_cpl_ready[g])
      );
    end
  endgenerate

  // ---- Arbiter (Multiplex Requests, Demux Completions) ----
  chan_arbiter #(.NUM_CH(NUM_CH)) u_arb (
    .core_clk     (core_clk),
    .core_rst_n   (core_rst_n),
    .ch_req       (ch_req),
    .ch_req_ready (ch_req_ready),
    .ch_cpl       (ch_cpl),
    .ch_cpl_ready (ch_cpl_ready),
    .m_req        (arb_req),
    .m_req_ready  (arb_req_ready),
    .m_cpl        (arb_cpl),
    .m_cpl_ready  (arb_cpl_ready)
  );

  // ---- ATS/PASID/PRI Subsystem (Transparent Passthrough in M2) ----
  ats_pri_subsys u_ats (
    .core_clk     (core_clk),
    .core_rst_n   (core_rst_n),
    .up_req       (arb_req),
    .up_req_ready (arb_req_ready),
    .up_cpl       (arb_cpl),
    .up_cpl_ready (arb_cpl_ready),
    .dn_req       (host_req),
    .dn_req_ready (host_req_ready),
    .dn_cpl       (host_cpl),
    .dn_cpl_ready (host_cpl_ready),
    .ats_up       (ats_up),
    .ats_dn       (ats_dn),
    .ats_en       (1'b0),
    .pri_en       (1'b0)
  );

  // ---- CSR Register Block (Connected Directly to Top APB) ----
  csr_block #(.NUM_RD_CH(NUM_RD_CH), .NUM_WR_CH(NUM_WR_CH), .APB_AW(APB_AW)) u_csr (
    .csr_clk      (csr_clk),
    .csr_rst_n    (csr_rst_n),
    .paddr        (s_apb_paddr),
    .psel         (s_apb_psel),
    .penable      (s_apb_penable),
    .pwrite       (s_apb_pwrite),
    .pwdata       (s_apb_pwdata),
    .pstrb        (s_apb_pstrb),
    .prdata       (s_apb_prdata),
    .pready       (s_apb_pready),
    .pslverr      (s_apb_pslverr),
    .ch_ctrl      (ch_ctrl),
    .ch_stat      (ch_stat),
    .ch_msi       (ch_msi),
    .ch_irq       (ch_irq),
    .rd_engine_en (rd_en),
    .wr_engine_en (wr_en),
    .soft_reset   (soft_reset)
  );

endmodule
```

- [ ] **Step 2: Update `Makefile` RTL list to remove `dwc_paa.sv`**

Modify `Makefile` line 5-9:
```makefile
BLOCKS  := rtl/blocks/dma_channel.sv \
           rtl/blocks/chan_arbiter.sv \
           rtl/blocks/ats_pri_subsys.sv \
           rtl/blocks/csr_block.sv
```

- [ ] **Step 3: Run lint verification**

Run: `make lint`  
Expected: Clean exit code 0.

- [ ] **Step 4: Commit**

```bash
git add rtl/dma_top.sv Makefile
git commit -m "refactor(core): decouple top-level from DWC shim and expose native PCIe transaction bus"
```

---

### Task 3: Channel Arbiter & Demux Implementation (`rtl/blocks/chan_arbiter.sv`)

**Files:**
- Modify: `rtl/blocks/chan_arbiter.sv`
- Test: `make lint`

**Interfaces:**
- Consumes: `ch_req[NUM_CH]`, `m_req_ready`, `m_cpl`, `ch_cpl_ready[NUM_CH]`.
- Produces: `m_req`, `ch_req_ready[NUM_CH]`, `ch_cpl[NUM_CH]`, `m_cpl_ready`. Arbitrates requests between channels; routes returning completions by tag.

- [ ] **Step 1: Implement `chan_arbiter.sv`**

Implement round-robin request arbitration and tag demuxing:

```systemverilog
// ============================================================================
// chan_arbiter.sv — Multi-channel request arbiter and completion demux.
// ============================================================================
module chan_arbiter #(
  parameter int NUM_CH = 8
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  // Per-channel requester & completion side
  input  dma_pkg::pcie_req_bus_t ch_req       [NUM_CH],
  output logic                   ch_req_ready [NUM_CH],
  output dma_pkg::pcie_cpl_bus_t ch_cpl       [NUM_CH],
  input  logic                   ch_cpl_ready [NUM_CH],
  // Merged PCIe transaction side
  output dma_pkg::pcie_req_bus_t m_req,
  input  logic                   m_req_ready,
  input  dma_pkg::pcie_cpl_bus_t m_cpl,
  output logic                   m_cpl_ready
);
  import dma_pkg::*;

  localparam int CH_WIDTH = (NUM_CH > 1) ? $clog2(NUM_CH) : 1;

  logic [CH_WIDTH-1:0] gnt;
  logic                gnt_valid;
  logic                in_burst;

  // Round-robin pointer
  logic [CH_WIDTH-1:0] rr_ptr;

  // Request arbitration logic
  always_comb begin
    gnt = rr_ptr;
    gnt_valid = 1'b0;
    for (int i = 0; i < NUM_CH; i++) begin
      int unsigned idx = (int'(rr_ptr) + i) % NUM_CH;
      if (ch_req[idx].valid && !gnt_valid) begin
        gnt = CH_WIDTH'(idx);
        gnt_valid = 1'b1;
      end
    end
  end

  // Lock grant during multi-beat write bursts
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      rr_ptr   <= '0;
      in_burst <= 1'b0;
    end else begin
      if (gnt_valid && m_req_ready) begin
        if (!ch_req[gnt].req.is_read && !ch_req[gnt].wlast) begin
          in_burst <= 1'b1;
        end else begin
          in_burst <= 1'b0;
          rr_ptr   <= (gnt == CH_WIDTH'(NUM_CH - 1)) ? '0 : gnt + 1'b1;
        end
      end
    end
  end

  // Multiplex request to master
  always_comb begin
    m_req = ch_req[gnt];
    m_req.valid = gnt_valid;
    for (int i = 0; i < NUM_CH; i++) begin
      ch_req_ready[i] = (gnt_valid && (gnt == CH_WIDTH'(i))) ? m_req_ready : 1'b0;
    end
  end

  // Completion demultiplexing by tag low bits
  // In M2: tag[0] = channel 0 (H2C), tag[1] = channel 1 (C2H), etc.
  wire [CH_WIDTH-1:0] cpl_dest_ch = CH_WIDTH'(m_cpl.meta.tag % NUM_CH);

  always_comb begin
    m_cpl_ready = ch_cpl_ready[cpl_dest_ch];
    for (int i = 0; i < NUM_CH; i++) begin
      ch_cpl[i]       = m_cpl;
      ch_cpl[i].valid = m_cpl.valid && (cpl_dest_ch == CH_WIDTH'(i));
    end
  end

endmodule
```

- [ ] **Step 2: Run lint verification**

Run: `make lint`  
Expected: Clean exit code 0.

- [ ] **Step 3: Commit**

```bash
git add rtl/blocks/chan_arbiter.sv
git commit -m "feat(arbiter): implement round-robin request arbitration and tag completion demux"
```

---

### Task 4: Implement H2C (Read) and C2H (Write) Movers in `rtl/blocks/dma_channel.sv`

**Files:**
- Modify: `rtl/blocks/dma_channel.sv`
- Test: `make lint`

**Interfaces:**
- Consumes: `ctrl` (`ch_ctrl_t`), `mm_s2m` (`axi_mm_s2m_t`), `cpl` (`pcie_cpl_bus_t`), `req_ready`.
- Produces: `stat` (`ch_stat_t`), `mm_m2s` (`axi_mm_m2s_t`), `req` (`pcie_req_bus_t`), `cpl_ready`.

- [ ] **Step 1: Implement `dma_channel.sv` Data Movers**

Implement the H2C mover (`IS_WRITE=0`) and C2H mover (`IS_WRITE=1`) with 4KB segmentation logic:

```systemverilog
// ============================================================================
// dma_channel.sv — DMA Channel Data Mover (Register Mode, DATA_WIDTH=512).
// IS_WRITE = 0: Read Channel (H2C) Host -> Device
// IS_WRITE = 1: Write Channel (C2H) Device -> Host
// ============================================================================
module dma_channel #(
  parameter bit IS_WRITE = 1'b0
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  input  logic                   axi_clk,
  input  logic                   axi_rst_n,
  // Control & Status (from/to CSR)
  input  dma_pkg::ch_ctrl_t      ctrl,
  output dma_pkg::ch_stat_t      stat,
  // Device-side AXI4-MM
  output dma_pkg::axi_mm_m2s_t   mm_m2s,
  input  dma_pkg::axi_mm_s2m_t   mm_s2m,
  // Device-side AXI4-Stream
  output dma_pkg::axis_data_t    st_o,
  input  logic                   st_o_ready,
  input  dma_pkg::axis_data_t    st_i,
  output logic                   st_i_ready,
  // Shared PCIe Requester / Completion
  output dma_pkg::pcie_req_bus_t req,
  input  logic                   req_ready,
  input  dma_pkg::pcie_cpl_bus_t cpl,
  output logic                   cpl_ready
);
  import dma_pkg::*;

  localparam int STRB_W = DATA_WIDTH / 8; // 64 bytes per beat

  // Working register pointers
  logic [63:0] cur_sar, cur_dar;
  logic [31:0] rem_bytes;
  logic [63:0] bytes_xferred_r;
  logic        done_irq_r;
  ch_state_e   ch_state_r;

  // 4KB Segmentation logic
  wire [63:0] host_addr = IS_WRITE ? cur_dar : cur_sar;
  wire [12:0] bytes_to_4k = 13'h1000 - {1'b0, host_addr[11:0]};
  wire [12:0] seg_len_calc = (rem_bytes < {19'b0, bytes_to_4k}) ? rem_bytes[12:0] : bytes_to_4k;
  wire [12:0] seg_len = (seg_len_calc > 13'd512) ? 13'd512 : seg_len_calc;
  wire [7:0]  burst_len = 8'((seg_len + 13'd63) >> 6) - 8'd1;

  // Stream tie-offs in M2
  assign st_o       = '0;
  assign st_i_ready = 1'b0;

  generate
    if (!IS_WRITE) begin : g_h2c
      // ------------------------------------------------------------------------
      // H2C (Read Channel): Host -> Device Local Memory
      // ------------------------------------------------------------------------
      typedef enum logic [2:0] {
        H_IDLE, H_REQ, H_CPL, H_BRESP, H_DONE
      } h_state_e;
      h_state_e state;
      logic [7:0] w_beats_rem;

      assign cpl_ready = (state == H_CPL) ? mm_s2m.wready : 1'b0;

      always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
          state           <= H_IDLE;
          cur_sar         <= '0;
          cur_dar         <= '0;
          rem_bytes       <= '0;
          bytes_xferred_r <= '0;
          done_irq_r      <= 1'b0;
          ch_state_r      <= ST_IDLE;
          w_beats_rem     <= '0;
          req.valid       <= 1'b0;
          req.req         <= '0;
          req.wdata       <= '0;
          req.wlast       <= 1'b0;
          mm_m2s          <= '0;
        end else begin
          done_irq_r <= 1'b0;
          case (state)
            H_IDLE: begin
              ch_state_r <= ST_IDLE;
              if (ctrl.enable && ctrl.doorbell) begin
                cur_sar         <= ctrl.sar;
                cur_dar         <= ctrl.dar;
                rem_bytes       <= ctrl.xfer_size;
                bytes_xferred_r <= '0;
                ch_state_r      <= ST_RUN;
                state           <= H_REQ;
              end
            end

            H_REQ: begin
              req.valid        <= 1'b1;
              req.req.is_read  <= 1'b1;
              req.req.addr     <= cur_sar;
              req.req.len_bytes<= seg_len;
              req.req.tag      <= 10'h00; // Channel 0
              req.req.first_be <= 4'hF;
              req.req.last_be  <= 4'hF;
              req.wlast        <= 1'b0;
              w_beats_rem      <= burst_len;
              if (req.valid && req_ready) begin
                req.valid <= 1'b0;
                // Issue AXI AW transaction
                mm_m2s.awaddr  <= cur_dar;
                mm_m2s.awlen   <= burst_len;
                mm_m2s.awsize  <= 3'b110; // 64 bytes
                mm_m2s.awburst <= 2'b01;  // INCR
                mm_m2s.awvalid <= 1'b1;
                state          <= H_CPL;
              end
            end

            H_CPL: begin
              if (mm_m2s.awvalid && mm_s2m.awready) begin
                mm_m2s.awvalid <= 1'b0;
              end
              if (cpl.valid && cpl_ready) begin
                mm_m2s.wdata  <= cpl.rdata;
                mm_m2s.wstrb  <= '1;
                mm_m2s.wvalid <= 1'b1;
                mm_m2s.wlast  <= (w_beats_rem == 0) || cpl.meta.last;
                if (w_beats_rem != 0) w_beats_rem <= w_beats_rem - 1'b1;
                if ((w_beats_rem == 0) || cpl.meta.last) begin
                  mm_m2s.bready <= 1'b1;
                  state <= H_BRESP;
                end
              end else if (mm_m2s.wvalid && mm_s2m.wready) begin
                mm_m2s.wvalid <= 1'b0;
              end
            end

            H_BRESP: begin
              if (mm_m2s.wvalid && mm_s2m.wready) mm_m2s.wvalid <= 1'b0;
              if (mm_m2s.bready && mm_s2m.bvalid) begin
                mm_m2s.bready   <= 1'b0;
                cur_sar         <= cur_sar + 64'(seg_len);
                cur_dar         <= cur_dar + 64'(seg_len);
                bytes_xferred_r <= bytes_xferred_r + 64'(seg_len);
                rem_bytes       <= rem_bytes - 32'(seg_len);
                if (rem_bytes <= 32'(seg_len)) begin
                  state <= H_DONE;
                end else begin
                  state <= H_REQ;
                end
              end
            end

            H_DONE: begin
              done_irq_r <= 1'b1;
              ch_state_r <= ST_IDLE;
              state      <= H_IDLE;
            end

            default: state <= H_IDLE;
          endcase
        end
      end
    end else begin : g_c2h
      // ------------------------------------------------------------------------
      // C2H (Write Channel): Device Local Memory -> Host
      // ------------------------------------------------------------------------
      typedef enum logic [2:0] {
        C_IDLE, C_AR, C_STG, C_MWR, C_NEXT, C_DONE
      } c_state_e;
      c_state_e state;
      logic [7:0] r_beats_rem;
      logic [DATA_WIDTH-1:0] fifo_data [4];
      logic [1:0] fifo_wr_ptr, fifo_rd_ptr;
      logic [2:0] fifo_cnt;

      always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
          state           <= C_IDLE;
          cur_sar         <= '0;
          cur_dar         <= '0;
          rem_bytes       <= '0;
          bytes_xferred_r <= '0;
          done_irq_r      <= 1'b0;
          ch_state_r      <= ST_IDLE;
          r_beats_rem     <= '0;
          fifo_wr_ptr     <= '0;
          fifo_rd_ptr     <= '0;
          fifo_cnt        <= '0;
          req.valid       <= 1'b0;
          req.req         <= '0;
          req.wdata       <= '0;
          req.wlast       <= 1'b0;
          mm_m2s          <= '0;
        end else begin
          done_irq_r <= 1'b0;
          case (state)
            C_IDLE: begin
              ch_state_r <= ST_IDLE;
              if (ctrl.enable && ctrl.doorbell) begin
                cur_sar         <= ctrl.sar;
                cur_dar         <= ctrl.dar;
                rem_bytes       <= ctrl.xfer_size;
                bytes_xferred_r <= '0;
                ch_state_r      <= ST_RUN;
                state           <= C_AR;
              end
            end

            C_AR: begin
              mm_m2s.araddr  <= cur_sar;
              mm_m2s.arlen   <= burst_len;
              mm_m2s.arsize  <= 3'b110;
              mm_m2s.arburst <= 2'b01;
              mm_m2s.arvalid <= 1'b1;
              r_beats_rem    <= burst_len;
              fifo_wr_ptr    <= '0;
              fifo_rd_ptr    <= '0;
              fifo_cnt       <= '0;
              if (mm_m2s.arvalid && mm_s2m.arready) begin
                mm_m2s.arvalid <= 1'b0;
                mm_m2s.rready  <= 1'b1;
                state          <= C_STG;
              end
            end

            C_STG: begin
              if (mm_m2s.rready && mm_s2m.rvalid) begin
                fifo_data[fifo_wr_ptr] <= mm_s2m.rdata;
                fifo_wr_ptr            <= fifo_wr_ptr + 1'b1;
                fifo_cnt               <= fifo_cnt + 1'b1;
                if (r_beats_rem != 0) begin
                  r_beats_rem <= r_beats_rem - 1'b1;
                end else begin
                  mm_m2s.rready <= 1'b0;
                  state         <= C_MWR;
                end
              end
            end

            C_MWR: begin
              req.valid        <= 1'b1;
              req.req.is_read  <= 1'b0;
              req.req.addr     <= cur_dar;
              req.req.len_bytes<= seg_len;
              req.req.tag      <= 10'h01; // Channel 1
              req.req.first_be <= 4'hF;
              req.req.last_be  <= 4'hF;
              req.wdata        <= fifo_data[fifo_rd_ptr];
              req.wlast        <= (fifo_cnt == 1);
              if (req.valid && req_ready) begin
                fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
                fifo_cnt    <= fifo_cnt - 1'b1;
                if (fifo_cnt == 1) begin
                  req.valid <= 1'b0;
                  state     <= C_NEXT;
                end
              end
            end

            C_NEXT: begin
              cur_sar         <= cur_sar + 64'(seg_len);
              cur_dar         <= cur_dar + 64'(seg_len);
              bytes_xferred_r <= bytes_xferred_r + 64'(seg_len);
              rem_bytes       <= rem_bytes - 32'(seg_len);
              if (rem_bytes <= 32'(seg_len)) begin
                state <= C_DONE;
              end else begin
                state <= C_AR;
              end
            end

            C_DONE: begin
              done_irq_r <= 1'b1;
              ch_state_r <= ST_IDLE;
              state      <= C_IDLE;
            end

            default: state <= C_IDLE;
          endcase
        end
      end
    end
  endgenerate

  // Output status assignments
  assign stat.state         = ch_state_r;
  assign stat.err           = ERR_NONE;
  assign stat.bytes_xferred = bytes_xferred_r;
  assign stat.done_irq      = done_irq_r;
  assign stat.abort_irq     = 1'b0;
  assign stat.watermark_irq = 1'b0;

endmodule
```

- [ ] **Step 2: Run lint verification**

Run: `make lint`  
Expected: Clean exit code 0.

- [ ] **Step 3: Commit**

```bash
git add rtl/blocks/dma_channel.sv
git commit -m "feat(channel): implement H2C and C2H register-mode data movers with 4KB segmentation"
```

---

### Task 5: Directed Testbench Implementation (`sim/tb_m2.sv`) & Makefile Automation

**Files:**
- Create: `sim/tb_m2.sv`
- Modify: `Makefile`
- Test: `make sim-m2`

**Interfaces:**
- Consumes: `dma_top`, `axi_ram`, `dma_pkg`.
- Produces: Executable self-checking testbench executing Tests 1 through 7 and reporting `[tb_m2] ALL TESTS PASSED`.

- [ ] **Step 1: Write `sim/tb_m2.sv`**

Implement `sim/tb_m2.sv` with the PCIe Root Complex BFM (host memory, APB driver, completion responder, write sink) and all 7 verification scenarios:

```systemverilog
// ============================================================================
// tb_m2.sv — Directed verification testbench for Milestone M2.
// Exercises: CSR sanity, single-beat H2C, multi-beat H2C, single-beat C2H,
// multi-beat C2H, 4KB page boundary split, and concurrent full-duplex operation.
// ============================================================================
module tb_m2;
  import dma_pkg::*;

  localparam int DW = DATA_WIDTH;
  localparam int NCH = NUM_RD_CH + NUM_WR_CH;
  localparam int HOST_MEM_SIZE = 1024 * 1024; // 1 MB host memory model

  // Clocks and resets
  logic core_clk = 0, core_rst_n = 0;
  logic axi_clk  = 0, axi_rst_n  = 0;
  logic csr_clk  = 0, csr_rst_n  = 0;

  always #1 core_clk = ~core_clk; // 500 MHz
  always #1.25 axi_clk = ~axi_clk; // 400 MHz
  always #5 csr_clk = ~csr_clk;   // 100 MHz

  // Host PCIe Transaction Buses
  pcie_req_bus_t host_req;
  logic          host_req_ready;
  pcie_cpl_bus_t host_cpl;
  logic          host_cpl_ready;

  // Host APB Slave Interface
  logic [19:0] s_apb_paddr;
  logic        s_apb_psel;
  logic        s_apb_penable;
  logic        s_apb_pwrite;
  logic [31:0] s_apb_pwdata;
  logic [3:0]  s_apb_pstrb;
  logic [31:0] s_apb_prdata;
  logic        s_apb_pready;
  logic        s_apb_pslverr;

  // Device AXI4-MM Buses
  axi_mm_m2s_t dev_mm_m2s     [NCH];
  axi_mm_s2m_t dev_mm_s2m     [NCH];

  // Device AXI4-Stream Buses
  axis_data_t  dev_st_o       [NCH];
  logic        dev_st_o_ready [NCH];
  axis_data_t  dev_st_i       [NCH];
  logic        dev_st_i_ready [NCH];

  // Interrupts
  logic        ch_irq         [NCH];
  ch_msi_t     ch_msi         [NCH];

  // Host memory array
  logic [7:0] host_mem [HOST_MEM_SIZE];

  int errors = 0;

  // DUT instantiation
  dma_top u_dut (
    .core_clk       (core_clk),
    .core_rst_n     (core_rst_n),
    .axi_clk        (axi_clk),
    .axi_rst_n      (axi_rst_n),
    .csr_clk        (csr_clk),
    .csr_rst_n      (csr_rst_n),
    .host_req       (host_req),
    .host_req_ready (host_req_ready),
    .host_cpl       (host_cpl),
    .host_cpl_ready (host_cpl_ready),
    .s_apb_paddr    (s_apb_paddr),
    .s_apb_psel     (s_apb_psel),
    .s_apb_penable  (s_apb_penable),
    .s_apb_pwrite   (s_apb_pwrite),
    .s_apb_pwdata   (s_apb_pwdata),
    .s_apb_pstrb    (s_apb_pstrb),
    .s_apb_prdata   (s_apb_prdata),
    .s_apb_pready   (s_apb_pready),
    .s_apb_pslverr  (s_apb_pslverr),
    .dev_mm_m2s     (dev_mm_m2s),
    .dev_mm_s2m     (dev_mm_s2m),
    .dev_st_o       (dev_st_o),
    .dev_st_o_ready (dev_st_o_ready),
    .dev_st_i       (dev_st_i),
    .dev_st_i_ready (dev_st_i_ready),
    .ch_irq         (ch_irq),
    .ch_msi         (ch_msi)
  );

  // Device AXI RAM models
  axi_ram u_ram_h2c (
    .clk       (axi_clk),
    .rst_n     (axi_rst_n),
    .s_axi_m2s (dev_mm_m2s[0]),
    .s_axi_s2m (dev_mm_s2m[0])
  );

  axi_ram u_ram_c2h (
    .clk       (axi_clk),
    .rst_n     (axi_rst_n),
    .s_axi_m2s (dev_mm_m2s[NUM_RD_CH]),
    .s_axi_s2m (dev_mm_s2m[NUM_RD_CH])
  );

  // Tie-off remaining unused channels in M2
  genvar k;
  generate
    for (k = 0; k < NCH; k++) begin : g_ties
      if (k != 0 && k != NUM_RD_CH) begin
        assign dev_mm_s2m[k]     = '0;
      end
      assign dev_st_o_ready[k] = 1'b1;
      assign dev_st_i[k]       = '0;
    end
  endgenerate

  // PCIe BFM: Request & Completion handler
  assign host_req_ready = 1'b1;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      host_cpl <= '0;
    end else begin
      host_cpl.valid <= 1'b0;
      if (host_req.valid && host_req.req.is_read) begin
        // Return 512-bit completion beat
        host_cpl.valid           <= 1'b1;
        host_cpl.meta.tag        <= host_req.req.tag;
        host_cpl.meta.byte_count <= host_req.req.len_bytes;
        host_cpl.meta.lower_addr <= host_req.req.addr[6:0];
        host_cpl.meta.status     <= CPL_SC;
        host_cpl.meta.last       <= 1'b1;
        for (int i = 0; i < 64; i++) begin
          int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
          host_cpl.rdata[8*i +: 8] <= host_mem[off];
        end
      end else if (host_req.valid && !host_req.req.is_read) begin
        // Capture write beat into host memory
        for (int i = 0; i < 64; i++) begin
          int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
          host_mem[off] <= host_req.wdata[8*i +: 8];
        end
      end
    end
  end

  // APB Tasks
  task automatic apb_write(input logic [19:0] addr, input logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1;
    s_apb_pwrite  = 1;
    s_apb_paddr   = addr;
    s_apb_pwdata  = data;
    s_apb_pstrb   = 4'hF;
    s_apb_penable = 0;
    @(negedge csr_clk);
    s_apb_penable = 1;
    @(negedge csr_clk);
    s_apb_psel    = 0;
    s_apb_penable = 0;
    s_apb_pwrite  = 0;
  endtask

  task automatic apb_read(input logic [19:0] addr, output logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1;
    s_apb_pwrite  = 0;
    s_apb_paddr   = addr;
    s_apb_penable = 0;
    @(negedge csr_clk);
    s_apb_penable = 1;
    @(negedge csr_clk);
    data          = s_apb_prdata;
    s_apb_psel    = 0;
    s_apb_penable = 0;
  endtask

  // Helper check function
  task automatic check_val(input string msg, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin
      $display("  [FAIL] %s: got=%016x exp=%016x", msg, got, exp);
      errors++;
    end else begin
      $display("  [OK]   %s = %08x", msg, got[31:0]);
    end
  endtask

  // Test sequence
  initial begin
    logic [31:0] rdata;
    $display("[tb_m2] Starting Milestone M2 Verification Suite...");

    // Reset sequence
    core_rst_n = 0; axi_rst_n = 0; csr_rst_n = 0;
    s_apb_psel = 0; s_apb_penable = 0; s_apb_pwrite = 0;
    repeat (10) @(posedge csr_clk);
    core_rst_n = 1; axi_rst_n = 1; csr_rst_n = 1;
    repeat (5) @(posedge csr_clk);

    // ------------------------------------------------------------------------
    // Test 1: CSR Sanity
    // ------------------------------------------------------------------------
    $display("\n--- Test 1: CSR Sanity ---");
    apb_read(20'h0000, rdata); check_val("ID_VERSION", rdata, 32'h444D_0100);
    apb_read(20'h0004, rdata); check_val("CAPABILITIES", rdata[5:0], 6'h3F);

    // ------------------------------------------------------------------------
    // Test 2: Single-Beat H2C (64 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 2: Single-Beat H2C (64 Bytes) ---");
    for (int i = 0; i < 64; i++) host_mem[32'h0001_0000 + i] = 8'(i + 1);
    apb_write(20'h0008, 32'h1); // RD_ENGINE_EN
    apb_write(20'h1000, 32'h1); // CH0_EN
    apb_write(20'h1020, 32'h0001_0000); // SAR_LO
    apb_write(20'h1024, 32'h0000_0000); // SAR_HI
    apb_write(20'h1028, 32'h0002_0000); // DAR_LO
    apb_write(20'h102C, 32'h0000_0000); // DAR_HI
    apb_write(20'h101C, 32'd64);        // TRANSFER_SIZE
    apb_write(20'h1004, 32'h1);         // DOORBELL

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 IRQ Asserted", ch_irq[0], 1);
    check_val("H2C Byte 0 in RAM", u_ram_h2c.read_byte(64'h0002_0000), 8'h01);
    check_val("H2C Byte 63 in RAM", u_ram_h2c.read_byte(64'h0002_003F), 8'd64);
    apb_write(20'h1054, 32'h1); // Clear done IRQ

    // ------------------------------------------------------------------------
    // Test 3: Multi-Beat H2C Burst (512 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 3: Multi-Beat H2C Burst (512 Bytes) ---");
    for (int i = 0; i < 512; i++) host_mem[32'h0001_0200 + i] = 8'(i ^ 8'hA5);
    apb_write(20'h1020, 32'h0001_0200);
    apb_write(20'h1028, 32'h0002_0200);
    apb_write(20'h101C, 32'd512);
    apb_write(20'h1004, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Multi-Beat Done", ch_irq[0], 1);
    check_val("H2C Burst Byte 0", u_ram_h2c.read_byte(64'h0002_0200), 8'hA5);
    check_val("H2C Burst Byte 511", u_ram_h2c.read_byte(64'h0002_03FF), 8'(511 ^ 8'hA5));
    apb_write(20'h1054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 4: Single-Beat C2H (64 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 4: Single-Beat C2H (64 Bytes) ---");
    for (int i = 0; i < 64; i++) u_ram_c2h.write_byte(64'h0003_0000 + i, 8'(i + 8'h50));
    apb_write(20'h000C, 32'h1); // WR_ENGINE_EN
    apb_write(20'h5000, 32'h1); // CH1_EN (WR Ch 0 = global ch 4 in 4+4)
    apb_write(20'h5020, 32'h0003_0000); // SAR_LO
    apb_write(20'h5028, 32'h0004_0000); // DAR_LO
    apb_write(20'h501C, 32'd64);        // TRANSFER_SIZE
    apb_write(20'h5004, 32'h1);         // DOORBELL

    wait (ch_irq[NUM_RD_CH] == 1'b1);
    check_val("Ch1 IRQ Asserted", ch_irq[NUM_RD_CH], 1);
    check_val("C2H Byte 0 in Host Mem", host_mem[32'h0004_0000], 8'h50);
    check_val("C2H Byte 63 in Host Mem", host_mem[32'h0004_003F], 8'(63 + 8'h50));
    apb_write(20'h5054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 5: Multi-Beat C2H Burst (512 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 5: Multi-Beat C2H Burst (512 Bytes) ---");
    for (int i = 0; i < 512; i++) u_ram_c2h.write_byte(64'h0003_0200 + i, 8'(i ^ 8'h3C));
    apb_write(20'h5020, 32'h0003_0200);
    apb_write(20'h5028, 32'h0004_0200);
    apb_write(20'h501C, 32'd512);
    apb_write(20'h5004, 32'h1);

    wait (ch_irq[NUM_RD_CH] == 1'b1);
    check_val("C2H Multi-Beat Done", ch_irq[NUM_RD_CH], 1);
    check_val("C2H Burst Byte 0", host_mem[32'h0004_0200], 8'h3C);
    check_val("C2H Burst Byte 511", host_mem[32'h0004_03FF], 8'(511 ^ 8'h3C));
    apb_write(20'h5054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 6: 4KB Page Boundary Splitting Test
    // ------------------------------------------------------------------------
    $display("\n--- Test 6: 4KB Page Boundary Splitting Test ---");
    // Start address at 0x0001_0F00 (256 bytes before 0x0001_1000 4KB boundary)
    for (int i = 0; i < 512; i++) host_mem[32'h0001_0F00 + i] = 8'(i + 8'h11);
    apb_write(20'h1020, 32'h0001_0F00); // 256B to 4K boundary
    apb_write(20'h1028, 32'h0002_0F00);
    apb_write(20'h101C, 32'd512);        // spans across 4KB boundary
    apb_write(20'h1004, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("4KB Split Transfer Done", ch_irq[0], 1);
    check_val("4KB Split First Byte", u_ram_h2c.read_byte(64'h0002_0F00), 8'h11);
    check_val("4KB Boundary Crossing Byte", u_ram_h2c.read_byte(64'h0002_1000), 8'(256 + 8'h11));
    check_val("4KB Split Last Byte", u_ram_h2c.read_byte(64'h0002_10FF), 8'(511 + 8'h11));
    apb_write(20'h1054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 7: Concurrent Full-Duplex Operation
    // ------------------------------------------------------------------------
    $display("\n--- Test 7: Concurrent Full-Duplex Operation ---");
    for (int i = 0; i < 128; i++) begin
      host_mem[32'h0001_2000 + i] = 8'(i + 8'hAA);
      u_ram_c2h.write_byte(64'h0003_2000 + i, 8'(i + 8'h55));
    end
    // Launch H2C
    apb_write(20'h1020, 32'h0001_2000);
    apb_write(20'h1028, 32'h0002_2000);
    apb_write(20'h101C, 32'd128);
    apb_write(20'h1004, 32'h1);
    // Launch C2H
    apb_write(20'h5020, 32'h0003_2000);
    apb_write(20'h5028, 32'h0004_2000);
    apb_write(20'h501C, 32'd128);
    apb_write(20'h5004, 32'h1);

    wait (ch_irq[0] == 1'b1 && ch_irq[NUM_RD_CH] == 1'b1);
    check_val("Concurrent H2C Byte 0", u_ram_h2c.read_byte(64'h0002_2000), 8'hAA);
    check_val("Concurrent C2H Byte 0", host_mem[32'h0004_2000], 8'h55);
    apb_write(20'h1054, 32'h1);
    apb_write(20'h5054, 32'h1);

    // Final summary
    if (errors == 0) begin
      $display("\n=======================================================");
      $display(" [tb_m2] ALL TESTS PASSED (0 errors)");
      $display("=======================================================\n");
    end else begin
      $display("\n[tb_m2] FAILED with %0d errors\n", errors);
    end
    $finish;
  end

  // Watchdog timeout
  initial begin
    #200000;
    $display("\n[tb_m2] ERROR: Simulation timed out!\n");
    $finish;
  end

endmodule
```

- [ ] **Step 2: Add `sim-m2` target to `Makefile`**

Append `sim-m2` to `Makefile`:

```makefile
# Milestone M2 directed simulation
sim-m2:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m2.sv --top-module tb_m2 -o tb_m2
	@./obj_dir/tb_m2
```

- [ ] **Step 3: Run `make sim-m2`**

Run: `make sim-m2`  
Expected: All 7 tests pass with `[tb_m2] ALL TESTS PASSED (0 errors)`.

- [ ] **Step 4: Commit**

```bash
git add sim/tb_m2.sv Makefile
git commit -m "feat(sim): add Milestone M2 directed testbench and make sim-m2 target"
```

---

### Task 6: Full Regression & Documentation Update

**Files:**
- Modify: `README.md`
- Test: All make targets (`make lint`, `make lint-if`, `make sim-csr`, `make sim-m2`)

- [ ] **Step 1: Run full regression**

Run: `make clean && make lint && make lint-if && make sim-csr && make sim-m2`  
Expected: All targets exit with code 0.

- [ ] **Step 2: Update `README.md`**

Update Milestone M2 status in `README.md` to reflect the completed IP-agnostic implementation, verification results, and simulation instructions.

- [ ] **Step 3: Commit and Push**

```bash
git add README.md
git commit -m "docs: mark Milestone M2 complete in README with verification instructions"
git push origin main
```
