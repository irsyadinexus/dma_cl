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
        .CH_ID      (g),
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

  // ---- Arbiter (Multiplex Requests, Demux Completions, WRR QoS) ----
  logic [3:0] ch_qos [NUM_CH];
  always_comb begin
    for (int i = 0; i < NUM_CH; i++) begin
      ch_qos[i] = ch_ctrl[i].qos;
    end
  end

  chan_arbiter #(.NUM_CH(NUM_CH)) u_arb (
    .core_clk     (core_clk),
    .core_rst_n   (core_rst_n),
    .ch_qos       (ch_qos),
    .ch_req       (ch_req),
    .ch_req_ready (ch_req_ready),
    .ch_cpl       (ch_cpl),
    .ch_cpl_ready (ch_cpl_ready),
    .m_req        (arb_req),
    .m_req_ready  (arb_req_ready),
    .m_cpl        (arb_cpl),
    .m_cpl_ready  (arb_cpl_ready)
  );

  // ---- ATS/ATC CSR Interface Signals ----
  logic [31:0]          ats_ctrl;
  logic [31:0]          atc_inval;
  logic                 atc_inval_pulse;
  dma_pkg::atc_status_t atc_stat;

  // ---- ATS/PASID/PRI Subsystem (Milestone M4) ----
  ats_pri_subsys u_ats (
    .core_clk        (core_clk),
    .core_rst_n      (core_rst_n),
    .up_req          (arb_req),
    .up_req_ready    (arb_req_ready),
    .up_cpl          (arb_cpl),
    .up_cpl_ready    (arb_cpl_ready),
    .dn_req          (host_req),
    .dn_req_ready    (host_req_ready),
    .dn_cpl          (host_cpl),
    .dn_cpl_ready    (host_cpl_ready),
    .ats_up          (ats_up),
    .ats_dn          (ats_dn),
    .ats_en          (ats_ctrl[0]),
    .pri_en          (1'b0),
    .atc_inval_reg   (atc_inval),
    .atc_inval_pulse (atc_inval_pulse),
    .atc_stat        (atc_stat)
  );

  // ---- CSR Register Block (Connected Directly to Top APB) ----
  csr_block #(.NUM_RD_CH(NUM_RD_CH), .NUM_WR_CH(NUM_WR_CH), .APB_AW(APB_AW)) u_csr (
    .csr_clk           (csr_clk),
    .csr_rst_n         (csr_rst_n),
    .paddr             (s_apb_paddr),
    .psel              (s_apb_psel),
    .penable           (s_apb_penable),
    .pwrite            (s_apb_pwrite),
    .pwdata            (s_apb_pwdata),
    .pstrb             (s_apb_pstrb),
    .prdata            (s_apb_prdata),
    .pready            (s_apb_pready),
    .pslverr           (s_apb_pslverr),
    .ch_ctrl           (ch_ctrl),
    .ch_stat           (ch_stat),
    .ch_msi            (ch_msi),
    .ch_irq            (ch_irq),
    .rd_engine_en      (rd_en),
    .wr_engine_en      (wr_en),
    .soft_reset        (soft_reset),
    .ats_ctrl_o        (ats_ctrl),
    .atc_inval_o       (atc_inval),
    .atc_inval_pulse_o (atc_inval_pulse),
    .atc_stat_i        (atc_stat)
  );

endmodule
