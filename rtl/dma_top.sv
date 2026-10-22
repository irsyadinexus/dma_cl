// ============================================================================
// dma_top.sv — PCIe Gen4 HDMA-native scatter-gather DMA engine, top level.
//
// Datapath pipeline:
//   dma_channel[N] --req--> chan_arbiter --> ats_pri_subsys --> dwc_paa <=> DWC
//                    <--cpl--            <--             <--
// Control plane:
//   host BAR --(DWC)--> dwc_paa completer --APB--> csr_block --ctrl/stat--> chan[N]
//
// Module boundaries use packed-struct bundles (dma_pkg) for tool portability
// (Verilator + Icarus + commercial) and synthesis. The rtl/if/*.sv interfaces
// mirror these bundles 1:1 for TB/BFM use. M1 blocks are structural stubs.
// ============================================================================
module dma_top (
  // ---- clocks / resets ----
  input  logic pcie_app_clk,   // DWC application clock (~500 MHz)
  input  logic pcie_app_rst_n,
  input  logic core_clk,       // DMA datapath
  input  logic core_rst_n,
  input  logic axi_clk,        // device-side AXI
  input  logic axi_rst_n,
  input  logic csr_clk,        // register access
  input  logic csr_rst_n,

  // ---- DWC controller status (native app bus attaches in dwc_paa) ----
  input  logic dwc_link_up,

  // ---- device-side AXI4-MM, per channel ----
  output dma_pkg::axi_mm_m2s_t dev_mm_m2s     [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  dma_pkg::axi_mm_s2m_t dev_mm_s2m     [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],

  // ---- device-side AXI4-Stream, per channel (H2C: st_o ; C2H: st_i) ----
  output dma_pkg::axis_data_t  dev_st_o       [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  logic                 dev_st_o_ready [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  input  dma_pkg::axis_data_t  dev_st_i       [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH],
  output logic                 dev_st_i_ready [dma_pkg::NUM_RD_CH + dma_pkg::NUM_WR_CH]
);
  import dma_pkg::*;

  localparam int NUM_CH = NUM_RD_CH + NUM_WR_CH;
  localparam int APB_AW = 20;

  // ---- internal buses ----
  pcie_req_bus_t ch_req       [NUM_CH];
  logic          ch_req_ready [NUM_CH];
  pcie_cpl_bus_t ch_cpl       [NUM_CH];
  logic          ch_cpl_ready [NUM_CH];

  pcie_req_bus_t arb_req;  logic arb_req_ready;
  pcie_cpl_bus_t arb_cpl;  logic arb_cpl_ready;
  pcie_req_bus_t paa_req;  logic paa_req_ready;
  pcie_cpl_bus_t paa_cpl;  logic paa_cpl_ready;

  ats_pri_up_t   ats_up;
  ats_pri_dn_t   ats_dn;

  ch_ctrl_t      ch_ctrl [NUM_CH];
  ch_stat_t      ch_stat [NUM_CH];
  ch_msi_t       ch_msi  [NUM_CH];
  logic          ch_irq  [NUM_CH];

  logic          fc_np_ok, fc_p_ok;
  logic          rd_en, wr_en, soft_reset;

  // APB: PAA completer -> CSR
  logic [APB_AW-1:0] apb_paddr;
  logic              apb_psel, apb_penable, apb_pwrite;
  logic [31:0]       apb_pwdata;
  logic [3:0]        apb_pstrb;
  logic [31:0]       apb_prdata;
  logic              apb_pready, apb_pslverr;

  // ---- per-channel movers ----
  genvar g;
  generate
    for (g = 0; g < NUM_CH; g++) begin : g_chan
      dma_channel #(
        .IS_WRITE   (g >= NUM_RD_CH)   // first NUM_RD_CH are Read/H2C
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

  // ---- arbiter / QoS + completion demux ----
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

  // ---- ATS/PASID/PRI (transparent stub at M1) ----
  ats_pri_subsys u_ats (
    .core_clk     (core_clk),
    .core_rst_n   (core_rst_n),
    .up_req       (arb_req),
    .up_req_ready (arb_req_ready),
    .up_cpl       (arb_cpl),
    .up_cpl_ready (arb_cpl_ready),
    .dn_req       (paa_req),
    .dn_req_ready (paa_req_ready),
    .dn_cpl       (paa_cpl),
    .dn_cpl_ready (paa_cpl_ready),
    .ats_up       (ats_up),
    .ats_dn       (ats_dn),
    .ats_en       (1'b0),
    .pri_en       (1'b0)
  );

  // ---- DWC PCIe application adapter ----
  dwc_paa #(.NUM_CH(NUM_CH), .APB_AW(APB_AW)) u_paa (
    .pcie_app_clk   (pcie_app_clk),
    .pcie_app_rst_n (pcie_app_rst_n),
    .core_clk       (core_clk),
    .core_rst_n     (core_rst_n),
    .link_up        (dwc_link_up),
    .ch_msi         (ch_msi),
    .ch_irq         (ch_irq),
    .req            (paa_req),
    .req_ready      (paa_req_ready),
    .cpl            (paa_cpl),
    .cpl_ready      (paa_cpl_ready),
    .fc_np_ok       (fc_np_ok),
    .fc_p_ok        (fc_p_ok),
    .ats_up         (ats_up),
    .ats_dn         (ats_dn),
    .csr_paddr      (apb_paddr),
    .csr_psel       (apb_psel),
    .csr_penable    (apb_penable),
    .csr_pwrite     (apb_pwrite),
    .csr_pwdata     (apb_pwdata),
    .csr_pstrb      (apb_pstrb),
    .csr_prdata     (apb_prdata),
    .csr_pready     (apb_pready),
    .csr_pslverr    (apb_pslverr)
  );

  // ---- CSR / register block ----
  csr_block #(.NUM_RD_CH(NUM_RD_CH), .NUM_WR_CH(NUM_WR_CH), .APB_AW(APB_AW)) u_csr (
    .csr_clk      (csr_clk),
    .csr_rst_n    (csr_rst_n),
    .paddr        (apb_paddr),
    .psel         (apb_psel),
    .penable      (apb_penable),
    .pwrite       (apb_pwrite),
    .pwdata       (apb_pwdata),
    .pstrb        (apb_pstrb),
    .prdata       (apb_prdata),
    .pready       (apb_pready),
    .pslverr      (apb_pslverr),
    .ch_ctrl      (ch_ctrl),
    .ch_stat      (ch_stat),
    .ch_msi       (ch_msi),
    .ch_irq       (ch_irq),
    .rd_engine_en (rd_en),
    .wr_engine_en (wr_en),
    .soft_reset   (soft_reset)
  );

endmodule
