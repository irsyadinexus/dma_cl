// ============================================================================
// dma_channel.sv — one DMA channel (descriptor engine + data mover).
// M1 STRUCTURAL STUB: ports/hierarchy only; datapath logic lands at M2/M3.
// IS_WRITE fixes direction: 0 = Read/H2C (host->device), 1 = Write/C2H.
// TODO(M3): LL descriptor walk, 4KB/MPS/MRRS segmentation, tag/reorder, mover FSM.
// ============================================================================
module dma_channel #(
  parameter bit IS_WRITE = 1'b0
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  input  logic                   axi_clk,
  input  logic                   axi_rst_n,
  // control / status (from/to CSR)
  input  dma_pkg::ch_ctrl_t      ctrl,
  output dma_pkg::ch_stat_t      stat,
  // device-side AXI4-MM (first-class)
  output dma_pkg::axi_mm_m2s_t   mm_m2s,
  input  dma_pkg::axi_mm_s2m_t   mm_s2m,
  // device-side AXI4-Stream (H2C drives st_o; C2H consumes st_i)
  output dma_pkg::axis_data_t    st_o,
  input  logic                   st_o_ready,
  input  dma_pkg::axis_data_t    st_i,
  output logic                   st_i_ready,
  // shared requester / completion (to arbiter)
  output dma_pkg::pcie_req_bus_t req,
  input  logic                   req_ready,
  input  dma_pkg::pcie_cpl_bus_t cpl,
  output logic                   cpl_ready
);
  import dma_pkg::*;

  // ---- STUB tie-offs (idle) ----
  assign stat       = '0;        // state = ST_IDLE, err = ERR_NONE
  assign req        = '0;
  assign mm_m2s     = '0;
  assign st_o       = '0;
  assign st_i_ready = 1'b0;
  assign cpl_ready  = 1'b0;

endmodule
