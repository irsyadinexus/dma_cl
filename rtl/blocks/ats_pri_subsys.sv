// ============================================================================
// ats_pri_subsys.sv — ATS/PASID/PRI subsystem on the merged requester path.
// Translates request addresses (ATC), issues ATS translation requests, and runs
// the PRI page-fault suspend/resume FSM. See docs/m1_ats_pri_microspec.md.
// M1 STRUCTURAL STUB: transparent passthrough (no translation) so the datapath
// is wired end-to-end. TODO(M4/M5): ATC, translation agent, PRI fault engine.
// ============================================================================
module ats_pri_subsys (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  // upstream (arbiter side)
  input  dma_pkg::pcie_req_bus_t up_req,
  output logic                   up_req_ready,
  output dma_pkg::pcie_cpl_bus_t up_cpl,
  input  logic                   up_cpl_ready,
  // downstream (PAA side)
  output dma_pkg::pcie_req_bus_t dn_req,
  input  logic                   dn_req_ready,
  input  dma_pkg::pcie_cpl_bus_t dn_cpl,
  output logic                   dn_cpl_ready,
  // ATS/PRI messages to/from PAA
  output dma_pkg::ats_pri_up_t   ats_up,
  input  dma_pkg::ats_pri_dn_t   ats_dn,
  // enables (from CSR)
  input  logic                   ats_en,
  input  logic                   pri_en
);
  import dma_pkg::*;

  // ---- STUB: transparent passthrough (translation inserted at M4/M5) ----
  assign dn_req       = up_req;
  assign up_req_ready = dn_req_ready;
  assign up_cpl       = dn_cpl;
  assign dn_cpl_ready = up_cpl_ready;
  assign ats_up       = '0;

endmodule
