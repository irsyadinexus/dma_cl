// ============================================================================
// dwc_paa.sv — PCIe Application Adapter (Synopsys DWC native app-layer shim).
// The only DWC-specific block. Assembles/parses TLPs, manages tags/credits/
// straddle framing, carries ATS/PRI messages, drives MSI-X, and presents the
// inbound BAR (completer) path to the CSR block as an APB master.
// M1 STRUCTURAL STUB. The DWC native application bus is databook-dependent
// (see docs/m1_paa_contract.md) and attaches here at reconciliation.
// TODO(M2): TLP framing, tag mgr, completion reassembly, MSI-X, credit tracking.
// ============================================================================
module dwc_paa #(
  parameter int NUM_CH = 8,
  parameter int APB_AW = 20
) (
  input  logic                   pcie_app_clk,
  input  logic                   pcie_app_rst_n,
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  // DWC controller status (placeholder for the native app bus)
  input  logic                   link_up,
  // MSI-X config + per-channel interrupt requests (from CSR) -> IMWr generator
  input  dma_pkg::ch_msi_t       ch_msi [NUM_CH],
  input  logic                   ch_irq [NUM_CH],
  // shared requester / completion (core side)
  input  dma_pkg::pcie_req_bus_t req,
  output logic                   req_ready,
  output dma_pkg::pcie_cpl_bus_t cpl,
  input  logic                   cpl_ready,
  output logic                   fc_np_ok,
  output logic                   fc_p_ok,
  // ATS/PRI messages
  input  dma_pkg::ats_pri_up_t   ats_up,
  output dma_pkg::ats_pri_dn_t   ats_dn,
  // CSR completer -> APB master to csr_block
  output logic [APB_AW-1:0]      csr_paddr,
  output logic                   csr_psel,
  output logic                   csr_penable,
  output logic                   csr_pwrite,
  output logic [31:0]            csr_pwdata,
  output logic [3:0]             csr_pstrb,
  input  logic [31:0]            csr_prdata,
  input  logic                   csr_pready,
  input  logic                   csr_pslverr
);
  import dma_pkg::*;

  // ---- STUB tie-offs ----
  assign req_ready   = 1'b0;
  assign cpl         = '0;
  assign fc_np_ok    = 1'b0;
  assign fc_p_ok     = 1'b0;
  assign ats_dn      = '0;
  assign csr_paddr   = '0;
  assign csr_psel    = 1'b0;
  assign csr_penable = 1'b0;
  assign csr_pwrite  = 1'b0;
  assign csr_pwdata  = '0;
  assign csr_pstrb   = '0;

endmodule
