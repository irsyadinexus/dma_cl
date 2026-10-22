// ============================================================================
// pcie_req_if.sv — vendor-neutral requester/completion contract (DMA core <-> PAA)
// M1 skeleton (no logic). The DWC-specific TLP framing lives in the PAA behind
// this interface. See spec §3.1 and docs/m1_paa_contract.md.
// ============================================================================
interface pcie_req_if #(
  parameter int DW = dma_pkg::DATA_WIDTH
) (input logic clk, input logic rst_n);

  import dma_pkg::*;

  // ---- Request issue (mover -> PAA) ----
  logic            req_valid;
  logic            req_ready;
  pcie_req_t       req;                 // hdr fields (addr/len/tag/pasid/at/...)
  logic [DW-1:0]   req_wdata;           // write data (MWr); ignored for MRd
  logic            req_wlast;           // last beat of a multi-beat write

  // ---- Completion return (PAA -> mover), for MRd ----
  logic            cpl_valid;
  logic            cpl_ready;
  pcie_cpl_meta_t  cpl_meta;            // tag/byte_count/lower_addr/status/last
  logic [DW-1:0]   cpl_rdata;

  // ---- Flow-control / status sideband ----
  logic            fc_np_ok;            // non-posted credit available (reads)
  logic            fc_p_ok;             // posted credit available (writes)

  modport mover (
    input  clk, rst_n, req_ready, cpl_valid, cpl_meta, cpl_rdata, fc_np_ok, fc_p_ok,
    output req_valid, req, req_wdata, req_wlast, cpl_ready
  );

  modport paa (
    input  clk, rst_n, req_valid, req, req_wdata, req_wlast, cpl_ready,
    output req_ready, cpl_valid, cpl_meta, cpl_rdata, fc_np_ok, fc_p_ok
  );

endinterface : pcie_req_if
