// ============================================================================
// chan_arbiter.sv — multiplex N channel requesters onto the shared requester,
// and demux completions back to the owning channel by tag.
// M1 STRUCTURAL STUB. TODO(M3): weighted round-robin + credit-aware issue +
// tag->channel completion routing table.
// ============================================================================
module chan_arbiter #(
  parameter int NUM_CH = 8
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  // per-channel side
  input  dma_pkg::pcie_req_bus_t ch_req       [NUM_CH],
  output logic                   ch_req_ready [NUM_CH],
  output dma_pkg::pcie_cpl_bus_t ch_cpl       [NUM_CH],
  input  logic                   ch_cpl_ready [NUM_CH],
  // merged side (to ATS/PAA)
  output dma_pkg::pcie_req_bus_t m_req,
  input  logic                   m_req_ready,
  input  dma_pkg::pcie_cpl_bus_t m_cpl,
  output logic                   m_cpl_ready
);
  import dma_pkg::*;

  // ---- STUB tie-offs ----
  assign m_req       = '0;
  assign m_cpl_ready = 1'b0;

  genvar i;
  generate
    for (i = 0; i < NUM_CH; i++) begin : g_tie
      assign ch_req_ready[i] = 1'b0;
      assign ch_cpl[i]       = '0;
    end
  endgenerate

endmodule
