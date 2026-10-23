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
  logic [CH_WIDTH-1:0] burst_ch;

  // Round-robin pointer
  logic [CH_WIDTH-1:0] rr_ptr;

  // Helper function: select next requesting channel
  function automatic logic [CH_WIDTH-1:0] pick_grant(
    input  logic [CH_WIDTH-1:0]    ptr,
    input  dma_pkg::pcie_req_bus_t reqs [NUM_CH],
    output logic                   valid
  );
    logic [CH_WIDTH-1:0] chosen;
    valid = 1'b0;
    chosen = ptr;
    for (int i = 0; i < NUM_CH; i++) begin
      int unsigned idx = (int'(ptr) + i) % NUM_CH;
      if (reqs[idx].valid && !valid) begin
        chosen = CH_WIDTH'(idx);
        valid = 1'b1;
      end
    end
    return chosen;
  endfunction

  // Request arbitration logic
  always_comb begin
    if (in_burst) begin
      gnt       = burst_ch;
      gnt_valid = ch_req[burst_ch].valid;
    end else begin
      gnt       = pick_grant(rr_ptr, ch_req, gnt_valid);
    end
  end

  // Lock grant during multi-beat write bursts
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      rr_ptr   <= '0;
      in_burst <= 1'b0;
      burst_ch <= '0;
    end else begin
      if (gnt_valid && m_req_ready) begin
        if (!ch_req[gnt].req.is_read && !ch_req[gnt].wlast) begin
          in_burst <= 1'b1;
          burst_ch <= gnt;
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

  // Completion demultiplexing by tag channel slice
  // Slices upper CH_WIDTH bits of tag (tag[9:7] for 8 channels)
  wire [CH_WIDTH-1:0] cpl_dest_ch = m_cpl.meta.tag[9 -: CH_WIDTH];

  always_comb begin
    m_cpl_ready = ch_cpl_ready[cpl_dest_ch];
    for (int i = 0; i < NUM_CH; i++) begin
      ch_cpl[i]       = m_cpl;
      ch_cpl[i].valid = m_cpl.valid && (cpl_dest_ch == CH_WIDTH'(i));
    end
  end

endmodule
