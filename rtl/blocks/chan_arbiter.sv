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
  wire [CH_WIDTH-1:0] cpl_dest_ch = CH_WIDTH'(int'(m_cpl.meta.tag) % NUM_CH);

  always_comb begin
    m_cpl_ready = ch_cpl_ready[cpl_dest_ch];
    for (int i = 0; i < NUM_CH; i++) begin
      ch_cpl[i]       = m_cpl;
      ch_cpl[i].valid = m_cpl.valid && (cpl_dest_ch == CH_WIDTH'(i));
    end
  end

endmodule
