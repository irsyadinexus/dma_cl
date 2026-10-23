// ============================================================================
// chan_arbiter.sv — Multi-channel request arbiter and completion demux.
// Features: Weighted Round-Robin (WRR) QoS with atomic multi-beat burst locking
// and tag-sliced completion demultiplexer.
// ============================================================================
module chan_arbiter #(
  parameter int NUM_CH = 8
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  // Per-channel QoS weights (from ch_ctrl.qos)
  input  logic [3:0]             ch_qos       [NUM_CH],
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
  logic                need_refresh;
  logic                in_burst;
  logic [CH_WIDTH-1:0] burst_ch;

  // Round-robin pointer and per-channel credit counters
  logic [CH_WIDTH-1:0] rr_ptr;
  logic [7:0]          credit [NUM_CH];

  // Helper function: select next requesting channel via WRR
  function automatic logic [CH_WIDTH-1:0] pick_wrr_grant(
    input logic [CH_WIDTH-1:0]    ptr,
    input dma_pkg::pcie_req_bus_t reqs [NUM_CH],
    input logic [7:0]             cr   [NUM_CH],
    output logic                  valid,
    output logic                  refresh_req
  );
    logic [CH_WIDTH-1:0] chosen;
    valid = 1'b0;
    refresh_req = 1'b0;
    chosen = ptr;

    // Pass 1: find channel with valid request and available credit
    for (int i = 0; i < NUM_CH; i++) begin
      int unsigned idx = (int'(ptr) + i) % NUM_CH;
      if (reqs[idx].valid && cr[idx] > 0 && !valid) begin
        chosen = CH_WIDTH'(idx);
        valid = 1'b1;
      end
    end

    // Pass 2: if all requesting channels exhausted credits, trigger refresh
    if (!valid) begin
      for (int i = 0; i < NUM_CH; i++) begin
        int unsigned idx = (int'(ptr) + i) % NUM_CH;
        if (reqs[idx].valid && !valid) begin
          chosen = CH_WIDTH'(idx);
          valid = 1'b1;
          refresh_req = 1'b1;
        end
      end
    end
    return chosen;
  endfunction

  // Request arbitration logic
  always_comb begin
    if (in_burst) begin
      gnt          = burst_ch;
      gnt_valid    = ch_req[burst_ch].valid;
      need_refresh = 1'b0;
    end else begin
      gnt = pick_wrr_grant(rr_ptr, ch_req, credit, gnt_valid, need_refresh);
    end
  end

  // Lock grant during multi-beat write bursts and manage credits
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      rr_ptr   <= '0;
      in_burst <= 1'b0;
      burst_ch <= '0;
      for (int i = 0; i < NUM_CH; i++) begin
        credit[i] <= (ch_qos[i] == 4'd0) ? 8'd1 : {4'b0, ch_qos[i]};
      end
    end else begin
      // Refresh credits when exhausted
      if (need_refresh) begin
        for (int i = 0; i < NUM_CH; i++) begin
          credit[i] <= (ch_qos[i] == 4'd0) ? 8'd1 : {4'b0, ch_qos[i]};
        end
      end

      if (gnt_valid && m_req_ready) begin
        if (!ch_req[gnt].req.is_read && !ch_req[gnt].wlast) begin
          // Lock burst
          in_burst <= 1'b1;
          burst_ch <= gnt;
        end else begin
          in_burst <= 1'b0;
          if (need_refresh) begin
            logic [7:0] w;
            w = (ch_qos[gnt] == 4'd0) ? 8'd1 : {4'b0, ch_qos[gnt]};
            credit[gnt] <= (w > 8'd1) ? w - 8'd1 : 8'd0;
            if (w <= 8'd1) begin
              rr_ptr <= (gnt == CH_WIDTH'(NUM_CH - 1)) ? '0 : gnt + 1'b1;
            end
          end else begin
            if (credit[gnt] > 8'd1) begin
              credit[gnt] <= credit[gnt] - 8'd1;
            end else begin
              credit[gnt] <= (ch_qos[gnt] == 4'd0) ? 8'd1 : {4'b0, ch_qos[gnt]};
              rr_ptr <= (gnt == CH_WIDTH'(NUM_CH - 1)) ? '0 : gnt + 1'b1;
            end
          end
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
