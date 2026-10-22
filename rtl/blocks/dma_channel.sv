// ============================================================================
// dma_channel.sv — DMA Channel Data Mover (Register Mode, DATA_WIDTH=512).
// IS_WRITE = 0: Read Channel (H2C) Host -> Device
// IS_WRITE = 1: Write Channel (C2H) Device -> Host
// ============================================================================
module dma_channel #(
  parameter bit IS_WRITE = 1'b0
) (
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  input  logic                   axi_clk,
  input  logic                   axi_rst_n,
  // Control & Status (from/to CSR)
  input  dma_pkg::ch_ctrl_t      ctrl,
  output dma_pkg::ch_stat_t      stat,
  // Device-side AXI4-MM
  output dma_pkg::axi_mm_m2s_t   mm_m2s,
  input  dma_pkg::axi_mm_s2m_t   mm_s2m,
  // Device-side AXI4-Stream
  output dma_pkg::axis_data_t    st_o,
  input  logic                   st_o_ready,
  input  dma_pkg::axis_data_t    st_i,
  output logic                   st_i_ready,
  // Shared PCIe Requester / Completion
  output dma_pkg::pcie_req_bus_t req,
  input  logic                   req_ready,
  input  dma_pkg::pcie_cpl_bus_t cpl,
  output logic                   cpl_ready
);
  import dma_pkg::*;

  localparam int STRB_W = DATA_WIDTH / 8; // 64 bytes per beat

  // Working register pointers
  logic [63:0] cur_sar, cur_dar;
  logic [31:0] rem_bytes;
  logic [63:0] bytes_xferred_r;
  logic        done_irq_r;
  ch_state_e   ch_state_r;

  // 4KB Segmentation logic
  wire [63:0] host_addr = IS_WRITE ? cur_dar : cur_sar;
  wire [12:0] bytes_to_4k = 13'h1000 - {1'b0, host_addr[11:0]};
  wire [12:0] seg_len_calc = (rem_bytes < {19'b0, bytes_to_4k}) ? rem_bytes[12:0] : bytes_to_4k;
  wire [12:0] seg_len = (seg_len_calc > 13'd512) ? 13'd512 : seg_len_calc;
  wire [7:0]  burst_len = 8'((seg_len + 13'd63) >> 6) - 8'd1;

  // Stream tie-offs in M2
  assign st_o       = '0;
  assign st_i_ready = 1'b0;

  generate
    if (!IS_WRITE) begin : g_h2c
      // ------------------------------------------------------------------------
      // H2C (Read Channel): Host -> Device Local Memory
      // ------------------------------------------------------------------------
      typedef enum logic [2:0] {
        H_IDLE, H_REQ, H_CPL, H_BRESP, H_DONE
      } h_state_e;
      h_state_e state;
      logic [7:0] w_beats_rem;
      logic [3:0] done_cnt;

      assign cpl_ready = (state == H_CPL && !mm_m2s.wvalid) ? mm_s2m.wready : 1'b0;

      always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
          state           <= H_IDLE;
          cur_sar         <= '0;
          cur_dar         <= '0;
          rem_bytes       <= '0;
          bytes_xferred_r <= '0;
          done_irq_r      <= 1'b0;
          done_cnt        <= '0;
          ch_state_r      <= ST_IDLE;
          w_beats_rem     <= '0;
          req.valid       <= 1'b0;
          req.req         <= '0;
          req.wdata       <= '0;
          req.wlast       <= 1'b0;
          mm_m2s          <= '0;
        end else begin
          if (done_cnt != 0) begin
            done_cnt   <= done_cnt - 1'b1;
            done_irq_r <= 1'b1;
          end else begin
            done_irq_r <= 1'b0;
          end
          case (state)
            H_IDLE: begin
              ch_state_r <= ST_IDLE;
              if (ctrl.enable && ctrl.doorbell && ctrl.xfer_size != 0) begin
                cur_sar         <= ctrl.sar;
                cur_dar         <= ctrl.dar;
                rem_bytes       <= ctrl.xfer_size;
                bytes_xferred_r <= '0;
                ch_state_r      <= ST_RUN;
                state           <= H_REQ;
              end
            end

            H_REQ: begin
              req.valid        <= 1'b1;
              req.req.is_read  <= 1'b1;
              req.req.addr     <= cur_sar;
              req.req.len_bytes<= seg_len;
              req.req.tag      <= 10'h00; // Channel 0
              req.req.first_be <= 4'hF;
              req.req.last_be  <= 4'hF;
              req.wlast        <= 1'b0;
              w_beats_rem      <= burst_len;
              if (req.valid && req_ready) begin
                req.valid <= 1'b0;
                // Issue AXI AW transaction
                mm_m2s.awaddr  <= cur_dar;
                mm_m2s.awlen   <= burst_len;
                mm_m2s.awsize  <= 3'b110; // 64 bytes
                mm_m2s.awburst <= 2'b01;  // INCR
                mm_m2s.awvalid <= 1'b1;
                state          <= H_CPL;
              end
            end

            H_CPL: begin
              if (mm_m2s.awvalid && mm_s2m.awready) begin
                mm_m2s.awvalid <= 1'b0;
              end
              if (cpl.valid && cpl_ready) begin
                mm_m2s.wdata  <= cpl.rdata;
                mm_m2s.wstrb  <= '1;
                mm_m2s.wvalid <= 1'b1;
                mm_m2s.wlast  <= (w_beats_rem == 0) || cpl.meta.last;
                if (w_beats_rem != 0) w_beats_rem <= w_beats_rem - 1'b1;
                if ((w_beats_rem == 0) || cpl.meta.last) begin
                  mm_m2s.bready <= 1'b1;
                  state <= H_BRESP;
                end
              end else if (mm_m2s.wvalid && mm_s2m.wready) begin
                mm_m2s.wvalid <= 1'b0;
              end
            end

            H_BRESP: begin
              if (mm_m2s.wvalid && mm_s2m.wready) mm_m2s.wvalid <= 1'b0;
              if (mm_m2s.bready && mm_s2m.bvalid) begin
                mm_m2s.bready   <= 1'b0;
                cur_sar         <= cur_sar + 64'(seg_len);
                cur_dar         <= cur_dar + 64'(seg_len);
                bytes_xferred_r <= bytes_xferred_r + 64'(seg_len);
                rem_bytes       <= rem_bytes - 32'(seg_len);
                if (rem_bytes <= 32'(seg_len)) begin
                  state <= H_DONE;
                end else begin
                  state <= H_REQ;
                end
              end
            end

            H_DONE: begin
              done_cnt   <= 4'd10;
              ch_state_r <= ST_IDLE;
              state      <= H_IDLE;
            end

            default: state <= H_IDLE;
          endcase
        end
      end
    end else begin : g_c2h
      // ------------------------------------------------------------------------
      // C2H (Write Channel): Device Local Memory -> Host
      // ------------------------------------------------------------------------
      typedef enum logic [2:0] {
        C_IDLE, C_AR, C_STG, C_MWR, C_NEXT, C_DONE
      } c_state_e;
      c_state_e state;
      logic [7:0] r_beats_rem;
      logic [DATA_WIDTH-1:0] fifo_data [8];
      logic [2:0]            fifo_wr_ptr, fifo_rd_ptr;
      logic [3:0]            done_cnt;

      assign cpl_ready = 1'b0;

      always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
          state           <= C_IDLE;
          cur_sar         <= '0;
          cur_dar         <= '0;
          rem_bytes       <= '0;
          bytes_xferred_r <= '0;
          done_irq_r      <= 1'b0;
          done_cnt        <= '0;
          ch_state_r      <= ST_IDLE;
          r_beats_rem     <= '0;
          fifo_wr_ptr     <= '0;
          fifo_rd_ptr     <= '0;
          req.valid       <= 1'b0;
          req.req         <= '0;
          req.wdata       <= '0;
          req.wlast       <= 1'b0;
          mm_m2s          <= '0;
        end else begin
          if (done_cnt != 0) begin
            done_cnt   <= done_cnt - 1'b1;
            done_irq_r <= 1'b1;
          end else begin
            done_irq_r <= 1'b0;
          end
          case (state)
            C_IDLE: begin
              ch_state_r <= ST_IDLE;
              if (ctrl.enable && ctrl.doorbell && ctrl.xfer_size != 0) begin
                cur_sar         <= ctrl.sar;
                cur_dar         <= ctrl.dar;
                rem_bytes       <= ctrl.xfer_size;
                bytes_xferred_r <= '0;
                ch_state_r      <= ST_RUN;
                state           <= C_AR;
              end
            end

            C_AR: begin
              mm_m2s.araddr  <= cur_sar;
              mm_m2s.arlen   <= burst_len;
              mm_m2s.arsize  <= 3'b110;
              mm_m2s.arburst <= 2'b01;
              mm_m2s.arvalid <= 1'b1;
              r_beats_rem    <= burst_len;
              fifo_wr_ptr    <= '0;
              fifo_rd_ptr    <= '0;
              if (mm_m2s.arvalid && mm_s2m.arready) begin
                mm_m2s.arvalid <= 1'b0;
                mm_m2s.rready  <= 1'b1;
                state          <= C_STG;
              end
            end

            C_STG: begin
              if (mm_m2s.rready && mm_s2m.rvalid) begin
                fifo_data[fifo_wr_ptr] <= mm_s2m.rdata;
                fifo_wr_ptr            <= fifo_wr_ptr + 1'b1;
                if (r_beats_rem != 0) begin
                  r_beats_rem <= r_beats_rem - 1'b1;
                end else begin
                  mm_m2s.rready    <= 1'b0;
                  state            <= C_MWR;
                  req.valid        <= 1'b1;
                  req.req.is_read  <= 1'b0;
                  req.req.addr     <= cur_dar;
                  req.req.len_bytes<= seg_len;
                  req.req.tag      <= 10'h01; // Channel 1
                  req.req.first_be <= 4'hF;
                  req.req.last_be  <= 4'hF;
                  req.wdata        <= (fifo_wr_ptr == 0) ? mm_s2m.rdata : fifo_data[0];
                  req.wlast        <= (burst_len == 0);
                  fifo_rd_ptr      <= 3'd1;
                  r_beats_rem      <= burst_len;
                end
              end
            end

            C_MWR: begin
              if (req.valid && req_ready) begin
                if (r_beats_rem == 0) begin
                  req.valid <= 1'b0;
                  req.wlast <= 1'b0;
                  state     <= C_NEXT;
                end else begin
                  req.valid        <= 1'b1;
                  req.req.addr     <= req.req.addr + 64;
                  req.wdata        <= fifo_data[fifo_rd_ptr];
                  req.wlast        <= (r_beats_rem == 8'd1);
                  fifo_rd_ptr      <= fifo_rd_ptr + 1'b1;
                  r_beats_rem      <= r_beats_rem - 1'b1;
                end
              end
            end

            C_NEXT: begin
              cur_sar         <= cur_sar + 64'(seg_len);
              cur_dar         <= cur_dar + 64'(seg_len);
              bytes_xferred_r <= bytes_xferred_r + 64'(seg_len);
              rem_bytes       <= rem_bytes - 32'(seg_len);
              if (rem_bytes <= 32'(seg_len)) begin
                state <= C_DONE;
              end else begin
                state <= C_AR;
              end
            end

            C_DONE: begin
              done_cnt   <= 4'd10;
              ch_state_r <= ST_IDLE;
              state      <= C_IDLE;
            end

            default: state <= C_IDLE;
          endcase
        end
      end
    end
  endgenerate

  // Output status assignments
  assign stat.state         = ch_state_r;
  assign stat.err           = ERR_NONE;
  assign stat.bytes_xferred = bytes_xferred_r;
  assign stat.done_irq      = done_irq_r;
  assign stat.abort_irq     = 1'b0;
  assign stat.watermark_irq = 1'b0;

endmodule
