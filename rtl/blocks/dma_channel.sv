// ============================================================================
// dma_channel.sv — DMA Channel Data Mover (Register & Linked-List Mode).
// DATA_WIDTH = 512, with Out-of-Order Completion Reorder Buffer on H2C.
// IS_WRITE = 0: Read Channel (H2C) Host -> Device
// IS_WRITE = 1: Write Channel (C2H) Device -> Host
// ============================================================================
module dma_channel #(
  parameter int unsigned CH_ID    = 0,
  parameter bit          IS_WRITE = 1'b0
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
      // Out-of-Order Completion Reorder Buffer, Commit Engine, & Linked-List FSM
      // ------------------------------------------------------------------------
      typedef struct packed {
        tag_state_e  state;
        logic [12:0] expected_bytes;
        logic [12:0] bytes_received;
        logic [6:0]  ram_slot_offset; // beat offset in reorder RAM
        logic [63:0] dar;
      } tag_entry_t;

      tag_entry_t scoreboard [TAGS_PER_CH];
      logic [DATA_WIDTH-1:0] reorder_ram [REORDER_BEATS];

      logic [$clog2(TAGS_PER_CH)-1:0] alloc_tag_ptr;
      logic [$clog2(TAGS_PER_CH)-1:0] commit_tag_ptr;
      logic [31:0] req_rem_bytes;
      logic [31:0] total_xfer_bytes;
      logic [3:0]  done_cnt;

      typedef enum logic [1:0] {
        C_IDLE, C_AW, C_W, C_BRESP
      } commit_state_e;
      commit_state_e c_state;
      logic [7:0] commit_beat_cnt;
      logic [7:0] commit_burst_len;

      typedef enum logic [1:0] {
        H_IDLE, H_FETCH_REQ, H_FETCH_WAIT, H_RUN
      } h_fsm_e;
      h_fsm_e h_fsm;

      logic [63:0] llp_ptr;
      logic [63:0] last_llp_base;
      logic        expected_cb;
      hdma_elem_t  cur_elem;
      logic        in_ll_mode;
      logic        elem_lie;

      assign cpl_ready = 1'b1; // Reorder buffer always sinks completions for allocated tags

      // 4KB segmentation for H2C request pipeline
      wire [12:0] h2c_bytes_to_4k = 13'h1000 - {1'b0, cur_sar[11:0]};
      wire [12:0] h2c_seg_calc = (req_rem_bytes < {19'b0, h2c_bytes_to_4k}) ? req_rem_bytes[12:0] : h2c_bytes_to_4k;
      wire [12:0] h2c_seg_len = (h2c_seg_calc > 13'd512) ? 13'd512 : h2c_seg_calc;

      // Completion write port into reorder RAM
      wire is_desc_cpl = (cpl.meta.tag == {CH_ID[2:0], 7'h7F});
      wire [$clog2(TAGS_PER_CH)-1:0] cpl_local_tag = cpl.meta.tag[$clog2(TAGS_PER_CH)-1:0];

      always_ff @(posedge core_clk) begin
        if (cpl.valid && cpl_ready && !is_desc_cpl) begin
          int unsigned wr_idx;
          wr_idx = int'(scoreboard[cpl_local_tag].ram_slot_offset) +
                   (int'(scoreboard[cpl_local_tag].bytes_received) >> 6);
          reorder_ram[7'(wr_idx)] <= cpl.rdata;
        end
      end

      // Requester, Descriptor Fetch & Scoreboard FSM
      always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
          cur_sar          <= '0;
          cur_dar          <= '0;
          rem_bytes        <= '0;
          req_rem_bytes    <= '0;
          total_xfer_bytes <= '0;
          bytes_xferred_r  <= '0;
          done_irq_r       <= 1'b0;
          done_cnt         <= '0;
          ch_state_r       <= ST_IDLE;
          alloc_tag_ptr    <= '0;
          commit_tag_ptr   <= '0;
          c_state          <= C_IDLE;
          commit_beat_cnt  <= '0;
          commit_burst_len <= '0;
          h_fsm            <= H_IDLE;
          llp_ptr          <= '0;
          last_llp_base    <= '0;
          expected_cb      <= 1'b1;
          cur_elem         <= '0;
          in_ll_mode       <= 1'b0;
          elem_lie         <= 1'b0;
          req.valid        <= 1'b0;
          req.req          <= '0;
          req.wdata        <= '0;
          req.wlast        <= 1'b0;
          mm_m2s           <= '0;
          for (int i = 0; i < TAGS_PER_CH; i++) begin
            scoreboard[i] <= '0;
          end
        end else if (!ctrl.enable) begin
          ch_state_r    <= ST_IDLE;
          h_fsm         <= H_IDLE;
          in_ll_mode    <= 1'b0;
          expected_cb   <= 1'b1;
          last_llp_base <= '0;
          done_irq_r    <= 1'b0;
          done_cnt      <= '0;
          req.valid     <= 1'b0;
          c_state       <= C_IDLE;
        end else begin
          // IRQ strobe
          if (done_cnt != 0) begin
            done_cnt   <= done_cnt - 1'b1;
            done_irq_r <= 1'b1;
          end else begin
            done_irq_r <= 1'b0;
          end

          // Completion status update in scoreboard (data completions)
          if (cpl.valid && cpl_ready && !is_desc_cpl) begin
            if (scoreboard[cpl_local_tag].state == TAG_WAIT_CPL) begin
              scoreboard[cpl_local_tag].bytes_received <= scoreboard[cpl_local_tag].bytes_received + 13'd64;
              if ((scoreboard[cpl_local_tag].bytes_received + 13'd64 >= scoreboard[cpl_local_tag].expected_bytes) || cpl.meta.last) begin
                scoreboard[cpl_local_tag].state <= TAG_COMMITTED;
              end
            end
          end

          // Top-level channel state machine
          case (h_fsm)
            H_IDLE: begin
              ch_state_r <= ST_IDLE;
              if (ctrl.enable && ctrl.doorbell) begin
                if (ctrl.lle) begin
                  // Linked-list mode
                  if (!in_ll_mode || (ctrl.llp_base != last_llp_base)) begin
                    llp_ptr       <= ctrl.llp_base;
                    last_llp_base <= ctrl.llp_base;
                    expected_cb   <= 1'b1;
                  end
                  in_ll_mode  <= 1'b1;
                  ch_state_r  <= ST_FETCH;
                  h_fsm       <= H_FETCH_REQ;
                end else if (ctrl.xfer_size != 0) begin
                  // Register mode
                  in_ll_mode       <= 1'b0;
                  cur_sar          <= ctrl.sar;
                  cur_dar          <= ctrl.dar;
                  req_rem_bytes    <= ctrl.xfer_size;
                  total_xfer_bytes <= ctrl.xfer_size;
                  rem_bytes        <= ctrl.xfer_size;
                  bytes_xferred_r  <= '0;
                  alloc_tag_ptr    <= '0;
                  commit_tag_ptr   <= '0;
                  elem_lie         <= ctrl.lie;
                  ch_state_r       <= ST_RUN;
                  h_fsm            <= H_RUN;
                  for (int i = 0; i < TAGS_PER_CH; i++) begin
                    scoreboard[i].state <= TAG_FREE;
                  end
                end
              end
            end

            H_FETCH_REQ: begin
              ch_state_r        <= ST_FETCH;
              req.valid         <= 1'b1;
              req.req.is_read   <= 1'b1;
              req.req.addr      <= llp_ptr;
              req.req.len_bytes <= 13'd64;
              req.req.tag       <= {CH_ID[2:0], 7'h7F};
              req.req.first_be  <= 4'hF;
              req.req.last_be   <= 4'hF;
              req.req.pasid     <= ctrl.pasid;
              req.req.pasid_en  <= ctrl.pasid_en;
              req.req.tc        <= ctrl.tc;
              req.req.relaxed   <= ctrl.ro;
              req.req.at        <= AT_UNTRANSLATED;
              req.wlast         <= 1'b0;
              if (req.valid && req_ready) begin
                req.valid <= 1'b0;
                h_fsm     <= H_FETCH_WAIT;
              end
            end

            H_FETCH_WAIT: begin
              ch_state_r <= ST_FETCH;
              if (cpl.valid && is_desc_cpl) begin
                // Unpack 24-byte descriptor
                cur_elem.control <= cpl.rdata[31:0];
                cur_elem.size    <= cpl.rdata[63:32];
                cur_elem.sar_lo  <= cpl.rdata[95:64];
                cur_elem.sar_hi  <= cpl.rdata[127:96];
                cur_elem.dar_lo  <= cpl.rdata[159:128];
                cur_elem.dar_hi  <= cpl.rdata[191:160];

                // Check ownership and element type
                if (cpl.rdata[CTRL_CB] != expected_cb) begin
                  // Ownership belongs to software — end of list
                  ch_state_r <= ST_IDLE;
                  h_fsm      <= H_IDLE;
                end else if (cpl.rdata[CTRL_LLP] == 1'b1) begin
                  // Link element: jump to next list pointer
                  llp_ptr <= {cpl.rdata[127:96], cpl.rdata[95:64]};
                  if (cpl.rdata[CTRL_TCB] == 1'b1) begin
                    expected_cb <= ~expected_cb;
                  end
                  h_fsm <= H_FETCH_REQ;
                end else begin
                  // Data element
                  cur_sar          <= {cpl.rdata[127:96], cpl.rdata[95:64]};
                  cur_dar          <= {cpl.rdata[191:160], cpl.rdata[159:128]};
                  req_rem_bytes    <= cpl.rdata[63:32];
                  total_xfer_bytes <= cpl.rdata[63:32];
                  rem_bytes        <= cpl.rdata[63:32];
                  elem_lie         <= cpl.rdata[CTRL_LIE];
                  alloc_tag_ptr    <= '0;
                  commit_tag_ptr   <= '0;
                  ch_state_r       <= ST_RUN;
                  h_fsm            <= H_RUN;
                  for (int i = 0; i < TAGS_PER_CH; i++) begin
                    scoreboard[i].state <= TAG_FREE;
                  end
                end
              end
            end

            H_RUN: begin
              ch_state_r <= ST_RUN;
              // Issue Memory Reads (MRd) for data payload
              if (req_rem_bytes != 0 && scoreboard[alloc_tag_ptr].state == TAG_FREE && !req.valid) begin
                req.valid         <= 1'b1;
                req.req.is_read   <= 1'b1;
                req.req.addr      <= cur_sar;
                req.req.len_bytes <= h2c_seg_len;
                req.req.tag       <= {CH_ID[2:0], 3'b0, alloc_tag_ptr};
                req.req.first_be  <= 4'hF;
                req.req.last_be   <= 4'hF;
                req.req.pasid     <= ctrl.pasid;
                req.req.pasid_en  <= ctrl.pasid_en;
                req.req.tc        <= ctrl.tc;
                req.req.relaxed   <= ctrl.ro;
                req.req.at        <= AT_UNTRANSLATED;
                req.wlast         <= 1'b0;
              end

              if (req.valid && req_ready) begin
                req.valid <= 1'b0;
                scoreboard[alloc_tag_ptr].state           <= TAG_WAIT_CPL;
                scoreboard[alloc_tag_ptr].expected_bytes  <= req.req.len_bytes;
                scoreboard[alloc_tag_ptr].bytes_received  <= '0;
                scoreboard[alloc_tag_ptr].ram_slot_offset <= 7'(alloc_tag_ptr * MAX_BURST_BEATS);
                scoreboard[alloc_tag_ptr].dar             <= cur_dar;

                cur_sar       <= cur_sar + 64'(req.req.len_bytes);
                cur_dar       <= cur_dar + 64'(req.req.len_bytes);
                req_rem_bytes <= req_rem_bytes - 32'(req.req.len_bytes);
                alloc_tag_ptr <= alloc_tag_ptr + 1'b1;
              end
            end

            default: h_fsm <= H_IDLE;
          endcase

          // In-Order Commit Engine (driving AXI-MM Master)
          case (c_state)
            C_IDLE: begin
              if (scoreboard[commit_tag_ptr].state == TAG_COMMITTED) begin
                commit_burst_len <= 8'((scoreboard[commit_tag_ptr].expected_bytes + 13'd63) >> 6) - 8'd1;
                mm_m2s.awaddr    <= scoreboard[commit_tag_ptr].dar;
                mm_m2s.awlen     <= 8'((scoreboard[commit_tag_ptr].expected_bytes + 13'd63) >> 6) - 8'd1;
                mm_m2s.awsize    <= 3'b110; // 64 bytes
                mm_m2s.awburst   <= 2'b01;  // INCR
                mm_m2s.awvalid   <= 1'b1;
                commit_beat_cnt  <= '0;
                c_state          <= C_AW;
              end
            end

            C_AW: begin
              if (mm_m2s.awvalid && mm_s2m.awready) begin
                mm_m2s.awvalid <= 1'b0;
                // Output first data beat
                mm_m2s.wdata   <= reorder_ram[scoreboard[commit_tag_ptr].ram_slot_offset];
                mm_m2s.wstrb   <= '1;
                mm_m2s.wvalid  <= 1'b1;
                mm_m2s.wlast   <= (commit_burst_len == 0);
                c_state        <= C_W;
              end
            end

            C_W: begin
              if (mm_m2s.wvalid && mm_s2m.wready) begin
                if (commit_beat_cnt == commit_burst_len) begin
                  mm_m2s.wvalid <= 1'b0;
                  mm_m2s.wlast  <= 1'b0;
                  mm_m2s.bready <= 1'b1;
                  c_state       <= C_BRESP;
                end else begin
                  commit_beat_cnt <= commit_beat_cnt + 1'b1;
                  mm_m2s.wdata    <= reorder_ram[7'(scoreboard[commit_tag_ptr].ram_slot_offset + 7'(commit_beat_cnt) + 7'd1)];
                  mm_m2s.wvalid   <= 1'b1;
                  mm_m2s.wlast    <= (commit_beat_cnt + 1'b1 == commit_burst_len);
                end
              end
            end

            C_BRESP: begin
              if (mm_m2s.wvalid && mm_s2m.wready) mm_m2s.wvalid <= 1'b0;
              if (mm_m2s.bready && mm_s2m.bvalid) begin
                mm_m2s.bready   <= 1'b0;
                bytes_xferred_r <= bytes_xferred_r + 64'(scoreboard[commit_tag_ptr].expected_bytes);
                rem_bytes       <= (rem_bytes > 32'(scoreboard[commit_tag_ptr].expected_bytes)) ?
                                   rem_bytes - 32'(scoreboard[commit_tag_ptr].expected_bytes) : 32'd0;
                scoreboard[commit_tag_ptr].state <= TAG_FREE;
                commit_tag_ptr <= commit_tag_ptr + 1'b1;

                if (rem_bytes <= 32'(scoreboard[commit_tag_ptr].expected_bytes)) begin
                  if (elem_lie) begin
                    done_cnt   <= 4'd10;
                    done_irq_r <= 1'b1;
                  end
                  if (in_ll_mode) begin
                    // Advance to next descriptor in memory
                    llp_ptr    <= llp_ptr + 64'd24;
                    ch_state_r <= ST_FETCH;
                    h_fsm      <= H_FETCH_REQ;
                    c_state    <= C_IDLE;
                  end else begin
                    done_cnt   <= 4'd10;
                    ch_state_r <= ST_IDLE;
                    h_fsm      <= H_IDLE;
                    c_state    <= C_IDLE;
                  end
                end else begin
                  c_state    <= C_IDLE;
                end
              end
            end

            default: c_state <= C_IDLE;
          endcase
        end
      end
    end else begin : g_c2h
      // ------------------------------------------------------------------------
      // C2H (Write Channel): Device Local Memory -> Host
      // Linked-List Engine & Data Mover
      // ------------------------------------------------------------------------
      typedef enum logic [2:0] {
        C_IDLE, C_FETCH_REQ, C_FETCH_WAIT, C_AR, C_STG, C_MWR, C_NEXT
      } c_state_e;
      c_state_e state;
      logic [7:0] r_beats_rem;
      logic [DATA_WIDTH-1:0] fifo_data [8];
      logic [2:0]            fifo_wr_ptr, fifo_rd_ptr;
      logic [3:0]            done_cnt;

      logic [63:0] llp_ptr;
      logic [63:0] last_llp_base;
      logic        expected_cb;
      hdma_elem_t  cur_elem;
      logic        in_ll_mode;
      logic        elem_lie;

      assign cpl_ready = (state == C_FETCH_WAIT);
      wire is_c2h_desc_cpl = (cpl.meta.tag == {CH_ID[2:0], 7'h7F});

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
          llp_ptr         <= '0;
          last_llp_base   <= '0;
          expected_cb     <= 1'b1;
          cur_elem        <= '0;
          in_ll_mode      <= 1'b0;
          elem_lie        <= 1'b0;
          req.valid       <= 1'b0;
          req.req         <= '0;
          req.wdata       <= '0;
          req.wlast       <= 1'b0;
          mm_m2s          <= '0;
        end else if (!ctrl.enable) begin
          ch_state_r    <= ST_IDLE;
          state         <= C_IDLE;
          in_ll_mode    <= 1'b0;
          expected_cb   <= 1'b1;
          last_llp_base <= '0;
          done_irq_r    <= 1'b0;
          done_cnt      <= '0;
          req.valid     <= 1'b0;
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
              if (ctrl.enable && ctrl.doorbell) begin
                if (ctrl.lle) begin
                  if (!in_ll_mode || (ctrl.llp_base != last_llp_base)) begin
                    llp_ptr       <= ctrl.llp_base;
                    last_llp_base <= ctrl.llp_base;
                    expected_cb   <= 1'b1;
                  end
                  in_ll_mode  <= 1'b1;
                  ch_state_r  <= ST_FETCH;
                  state       <= C_FETCH_REQ;
                end else if (ctrl.xfer_size != 0) begin
                  in_ll_mode      <= 1'b0;
                  cur_sar         <= ctrl.sar;
                  cur_dar         <= ctrl.dar;
                  rem_bytes       <= ctrl.xfer_size;
                  bytes_xferred_r <= '0;
                  elem_lie        <= ctrl.lie;
                  ch_state_r      <= ST_RUN;
                  state           <= C_AR;
                end
              end
            end

            C_FETCH_REQ: begin
              ch_state_r        <= ST_FETCH;
              req.valid         <= 1'b1;
              req.req.is_read   <= 1'b1;
              req.req.addr      <= llp_ptr;
              req.req.len_bytes <= 13'd64;
              req.req.tag       <= {CH_ID[2:0], 7'h7F};
              req.req.first_be  <= 4'hF;
              req.req.last_be   <= 4'hF;
              req.req.pasid     <= ctrl.pasid;
              req.req.pasid_en  <= ctrl.pasid_en;
              req.req.tc        <= ctrl.tc;
              req.req.relaxed   <= ctrl.ro;
              req.req.at        <= AT_UNTRANSLATED;
              req.wlast         <= 1'b0;
              if (req.valid && req_ready) begin
                req.valid <= 1'b0;
                state     <= C_FETCH_WAIT;
              end
            end

            C_FETCH_WAIT: begin
              ch_state_r <= ST_FETCH;
              if (cpl.valid && is_c2h_desc_cpl) begin
                cur_elem.control <= cpl.rdata[31:0];
                cur_elem.size    <= cpl.rdata[63:32];
                cur_elem.sar_lo  <= cpl.rdata[95:64];
                cur_elem.sar_hi  <= cpl.rdata[127:96];
                cur_elem.dar_lo  <= cpl.rdata[159:128];
                cur_elem.dar_hi  <= cpl.rdata[191:160];

                if (cpl.rdata[CTRL_CB] != expected_cb) begin
                  ch_state_r <= ST_IDLE;
                  state      <= C_IDLE;
                end else if (cpl.rdata[CTRL_LLP] == 1'b1) begin
                  llp_ptr <= {cpl.rdata[127:96], cpl.rdata[95:64]};
                  if (cpl.rdata[CTRL_TCB] == 1'b1) begin
                    expected_cb <= ~expected_cb;
                  end
                  state <= C_FETCH_REQ;
                end else begin
                  cur_sar    <= {cpl.rdata[127:96], cpl.rdata[95:64]};
                  cur_dar    <= {cpl.rdata[191:160], cpl.rdata[159:128]};
                  rem_bytes  <= cpl.rdata[63:32];
                  elem_lie   <= cpl.rdata[CTRL_LIE];
                  ch_state_r <= ST_RUN;
                  state      <= C_AR;
                end
              end
            end

            C_AR: begin
              ch_state_r     <= ST_RUN;
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
                  req.req.tag      <= {CH_ID[2:0], 7'b0};
                  req.req.first_be <= 4'hF;
                  req.req.last_be  <= 4'hF;
                  req.req.pasid    <= ctrl.pasid;
                  req.req.pasid_en <= ctrl.pasid_en;
                  req.req.tc       <= ctrl.tc;
                  req.req.relaxed  <= ctrl.ro;
                  req.req.at       <= AT_UNTRANSLATED;
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
                if (elem_lie) begin
                  done_cnt   <= 4'd10;
                  done_irq_r <= 1'b1;
                end
                if (in_ll_mode) begin
                  llp_ptr    <= llp_ptr + 64'd24;
                  ch_state_r <= ST_FETCH;
                  state      <= C_FETCH_REQ;
                end else begin
                  done_cnt   <= 4'd10;
                  ch_state_r <= ST_IDLE;
                  state      <= C_IDLE;
                end
              end else begin
                state <= C_AR;
              end
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
