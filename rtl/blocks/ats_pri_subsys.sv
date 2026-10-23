// ============================================================================
// ats_pri_subsys.sv — ATS/PASID/PRI subsystem on the merged requester path.
// Translates request addresses (ATC), issues ATS translation requests, and runs
// the PRI page-fault suspend/resume FSM. See docs/m1_ats_pri_microspec.md.
// Milestone M4: 64-entry fully associative ATC, invalidation engine, ATS agent.
// ============================================================================
module ats_pri_subsys #(
  parameter int unsigned NUM_CH = 8
) (
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
  input  logic                   pri_en,
  // CSR ATC invalidation & status interface
  input  logic [31:0]            atc_inval_reg,
  input  logic                   atc_inval_pulse,
  output dma_pkg::atc_status_t   atc_stat,
  // Channel fault reporting (M4)
  output logic [NUM_CH-1:0]      fault_abort_o,
  output dma_pkg::err_code_e     fault_err_o [NUM_CH]
);
  import dma_pkg::*;

  // ==========================================================================
  // Translation Agent FSM State & Skid Buffer
  // ==========================================================================
  typedef enum logic [2:0] {
    T_PASSTHROUGH = 3'd0,
    T_IDLE        = 3'd1,
    T_REQ         = 3'd2,
    T_WAIT        = 3'd3,
    T_EMIT        = 3'd4,
    T_FAULT       = 3'd5
  } xlate_state_e;

  xlate_state_e state;
  pcie_req_bus_t skid_req;
  logic [7:0]    cur_req_id;
  logic [2:0]    fault_ch;
  logic          drain_fault;
  logic          in_write_burst;

  // ==========================================================================
  // ATC Storage and Counters (64 entries, fully associative)
  // ==========================================================================
  atc_entry_t atc_mem [ATC_ENTRIES];
  logic [$clog2(ATC_ENTRIES)-1:0] replace_ptr;
  logic [15:0] hit_cnt_r, miss_cnt_r;

  assign atc_stat.hit_count  = hit_cnt_r;
  assign atc_stat.miss_count = miss_cnt_r;

  // Replacement / allocation slot selector
  logic [$clog2(ATC_ENTRIES)-1:0] alloc_slot;
  logic has_invalid;
  always_comb begin
    alloc_slot = replace_ptr;
    has_invalid = 1'b0;
    for (int i = ATC_ENTRIES-1; i >= 0; i--) begin
      if (!atc_mem[i].valid) begin
        alloc_slot = 6'(i);
        has_invalid = 1'b1;
      end
    end
  end

  // ==========================================================================
  // Combinatorial Tag Lookup & Permission Checking
  // ==========================================================================
  wire [51:0] req_vpn = up_req.req.addr[63:12];
  logic [ATC_ENTRIES-1:0] hit_mask;
  atc_entry_t match_entry;
  logic atc_hit;

  always_comb begin
    atc_hit = 1'b0;
    match_entry = '0;
    hit_mask = '0;
    for (int i = 0; i < ATC_ENTRIES; i++) begin
      hit_mask[i] = atc_mem[i].valid &&
                    (atc_mem[i].vpn == req_vpn) &&
                    (atc_mem[i].pasid_en == up_req.req.pasid_en) &&
                    (!up_req.req.pasid_en || (atc_mem[i].pasid == up_req.req.pasid));
      if (hit_mask[i]) begin
        atc_hit = 1'b1;
        match_entry = atc_mem[i];
      end
    end
  end

  wire perm_ok = up_req.req.is_read ? match_entry.r_perm : match_entry.w_perm;

  // ==========================================================================
  // Invalidation Engine (Single-Cycle Flush)
  // Sources: Software CSR writes (atc_inval_pulse) or Host IOMMU (ats_dn.inval_valid)
  // ==========================================================================
  wire do_inval_csr  = atc_inval_pulse;
  wire do_inval_host = ats_dn.inval_valid;
  wire do_inval      = do_inval_csr || do_inval_host;

  wire        inval_global    = do_inval_csr ? atc_inval_reg[1] : ats_dn.inval_global;
  wire        inval_pasid_all = do_inval_csr ? atc_inval_reg[2] : 1'b0;
  wire        inval_pasid_en  = do_inval_csr ? (atc_inval_reg[2] || (atc_inval_reg[19:4] != '0)) : ats_dn.inval_pasid_en;
  wire [19:0] inval_pasid     = do_inval_csr ? {4'b0, atc_inval_reg[19:4]} : ats_dn.inval_pasid;
  wire [51:0] inval_vpn       = do_inval_csr ? {40'b0, atc_inval_reg[31:20]} : ats_dn.inval_addr[63:12];

  // Invalidation memory clear
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      for (int i = 0; i < ATC_ENTRIES; i++) atc_mem[i] <= '0;
      replace_ptr <= '0;
    end else if (do_inval) begin
      for (int i = 0; i < ATC_ENTRIES; i++) begin
        if (inval_global) begin
          atc_mem[i].valid <= 1'b0;
        end else if (atc_mem[i].valid) begin
          if (inval_pasid_en && (atc_mem[i].pasid == inval_pasid)) begin
            if (inval_pasid_all || (atc_mem[i].vpn == inval_vpn)) begin
              atc_mem[i].valid <= 1'b0;
            end
          end else if (!inval_pasid_en && (atc_mem[i].vpn == inval_vpn)) begin
            atc_mem[i].valid <= 1'b0;
          end
        end
      end
    end else if (state == T_WAIT && ats_dn.ats_rsp_valid && (ats_dn.ats_rsp_id == cur_req_id)) begin
      if (!ats_dn.ats_rsp.fail && !ats_dn.ats_rsp.non_present &&
          (skid_req.req.is_read ? ats_dn.ats_rsp.r : ats_dn.ats_rsp.w)) begin
        atc_mem[alloc_slot] <= '{
          valid:    1'b1,
          pasid_en: skid_req.req.pasid_en,
          pasid:    skid_req.req.pasid,
          vpn:      skid_req.req.addr[63:12],
          ppn:      ats_dn.ats_rsp.trans_addr[63:12],
          r_perm:   ats_dn.ats_rsp.r,
          w_perm:   ats_dn.ats_rsp.w,
          priv:     ats_dn.ats_rsp.priv
        };
        if (!has_invalid) replace_ptr <= replace_ptr + 1'b1;
      end
    end
  end

  // ==========================================================================
  // Translation Agent FSM
  // ==========================================================================
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      state          <= T_IDLE;
      skid_req       <= '0;
      cur_req_id     <= '0;
      fault_ch       <= '0;
      drain_fault    <= 1'b0;
      in_write_burst <= 1'b0;
      hit_cnt_r      <= '0;
      miss_cnt_r     <= '0;
    end else begin
      case (state)
        T_IDLE: begin
          if (!ats_en) begin
            state <= T_IDLE;
          end else if (up_req.valid) begin
            if (atc_hit && perm_ok) begin
              // Hit: translate in-flight
              if (dn_req_ready) begin
                if (!in_write_burst) hit_cnt_r <= hit_cnt_r + 1'b1;
                if (!up_req.req.is_read) in_write_burst <= !up_req.wlast;
              end
            end else if (atc_hit && !perm_ok) begin
              // Permission fault
              miss_cnt_r  <= miss_cnt_r + 1'b1;
              fault_ch    <= up_req.req.tag[9:7];
              drain_fault <= !up_req.req.is_read && !up_req.wlast;
              state       <= T_FAULT;
            end else begin
              // ATC miss: capture into skid buffer, issue ATS request
              miss_cnt_r <= miss_cnt_r + 1'b1;
              skid_req   <= up_req;
              state      <= T_REQ;
            end
          end
        end

        T_REQ: begin
          if (ats_dn.ats_req_ready) begin
            state <= T_WAIT;
          end
        end

        T_WAIT: begin
          if (ats_dn.ats_rsp_valid && (ats_dn.ats_rsp_id == cur_req_id)) begin
            cur_req_id <= cur_req_id + 1'b1;
            if (ats_dn.ats_rsp.fail || ats_dn.ats_rsp.non_present ||
                (skid_req.req.is_read ? !ats_dn.ats_rsp.r : !ats_dn.ats_rsp.w)) begin
              fault_ch    <= skid_req.req.tag[9:7];
              drain_fault <= !skid_req.req.is_read && !skid_req.wlast;
              state       <= T_FAULT;
            end else begin
              skid_req.req.addr <= {ats_dn.ats_rsp.trans_addr[63:12], skid_req.req.addr[11:0]};
              skid_req.req.at   <= AT_TRANSLATED;
              state             <= T_EMIT;
            end
          end
        end

        T_EMIT: begin
          if (dn_req_ready) begin
            if (!skid_req.req.is_read) in_write_burst <= !skid_req.wlast;
            state <= T_IDLE;
          end
        end

        T_FAULT: begin
          if (drain_fault) begin
            if (up_req.valid && up_req.wlast) begin
              drain_fault <= 1'b0;
              state       <= T_IDLE;
            end
          end else begin
            state <= T_IDLE;
          end
        end

        default: state <= T_IDLE;
      endcase
    end
  end

  // ==========================================================================
  // Combinatorial Datapath & Ready Generation
  // ==========================================================================
  always_comb begin
    up_req_ready = 1'b0;
    dn_req       = '0;

    if (!ats_en) begin
      // Transparent passthrough when ATS is disabled
      dn_req       = up_req;
      up_req_ready = dn_req_ready;
    end else begin
      case (state)
        T_IDLE: begin
          if (up_req.valid && atc_hit && perm_ok) begin
            dn_req              = up_req;
            dn_req.req.addr     = {match_entry.ppn, up_req.req.addr[11:0]};
            dn_req.req.at       = AT_TRANSLATED;
            up_req_ready        = dn_req_ready;
          end else if (up_req.valid && atc_hit && !perm_ok) begin
            up_req_ready        = 1'b1; // sink faulted beat into FSM
          end else if (up_req.valid && !atc_hit) begin
            up_req_ready        = 1'b1; // sink miss beat into skid buffer
          end else begin
            up_req_ready        = 1'b1;
          end
        end

        T_REQ, T_WAIT: begin
          up_req_ready = 1'b0;
        end

        T_EMIT: begin
          dn_req       = skid_req;
          up_req_ready = 1'b0;
        end

        T_FAULT: begin
          up_req_ready = drain_fault;
        end

        default: begin
          up_req_ready = 1'b0;
        end
      endcase
    end
  end

  // ATS / PRI upstream messages to PAA
  always_comb begin
    ats_up                  = '0;
    ats_up.ats_rsp_ready    = 1'b1;
    ats_up.pri_rsp_ready    = 1'b1;

    if (state == T_REQ) begin
      ats_up.ats_req_valid    = 1'b1;
      ats_up.ats_req_addr     = {skid_req.req.addr[63:12], 12'b0};
      ats_up.ats_req_pasid    = skid_req.req.pasid;
      ats_up.ats_req_pasid_en = skid_req.req.pasid_en;
      ats_up.ats_req_is_write = !skid_req.req.is_read;
      ats_up.ats_req_id       = cur_req_id;
    end
  end

  // Fault abort strobes and error codes
  always_comb begin
    for (int i = 0; i < NUM_CH; i++) begin
      fault_abort_o[i] = (state == T_FAULT) && (fault_ch == 3'(i));
      fault_err_o[i]   = ((state == T_FAULT) && (fault_ch == 3'(i))) ? ERR_ATS_FAIL : ERR_NONE;
    end
  end

  // Completions pass through transparently
  assign up_cpl       = dn_cpl;
  assign dn_cpl_ready = up_cpl_ready;

endmodule
