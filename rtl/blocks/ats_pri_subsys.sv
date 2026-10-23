// ============================================================================
// ats_pri_subsys.sv — ATS/PASID/PRI subsystem on the merged requester path.
// Translates request addresses (ATC), issues ATS translation requests, and runs
// the PRI page-fault suspend/resume FSM. See docs/m1_ats_pri_microspec.md.
// Milestone M4: 64-entry fully associative ATC, invalidation engine, ATS agent.
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
  input  logic                   pri_en,
  // CSR ATC invalidation & status interface
  input  logic [31:0]            atc_inval_reg,
  input  logic                   atc_inval_pulse,
  output dma_pkg::atc_status_t   atc_stat
);
  import dma_pkg::*;

  // ==========================================================================
  // ATC Storage and Counters (64 entries, fully associative)
  // ==========================================================================
  atc_entry_t atc_mem [ATC_ENTRIES];
  logic [$clog2(ATC_ENTRIES)-1:0] replace_ptr;
  logic [15:0] hit_cnt_r, miss_cnt_r;

  assign atc_stat.hit_count  = hit_cnt_r;
  assign atc_stat.miss_count = miss_cnt_r;

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
                    (!atc_mem[i].pasid_en || (atc_mem[i].pasid == up_req.req.pasid));
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

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      for (int i = 0; i < ATC_ENTRIES; i++) atc_mem[i] <= '0;
      replace_ptr <= '0;
      hit_cnt_r   <= '0;
      miss_cnt_r  <= '0;
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
    end
  end

  // ---- Datapath passthrough (Translation Agent FSM inserted in Task 4) ----
  assign dn_req       = up_req;
  assign up_req_ready = dn_req_ready;
  assign up_cpl       = dn_cpl;
  assign dn_cpl_ready = up_cpl_ready;
  assign ats_up       = '0;

endmodule
