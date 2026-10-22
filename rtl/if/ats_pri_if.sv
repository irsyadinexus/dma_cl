// ============================================================================
// ats_pri_if.sv — ATS/PASID translation + PRI page-request contract
// (ATS/PRI subsystem <-> PAA). M1 skeleton. See docs/m1_ats_pri_microspec.md.
// ============================================================================
interface ats_pri_if (input logic clk, input logic rst_n);

  import dma_pkg::*;

  // ---- ATS translation request (Xlate agent -> PAA -> host) ----
  logic                    ats_req_valid;
  logic                    ats_req_ready;
  logic [63:0]             ats_req_addr;   // untranslated IOVA
  logic [PASID_W-1:0]      ats_req_pasid;
  logic                    ats_req_pasid_en;
  logic                    ats_req_is_write;
  logic [7:0]              ats_req_id;      // outstanding-request id

  // ---- ATS translation response (PAA -> Xlate agent) ----
  logic                    ats_rsp_valid;
  logic                    ats_rsp_ready;
  logic [7:0]              ats_rsp_id;
  ats_rsp_t                ats_rsp;         // trans_addr/size/perm/non_present/fail

  // ---- PRI page request (PRI engine -> PAA -> host) ----
  logic                    pri_req_valid;
  logic                    pri_req_ready;
  logic [63:0]             pri_req_addr;    // page-aligned faulting address
  logic [PASID_W-1:0]      pri_req_pasid;
  logic                    pri_req_pasid_en;
  logic                    pri_req_rd;      // access was a read
  logic                    pri_req_wr;      // access was a write
  logic [8:0]              pri_req_prg_idx; // page-request group index

  // ---- PRI response / PRG response (PAA -> PRI engine) ----
  logic                    pri_rsp_valid;
  logic                    pri_rsp_ready;
  logic [8:0]              pri_rsp_prg_idx;
  logic [3:0]              pri_rsp_code;    // success / invalid / failure ⚑RECONCILE

  modport agent (   // ATS/PASID/PRI subsystem side
    input  clk, rst_n,
    input  ats_req_ready, ats_rsp_valid, ats_rsp_id, ats_rsp,
    input  pri_req_ready, pri_rsp_valid, pri_rsp_prg_idx, pri_rsp_code,
    output ats_req_valid, ats_req_addr, ats_req_pasid, ats_req_pasid_en,
    output ats_req_is_write, ats_req_id, ats_rsp_ready,
    output pri_req_valid, pri_req_addr, pri_req_pasid, pri_req_pasid_en,
    output pri_req_rd, pri_req_wr, pri_req_prg_idx, pri_rsp_ready
  );

  modport paa (
    input  clk, rst_n,
    input  ats_req_valid, ats_req_addr, ats_req_pasid, ats_req_pasid_en,
    input  ats_req_is_write, ats_req_id, ats_rsp_ready,
    input  pri_req_valid, pri_req_addr, pri_req_pasid, pri_req_pasid_en,
    input  pri_req_rd, pri_req_wr, pri_req_prg_idx, pri_rsp_ready,
    output ats_req_ready, ats_rsp_valid, ats_rsp_id, ats_rsp,
    output pri_req_ready, pri_rsp_valid, pri_rsp_prg_idx, pri_rsp_code
  );

endinterface : ats_pri_if
