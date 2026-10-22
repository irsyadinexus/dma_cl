// Lint/elaboration harness — instantiates every M1 interface so tools have a
// top to elaborate. Not a functional module. Used by `make lint`.
module lint_top;
  logic clk, rst_n;
  logic pclk, presetn;

  pcie_req_if u_req  (.clk(clk),   .rst_n(rst_n));
  ats_pri_if  u_ats  (.clk(clk),   .rst_n(rst_n));
  axi4_if     u_axi  (.clk(clk),   .rst_n(rst_n));
  axi4s_if    u_axis (.clk(clk),   .rst_n(rst_n));
  csr_apb_if  u_csr  (.pclk(pclk), .presetn(presetn));

  // touch a package symbol so the package is elaborated
  dma_pkg::err_code_e e;
  initial e = dma_pkg::ERR_NONE;
endmodule
