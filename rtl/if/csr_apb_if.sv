// ============================================================================
// csr_apb_if.sv — control/status register access (host BAR completer -> CSR).
// APB-style slave on csr_clk. Presents the HDMA-native register map (BAR0) plus
// the vendor extension region. M1 skeleton. See docs/m1_register_map.md.
// ============================================================================
interface csr_apb_if #(
  parameter int AW = 20,   // BAR0 aperture: 1 MB
  parameter int DW = 32
) (input logic pclk, input logic presetn);

  logic [AW-1:0]  paddr;
  logic           psel;
  logic           penable;
  logic           pwrite;
  logic [DW-1:0]  pwdata;
  logic [DW/8-1:0] pstrb;
  logic [DW-1:0]  prdata;
  logic           pready;
  logic           pslverr;

  modport slave (   // the CSR block
    input  pclk, presetn, paddr, psel, penable, pwrite, pwdata, pstrb,
    output prdata, pready, pslverr
  );

  modport master ( // driven from the PAA completer path
    input  pclk, presetn, prdata, pready, pslverr,
    output paddr, psel, penable, pwrite, pwdata, pstrb
  );

endinterface : csr_apb_if
