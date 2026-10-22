// ============================================================================
// axi4s_if.sv — AXI4-Stream device datapath (extension path, spec §10.1).
// Read/H2C channel drives master; Write/C2H channel consumes as slave.
// M1 skeleton. TUSER carries per-packet sideband (SOP/EOP meta, stream tag).
// ============================================================================
interface axi4s_if #(
  parameter int DW = dma_pkg::DATA_WIDTH,
  parameter int UW = 32
) (input logic clk, input logic rst_n);

  localparam int KW = DW/8;

  logic [DW-1:0]  tdata;
  logic [KW-1:0]  tkeep;
  logic           tlast;
  logic [UW-1:0]  tuser;
  logic           tvalid;
  logic           tready;

  modport master (          // producer: H2C (host -> stream)
    input  clk, rst_n, tready,
    output tdata, tkeep, tlast, tuser, tvalid
  );

  modport slave (           // consumer: C2H (stream -> host)
    input  clk, rst_n, tdata, tkeep, tlast, tuser, tvalid,
    output tready
  );

endinterface : axi4s_if
