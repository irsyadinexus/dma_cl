// ============================================================================
// axi4_if.sv — AXI4 memory-mapped device datapath (first-class MM path).
// M1 skeleton. DMA channel is the AXI master (to on-card DDR / local memory).
// ============================================================================
interface axi4_if #(
  parameter int DW = dma_pkg::DATA_WIDTH,
  parameter int AW = 64,
  parameter int IW = dma_pkg::AXI_ID_W
) (input logic clk, input logic rst_n);

  localparam int SW = DW/8;

  // Write address
  logic [IW-1:0]  awid;   logic [AW-1:0] awaddr;  logic [7:0] awlen;
  logic [2:0]     awsize; logic [1:0]    awburst; logic       awvalid, awready;
  // Write data
  logic [DW-1:0]  wdata;  logic [SW-1:0] wstrb;   logic wlast, wvalid, wready;
  // Write response
  logic [IW-1:0]  bid;    logic [1:0]    bresp;   logic bvalid, bready;
  // Read address
  logic [IW-1:0]  arid;   logic [AW-1:0] araddr;  logic [7:0] arlen;
  logic [2:0]     arsize; logic [1:0]    arburst; logic       arvalid, arready;
  // Read data
  logic [IW-1:0]  rid;    logic [DW-1:0] rdata;   logic [1:0] rresp;
  logic           rlast;  logic          rvalid,  rready;

  modport master (
    input  clk, rst_n, awready, wready, bid, bresp, bvalid,
           arready, rid, rdata, rresp, rlast, rvalid,
    output awid, awaddr, awlen, awsize, awburst, awvalid,
           wdata, wstrb, wlast, wvalid, bready,
           arid, araddr, arlen, arsize, arburst, arvalid, rready
  );

  modport slave (
    input  clk, rst_n, awid, awaddr, awlen, awsize, awburst, awvalid,
           wdata, wstrb, wlast, wvalid, bready,
           arid, araddr, arlen, arsize, arburst, arvalid, rready,
    output awready, wready, bid, bresp, bvalid,
           arready, rid, rdata, rresp, rlast, rvalid
  );

endinterface : axi4_if
