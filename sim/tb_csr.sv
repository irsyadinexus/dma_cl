// ============================================================================
// tb_csr.sv — self-checking APB testbench for csr_block (Verilator --binary).
// Exercises: identity regs, per-channel R/W, 64-bit reassembly, doorbell pulse,
// extension region (stream mode / PASID), interrupt set + W1C clear, direction,
// and pslverr on undecoded access. Run: make sim-csr
// ============================================================================
module tb_csr;
  import dma_pkg::*;

  localparam int NRD = 2, NWR = 2, NCH = NRD + NWR, AW = 20;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // APB
  logic [AW-1:0] paddr;  logic psel, penable, pwrite;
  logic [31:0]   pwdata; logic [3:0] pstrb;
  logic [31:0]   prdata; logic pready, pslverr;

  // DUT side-channels
  ch_ctrl_t ch_ctrl [NCH];
  ch_stat_t ch_stat [NCH];
  ch_msi_t  ch_msi  [NCH];
  logic     ch_irq  [NCH];
  logic     rd_en, wr_en, soft_reset;

  csr_block #(.NUM_RD_CH(NRD), .NUM_WR_CH(NWR), .APB_AW(AW)) dut (
    .csr_clk(clk), .csr_rst_n(rst_n),
    .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite),
    .pwdata(pwdata), .pstrb(pstrb), .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .ch_ctrl(ch_ctrl), .ch_stat(ch_stat), .ch_msi(ch_msi), .ch_irq(ch_irq),
    .rd_engine_en(rd_en), .wr_engine_en(wr_en), .soft_reset(soft_reset)
  );

  // sticky doorbell capture
  logic db_seen [NCH];
  always @(posedge clk) for (int i = 0; i < NCH; i++) if (ch_ctrl[i].doorbell) db_seen[i] <= 1'b1;

  int errors = 0;

  task automatic apb_write(input logic [AW-1:0] a, input logic [31:0] d);
    @(negedge clk); psel = 1; pwrite = 1; paddr = a; pwdata = d; pstrb = 4'hF; penable = 0;
    @(negedge clk); penable = 1;
    @(negedge clk); psel = 0; penable = 0; pwrite = 0;
  endtask

  task automatic apb_read(input logic [AW-1:0] a, output logic [31:0] d);
    @(negedge clk); psel = 1; pwrite = 0; paddr = a; penable = 0;
    @(negedge clk); penable = 1;
    @(negedge clk); d = prdata; psel = 0; penable = 0;
  endtask

  task automatic pulse_done(input int ch);
    @(negedge clk); ch_stat[ch].done_irq = 1'b1;
    @(negedge clk); ch_stat[ch].done_irq = 1'b0;
  endtask

  function automatic void ck(input string nm, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin errors++; $display("  FAIL %-22s got=%016x exp=%016x", nm, got, exp); end
    else                         $display("  ok   %-22s = %016x", nm, got);
  endfunction

  // channel block base
  function automatic logic [AW-1:0] rd_ch(input int n); return 20'h1000 + n*20'h200; endfunction
  function automatic logic [AW-1:0] wr_ch(input int n); return 20'h5000 + n*20'h200; endfunction

  logic [31:0] rd;

  initial begin
    // reset
    psel = 0; penable = 0; pwrite = 0; paddr = 0; pwdata = 0; pstrb = 0;
    for (int i = 0; i < NCH; i++) begin ch_stat[i] = '0; db_seen[i] = 0; end
    repeat (4) @(negedge clk);
    rst_n = 1;
    @(negedge clk);

    $display("[tb_csr] start");

    // ---- identity ----
    apb_read(20'h000, rd); ck("ID_VERSION",   rd, 64'h444D0100);
    apb_read(20'h004, rd); ck("CAPABILITIES", rd, 64'h0200223F); // DW512,WR2,RD2,caps=0x3F

    // ---- per-channel RD ch0 ----
    apb_write(rd_ch(0) + 9'h34, 32'h0000_0014);          // CONTROL1: LLE(2)+LIE(4)
    apb_read (rd_ch(0) + 9'h34, rd); ck("CONTROL1 rb", rd, 64'h14);
    apb_write(rd_ch(0) + 9'h10, 32'hDEAD_BEEF);          // LLP_LO
    apb_write(rd_ch(0) + 9'h14, 32'h1234_5678);          // LLP_HI
    apb_write(rd_ch(0) + 9'h00, 32'h1);                  // CH_EN
    ck("ch0.enable", ch_ctrl[0].enable, 1);
    ck("ch0.lle",    ch_ctrl[0].lle,    1);
    ck("ch0.lie",    ch_ctrl[0].lie,    1);
    ck("ch0.llp",    ch_ctrl[0].llp_base, 64'h12345678DEADBEEF);
    ck("ch0.dir(RD)",ch_ctrl[0].dir,    CH_READ);

    // byte-strobe write (only lane 1)
    apb_write(rd_ch(0) + 9'h1C, 32'hFFFF_FFFF);          // TRANSFER_SIZE full
    @(negedge clk); psel=1; pwrite=1; paddr=rd_ch(0)+9'h1C; pwdata=32'h0000_0000; pstrb=4'b0010; penable=0;
    @(negedge clk); penable=1; @(negedge clk); psel=0; penable=0; pwrite=0;
    apb_read (rd_ch(0) + 9'h1C, rd); ck("size be-write", rd, 64'hFFFF_00FF);

    // ---- doorbell pulse ----
    apb_write(rd_ch(0) + 9'h04, 32'h1);                  // DOORBELL
    repeat (2) @(negedge clk);
    ck("ch0.doorbell pulse", db_seen[0], 1);

    // ---- extension: stream mode + PASID (RD ch0) ----
    apb_write(20'h9010 + 0*4, 32'hBEEF_0001);            // RD_CH0 STREAM_CTRL: mode=1, tuser=0xBEEF
    ck("ch0.mode(ST)",   ch_ctrl[0].mode,        DEV_AXI_ST);
    ck("ch0.stream_user",ch_ctrl[0].stream_user, 64'hBEEF);
    apb_write(20'h9110 + 0*8, 32'h0030_0005);            // RD_CH0 PASID: pasid=5, en(20), priv(21)
    ck("ch0.pasid",    ch_ctrl[0].pasid,    5);
    ck("ch0.pasid_en", ch_ctrl[0].pasid_en, 1);
    ck("ch0.pasid_prv",ch_ctrl[0].pasid_priv,1);

    // ---- WR ch0 is global index NRD (=2): direction check ----
    apb_write(wr_ch(0) + 9'h00, 32'h1);
    ck("ch2.enable",   ch_ctrl[2].enable, 1);
    ck("ch2.dir(WR)",  ch_ctrl[2].dir,    CH_WRITE);

    // ---- interrupt set + read + W1C ----
    pulse_done(0);
    @(negedge clk);
    apb_read(20'h010, rd);              ck("RD_INT_STATUS bit0", rd[0], 1);
    apb_read(rd_ch(0) + 9'h58, rd);     ck("ch0 INT_STATUS done", rd[0], 1);
    ck("ch0.ch_irq", ch_irq[0], 1);
    apb_write(rd_ch(0) + 9'h54, 32'h1); // per-channel INT_CLEAR done
    @(negedge clk);
    apb_read(20'h010, rd);              ck("RD_INT_STATUS cleared", rd[0], 0);

    // global clear path
    pulse_done(1); @(negedge clk);
    apb_read(20'h010, rd);              ck("RD_INT_STATUS bit1", rd[1], 1);
    apb_write(20'h014, 32'h2);          // RD_INT_CLEAR bit1
    @(negedge clk);
    apb_read(20'h010, rd);              ck("global clear ch1", rd[1], 0);

    // ---- pslverr on undecoded ----
    @(negedge clk); psel=1; pwrite=0; paddr=20'hA000; penable=0;
    @(negedge clk); penable=1;
    @(negedge clk); ck("pslverr undecoded", pslverr, 1); psel=0; penable=0;

    repeat (2) @(negedge clk);
    if (errors == 0) $display("[tb_csr] ALL PASS");
    else             $display("[tb_csr] %0d FAILURE(S)", errors);
    $finish;
  end

  // watchdog
  initial begin #100000; $display("[tb_csr] TIMEOUT"); $finish; end

endmodule
