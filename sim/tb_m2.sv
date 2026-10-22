// ============================================================================
// tb_m2.sv — Directed verification testbench for Milestone M2.
// Exercises: CSR sanity, single-beat H2C, multi-beat H2C, single-beat C2H,
// multi-beat C2H, 4KB page boundary split, and concurrent full-duplex operation.
// ============================================================================
module tb_m2;
  import dma_pkg::*;

  localparam int DW = DATA_WIDTH;
  localparam int NCH = NUM_RD_CH + NUM_WR_CH;
  localparam int HOST_MEM_SIZE = 1024 * 1024; // 1 MB host memory model

  // Clocks and resets
  logic core_clk = 0, core_rst_n = 0;
  logic axi_clk  = 0, axi_rst_n  = 0;
  logic csr_clk  = 0, csr_rst_n  = 0;

  always #1 core_clk = ~core_clk; // 500 MHz
  always #1.25 axi_clk = ~axi_clk; // 400 MHz
  always #5 csr_clk = ~csr_clk;   // 100 MHz

  // Host PCIe Transaction Buses
  pcie_req_bus_t host_req;
  logic          host_req_ready;
  pcie_cpl_bus_t host_cpl;
  logic          host_cpl_ready;

  // Host APB Slave Interface
  logic [19:0] s_apb_paddr;
  logic        s_apb_psel;
  logic        s_apb_penable;
  logic        s_apb_pwrite;
  logic [31:0] s_apb_pwdata;
  logic [3:0]  s_apb_pstrb;
  logic [31:0] s_apb_prdata;
  logic        s_apb_pready;
  logic        s_apb_pslverr;

  // Device AXI4-MM Buses
  axi_mm_m2s_t dev_mm_m2s     [NCH];
  axi_mm_s2m_t dev_mm_s2m     [NCH];

  // Device AXI4-Stream Buses
  axis_data_t  dev_st_o       [NCH];
  logic        dev_st_o_ready [NCH];
  axis_data_t  dev_st_i       [NCH];
  logic        dev_st_i_ready [NCH];

  // Interrupts
  logic        ch_irq         [NCH];
  ch_msi_t     ch_msi         [NCH];

  // Host memory array
  logic [7:0] host_mem [HOST_MEM_SIZE];

  int errors = 0;

  // DUT instantiation
  dma_top u_dut (
    .core_clk       (core_clk),
    .core_rst_n     (core_rst_n),
    .axi_clk        (axi_clk),
    .axi_rst_n      (axi_rst_n),
    .csr_clk        (csr_clk),
    .csr_rst_n      (csr_rst_n),
    .host_req       (host_req),
    .host_req_ready (host_req_ready),
    .host_cpl       (host_cpl),
    .host_cpl_ready (host_cpl_ready),
    .s_apb_paddr    (s_apb_paddr),
    .s_apb_psel     (s_apb_psel),
    .s_apb_penable  (s_apb_penable),
    .s_apb_pwrite   (s_apb_pwrite),
    .s_apb_pwdata   (s_apb_pwdata),
    .s_apb_pstrb    (s_apb_pstrb),
    .s_apb_prdata   (s_apb_prdata),
    .s_apb_pready   (s_apb_pready),
    .s_apb_pslverr  (s_apb_pslverr),
    .dev_mm_m2s     (dev_mm_m2s),
    .dev_mm_s2m     (dev_mm_s2m),
    .dev_st_o       (dev_st_o),
    .dev_st_o_ready (dev_st_o_ready),
    .dev_st_i       (dev_st_i),
    .dev_st_i_ready (dev_st_i_ready),
    .ch_irq         (ch_irq),
    .ch_msi         (ch_msi)
  );

  // Device AXI RAM models
  axi_ram u_ram_h2c (
    .clk       (axi_clk),
    .rst_n     (axi_rst_n),
    .s_axi_m2s (dev_mm_m2s[0]),
    .s_axi_s2m (dev_mm_s2m[0])
  );

  axi_ram u_ram_c2h (
    .clk       (axi_clk),
    .rst_n     (axi_rst_n),
    .s_axi_m2s (dev_mm_m2s[NUM_RD_CH]),
    .s_axi_s2m (dev_mm_s2m[NUM_RD_CH])
  );

  // Tie-off remaining unused channels in M2
  genvar k;
  generate
    for (k = 0; k < NCH; k++) begin : g_ties
      if (k != 0 && k != NUM_RD_CH) begin
        assign dev_mm_s2m[k]     = '0;
      end
      assign dev_st_o_ready[k] = 1'b1;
      assign dev_st_i[k]       = '0;
    end
  endgenerate

  // PCIe BFM: Request & Completion handler
  assign host_req_ready = 1'b1;

  logic [63:0]       cpl_addr;
  logic [TAG_W-1:0]  cpl_tag;
  logic [12:0]       cpl_byte_count;
  logic [7:0]        cpl_beats_rem;
  logic              cpl_active;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      host_cpl       <= '0;
      cpl_addr       <= '0;
      cpl_tag        <= '0;
      cpl_byte_count <= '0;
      cpl_beats_rem  <= '0;
      cpl_active     <= 1'b0;
    end else begin
      // Write sink: capture write beat into host memory
      if (host_req.valid && !host_req.req.is_read) begin
        for (int i = 0; i < 64; i++) begin
          int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
          host_mem[off] <= host_req.wdata[8*i +: 8];
        end
      end

      // Read completion engine
      if (!cpl_active) begin
        if (host_req.valid && host_req.req.is_read) begin
          logic [7:0] total_beats;
          total_beats = 8'((host_req.req.len_bytes + 13'd63) >> 6);
          cpl_active     <= 1'b1;
          cpl_addr       <= host_req.req.addr + 64;
          cpl_tag        <= host_req.req.tag;
          cpl_byte_count <= host_req.req.len_bytes;
          cpl_beats_rem  <= total_beats - 1'b1;

          host_cpl.valid           <= 1'b1;
          host_cpl.meta.tag        <= host_req.req.tag;
          host_cpl.meta.byte_count <= host_req.req.len_bytes;
          host_cpl.meta.lower_addr <= host_req.req.addr[6:0];
          host_cpl.meta.status     <= CPL_SC;
          host_cpl.meta.last       <= (total_beats <= 8'd1);
          for (int i = 0; i < 64; i++) begin
            int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
            host_cpl.rdata[8*i +: 8] <= host_mem[off];
          end
        end else begin
          host_cpl.valid <= 1'b0;
        end
      end else begin
        if (host_cpl.valid && host_cpl_ready) begin
          if (cpl_beats_rem == 0) begin
            cpl_active     <= 1'b0;
            host_cpl.valid <= 1'b0;
          end else begin
            cpl_beats_rem  <= cpl_beats_rem - 1'b1;
            cpl_addr       <= cpl_addr + 64;

            host_cpl.valid           <= 1'b1;
            host_cpl.meta.tag        <= cpl_tag;
            host_cpl.meta.byte_count <= cpl_byte_count;
            host_cpl.meta.lower_addr <= cpl_addr[6:0];
            host_cpl.meta.status     <= CPL_SC;
            host_cpl.meta.last       <= (cpl_beats_rem == 8'd1);
            for (int i = 0; i < 64; i++) begin
              int unsigned off = (int'(cpl_addr) + i) % HOST_MEM_SIZE;
              host_cpl.rdata[8*i +: 8] <= host_mem[off];
            end
          end
        end
      end
    end
  end

  // APB Tasks
  task automatic apb_write(input logic [19:0] addr, input logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1;
    s_apb_pwrite  = 1;
    s_apb_paddr   = addr;
    s_apb_pwdata  = data;
    s_apb_pstrb   = 4'hF;
    s_apb_penable = 0;
    @(negedge csr_clk);
    s_apb_penable = 1;
    @(negedge csr_clk);
    s_apb_psel    = 0;
    s_apb_penable = 0;
    s_apb_pwrite  = 0;
  endtask

  task automatic apb_read(input logic [19:0] addr, output logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1;
    s_apb_pwrite  = 0;
    s_apb_paddr   = addr;
    s_apb_penable = 0;
    @(negedge csr_clk);
    s_apb_penable = 1;
    @(negedge csr_clk);
    data          = s_apb_prdata;
    s_apb_psel    = 0;
    s_apb_penable = 0;
  endtask

  // Helper check function
  task automatic check_val(input string msg, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin
      $display("  [FAIL] %s: got=%016x exp=%016x", msg, got, exp);
      errors++;
    end else begin
      $display("  [OK]   %s = %08x", msg, got[31:0]);
    end
  endtask

  // Test sequence
  initial begin
    logic [31:0] rdata;
    $display("[tb_m2] Starting Milestone M2 Verification Suite...");

    // Reset sequence
    core_rst_n = 0; axi_rst_n = 0; csr_rst_n = 0;
    s_apb_psel = 0; s_apb_penable = 0; s_apb_pwrite = 0;
    repeat (10) @(posedge csr_clk);
    core_rst_n = 1; axi_rst_n = 1; csr_rst_n = 1;
    repeat (5) @(posedge csr_clk);

    // ------------------------------------------------------------------------
    // Test 1: CSR Sanity
    // ------------------------------------------------------------------------
    $display("\n--- Test 1: CSR Sanity ---");
    apb_read(20'h0000, rdata); check_val("ID_VERSION", rdata, 32'h444D_0100);
    apb_read(20'h0004, rdata); check_val("CAPABILITIES", rdata[5:0], 6'h3F);

    // ------------------------------------------------------------------------
    // Test 2: Single-Beat H2C (64 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 2: Single-Beat H2C (64 Bytes) ---");
    for (int i = 0; i < 64; i++) host_mem[32'h0001_0000 + i] = 8'(i + 1);
    apb_write(20'h0008, 32'h1); // RD_ENGINE_EN
    apb_write(20'h1000, 32'h1); // CH0_EN
    apb_write(20'h1020, 32'h0001_0000); // SAR_LO
    apb_write(20'h1024, 32'h0000_0000); // SAR_HI
    apb_write(20'h1028, 32'h0002_0000); // DAR_LO
    apb_write(20'h102C, 32'h0000_0000); // DAR_HI
    apb_write(20'h101C, 32'd64);        // TRANSFER_SIZE
    apb_write(20'h1004, 32'h1);         // DOORBELL

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 IRQ Asserted", ch_irq[0], 1);
    check_val("H2C Byte 0 in RAM", u_ram_h2c.read_byte(64'h0002_0000), 8'h01);
    check_val("H2C Byte 63 in RAM", u_ram_h2c.read_byte(64'h0002_003F), 8'd64);
    apb_write(20'h1054, 32'h1); // Clear done IRQ

    // ------------------------------------------------------------------------
    // Test 3: Multi-Beat H2C Burst (512 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 3: Multi-Beat H2C Burst (512 Bytes) ---");
    for (int i = 0; i < 512; i++) host_mem[32'h0001_0200 + i] = 8'(i ^ 8'hA5);
    apb_write(20'h1020, 32'h0001_0200);
    apb_write(20'h1028, 32'h0002_0200);
    apb_write(20'h101C, 32'd512);
    apb_write(20'h1004, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Multi-Beat Done", ch_irq[0], 1);
    check_val("H2C Burst Byte 0", u_ram_h2c.read_byte(64'h0002_0200), 8'hA5);
    check_val("H2C Burst Byte 511", u_ram_h2c.read_byte(64'h0002_03FF), 8'(511 ^ 8'hA5));
    apb_write(20'h1054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 4: Single-Beat C2H (64 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 4: Single-Beat C2H (64 Bytes) ---");
    for (int i = 0; i < 64; i++) u_ram_c2h.write_byte(64'h0003_0000 + i, 8'(i + 8'h50));
    apb_write(20'h000C, 32'h1); // WR_ENGINE_EN
    apb_write(20'h5000, 32'h1); // CH1_EN (WR Ch 0 = global ch 4 in 4+4)
    apb_write(20'h5020, 32'h0003_0000); // SAR_LO
    apb_write(20'h5028, 32'h0004_0000); // DAR_LO
    apb_write(20'h501C, 32'd64);        // TRANSFER_SIZE
    apb_write(20'h5004, 32'h1);         // DOORBELL

    wait (ch_irq[NUM_RD_CH] == 1'b1);
    check_val("Ch1 IRQ Asserted", ch_irq[NUM_RD_CH], 1);
    check_val("C2H Byte 0 in Host Mem", host_mem[32'h0004_0000], 8'h50);
    check_val("C2H Byte 63 in Host Mem", host_mem[32'h0004_003F], 8'(63 + 8'h50));
    apb_write(20'h5054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 5: Multi-Beat C2H Burst (512 Bytes)
    // ------------------------------------------------------------------------
    $display("\n--- Test 5: Multi-Beat C2H Burst (512 Bytes) ---");
    for (int i = 0; i < 512; i++) u_ram_c2h.write_byte(64'h0003_0200 + i, 8'(i ^ 8'h3C));
    apb_write(20'h5020, 32'h0003_0200);
    apb_write(20'h5028, 32'h0004_0200);
    apb_write(20'h501C, 32'd512);
    apb_write(20'h5004, 32'h1);

    wait (ch_irq[NUM_RD_CH] == 1'b1);
    check_val("C2H Multi-Beat Done", ch_irq[NUM_RD_CH], 1);
    check_val("C2H Burst Byte 0", host_mem[32'h0004_0200], 8'h3C);
    check_val("C2H Burst Byte 511", host_mem[32'h0004_03FF], 8'(511 ^ 8'h3C));
    apb_write(20'h5054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 6: 4KB Page Boundary Splitting Test
    // ------------------------------------------------------------------------
    $display("\n--- Test 6: 4KB Page Boundary Splitting Test ---");
    // Start address at 0x0001_0F00 (256 bytes before 0x0001_1000 4KB boundary)
    for (int i = 0; i < 512; i++) host_mem[32'h0001_0F00 + i] = 8'(i + 8'h11);
    apb_write(20'h1020, 32'h0001_0F00); // 256B to 4K boundary
    apb_write(20'h1028, 32'h0002_0F00);
    apb_write(20'h101C, 32'd512);        // spans across 4KB boundary
    apb_write(20'h1004, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("4KB Split Transfer Done", ch_irq[0], 1);
    check_val("4KB Split First Byte", u_ram_h2c.read_byte(64'h0002_0F00), 8'h11);
    check_val("4KB Boundary Crossing Byte", u_ram_h2c.read_byte(64'h0002_1000), 8'(256 + 8'h11));
    check_val("4KB Split Last Byte", u_ram_h2c.read_byte(64'h0002_10FF), 8'(511 + 8'h11));
    apb_write(20'h1054, 32'h1);

    // ------------------------------------------------------------------------
    // Test 7: Concurrent Full-Duplex Operation
    // ------------------------------------------------------------------------
    $display("\n--- Test 7: Concurrent Full-Duplex Operation ---");
    for (int i = 0; i < 128; i++) begin
      host_mem[32'h0001_2000 + i] = 8'(i + 8'hAA);
      u_ram_c2h.write_byte(64'h0003_2000 + i, 8'(i + 8'h55));
    end
    // Launch H2C
    apb_write(20'h1020, 32'h0001_2000);
    apb_write(20'h1028, 32'h0002_2000);
    apb_write(20'h101C, 32'd128);
    apb_write(20'h1004, 32'h1);
    // Launch C2H
    apb_write(20'h5020, 32'h0003_2000);
    apb_write(20'h5028, 32'h0004_2000);
    apb_write(20'h501C, 32'd128);
    apb_write(20'h5004, 32'h1);

    wait (ch_irq[0] == 1'b1 && ch_irq[NUM_RD_CH] == 1'b1);
    check_val("Concurrent H2C Byte 0", u_ram_h2c.read_byte(64'h0002_2000), 8'hAA);
    check_val("Concurrent C2H Byte 0", host_mem[32'h0004_2000], 8'h55);
    apb_write(20'h1054, 32'h1);
    apb_write(20'h5054, 32'h1);

    // Final summary
    if (errors == 0) begin
      $display("\n=======================================================");
      $display(" [tb_m2] ALL TESTS PASSED (0 errors)");
      $display("=======================================================\n");
    end else begin
      $display("\n[tb_m2] FAILED with %0d errors\n", errors);
    end
    $finish;
  end

  // Watchdog timeout
  initial begin
    #200000;
    $display("\n[tb_m2] ERROR: Simulation timed out!\n");
    $finish;
  end

endmodule
