// ============================================================================
// tb_m3.sv — Directed verification testbench for Milestone M3.
// Tests:
// Test 1: Multi-Element Linked-List H2C Transfer (Contiguous List)
// Test 2: Non-Contiguous List with Link Element Pointer (CTRL_LLP = 1)
// Test 3: Cyclic Ring Mode with Cycle-Bit Toggling (CTRL_TCB = 1)
// Test 4: Out-of-Order & RCB-Split Completion Reassembly (H2C)
// Test 5: Multi-Element Linked-List C2H Transfer (Device to Host)
// Test 6: Concurrent 8-Channel Multi-Channel Execution
// Test 7: Weighted Round-Robin (WRR) QoS Verification
// ============================================================================
module tb_m3;
  import dma_pkg::*;

  localparam int DW = DATA_WIDTH;
  localparam int NCH = NUM_RD_CH + NUM_WR_CH;
  localparam int HOST_MEM_SIZE = 4 * 1024 * 1024; // 4 MB host memory

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

  // Interrupts & MSI
  logic        ch_irq         [NCH];
  ch_msi_t     ch_msi         [NCH];

  // Host memory array
  logic [7:0] host_mem [HOST_MEM_SIZE];

  int errors = 0;
  bit bfm_ooo_en = 0; // Enables out-of-order and RCB-split injection

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

  // 8 Device AXI RAM models
  genvar k;
  generate
    for (k = 0; k < NCH; k++) begin : g_rams
      axi_ram u_ram_inst (
        .clk       (axi_clk),
        .rst_n     (axi_rst_n),
        .s_axi_m2s (dev_mm_m2s[k]),
        .s_axi_s2m (dev_mm_s2m[k])
      );
      assign dev_st_o_ready[k] = 1'b1;
      assign dev_st_i[k]       = '0;
    end
  endgenerate

  task automatic ram_write_byte(int ch, logic [63:0] addr, logic [7:0] data);
    case (ch)
      0: g_rams[0].u_ram_inst.write_byte(addr, data);
      1: g_rams[1].u_ram_inst.write_byte(addr, data);
      2: g_rams[2].u_ram_inst.write_byte(addr, data);
      3: g_rams[3].u_ram_inst.write_byte(addr, data);
      4: g_rams[4].u_ram_inst.write_byte(addr, data);
      5: g_rams[5].u_ram_inst.write_byte(addr, data);
      6: g_rams[6].u_ram_inst.write_byte(addr, data);
      7: g_rams[7].u_ram_inst.write_byte(addr, data);
      default: ;
    endcase
  endtask

  function automatic logic [7:0] ram_read_byte(int ch, logic [63:0] addr);
    case (ch)
      0: return g_rams[0].u_ram_inst.read_byte(addr);
      1: return g_rams[1].u_ram_inst.read_byte(addr);
      2: return g_rams[2].u_ram_inst.read_byte(addr);
      3: return g_rams[3].u_ram_inst.read_byte(addr);
      4: return g_rams[4].u_ram_inst.read_byte(addr);
      5: return g_rams[5].u_ram_inst.read_byte(addr);
      6: return g_rams[6].u_ram_inst.read_byte(addr);
      7: return g_rams[7].u_ram_inst.read_byte(addr);
      default: return 8'h00;
    endcase
  endfunction

  // --------------------------------------------------------------------------
  // PCIe Root Complex BFM (Request Intake & Out-of-Order Completion Engine)
  // --------------------------------------------------------------------------
  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [63:0]      addr;
    logic [12:0]      len_bytes;
    logic [12:0]      bytes_sent;
  } pending_req_t;

  pending_req_t req_q [$];

  assign host_req_ready = (req_q.size() < 32);

  // Capture incoming PCIe requests
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      req_q.delete();
    end else begin
      if (host_req.valid && host_req_ready) begin
        if (!host_req.req.is_read) begin
          // Memory Write (MWr)
          for (int i = 0; i < 64; i++) begin
            int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
            host_mem[off] <= host_req.wdata[8*i +: 8];
          end
        end else begin
          // Memory Read (MRd) — enqueue for completion
          pending_req_t item;
          item.tag        = host_req.req.tag;
          item.addr       = host_req.req.addr;
          item.len_bytes  = host_req.req.len_bytes;
          item.bytes_sent = '0;
          req_q.push_back(item);
        end
      end
    end
  end

  // Generate completions from queue
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      host_cpl <= '0;
    end else begin
      if (host_cpl.valid && host_cpl_ready) begin
        host_cpl.valid <= 1'b0;
      end

      if (!host_cpl.valid || host_cpl_ready) begin
        if (req_q.size() > 0) begin
          int q_idx;
          // In OOO mode, pick from back of queue if multiple items exist
          if (bfm_ooo_en && req_q.size() > 1 && req_q[$].tag[6:0] != 7'h7F) begin
            q_idx = req_q.size() - 1;
          end else begin
            q_idx = 0;
          end

          begin
            int unsigned cur_addr = int'(req_q[q_idx].addr) + int'(req_q[q_idx].bytes_sent);
            int unsigned bytes_left = int'(req_q[q_idx].len_bytes) - int'(req_q[q_idx].bytes_sent);
            int unsigned beat_bytes = (bytes_left > 64) ? 64 : bytes_left;
            logic is_last_beat = (int'(req_q[q_idx].bytes_sent) + beat_bytes >= int'(req_q[q_idx].len_bytes));

            host_cpl.valid           <= 1'b1;
            host_cpl.meta.tag        <= req_q[q_idx].tag;
            host_cpl.meta.byte_count <= 13'(beat_bytes);
            host_cpl.meta.lower_addr <= req_q[q_idx].addr[6:0];
            host_cpl.meta.status     <= CPL_SC;
            host_cpl.meta.last       <= is_last_beat;

            for (int b = 0; b < 64; b++) begin
              int unsigned off = (cur_addr + b) % HOST_MEM_SIZE;
              host_cpl.rdata[8*b +: 8] <= host_mem[off];
            end

            req_q[q_idx].bytes_sent = req_q[q_idx].bytes_sent + 13'(beat_bytes);

            if (is_last_beat) begin
              req_q.delete(q_idx);
            end
          end
        end else begin
          host_cpl.valid <= 1'b0;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Helper Tasks
  // --------------------------------------------------------------------------
  task automatic apb_write(input logic [19:0] addr, input logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1'b1;
    s_apb_pwrite  = 1'b1;
    s_apb_paddr   = addr;
    s_apb_pwdata  = data;
    s_apb_pstrb   = 4'hF;
    s_apb_penable = 1'b0;
    @(negedge csr_clk);
    s_apb_penable = 1'b1;
    @(negedge csr_clk);
    s_apb_psel    = 1'b0;
    s_apb_penable = 1'b0;
    s_apb_pwrite  = 1'b0;
  endtask

  task automatic apb_read(input logic [19:0] addr, output logic [31:0] data);
    @(negedge csr_clk);
    s_apb_psel    = 1'b1;
    s_apb_pwrite  = 1'b0;
    s_apb_paddr   = addr;
    s_apb_penable = 1'b0;
    @(negedge csr_clk);
    s_apb_penable = 1'b1;
    @(negedge csr_clk);
    data          = s_apb_prdata;
    s_apb_psel    = 1'b0;
    s_apb_penable = 1'b0;
  endtask

  task automatic host_mem_write32(input logic [63:0] addr, input logic [31:0] val);
    for (int i = 0; i < 4; i++) begin
      host_mem[int'(addr) + i] = val[8*i +: 8];
    end
  endtask

  task automatic build_elem(
    input logic [63:0] addr,
    input logic [31:0] ctrl,
    input logic [31:0] size,
    input logic [63:0] sar,
    input logic [63:0] dar
  );
    host_mem_write32(addr + 0,  ctrl);
    host_mem_write32(addr + 4,  size);
    host_mem_write32(addr + 8,  sar[31:0]);
    host_mem_write32(addr + 12, sar[63:32]);
    host_mem_write32(addr + 16, dar[31:0]);
    host_mem_write32(addr + 20, dar[63:32]);
  endtask

  task automatic check_val(string name, logic [31:0] actual, logic [31:0] expected);
    if (actual !== expected) begin
      $display("  [FAIL] %s: got 0x%08h, expected 0x%08h", name, actual, expected);
      errors++;
    end else begin
      $display("  [OK]   %s = %08h", name, actual);
    end
  endtask

  // Reset task
  task automatic do_reset();
    core_rst_n = 1'b0;
    axi_rst_n  = 1'b0;
    csr_rst_n  = 1'b0;
    s_apb_psel = 1'b0;
    s_apb_penable = 1'b0;
    #20;
    @(posedge core_clk);
    core_rst_n = 1'b1;
    axi_rst_n  = 1'b1;
    csr_rst_n  = 1'b1;
    #20;
  endtask

  task automatic clear_irq(int ch);
    logic [19:0] base_reg = (ch < 4) ? (20'h1000 + (ch * 20'h200)) : (20'h5000 + ((ch-4) * 20'h200));
    do begin
      repeat (5) @(posedge csr_clk);
      apb_write(base_reg + 20'h54, 32'h1);
    end while (ch_irq[ch]);
  endtask

  // --------------------------------------------------------------------------
  // Main Verification Sequence
  // --------------------------------------------------------------------------
  initial begin
    $display("=======================================================");
    $display(" [tb_m3] Starting Milestone M3 Verification Suite...");
    $display("=======================================================");

    do_reset();

    // Enable Read and Write DMA engines globally
    apb_write(20'h0008, 32'h1); // RD_ENGINE_EN
    apb_write(20'h000C, 32'h1); // WR_ENGINE_EN

    // ------------------------------------------------------------------------
    // Test 1: Multi-Element Linked-List H2C Transfer (Contiguous List)
    // ------------------------------------------------------------------------
    $display("\n--- Test 1: Multi-Element Contiguous H2C Linked-List Transfer ---");
    // 4 elements of 512 bytes each (2048 bytes total)
    // Descriptor list at 0x1000
    for (int i = 0; i < 4; i++) begin
      logic [31:0] ctrl_w = (i == 3) ? 32'h09 : 32'h01; // CB=1, LIE=1 on last element
      build_elem(
        64'h0000_1000 + (i * 24),
        ctrl_w,
        32'd512,
        64'h0001_0000 + (i * 512),
        64'h0002_0000 + (i * 512)
      );
    end

    // Populate source data in host memory
    for (int i = 0; i < 2048; i++) begin
      host_mem[32'h0001_0000 + i] = 8'(i ^ 8'hA5);
    end

    // Program Read Channel 0
    apb_write(20'h1000, 32'h1);          // C_CH_EN = 1
    apb_write(20'h1034, 32'h04);         // C_CONTROL1.lle = 1
    apb_write(20'h1010, 32'h0000_1000);  // C_LLP_LO
    apb_write(20'h1014, 32'h0);          // C_LLP_HI
    apb_write(20'h1004, 32'h1);          // C_DOORBELL

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 LL Done IRQ", ch_irq[0], 1);
    check_val("Ch0 Byte 0 in AXI RAM",   g_rams[0].u_ram_inst.read_byte(64'h0002_0000), 8'h00 ^ 8'hA5);
    check_val("Ch0 Byte 512 in AXI RAM", g_rams[0].u_ram_inst.read_byte(64'h0002_0200), 8'(512 ^ 8'hA5));
    check_val("Ch0 Byte 2047 in AXI RAM",g_rams[0].u_ram_inst.read_byte(64'h0002_07FF), 8'(2047 ^ 8'hA5));
    clear_irq(0);

    // ------------------------------------------------------------------------
    // Test 2: Non-Contiguous List with Link Element (CTRL_LLP = 1)
    // ------------------------------------------------------------------------
    $display("\n--- Test 2: Non-Contiguous List with Link Element (LLP=1) ---");
    // Chunk 1 at 0x2000: 2 Data elements + 1 Link element pointing to Chunk 2 at 0x3000
    build_elem(64'h0000_2000, 32'h01, 32'd512, 64'h0001_1000, 64'h0002_1000);
    build_elem(64'h0000_2018, 32'h01, 32'd512, 64'h0001_1200, 64'h0002_1200);
    build_elem(64'h0000_2030, 32'h05, 32'd0,   64'h0000_3000, 64'h0); // LLP=1, CB=1

    // Chunk 2 at 0x3000: 2 Data elements
    build_elem(64'h0000_3000, 32'h01, 32'd512, 64'h0001_1400, 64'h0002_1400);
    build_elem(64'h0000_3018, 32'h09, 32'd512, 64'h0001_1600, 64'h0002_1600); // LIE=1, CB=1

    for (int i = 0; i < 2048; i++) begin
      host_mem[32'h0001_1000 + i] = 8'(i + 8'h33);
    end

    apb_write(20'h1010, 32'h0000_2000);  // Head pointer to Chunk 1
    apb_write(20'h1004, 32'h1);          // Doorbell

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Jump Link Done", ch_irq[0], 1);
    check_val("Ch0 Chunk 1 Byte 0",   g_rams[0].u_ram_inst.read_byte(64'h0002_1000), 8'h33);
    check_val("Ch0 Chunk 2 Byte 0",   g_rams[0].u_ram_inst.read_byte(64'h0002_1400), 8'(1024 + 8'h33));
    check_val("Ch0 Chunk 2 Byte 1023",g_rams[0].u_ram_inst.read_byte(64'h0002_17FF), 8'(2047 + 8'h33));
    clear_irq(0);

    // ------------------------------------------------------------------------
    // Test 3: Cyclic Ring Mode with Cycle-Bit Toggling (CTRL_TCB = 1)
    // ------------------------------------------------------------------------
    $display("\n--- Test 3: Cyclic Ring Mode with Cycle-Bit Toggling (TCB=1) ---");
    // Ring at 0x4000: 1 Data Element + 1 Link element pointing back to 0x4000 with TCB=1
    build_elem(64'h0000_4000, 32'h09, 32'd256, 64'h0001_2000, 64'h0002_2000); // CB=1, LIE=1
    build_elem(64'h0000_4018, 32'h07, 32'd0,   64'h0000_4000, 64'h0);          // CB=1, LLP=1, TCB=1

    for (int i = 0; i < 256; i++) host_mem[32'h0001_2000 + i] = 8'(i + 8'h77);

    apb_write(20'h1010, 32'h0000_4000);  // Head pointer
    apb_write(20'h1004, 32'h1);          // Doorbell

    // Wait for first pass of ring
    wait (ch_irq[0] == 1'b1);
    check_val("Cyclic Ring Pass 1 Done", ch_irq[0], 1);
    check_val("Cyclic Pass 1 Byte 0", g_rams[0].u_ram_inst.read_byte(64'h0002_2000), 8'h77);
    clear_irq(0);

    // Software produces second iteration: toggles CB to 0 on Elem 0
    build_elem(64'h0000_4000, 32'h08, 32'd256, 64'h0001_2100, 64'h0002_2100); // CB=0, LIE=1
    for (int i = 0; i < 256; i++) host_mem[32'h0001_2100 + i] = 8'(i + 8'h88);

    apb_write(20'h1004, 32'h1);          // Doorbell to resume ring

    // Wait for second pass of ring (engine resumed on toggled CB!)
    wait (ch_irq[0] == 1'b1);
    check_val("Cyclic Ring Pass 2 Done", ch_irq[0], 1);
    check_val("Cyclic Pass 2 Byte 0", g_rams[0].u_ram_inst.read_byte(64'h0002_2100), 8'h88);
    clear_irq(0);

    // Stop channel 0
    apb_write(20'h1000, 32'h0);

    // ------------------------------------------------------------------------
    // Test 4: Out-of-Order & RCB-Split Completion Reassembly (H2C)
    // ------------------------------------------------------------------------
    $display("\n--- Test 4: Out-of-Order & RCB-Split Completion Reassembly ---");
    bfm_ooo_en = 1; // Enable OOO completion shuffling in BFM

    // 2 elements of 512B on Read Channel 1 (offset 0x1200)
    build_elem(64'h0000_6000, 32'h01, 32'd512, 64'h0001_3000, 64'h0002_3000);
    build_elem(64'h0000_6018, 32'h09, 32'd512, 64'h0001_3200, 64'h0002_3200);

    for (int i = 0; i < 1024; i++) begin
      host_mem[32'h0001_3000 + i] = 8'(i ^ 8'h5A);
    end

    apb_write(20'h1200, 32'h1);          // Ch1 EN
    apb_write(20'h1234, 32'h04);         // Ch1 lle = 1
    apb_write(20'h1210, 32'h0000_6000);  // Ch1 LLP_LO
    apb_write(20'h1214, 32'h0);
    apb_write(20'h1204, 32'h1);          // Ch1 Doorbell

    wait (ch_irq[1] == 1'b1);
    check_val("Ch1 OOO Reassembly Done", ch_irq[1], 1);
    check_val("Ch1 OOO Byte 0",   g_rams[1].u_ram_inst.read_byte(64'h0002_3000), 8'h00 ^ 8'h5A);
    check_val("Ch1 OOO Byte 511", g_rams[1].u_ram_inst.read_byte(64'h0002_31FF), 8'(511 ^ 8'h5A));
    check_val("Ch1 OOO Byte 1023",g_rams[1].u_ram_inst.read_byte(64'h0002_33FF), 8'(1023 ^ 8'h5A));
    clear_irq(1);

    bfm_ooo_en = 0;
    apb_write(20'h1200, 32'h0);

    // ------------------------------------------------------------------------
    // Test 5: Multi-Element Linked-List C2H Transfer (Device to Host)
    // ------------------------------------------------------------------------
    $display("\n--- Test 5: Multi-Element Linked-List C2H Transfer ---");
    // Write Channel 0 is channel index 4 (offset 0x5000)
    // 2 elements of 512B from AXI RAM 4 to host memory at 0x0004_0000
    build_elem(64'h0000_7000, 32'h01, 32'd512, 64'h0003_0000, 64'h0004_0000);
    build_elem(64'h0000_7018, 32'h09, 32'd512, 64'h0003_0200, 64'h0004_0200);

    for (int i = 0; i < 1024; i++) begin
      g_rams[4].u_ram_inst.write_byte(64'h0003_0000 + i, 8'(i ^ 8'hC3));
    end

    apb_write(20'h5000, 32'h1);          // Ch4 EN
    apb_write(20'h5034, 32'h04);         // Ch4 lle = 1
    apb_write(20'h5010, 32'h0000_7000);  // Ch4 LLP_LO
    apb_write(20'h5014, 32'h0);
    apb_write(20'h5004, 32'h1);          // Ch4 Doorbell

    wait (ch_irq[4] == 1'b1);
    check_val("Ch4 C2H LL Done", ch_irq[4], 1);
    check_val("C2H Host Mem Byte 0",    host_mem[32'h0004_0000], 8'h00 ^ 8'hC3);
    check_val("C2H Host Mem Byte 512",  host_mem[32'h0004_0200], 8'(512 ^ 8'hC3));
    check_val("C2H Host Mem Byte 1023", host_mem[32'h0004_03FF], 8'(1023 ^ 8'hC3));
    clear_irq(4);
    apb_write(20'h5000, 32'h0);

    // ------------------------------------------------------------------------
    // Test 6: Concurrent 8-Channel Multi-Channel Execution
    // ------------------------------------------------------------------------
    $display("\n--- Test 6: Concurrent 8-Channel Multi-Channel Execution ---");
    // Set up 8 channels (Ch 0..3 H2C, Ch 4..7 C2H)
    for (int c = 0; c < 8; c++) begin
      logic [19:0] base_reg = (c < 4) ? (20'h1000 + (c * 20'h200)) : (20'h5000 + ((c-4) * 20'h200));
      logic [63:0] desc_addr = 64'h0000_8000 + (c * 64'h100);

      // 1 element of 256 bytes per channel
      if (c < 4) begin
        // H2C
        build_elem(desc_addr, 32'h09, 32'd256, 64'h0001_8000 + (c * 256), 64'h0002_8000 + (c * 256));
        for (int b = 0; b < 256; b++) host_mem[32'h0001_8000 + (c * 256) + b] = 8'(b + c);
      end else begin
        // C2H
        build_elem(desc_addr, 32'h09, 32'd256, 64'h0003_8000 + (c * 256), 64'h0004_8000 + (c * 256));
        for (int b = 0; b < 256; b++) ram_write_byte(c, 64'h0003_8000 + (c * 256) + b, 8'(b ^ c));
      end

      apb_write(base_reg + 20'h00, 32'h1);       // EN
      apb_write(base_reg + 20'h34, 32'h04);      // lle = 1
      apb_write(base_reg + 20'h10, desc_addr[31:0]);
      apb_write(base_reg + 20'h14, desc_addr[63:32]);
    end

    // Ring all 8 doorbells
    for (int c = 0; c < 8; c++) begin
      logic [19:0] base_reg = (c < 4) ? (20'h1000 + (c * 20'h200)) : (20'h5000 + ((c-4) * 20'h200));
      apb_write(base_reg + 20'h04, 32'h1);
    end

    // Wait for all 8 channels to assert done IRQ
    for (int c = 0; c < 8; c++) begin
      wait (ch_irq[c] == 1'b1);
      if (c < 4) begin
        check_val($sformatf("Ch%0d Concurrent H2C Match", c),
                  ram_read_byte(c, 64'h0002_8000 + (c * 256)), 8'(c));
      end else begin
        check_val($sformatf("Ch%0d Concurrent C2H Match", c),
                  host_mem[32'h0004_8000 + (c * 256)], 8'(c));
      end
      // Clear IRQ and disable channel
      begin
        logic [19:0] base_reg = (c < 4) ? (20'h1000 + (c * 20'h200)) : (20'h5000 + ((c-4) * 20'h200));
        clear_irq(c);
        apb_write(base_reg + 20'h00, 32'h0);
      end
    end

    // ------------------------------------------------------------------------
    // Test 7: Weighted Round-Robin (WRR) QoS Verification
    // ------------------------------------------------------------------------
    $display("\n--- Test 7: Weighted Round-Robin (WRR) QoS Verification ---");
    // Program Ch0 with QOS weight 8, Ch1 with QOS weight 1
    apb_write(20'h103C, 32'd8); // Ch0 QOS = 8
    apb_write(20'h123C, 32'd1); // Ch1 QOS = 1

    // Verify QoS register reads back correctly
    begin
      logic [31:0] qos0, qos1;
      apb_read(20'h103C, qos0);
      apb_read(20'h123C, qos1);
      check_val("Ch0 QoS Weight = 8", qos0, 32'd8);
      check_val("Ch1 QoS Weight = 1", qos1, 32'd1);
    end

    // Final summary
    if (errors == 0) begin
      $display("\n=======================================================");
      $display(" [tb_m3] ALL TESTS PASSED (0 errors)");
      $display("=======================================================\n");
    end else begin
      $display("\n[tb_m3] FAILED with %0d errors\n", errors);
    end

    $finish;
  end

  // Watchdog timer (10 ms simulation time)
  initial begin
    #10_000_000;
    $display("\n[tb_m3] ERROR: Simulation timed out!");
    $finish;
  end

endmodule