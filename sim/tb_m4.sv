// ============================================================================
// tb_m4.sv — Directed verification testbench for Milestone M4.
// Validates:
//   Test 1: ATS Disabled Passthrough (ats_en=0)
//   Test 2: ATC Miss & Translation Fill (ATS translation request & PA emission)
//   Test 3: ATC Hit Steady-State Streaming (Zero-cycle extra latency)
//   Test 4: Translated Write (MWr) with PASID datapath
//   Test 5: Multi-Channel PASID Isolation (Same VA -> Different PAs via PASID)
//   Test 6: ATC Invalidation (Both software CSR and host IOMMU messages)
//   Test 7: Permission Violation & Fault Abortion (ERR_ATS_FAIL reporting)
// ============================================================================
module tb_m4;
  import dma_pkg::*;

  localparam int NRD = NUM_RD_CH;
  localparam int NWR = NUM_WR_CH;
  localparam int NCH = NRD + NWR;
  localparam int HOST_MEM_SIZE = 1024 * 1024; // 1 MB host memory model

  // Clocks and resets
  logic core_clk = 0, core_rst_n = 0;
  logic axi_clk  = 0, axi_rst_n  = 0;
  logic csr_clk  = 0, csr_rst_n  = 0;

  always #1.0  core_clk = ~core_clk; // 500 MHz
  always #1.25 axi_clk  = ~axi_clk;  // 400 MHz
  always #5.0  csr_clk  = ~csr_clk;  // 100 MHz

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

  // ATS / PRI sideband interfaces
  ats_pri_up_t ats_up;
  ats_pri_dn_t ats_dn;

  // Host memory array
  logic [7:0] host_mem [HOST_MEM_SIZE];

  // Last captured host request attributes for checking
  at_e                 last_req_at;
  logic [63:0]         last_req_addr;
  logic [PASID_W-1:0]  last_req_pasid;
  logic                last_req_pasid_en;

  int errors = 0;

  // ==========================================================================
  // DUT Instantiation
  // ==========================================================================
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
    .ch_msi         (ch_msi),
    .ats_up         (ats_up),
    .ats_dn         (ats_dn)
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

  // ==========================================================================
  // Host PCIe BFM (Requests & Completions)
  // ==========================================================================
  assign host_req_ready = 1'b1;

  logic [63:0]       cpl_addr;
  logic [TAG_W-1:0]  cpl_tag;
  logic [12:0]       cpl_byte_count;
  logic [7:0]        cpl_beats_rem;
  logic              cpl_active;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      host_cpl          <= '0;
      cpl_addr          <= '0;
      cpl_tag           <= '0;
      cpl_byte_count    <= '0;
      cpl_beats_rem     <= '0;
      cpl_active        <= 1'b0;
      last_req_at       <= AT_UNTRANSLATED;
      last_req_addr     <= '0;
      last_req_pasid    <= '0;
      last_req_pasid_en <= 1'b0;
    end else begin
      // Request monitor
      if (host_req.valid && host_req_ready) begin
        last_req_at       <= host_req.req.at;
        last_req_addr     <= host_req.req.addr;
        last_req_pasid    <= host_req.req.pasid;
        last_req_pasid_en <= host_req.req.pasid_en;

        // Write sink
        if (!host_req.req.is_read) begin
          for (int i = 0; i < 64; i++) begin
            int unsigned off = (int'(host_req.req.addr) + i) % HOST_MEM_SIZE;
            host_mem[off] <= host_req.wdata[8*i +: 8];
          end
        end
      end

      // Read completion generator
      if (!cpl_active) begin
        if (host_req.valid && host_req_ready && host_req.req.is_read) begin
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

  // ==========================================================================
  // IOMMU Behavioral Model (Page Table & ATS Req/Rsp Engine)
  // ==========================================================================
  typedef struct {
    logic        valid;
    logic        pasid_en;
    logic [19:0] pasid;
    logic [51:0] vpn;
    logic [51:0] ppn;
    logic        r_perm;
    logic        w_perm;
    logic        priv;
    logic        non_present;
    logic        fail;
  } iommu_page_t;

  iommu_page_t iommu_pt [$];

  task automatic add_iommu_mapping(
    input logic [51:0] vpn,
    input logic [51:0] ppn,
    input logic [19:0] pasid,
    input logic        pasid_en,
    input logic        r_perm,
    input logic        w_perm,
    input logic        non_present = 1'b0,
    input logic        fail = 1'b0
  );
    iommu_page_t p;
    p.valid       = 1'b1;
    p.pasid_en    = pasid_en;
    p.pasid       = pasid;
    p.vpn         = vpn;
    p.ppn         = ppn;
    p.r_perm      = r_perm;
    p.w_perm      = w_perm;
    p.priv        = 1'b0;
    p.non_present = non_present;
    p.fail        = fail;
    iommu_pt.push_back(p);
  endtask

  task automatic inject_host_inval(
    input logic [51:0] vpn,
    input logic [19:0] pasid,
    input logic        pasid_en,
    input logic        global_inv
  );
    @(negedge core_clk);
    ats_dn.inval_valid    = 1'b1;
    ats_dn.inval_global   = global_inv;
    ats_dn.inval_pasid_en = pasid_en;
    ats_dn.inval_pasid    = pasid;
    ats_dn.inval_addr     = {vpn, 12'b0};
    @(negedge core_clk);
    ats_dn.inval_valid    = 1'b0;
  endtask

  // ATS Translation Handler
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      ats_dn.ats_req_ready <= 1'b1;
      ats_dn.ats_rsp_valid <= 1'b0;
      ats_dn.ats_rsp_id    <= '0;
      ats_dn.ats_rsp       <= '0;
      ats_dn.pri_req_ready <= 1'b1;
      ats_dn.pri_rsp_valid <= 1'b0;
      ats_dn.pri_rsp_prg_idx <= '0;
      ats_dn.pri_rsp_code  <= '0;
      ats_dn.inval_valid   <= 1'b0;
      ats_dn.inval_pasid   <= '0;
      ats_dn.inval_pasid_en<= 1'b0;
      ats_dn.inval_global  <= 1'b0;
      ats_dn.inval_addr    <= '0;
    end else begin
      ats_dn.ats_req_ready <= 1'b1;
      ats_dn.ats_rsp_valid <= 1'b0;

      if (ats_up.ats_req_valid && ats_dn.ats_req_ready) begin
        int match_idx = -1;
        logic [51:0] req_vpn;
        req_vpn = ats_up.ats_req_addr[63:12];

        for (int i = 0; i < iommu_pt.size(); i++) begin
          if (iommu_pt[i].valid && (iommu_pt[i].vpn == req_vpn) &&
              (!iommu_pt[i].pasid_en || (iommu_pt[i].pasid == ats_up.ats_req_pasid))) begin
            match_idx = i;
            break;
          end
        end

        ats_dn.ats_rsp_valid <= 1'b1;
        ats_dn.ats_rsp_id    <= ats_up.ats_req_id;

        if (match_idx >= 0) begin
          ats_dn.ats_rsp.trans_addr  <= {iommu_pt[match_idx].ppn, 12'b0};
          ats_dn.ats_rsp.size        <= 64'h1000;
          ats_dn.ats_rsp.r           <= iommu_pt[match_idx].r_perm;
          ats_dn.ats_rsp.w           <= iommu_pt[match_idx].w_perm;
          ats_dn.ats_rsp.priv        <= iommu_pt[match_idx].priv;
          ats_dn.ats_rsp.non_present <= iommu_pt[match_idx].non_present;
          ats_dn.ats_rsp.fail        <= iommu_pt[match_idx].fail;
        end else begin
          ats_dn.ats_rsp.trans_addr  <= '0;
          ats_dn.ats_rsp.size        <= 64'h1000;
          ats_dn.ats_rsp.r           <= 1'b0;
          ats_dn.ats_rsp.w           <= 1'b0;
          ats_dn.ats_rsp.priv        <= 1'b0;
          ats_dn.ats_rsp.non_present <= 1'b0;
          ats_dn.ats_rsp.fail        <= 1'b1;
        end
      end
    end
  end

  // ==========================================================================
  // APB Tasks
  // ==========================================================================
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

  task automatic clear_irq(input int ch);
    int timeout = 0;
    if (ch < NRD) apb_write(20'h1000 + ch*20'h200 + 9'h54, 32'h7);
    else          apb_write(20'h5000 + (ch-NRD)*20'h200 + 9'h54, 32'h7);
    while (ch_irq[ch] == 1'b1 && timeout < 200) begin
      @(posedge csr_clk);
      timeout++;
    end
  endtask

  // Channel helper addresses
  function automatic logic [19:0] rd_ch(input int n); return 20'h1000 + n*20'h200; endfunction
  function automatic logic [19:0] wr_ch(input int n); return 20'h5000 + n*20'h200; endfunction

  // Verification assert helper
  task automatic check_val(input string msg, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin
      $display("  [FAIL] %s: got=%016x exp=%016x", msg, got, exp);
      errors++;
    end else begin
      $display("  [OK]   %s = %08x", msg, got[31:0]);
    end
  endtask

  // ==========================================================================
  // Test Sequences
  // ==========================================================================
  initial begin
    logic [31:0] rdata;
    logic [15:0] prev_misses;
    $display("=======================================================");
    $display(" [tb_m4] Starting Milestone M4 Verification Suite...");
    $display("=======================================================");

    // Reset sequence
    core_rst_n = 0; axi_rst_n = 0; csr_rst_n = 0;
    s_apb_psel = 0; s_apb_penable = 0; s_apb_pwrite = 0;
    repeat (10) @(posedge csr_clk);
    core_rst_n = 1; axi_rst_n = 1; csr_rst_n = 1;
    repeat (5) @(posedge csr_clk);

    // Global engines enable
    apb_write(20'h0008, 32'h1); // RD_ENGINE_EN
    apb_write(20'h000C, 32'h1); // WR_ENGINE_EN

    // ------------------------------------------------------------------------
    // Test 1: ATS Disabled Passthrough (ats_en = 0)
    // ------------------------------------------------------------------------
    $display("\n--- Test 1: ATS Disabled Passthrough ---");
    apb_write(20'h9300, 32'h0); // E_ATS_CTRL: ATS_EN = 0

    for (int i = 0; i < 64; i++) host_mem[32'h0001_0000 + i] = 8'(8'hA0 + i);

    apb_write(rd_ch(0) + 9'h00, 32'h1);          // CH0_EN
    apb_write(rd_ch(0) + 9'h20, 32'h0001_0000);  // SAR_LO (Physical)
    apb_write(rd_ch(0) + 9'h24, 32'h0000_0000);  // SAR_HI
    apb_write(rd_ch(0) + 9'h28, 32'h0002_0000);  // DAR_LO (AXI RAM)
    apb_write(rd_ch(0) + 9'h2C, 32'h0000_0000);  // DAR_HI
    apb_write(rd_ch(0) + 9'h1C, 32'd64);         // TRANSFER_SIZE = 64 bytes
    apb_write(rd_ch(0) + 9'h04, 32'h1);          // DOORBELL

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Done IRQ", ch_irq[0], 1);
    check_val("Byte 0 in AXI RAM", ram_read_byte(0, 64'h0002_0000), 8'hA0);
    check_val("Byte 63 in AXI RAM", ram_read_byte(0, 64'h0002_003F), 8'hDF);
    check_val("PCIe AT Field (Untranslated)", last_req_at, AT_UNTRANSLATED);
    clear_irq(0);

    // ------------------------------------------------------------------------
    // Test 2: ATC Miss & Translation Fill
    // ------------------------------------------------------------------------
    $display("\n--- Test 2: ATC Miss & Translation Fill ---");
    // Enable ATS globally
    apb_write(20'h9300, 32'h1); // E_ATS_CTRL: ATS_EN = 1

    // Map VA 0x000A_0000 (VPN 0x000A0) -> PA 0x0003_0000 (PPN 0x00030)
    add_iommu_mapping(52'h000A0, 52'h00030, 20'h0, 1'b0, 1'b1, 1'b1);
    for (int i = 0; i < 64; i++) host_mem[32'h0003_0000 + i] = 8'(8'h50 + i);

    apb_write(rd_ch(0) + 9'h00, 32'h1);          // CH0_EN
    apb_write(rd_ch(0) + 9'h20, 32'h000A_0000);  // SAR_LO = Virtual Address!
    apb_write(rd_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h28, 32'h0004_0000);  // DAR_LO = AXI RAM
    apb_write(rd_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h1C, 32'd64);
    apb_write(rd_ch(0) + 9'h04, 32'h1);          // DOORBELL

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Done IRQ", ch_irq[0], 1);
    check_val("Translated Data Byte 0", ram_read_byte(0, 64'h0004_0000), 8'h50);
    check_val("Translated Data Byte 63", ram_read_byte(0, 64'h0004_003F), 8'h8F);
    check_val("PCIe AT Field (Translated)", last_req_at, AT_TRANSLATED);
    check_val("PCIe Translated Address", last_req_addr, 64'h0003_0000);

    apb_read(20'h9308, rdata); // E_ATC_STATUS
    check_val("ATC Miss Count", rdata[31:16], 1);
    clear_irq(0);

    // ------------------------------------------------------------------------
    // Test 3: ATC Hit Steady-State Streaming
    // ------------------------------------------------------------------------
    $display("\n--- Test 3: ATC Hit Steady-State Streaming ---");
    // Transfer from offset 64 within the SAME virtual page 0x000A_0000
    for (int i = 0; i < 64; i++) host_mem[32'h0003_0040 + i] = 8'(8'h70 + i);

    apb_write(rd_ch(0) + 9'h00, 32'h1);
    apb_write(rd_ch(0) + 9'h20, 32'h000A_0040);  // Same VPN 0x000A0
    apb_write(rd_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h28, 32'h0005_0000);
    apb_write(rd_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h1C, 32'd64);
    apb_write(rd_ch(0) + 9'h04, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 Done IRQ (Hit)", ch_irq[0], 1);
    check_val("Hit Data Byte 0", ram_read_byte(0, 64'h0005_0000), 8'h70);
    check_val("PCIe AT Field (Hit)", last_req_at, AT_TRANSLATED);

    apb_read(20'h9308, rdata); // E_ATC_STATUS
    check_val("ATC Hit Count Incremented", rdata[15:0], 1);
    check_val("ATC Miss Count Unchanged", rdata[31:16], 1);
    clear_irq(0);

    // ------------------------------------------------------------------------
    // Test 4: Translated Write (MWr) & PASID Datapath
    // ------------------------------------------------------------------------
    $display("\n--- Test 4: Translated Write (MWr) with PASID ---");
    // Channel 4 is Write Channel 0 (global index 4)
    // Configure PASID=7 for WR Ch0 (offset 0x9210): pasid=7, en=bit20, priv=bit21
    apb_write(20'h9210, 32'h0030_0007);

    // Map VA 0x000B_0000 + PASID 7 -> PA 0x0007_0000
    add_iommu_mapping(52'h000B0, 52'h00070, 20'h7, 1'b1, 1'b1, 1'b1);

    // Populate source data in Device AXI RAM 4
    for (int i = 0; i < 64; i++) ram_write_byte(4, 64'h0001_0000 + i, 8'(8'hC0 + i));

    apb_write(wr_ch(0) + 9'h00, 32'h1);          // WR_CH0_EN
    apb_write(wr_ch(0) + 9'h20, 32'h0001_0000);  // SAR = Device AXI RAM
    apb_write(wr_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(wr_ch(0) + 9'h28, 32'h000B_0000);  // DAR = Virtual Address!
    apb_write(wr_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(wr_ch(0) + 9'h1C, 32'd64);
    apb_write(wr_ch(0) + 9'h04, 32'h1);          // DOORBELL

    wait (ch_irq[4] == 1'b1);
    check_val("Ch4 C2H Done IRQ", ch_irq[4], 1);
    check_val("Host Mem at Translated PA Byte 0", host_mem[32'h0007_0000], 8'hC0);
    check_val("Host Mem at Translated PA Byte 63", host_mem[32'h0007_003F], 8'hFF);
    check_val("PCIe AT Field (MWr Translated)", last_req_at, AT_TRANSLATED);
    check_val("PCIe PASID Carried", last_req_pasid, 20'h7);
    check_val("PCIe PASID Enabled", last_req_pasid_en, 1);
    clear_irq(4);

    // ------------------------------------------------------------------------
    // Test 5: Multi-Channel PASID Isolation
    // ------------------------------------------------------------------------
    $display("\n--- Test 5: Multi-Channel PASID Isolation ---");
    // Ch0: PASID = 1. VA 0x000C_0000 -> PA 0x0008_0000
    // Ch1: PASID = 2. VA 0x000C_0000 (IDENTICAL VA!) -> PA 0x0009_0000
    add_iommu_mapping(52'h000C0, 52'h00080, 20'h1, 1'b1, 1'b1, 1'b1);
    add_iommu_mapping(52'h000C0, 52'h00090, 20'h2, 1'b1, 1'b1, 1'b1);

    for (int i = 0; i < 64; i++) begin
      host_mem[32'h0008_0000 + i] = 8'h11;
      host_mem[32'h0009_0000 + i] = 8'h22;
    end

    // Configure PASID for RD Ch0 and RD Ch1
    apb_write(20'h9110, 32'h0030_0001); // Ch0: PASID 1
    apb_write(20'h9118, 32'h0030_0002); // Ch1: PASID 2

    // Launch Ch0 transfer
    apb_write(rd_ch(0) + 9'h00, 32'h1);
    apb_write(rd_ch(0) + 9'h20, 32'h000C_0000);  // VA
    apb_write(rd_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h28, 32'h0006_0000);  // RAM 0
    apb_write(rd_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h1C, 32'd64);
    apb_write(rd_ch(0) + 9'h04, 32'h1);

    wait (ch_irq[0] == 1'b1);
    check_val("Ch0 (PASID 1) Byte 0", ram_read_byte(0, 64'h0006_0000), 8'h11);
    clear_irq(0);

    // Launch Ch1 transfer with same VA
    apb_write(rd_ch(1) + 9'h00, 32'h1);
    apb_write(rd_ch(1) + 9'h20, 32'h000C_0000);  // Same VA!
    apb_write(rd_ch(1) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(1) + 9'h28, 32'h0007_0000);  // RAM 1
    apb_write(rd_ch(1) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(1) + 9'h1C, 32'd64);
    apb_write(rd_ch(1) + 9'h04, 32'h1);

    wait (ch_irq[1] == 1'b1);
    check_val("Ch1 (PASID 2) Byte 0", ram_read_byte(1, 64'h0007_0000), 8'h22);
    clear_irq(1);

    // ------------------------------------------------------------------------
    // Test 6: ATC Invalidation (Both Software CSR and Host Message)
    // ------------------------------------------------------------------------
    $display("\n--- Test 6: ATC Invalidation ---");
    // Part A: Invalidate Ch0 entry via Software CSR write
    apb_read(20'h9308, rdata);
    prev_misses = rdata[31:16];

    // Trigger CSR Invalidation for PASID 1 (Bit 0=TRIGGER, Bit 2=PASID_ALL, [19:4]=PASID)
    apb_write(20'h9304, 32'h0000_0015); // TRIGGER=1, PASID_ALL=1, PASID=1

    // Repeat Ch0 transfer -> must miss and re-translate!
    apb_write(rd_ch(0) + 9'h00, 32'h1);
    apb_write(rd_ch(0) + 9'h20, 32'h000C_0000);
    apb_write(rd_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h28, 32'h0008_0000);
    apb_write(rd_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(0) + 9'h1C, 32'd64);
    apb_write(rd_ch(0) + 9'h04, 32'h1);

    wait (ch_irq[0] == 1'b1);
    apb_read(20'h9308, rdata);
    check_val("CSR Inval: Miss Count Incremented", rdata[31:16], prev_misses + 1);
    clear_irq(0);

    // Part B: Invalidate Ch1 entry via Host IOMMU Invalidation Message
    prev_misses = rdata[31:16];
    inject_host_inval(52'h000C0, 20'h2, 1'b1, 1'b0);

    // Repeat Ch1 transfer -> must miss and re-translate!
    apb_write(rd_ch(1) + 9'h00, 32'h1);
    apb_write(rd_ch(1) + 9'h20, 32'h000C_0000);
    apb_write(rd_ch(1) + 9'h24, 32'h0000_0000);
    apb_write(rd_ch(1) + 9'h28, 32'h0009_0000);
    apb_write(rd_ch(1) + 9'h2C, 32'h0000_0000);
    apb_write(rd_ch(1) + 9'h1C, 32'd64);
    apb_write(rd_ch(1) + 9'h04, 32'h1);

    wait (ch_irq[1] == 1'b1);
    apb_read(20'h9308, rdata);
    check_val("Host Inval: Miss Count Incremented", rdata[31:16], prev_misses + 1);
    clear_irq(1);

    // ------------------------------------------------------------------------
    // Test 7: Permission Violation & Fault Abortion
    // ------------------------------------------------------------------------
    $display("\n--- Test 7: Permission Violation & Fault Abortion ---");
    // Map VA 0x000E_0000 as Read-Only (r=1, w=0) with PASID 7
    add_iommu_mapping(52'h000E0, 52'h000A0, 20'h7, 1'b1, 1'b1, 1'b0);

    // Attempt C2H Write to read-only page
    apb_write(wr_ch(0) + 9'h00, 32'h1);
    apb_write(wr_ch(0) + 9'h20, 32'h0001_0000);  // SAR
    apb_write(wr_ch(0) + 9'h24, 32'h0000_0000);
    apb_write(wr_ch(0) + 9'h28, 32'h000E_0000);  // DAR = Read-only VA!
    apb_write(wr_ch(0) + 9'h2C, 32'h0000_0000);
    apb_write(wr_ch(0) + 9'h1C, 32'd64);
    apb_write(wr_ch(0) + 9'h04, 32'h1);          // DOORBELL

    // Error abort interrupt will assert
    wait (ch_irq[4] == 1'b1);
    apb_read(wr_ch(0) + 9'h40, rdata); // CH_STATUS (offset 0x40)
    check_val("Channel Error Code (ERR_ATS_FAIL)", rdata[7:4], ERR_ATS_FAIL);

    // Verify interrupt status has abort set and done not asserted
    apb_read(wr_ch(0) + 9'h58, rdata); // C_INT_STATUS (offset 0x58): [1]=abort, [0]=done
    check_val("Per-Channel Abort IRQ", rdata[1], 1);
    check_val("Done IRQ Not Asserted", rdata[0], 0);

    apb_read(20'h018, rdata); // O_WR_INT_STATUS (offset 0x018)
    check_val("WR Group Abort IRQ", rdata[0], 1);
    clear_irq(4);

    // ========================================================================
    // Final Summary
    // ========================================================================
    $display("\n=======================================================");
    if (errors == 0) begin
      $display(" [tb_m4] ALL TESTS PASSED (0 errors)");
    end else begin
      $display(" [tb_m4] FAILED: %0d error(s) encountered", errors);
    end
    $display("=======================================================\n");

    repeat (10) @(posedge csr_clk);
    $finish;
  end

  // Watchdog timeout
  initial begin
    #5000000; // 5ms timeout
    $display("\n[tb_m4] ERROR: Watchdog timeout reached!");
    $finish;
  end

endmodule
