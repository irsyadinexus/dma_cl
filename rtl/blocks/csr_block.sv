// ============================================================================
// csr_block.sv — HDMA-native register file (BAR0), APB slave on csr_clk.
// Implements docs/m1_register_map.md: common region + per-channel RD/WR blocks
// + vendor extension region. Zero-wait APB, byte-strobed writes, W1C interrupts,
// WO doorbell pulse, RO status/xferred read-back, interrupt aggregation.
// Channel index convention: 0..NUM_RD_CH-1 = Read/H2C, then NUM_WR_CH Write/C2H.
// ⚑RECONCILE offsets/bitfields vs DWC HDMA databook + dw-hdma-v0-regs.h.
// ============================================================================
module csr_block #(
  parameter int NUM_RD_CH = 4,
  parameter int NUM_WR_CH = 4,
  parameter int APB_AW    = 20
) (
  input  logic              csr_clk,
  input  logic              csr_rst_n,
  // APB slave
  input  logic [APB_AW-1:0] paddr,
  input  logic              psel,
  input  logic              penable,
  input  logic              pwrite,
  input  logic [31:0]       pwdata,
  input  logic [3:0]        pstrb,
  output logic [31:0]       prdata,
  output logic              pready,
  output logic              pslverr,
  // control / status / interrupts (per channel)
  output dma_pkg::ch_ctrl_t ch_ctrl [NUM_RD_CH + NUM_WR_CH],
  input  dma_pkg::ch_stat_t ch_stat [NUM_RD_CH + NUM_WR_CH],
  output dma_pkg::ch_msi_t  ch_msi  [NUM_RD_CH + NUM_WR_CH],
  output logic              ch_irq  [NUM_RD_CH + NUM_WR_CH],
  // engine enables / soft reset
  output logic              rd_engine_en,
  output logic              wr_engine_en,
  output logic              soft_reset,
  // ATS & ATC subsystem controls
  output logic [31:0]       ats_ctrl_o,
  output logic [31:0]       atc_inval_o,
  output logic              atc_inval_pulse_o,
  input  dma_pkg::atc_status_t atc_stat_i
);
  import dma_pkg::*;

  localparam int NUM_CH = NUM_RD_CH + NUM_WR_CH;

  // ---- region bases (byte offsets) ⚑RECONCILE ----
  localparam logic [APB_AW-1:0] COMMON_BASE = 'h0_0000;
  localparam logic [APB_AW-1:0] RD_BASE     = 'h0_1000;
  localparam logic [APB_AW-1:0] WR_BASE     = 'h0_5000;
  localparam logic [APB_AW-1:0] EXT_BASE    = 'h0_9000;
  localparam int                CH_STRIDE   = 'h200;
  localparam logic [APB_AW-1:0] RD_END = RD_BASE + APB_AW'(NUM_RD_CH*CH_STRIDE);
  localparam logic [APB_AW-1:0] WR_END = WR_BASE + APB_AW'(NUM_WR_CH*CH_STRIDE);

  // ---- common-region offsets ----
  localparam logic [11:0] O_ID_VERSION    = 12'h000;
  localparam logic [11:0] O_CAPABILITIES  = 12'h004;
  localparam logic [11:0] O_RD_ENGINE_EN  = 12'h008;
  localparam logic [11:0] O_WR_ENGINE_EN  = 12'h00C;
  localparam logic [11:0] O_RD_INT_STATUS = 12'h010;
  localparam logic [11:0] O_RD_INT_CLEAR  = 12'h014;
  localparam logic [11:0] O_WR_INT_STATUS = 12'h018;
  localparam logic [11:0] O_WR_INT_CLEAR  = 12'h01C;
  localparam logic [11:0] O_RD_ERR_STATUS = 12'h020;
  localparam logic [11:0] O_WR_ERR_STATUS = 12'h024;
  localparam logic [11:0] O_SOFT_RESET    = 12'h028;

  // ---- per-channel offsets ----
  localparam logic [8:0] C_CH_EN         = 9'h00;
  localparam logic [8:0] C_DOORBELL      = 9'h04;
  localparam logic [8:0] C_PREFETCH      = 9'h08;
  localparam logic [8:0] C_HANDSHAKE     = 9'h0C;
  localparam logic [8:0] C_LLP_LO        = 9'h10;
  localparam logic [8:0] C_LLP_HI        = 9'h14;
  localparam logic [8:0] C_CYCLE_SYNC    = 9'h18;
  localparam logic [8:0] C_TRANSFER_SIZE = 9'h1C;
  localparam logic [8:0] C_SAR_LO        = 9'h20;
  localparam logic [8:0] C_SAR_HI        = 9'h24;
  localparam logic [8:0] C_DAR_LO        = 9'h28;
  localparam logic [8:0] C_DAR_HI        = 9'h2C;
  localparam logic [8:0] C_WATERMARK_EN  = 9'h30;
  localparam logic [8:0] C_CONTROL1      = 9'h34;
  localparam logic [8:0] C_FUNC_NUM      = 9'h38;
  localparam logic [8:0] C_QOS           = 9'h3C;
  localparam logic [8:0] C_CH_STATUS     = 9'h40;
  localparam logic [8:0] C_XFERRED_LO    = 9'h44;
  localparam logic [8:0] C_XFERRED_HI    = 9'h48;
  localparam logic [8:0] C_INT_SETUP     = 9'h50;
  localparam logic [8:0] C_INT_CLEAR     = 9'h54;
  localparam logic [8:0] C_INT_STATUS    = 9'h58;
  localparam logic [8:0] C_MSI_STOP_LO   = 9'h60;
  localparam logic [8:0] C_MSI_STOP_HI   = 9'h64;
  localparam logic [8:0] C_MSI_WMK_LO    = 9'h68;
  localparam logic [8:0] C_MSI_WMK_HI    = 9'h6C;
  localparam logic [8:0] C_MSI_ABORT_LO  = 9'h70;
  localparam logic [8:0] C_MSI_ABORT_HI  = 9'h74;
  localparam logic [8:0] C_MSI_MSGD      = 9'h78;

  // ---- extension-region offsets ----
  localparam logic [11:0] E_EXT_ID        = 12'h000;
  localparam logic [11:0] E_EXT_CAP       = 12'h004;
  localparam logic [11:0] E_RD_STREAM     = 12'h010; // + n*4
  localparam logic [11:0] E_WR_STREAM     = 12'h090; // + n*4
  localparam logic [11:0] E_RD_PASID      = 12'h110; // + n*8
  localparam logic [11:0] E_WR_PASID      = 12'h210; // + n*8
  localparam logic [11:0] E_ATS_CTRL      = 12'h300;
  localparam logic [11:0] E_ATC_INVAL     = 12'h304;
  localparam logic [11:0] E_ATC_STATUS    = 12'h308;
  localparam logic [11:0] E_PRI_CTRL      = 12'h310;
  localparam logic [11:0] E_PRI_STATUS    = 12'h314;
  localparam logic [11:0] E_P2P_CTRL      = 12'h320;
  // sized range ends (width-clean comparisons)
  localparam logic [11:0] E_RD_STREAM_END = E_RD_STREAM + 12'(NUM_RD_CH*4);
  localparam logic [11:0] E_WR_STREAM_END = E_WR_STREAM + 12'(NUM_WR_CH*4);
  localparam logic [11:0] E_RD_PASID_END  = E_RD_PASID  + 12'(NUM_RD_CH*8);
  localparam logic [11:0] E_WR_PASID_END  = E_WR_PASID  + 12'(NUM_WR_CH*8);

  // ---- RO identity constants ----
  localparam logic [31:0] ID_VERSION_VAL = 32'h444D_0100; // "DM" + v1.0 ⚑RECONCILE
  localparam logic [31:0] EXT_ID_VAL     = 32'h4558_5401; // "EXT" + v1
  // [31:16]=DATA_WIDTH [15:12]=NUM_WR_CH [11:8]=NUM_RD_CH [5:0]=caps
  // (built with shift-OR to avoid size-cast syntax for tool portability)
  localparam logic [31:0] CAP_VAL =
      (32'(AXIS_EN)<<0) | (32'(ATS_EN)<<1) | (32'(PASID_EN)<<2) | (32'(PRI_EN)<<3) |
      (32'(P2P_EN)<<4)  | (32'(ECC_EN)<<5) |
      (NUM_RD_CH<<8) | (NUM_WR_CH<<12) | (DATA_WIDTH<<16);

  // ==========================================================================
  // Register storage
  // ==========================================================================
  logic        rd_engine_en_r, wr_engine_en_r, soft_reset_r;
  logic [31:0] ats_ctrl_r, atc_inval_r, pri_ctrl_r, p2p_ctrl_r;
  logic        atc_inval_pulse_r;

  logic        ch_en_r        [NUM_CH];
  logic [31:0] prefetch_r     [NUM_CH];
  logic [31:0] handshake_r    [NUM_CH];
  logic [31:0] llp_lo_r       [NUM_CH];
  logic [31:0] llp_hi_r       [NUM_CH];
  logic [31:0] cycle_sync_r   [NUM_CH];
  logic [31:0] xfer_size_r    [NUM_CH];
  logic [31:0] sar_lo_r       [NUM_CH];
  logic [31:0] sar_hi_r       [NUM_CH];
  logic [31:0] dar_lo_r       [NUM_CH];
  logic [31:0] dar_hi_r       [NUM_CH];
  logic [31:0] watermark_en_r [NUM_CH];
  logic [31:0] control1_r     [NUM_CH];
  logic [31:0] func_num_r     [NUM_CH];
  logic [31:0] qos_r          [NUM_CH];
  logic [31:0] int_setup_r    [NUM_CH];
  logic [31:0] msi_stop_lo_r  [NUM_CH];
  logic [31:0] msi_stop_hi_r  [NUM_CH];
  logic [31:0] msi_wmk_lo_r   [NUM_CH];
  logic [31:0] msi_wmk_hi_r   [NUM_CH];
  logic [31:0] msi_abort_lo_r [NUM_CH];
  logic [31:0] msi_abort_hi_r [NUM_CH];
  logic [31:0] msi_msgd_r     [NUM_CH];
  logic [31:0] stream_ctrl_r  [NUM_CH];
  logic [31:0] pasid_r        [NUM_CH];

  // interrupt pending (W1C)
  logic        done_pend  [NUM_CH];
  logic        abort_pend [NUM_CH];
  logic        wmk_pend   [NUM_CH];

  // doorbell 1-cycle pulse
  logic        db_pulse   [NUM_CH];

  // ==========================================================================
  // APB access decode (zero wait states)
  // ==========================================================================
  wire access   = psel & penable;   // transfer completes this cycle (pready=1)
  wire do_write = access &  pwrite;
  wire do_read  = access & ~pwrite;

  // region decode
  wire in_common = (paddr[APB_AW-1:12] == COMMON_BASE[APB_AW-1:12]);
  wire in_ext    = (paddr[APB_AW-1:12] == EXT_BASE[APB_AW-1:12]);
  wire in_rd     = (paddr >= RD_BASE) && (paddr < RD_END);
  wire in_wr     = (paddr >= WR_BASE) && (paddr < WR_END);
  wire        ch_hit = in_rd | in_wr;

  // channel decode via explicit slices (stride 0x200 => low 9 bits = reg offset)
  localparam int CHW = (NUM_CH > 1) ? $clog2(NUM_CH) : 1; // channel-index width
  wire [APB_AW-1:0] off_rd  = paddr - RD_BASE;
  wire [APB_AW-1:0] off_wr  = paddr - WR_BASE;
  wire [8:0]      coff = in_rd ? off_rd[8:0] : off_wr[8:0]; // reg offset in block
  wire [CHW-1:0]  gch  = in_wr ? (CHW'(NUM_RD_CH) + CHW'(off_wr[APB_AW-1:9]))
                               : CHW'(off_rd[APB_AW-1:9]);   // global channel idx
  wire [11:0]     eoff = 12'(paddr - EXT_BASE);              // ext-region offset

  // extension-region channel decode (sized index) — one place for stream/pasid
  logic          ext_stream, ext_pasid;
  logic [CHW-1:0] ext_ch;
  always_comb begin
    ext_stream = 1'b0; ext_pasid = 1'b0; ext_ch = '0;
    if      ((eoff >= E_RD_STREAM) && (eoff < E_RD_STREAM_END)) begin
      ext_stream = 1'b1; ext_ch = CHW'((eoff - E_RD_STREAM) >> 2);
    end else if ((eoff >= E_WR_STREAM) && (eoff < E_WR_STREAM_END)) begin
      ext_stream = 1'b1; ext_ch = CHW'(NUM_RD_CH) + CHW'((eoff - E_WR_STREAM) >> 2);
    end else if ((eoff >= E_RD_PASID) && (eoff < E_RD_PASID_END) && (eoff[2:0] == 3'h0)) begin
      ext_pasid = 1'b1; ext_ch = CHW'((eoff - E_RD_PASID) >> 3);
    end else if ((eoff >= E_WR_PASID) && (eoff < E_WR_PASID_END) && (eoff[2:0] == 3'h0)) begin
      ext_pasid = 1'b1; ext_ch = CHW'(NUM_RD_CH) + CHW'((eoff - E_WR_PASID) >> 3);
    end
  end

  // helper: apply byte strobes
  function automatic logic [31:0] apply_be(input logic [31:0] cur,
                                           input logic [31:0] wr,
                                           input logic [3:0]  be);
    logic [31:0] r;
    for (int i = 0; i < 4; i++)
      r[8*i +: 8] = be[i] ? wr[8*i +: 8] : cur[8*i +: 8];
    return r;
  endfunction

  // ==========================================================================
  // Write path + interrupt capture + doorbell pulse (sequential)
  // ==========================================================================
  always_ff @(posedge csr_clk or negedge csr_rst_n) begin
    if (!csr_rst_n) begin
      rd_engine_en_r <= 1'b0; wr_engine_en_r <= 1'b0; soft_reset_r <= 1'b0;
      ats_ctrl_r <= '0; atc_inval_r <= '0; pri_ctrl_r <= '0; p2p_ctrl_r <= '0;
      atc_inval_pulse_r <= 1'b0;
      for (int g = 0; g < NUM_CH; g++) begin
        ch_en_r[g] <= 1'b0; prefetch_r[g] <= '0; handshake_r[g] <= '0;
        llp_lo_r[g] <= '0; llp_hi_r[g] <= '0; cycle_sync_r[g] <= '0;
        xfer_size_r[g] <= '0; sar_lo_r[g] <= '0; sar_hi_r[g] <= '0;
        dar_lo_r[g] <= '0; dar_hi_r[g] <= '0; watermark_en_r[g] <= '0;
        control1_r[g] <= '0; func_num_r[g] <= '0; qos_r[g] <= '0;
        int_setup_r[g] <= '0;
        msi_stop_lo_r[g] <= '0; msi_stop_hi_r[g] <= '0;
        msi_wmk_lo_r[g] <= '0; msi_wmk_hi_r[g] <= '0;
        msi_abort_lo_r[g] <= '0; msi_abort_hi_r[g] <= '0; msi_msgd_r[g] <= '0;
        stream_ctrl_r[g] <= '0; pasid_r[g] <= '0;
        done_pend[g] <= 1'b0; abort_pend[g] <= 1'b0; wmk_pend[g] <= 1'b0;
        db_pulse[g] <= 1'b0;
      end
    end else begin
      // doorbell pulse defaults low; soft_reset self-clears after one cycle
      for (int g = 0; g < NUM_CH; g++) db_pulse[g] <= 1'b0;
      soft_reset_r <= 1'b0;
      atc_inval_pulse_r <= do_write && in_ext && (eoff == E_ATC_INVAL) && pwdata[0];

      // ---- common region writes ----
      if (do_write && in_common) begin
        case (paddr[11:0])
          O_RD_ENGINE_EN: if (pstrb[0]) rd_engine_en_r <= pwdata[0];
          O_WR_ENGINE_EN: if (pstrb[0]) wr_engine_en_r <= pwdata[0];
          O_SOFT_RESET:   if (pstrb[0]) soft_reset_r   <= pwdata[0];
          default: ; // RO / status handled below or ignored
        endcase
      end

      // ---- per-channel writes ----
      if (do_write && ch_hit) begin
        case (coff)
          C_CH_EN:         if (pstrb[0]) ch_en_r[gch]     <= pwdata[0];
          C_DOORBELL:      if (pstrb[0]) db_pulse[gch]    <= pwdata[0]; // WO pulse
          C_PREFETCH:      prefetch_r[gch]     <= apply_be(prefetch_r[gch], pwdata, pstrb);
          C_HANDSHAKE:     handshake_r[gch]    <= apply_be(handshake_r[gch], pwdata, pstrb);
          C_LLP_LO:        llp_lo_r[gch]       <= apply_be(llp_lo_r[gch], pwdata, pstrb);
          C_LLP_HI:        llp_hi_r[gch]       <= apply_be(llp_hi_r[gch], pwdata, pstrb);
          C_CYCLE_SYNC:    cycle_sync_r[gch]   <= apply_be(cycle_sync_r[gch], pwdata, pstrb);
          C_TRANSFER_SIZE: xfer_size_r[gch]    <= apply_be(xfer_size_r[gch], pwdata, pstrb);
          C_SAR_LO:        sar_lo_r[gch]       <= apply_be(sar_lo_r[gch], pwdata, pstrb);
          C_SAR_HI:        sar_hi_r[gch]       <= apply_be(sar_hi_r[gch], pwdata, pstrb);
          C_DAR_LO:        dar_lo_r[gch]       <= apply_be(dar_lo_r[gch], pwdata, pstrb);
          C_DAR_HI:        dar_hi_r[gch]       <= apply_be(dar_hi_r[gch], pwdata, pstrb);
          C_WATERMARK_EN:  watermark_en_r[gch] <= apply_be(watermark_en_r[gch], pwdata, pstrb);
          C_CONTROL1:      control1_r[gch]     <= apply_be(control1_r[gch], pwdata, pstrb);
          C_FUNC_NUM:      func_num_r[gch]     <= apply_be(func_num_r[gch], pwdata, pstrb);
          C_QOS:           qos_r[gch]          <= apply_be(qos_r[gch], pwdata, pstrb);
          C_INT_SETUP:     int_setup_r[gch]    <= apply_be(int_setup_r[gch], pwdata, pstrb);
          C_MSI_STOP_LO:   msi_stop_lo_r[gch]  <= apply_be(msi_stop_lo_r[gch], pwdata, pstrb);
          C_MSI_STOP_HI:   msi_stop_hi_r[gch]  <= apply_be(msi_stop_hi_r[gch], pwdata, pstrb);
          C_MSI_WMK_LO:    msi_wmk_lo_r[gch]   <= apply_be(msi_wmk_lo_r[gch], pwdata, pstrb);
          C_MSI_WMK_HI:    msi_wmk_hi_r[gch]   <= apply_be(msi_wmk_hi_r[gch], pwdata, pstrb);
          C_MSI_ABORT_LO:  msi_abort_lo_r[gch] <= apply_be(msi_abort_lo_r[gch], pwdata, pstrb);
          C_MSI_ABORT_HI:  msi_abort_hi_r[gch] <= apply_be(msi_abort_hi_r[gch], pwdata, pstrb);
          C_MSI_MSGD:      msi_msgd_r[gch]     <= apply_be(msi_msgd_r[gch], pwdata, pstrb);
          default: ; // RO (CH_STATUS/XFERRED/INT_STATUS) or W1C (handled below)
        endcase
      end

      // ---- extension region writes ----
      if (do_write && in_ext) begin
        if      (eoff == E_ATS_CTRL)  ats_ctrl_r  <= apply_be(ats_ctrl_r, pwdata, pstrb);
        else if (eoff == E_ATC_INVAL) atc_inval_r <= apply_be(atc_inval_r, pwdata, pstrb);
        else if (eoff == E_PRI_CTRL)  pri_ctrl_r  <= apply_be(pri_ctrl_r, pwdata, pstrb);
        else if (eoff == E_P2P_CTRL)  p2p_ctrl_r  <= apply_be(p2p_ctrl_r, pwdata, pstrb);
        else if (ext_stream) stream_ctrl_r[ext_ch] <= apply_be(stream_ctrl_r[ext_ch], pwdata, pstrb);
        else if (ext_pasid)  pasid_r[ext_ch]       <= apply_be(pasid_r[ext_ch], pwdata, pstrb);
      end

      // ---- interrupt capture (set on event, clear on W1C; event wins) ----
      // per-channel INT_CLEAR (bit0 done, bit1 abort, bit2 watermark) OR global
      // RD/WR_INT_CLEAR (bit n clears channel n's whole group). NUM_CH <= 32.
      for (int g = 0; g < NUM_CH; g++) begin
        // per-channel clear match for this g
        // (gch is the addressed channel; compare against loop index)
        // global-group clear pulses:
        //   RD group: g < NUM_RD_CH, pwdata bit g
        //   WR group: g >= NUM_RD_CH, pwdata bit (g-NUM_RD_CH)
        done_pend[g] <= ((( do_write && ch_hit && (gch == CHW'(g)) && (coff == C_INT_CLEAR) && pwdata[0])
                        || ( do_write && in_common && (g <  NUM_RD_CH) && (paddr[11:0]==O_RD_INT_CLEAR) && pwdata[g])
                        || ( do_write && in_common && (g >= NUM_RD_CH) && (paddr[11:0]==O_WR_INT_CLEAR) && pwdata[(g >= NUM_RD_CH) ? (g-NUM_RD_CH) : 0]))
                        ? 1'b0 : done_pend[g]) | ch_stat[g].done_irq;
        abort_pend[g] <= ((( do_write && ch_hit && (gch == CHW'(g)) && (coff == C_INT_CLEAR) && pwdata[1])
                        || ( do_write && in_common && (g <  NUM_RD_CH) && (paddr[11:0]==O_RD_INT_CLEAR) && pwdata[g])
                        || ( do_write && in_common && (g >= NUM_RD_CH) && (paddr[11:0]==O_WR_INT_CLEAR) && pwdata[(g >= NUM_RD_CH) ? (g-NUM_RD_CH) : 0]))
                        ? 1'b0 : abort_pend[g]) | ch_stat[g].abort_irq;
        wmk_pend[g] <= ((( do_write && ch_hit && (gch == CHW'(g)) && (coff == C_INT_CLEAR) && pwdata[2])
                        || ( do_write && in_common && (g <  NUM_RD_CH) && (paddr[11:0]==O_RD_INT_CLEAR) && pwdata[g])
                        || ( do_write && in_common && (g >= NUM_RD_CH) && (paddr[11:0]==O_WR_INT_CLEAR) && pwdata[(g >= NUM_RD_CH) ? (g-NUM_RD_CH) : 0]))
                        ? 1'b0 : wmk_pend[g]) | ch_stat[g].watermark_irq;
      end
    end
  end

  // ==========================================================================
  // Read mux (combinational)
  // ==========================================================================
  logic [31:0] rdata;
  always_comb begin
    rdata = 32'h0;
    if (in_common) begin
      case (paddr[11:0])
        O_ID_VERSION:    rdata = ID_VERSION_VAL;
        O_CAPABILITIES:  rdata = CAP_VAL;
        O_RD_ENGINE_EN:  rdata = {31'b0, rd_engine_en_r};
        O_WR_ENGINE_EN:  rdata = {31'b0, wr_engine_en_r};
        O_SOFT_RESET:    rdata = {31'b0, soft_reset_r};
        O_RD_INT_STATUS: for (int g = 0; g < NUM_RD_CH; g++)
                           rdata[g] = done_pend[g] | abort_pend[g] | wmk_pend[g];
        O_WR_INT_STATUS: for (int g = 0; g < NUM_WR_CH; g++)
                           rdata[g] = done_pend[NUM_RD_CH+g] | abort_pend[NUM_RD_CH+g]
                                      | wmk_pend[NUM_RD_CH+g];
        O_RD_ERR_STATUS: for (int g = 0; g < NUM_RD_CH; g++)
                           rdata[g] = (ch_stat[g].err != ERR_NONE);
        O_WR_ERR_STATUS: for (int g = 0; g < NUM_WR_CH; g++)
                           rdata[g] = (ch_stat[NUM_RD_CH+g].err != ERR_NONE);
        default: rdata = 32'h0;
      endcase
    end else if (ch_hit) begin
      case (coff)
        C_CH_EN:         rdata = {31'b0, ch_en_r[gch]};
        C_PREFETCH:      rdata = prefetch_r[gch];
        C_HANDSHAKE:     rdata = handshake_r[gch];
        C_LLP_LO:        rdata = llp_lo_r[gch];
        C_LLP_HI:        rdata = llp_hi_r[gch];
        C_CYCLE_SYNC:    rdata = cycle_sync_r[gch];
        C_TRANSFER_SIZE: rdata = xfer_size_r[gch];
        C_SAR_LO:        rdata = sar_lo_r[gch];
        C_SAR_HI:        rdata = sar_hi_r[gch];
        C_DAR_LO:        rdata = dar_lo_r[gch];
        C_DAR_HI:        rdata = dar_hi_r[gch];
        C_WATERMARK_EN:  rdata = watermark_en_r[gch];
        C_CONTROL1:      rdata = control1_r[gch];
        C_FUNC_NUM:      rdata = func_num_r[gch];
        C_QOS:           rdata = qos_r[gch];
        C_CH_STATUS:     rdata = {24'b0, ch_stat[gch].err, 1'b0, ch_stat[gch].state};
        C_XFERRED_LO:    rdata = ch_stat[gch].bytes_xferred[31:0];
        C_XFERRED_HI:    rdata = ch_stat[gch].bytes_xferred[63:32];
        C_INT_SETUP:     rdata = int_setup_r[gch];
        C_INT_STATUS:    rdata = {29'b0, wmk_pend[gch], abort_pend[gch], done_pend[gch]};
        C_MSI_STOP_LO:   rdata = msi_stop_lo_r[gch];
        C_MSI_STOP_HI:   rdata = msi_stop_hi_r[gch];
        C_MSI_WMK_LO:    rdata = msi_wmk_lo_r[gch];
        C_MSI_WMK_HI:    rdata = msi_wmk_hi_r[gch];
        C_MSI_ABORT_LO:  rdata = msi_abort_lo_r[gch];
        C_MSI_ABORT_HI:  rdata = msi_abort_hi_r[gch];
        C_MSI_MSGD:      rdata = msi_msgd_r[gch];
        default: rdata = 32'h0;
      endcase
    end else if (in_ext) begin
      if      (eoff == E_EXT_ID)     rdata = EXT_ID_VAL;
      else if (eoff == E_EXT_CAP)    rdata = CAP_VAL;
      else if (eoff == E_ATS_CTRL)   rdata = ats_ctrl_r;
      else if (eoff == E_ATC_INVAL)  rdata = atc_inval_r;
      else if (eoff == E_ATC_STATUS) rdata = {atc_stat_i.miss_count, atc_stat_i.hit_count};
      else if (eoff == E_PRI_CTRL)   rdata = pri_ctrl_r;
      else if (eoff == E_PRI_STATUS) rdata = 32'h0; // driven by PRI engine (TODO M5)
      else if (eoff == E_P2P_CTRL)   rdata = p2p_ctrl_r;
      else if (ext_stream)           rdata = stream_ctrl_r[ext_ch];
      else if (ext_pasid)            rdata = pasid_r[ext_ch];
      else rdata = 32'h0;
    end
  end

  // ---- APB handshake ----
  assign prdata  = do_read ? rdata : 32'h0;
  assign pready  = 1'b1;                                   // zero wait states
  assign pslverr = access & ~(in_common | ch_hit | in_ext); // undecoded access

  // ==========================================================================
  // Output assembly
  // ==========================================================================
  assign rd_engine_en      = rd_engine_en_r;
  assign wr_engine_en      = wr_engine_en_r;
  assign soft_reset        = soft_reset_r;
  assign ats_ctrl_o        = ats_ctrl_r;
  assign atc_inval_o       = atc_inval_r;
  assign atc_inval_pulse_o = atc_inval_pulse_r;

  always_comb begin
    for (int g = 0; g < NUM_CH; g++) begin
      ch_ctrl[g]              = '0;
      ch_ctrl[g].enable       = ch_en_r[g];
      ch_ctrl[g].lle          = control1_r[g][2];
      ch_ctrl[g].dir          = (g < NUM_RD_CH) ? CH_READ : CH_WRITE;
      ch_ctrl[g].mode         = stream_ctrl_r[g][0] ? DEV_AXI_ST : DEV_AXI_MM;
      ch_ctrl[g].llp_base     = {llp_hi_r[g], llp_lo_r[g]};
      ch_ctrl[g].sar          = {sar_hi_r[g], sar_lo_r[g]};
      ch_ctrl[g].dar          = {dar_hi_r[g], dar_lo_r[g]};
      ch_ctrl[g].xfer_size    = xfer_size_r[g];
      ch_ctrl[g].tc           = control1_r[g][12:10];
      ch_ctrl[g].ro           = control1_r[g][8];
      ch_ctrl[g].ns           = control1_r[g][9];
      ch_ctrl[g].lie          = control1_r[g][4];
      ch_ctrl[g].rie          = control1_r[g][5];
      ch_ctrl[g].prefetch     = prefetch_r[g][15:0];
      ch_ctrl[g].watermark_en = watermark_en_r[g][0];
      ch_ctrl[g].func_num     = func_num_r[g][7:0];
      ch_ctrl[g].stream_user  = stream_ctrl_r[g][31:16];
      ch_ctrl[g].pasid        = pasid_r[g][PASID_W-1:0];
      ch_ctrl[g].pasid_en     = pasid_r[g][20];
      ch_ctrl[g].pasid_priv   = pasid_r[g][21];
      ch_ctrl[g].qos          = qos_r[g][3:0];
      ch_ctrl[g].doorbell     = db_pulse[g];

      ch_msi[g].stop_addr     = {msi_stop_hi_r[g],  msi_stop_lo_r[g]};
      ch_msi[g].wmk_addr      = {msi_wmk_hi_r[g],   msi_wmk_lo_r[g]};
      ch_msi[g].abort_addr    = {msi_abort_hi_r[g], msi_abort_lo_r[g]};
      ch_msi[g].msgd          = msi_msgd_r[g];

      ch_irq[g]               = done_pend[g] | abort_pend[g] | wmk_pend[g];
    end
  end

endmodule
