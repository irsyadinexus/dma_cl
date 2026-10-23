// ============================================================================
// dma_pkg.sv  —  PCIe Gen4 HDMA-native DMA engine: parameters, types, enums
// M1 interface-freeze artifact. SV-2012.
//
// ⚑RECONCILE markers: exact bit positions/widths must be checked against the
// Synopsys DWC_pcie HDMA databook + the upstream dw-edma/dw-hdma-v0-regs.h that
// the Linux driver assumes. Structure here mirrors that driver's expectations.
// ============================================================================
package dma_pkg;

  // ---------------------------------------------------------------------------
  // Top-level build parameters (see spec §14). Overridable at top module.
  // ---------------------------------------------------------------------------
  localparam int unsigned DATA_WIDTH    = 512;  // primary x16: 512b@500MHz=256Gb/s >= 252
                                                // (1024b@~250-300MHz = timing-relief alt)
  localparam bit          STRADDLE_EN   = 1'b1; // multi-TLP/beat pack+de-straddle (line rate)
  localparam int unsigned NUM_RD_CH     = 4;    // HDMA "Read"  = H2C ; up to 16
  localparam int unsigned NUM_WR_CH     = 4;    // HDMA "Write" = C2H ; up to 16
  localparam int unsigned AXI_ID_W      = 8;
  localparam int unsigned TAG_W         = 10;   // 10-bit extended tags
  localparam int unsigned MAX_TAGS      = (1 << TAG_W);
  localparam int unsigned PASID_W       = 20;
  localparam int unsigned ATC_ENTRIES   = 64;   // 64-entry fully associative ATC (M4)
  localparam int unsigned PRI_CONTEXTS  = 64;
  localparam int unsigned REORDER_BYTES = 65536; // per active read channel
  localparam int unsigned LEN_W         = 13;    // <=4096 bytes/request (4KB rule)

  // M3 Completion Reorder & Tag Allocator Parameters
  localparam int unsigned TAGS_PER_CH       = 16;   // in-flight MRd tags per channel
  localparam int unsigned MAX_BURST_BEATS   = 8;    // 512B / 64B = 8 beats max per MRd/MWr
  localparam int unsigned REORDER_BEATS     = TAGS_PER_CH * MAX_BURST_BEATS; // 128 beats (8KB)
  localparam int unsigned CPL_TIMEOUT_LIMIT = 65535;// watchdog cycles before ERR_CPL_TIMEOUT

  localparam int unsigned STRB_WIDTH    = DATA_WIDTH/8;

  // Build-time capability flags (reported via CAPABILITIES / EXT_CAP)
  localparam bit          AXIS_EN       = 1'b1; // AXI4-Stream device mode
  localparam bit          ATS_EN        = 1'b1;
  localparam bit          PASID_EN      = 1'b1;
  localparam bit          PRI_EN        = 1'b1;
  localparam bit          P2P_EN        = 1'b1;
  localparam bit          ECC_EN        = 1'b1;

  // ---------------------------------------------------------------------------
  // Enumerations
  // ---------------------------------------------------------------------------
  typedef enum logic [0:0] { CH_READ = 1'b0,   // H2C : host -> device
                             CH_WRITE = 1'b1    // C2H : device -> host
                           } ch_dir_e;

  typedef enum logic [0:0] { DEV_AXI_MM = 1'b0, // first-class (on-card DDR)
                             DEV_AXI_ST = 1'b1  // AXI4-Stream extension
                           } dev_mode_e;

  // Tag state for completion reassembly (M3)
  typedef enum logic [1:0] {
    TAG_FREE      = 2'b00,
    TAG_WAIT_CPL  = 2'b01,
    TAG_COMMITTED = 2'b10
  } tag_state_e;

  // PCIe AT[1:0] field (Address Type) — ATS
  typedef enum logic [1:0] { AT_UNTRANSLATED = 2'b00,
                             AT_TRANS_REQ    = 2'b01, // translation request
                             AT_TRANSLATED   = 2'b10
                           } at_e;

  // Completion status (PCIe Cpl "Completion Status" field)
  typedef enum logic [2:0] { CPL_SC  = 3'b000, // Successful
                             CPL_UR  = 3'b001, // Unsupported Request
                             CPL_CRS = 3'b010, // Config Retry Status
                             CPL_CA  = 3'b100  // Completer Abort
                           } cpl_status_e;

  // Engine/channel error codes (mirrors HDMA *_ERR_STATUS classes) ⚑RECONCILE
  typedef enum logic [3:0] { ERR_NONE          = 4'h0,
                             ERR_LL_FETCH       = 4'h1, // linked-list fetch
                             ERR_LL_FORMAT      = 4'h2, // bad element/version
                             ERR_CPL_TIMEOUT    = 4'h3,
                             ERR_CPL_UR         = 4'h4,
                             ERR_CPL_CA         = 4'h5,
                             ERR_POISONED       = 4'h6,
                             ERR_ECC_UNCORR     = 4'h7,
                             ERR_ATS_FAIL       = 4'h8, // translation failed
                             ERR_PRI_FAIL       = 4'h9, // page-request denied
                             ERR_PRI_TIMEOUT    = 4'hA,
                             ERR_4KB_VIOL       = 4'hB
                           } err_code_e;

  // ---------------------------------------------------------------------------
  // HDMA-native linked-list data element (6 DWORDs / 24 bytes).
  // DWORD0=control at byte offset 0. Packed struct: first field = MSBs, so the
  // parser/serializer maps DWORDs explicitly (do not rely on struct bit order
  // for memory layout). ⚑RECONCILE element layout vs HDMA databook.
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0] control;  // DWORD0 : see CTRL_* bit positions below
    logic [31:0] size;     // DWORD1 : transfer size in bytes
    logic [31:0] sar_hi;   // DWORD3 : source addr [63:32]
    logic [31:0] sar_lo;   // DWORD2 : source addr [31:0]
    logic [31:0] dar_hi;   // DWORD5 : dest   addr [63:32]
    logic [31:0] dar_lo;   // DWORD4 : dest   addr [31:0]
  } hdma_elem_t;

  // Element control bit positions ⚑RECONCILE (eDMA-like; HDMA may differ)
  localparam int CTRL_CB  = 0;  // Cycle Bit (HW/SW ownership)
  localparam int CTRL_TCB = 1;  // Toggle Cycle Bit
  localparam int CTRL_LLP = 2;  // Load Link Pointer (this is a link element)
  localparam int CTRL_LIE = 3;  // Local Interrupt Enable  (done on this elem)
  localparam int CTRL_RIE = 4;  // Remote Interrupt Enable

  // ---------------------------------------------------------------------------
  // Internal vendor-neutral requester descriptor (PAA contract, spec §3.1).
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic                is_read;    // 1 = MRd, 0 = MWr
    logic [63:0]         addr;       // already ATS-translated if at==TRANSLATED
    logic [LEN_W-1:0]    len_bytes;  // <=4096, never crosses 4KB
    logic [TAG_W-1:0]    tag;        // valid for reads
    logic [PASID_W-1:0]  pasid;
    logic                pasid_en;
    at_e                 at;
    logic [2:0]          tc;         // traffic class
    logic                no_snoop;
    logic                relaxed;    // relaxed ordering
    logic [3:0]          first_be;
    logic [3:0]          last_be;
  } pcie_req_t;

  // Completion beat metadata returned by PAA to the mover.
  typedef struct packed {
    logic [TAG_W-1:0]    tag;
    logic [12:0]         byte_count;
    logic [6:0]          lower_addr;
    cpl_status_e         status;
    logic                last;
  } pcie_cpl_meta_t;

  // ATS translation response (from PAA ATS interface)
  typedef struct packed {
    logic [63:0]         trans_addr; // translated PA
    logic [63:0]         size;       // translation range (power of two)
    logic                r;          // read allowed
    logic                w;          // write allowed
    logic                priv;       // privileged
    logic                non_present;// -> trigger PRI page request
    logic                fail;       // hard failure -> abort
  } ats_rsp_t;

  // ATC (Device TLB) configuration and structures

  typedef struct packed {
    logic        valid;
    logic        pasid_en;
    logic [19:0] pasid;
    logic [51:0] vpn;         // Virtual Page Number: addr[63:12]
    logic [51:0] ppn;         // Physical Page Number: trans_addr[63:12]
    logic        r_perm;      // Read permission allowed
    logic        w_perm;      // Write permission allowed
    logic        priv;        // Privileged access allowed
  } atc_entry_t;

  typedef struct packed {
    logic [15:0] hit_count;
    logic [15:0] miss_count;
  } atc_status_t;

  // ===========================================================================
  // Synthesizable module-boundary bundles.
  // Rationale: SV interface *ports* are unsupported by some tools (e.g. Icarus
  // 12). Packed-struct + array ports lint on Verilator + Icarus + commercial
  // tools and are synthesis-clean. The rtl/if/*.sv interfaces mirror these 1:1
  // and are used for TB/BFM connections on interface-capable simulators.
  // Convention: <bus>_bus_t carries producer->consumer payload+valid; the
  // matching *_ready flows back as a separate scalar.
  // ===========================================================================

  // ---- PCIe requester / completion bundles ----
  typedef struct packed {
    logic                  valid;
    pcie_req_t             req;
    logic [DATA_WIDTH-1:0] wdata;   // MWr write data (ignored for MRd)
    logic                  wlast;
  } pcie_req_bus_t;

  typedef struct packed {
    logic                  valid;
    pcie_cpl_meta_t        meta;
    logic [DATA_WIDTH-1:0] rdata;
  } pcie_cpl_bus_t;

  // ---- ATS/PRI message bundles (subsystem <-> PAA) ----
  typedef struct packed {
    logic                ats_req_valid;
    logic [63:0]         ats_req_addr;
    logic [PASID_W-1:0]  ats_req_pasid;
    logic                ats_req_pasid_en;
    logic                ats_req_is_write;
    logic [7:0]          ats_req_id;
    logic                ats_rsp_ready;
    logic                pri_req_valid;
    logic [63:0]         pri_req_addr;
    logic [PASID_W-1:0]  pri_req_pasid;
    logic                pri_req_pasid_en;
    logic                pri_req_rd;
    logic                pri_req_wr;
    logic [8:0]          pri_req_prg_idx;
    logic                pri_rsp_ready;
  } ats_pri_up_t;     // subsystem -> PAA

  typedef struct packed {
    logic                ats_req_ready;
    logic                ats_rsp_valid;
    logic [7:0]          ats_rsp_id;
    ats_rsp_t            ats_rsp;
    logic                pri_req_ready;
    logic                pri_rsp_valid;
    logic [8:0]          pri_rsp_prg_idx;
    logic [3:0]          pri_rsp_code;
  } ats_pri_dn_t;     // PAA -> subsystem

  // ---- AXI4-MM device bus (split by drive direction) ----
  typedef struct packed {
    logic [AXI_ID_W-1:0]   awid;   logic [63:0] awaddr;  logic [7:0] awlen;
    logic [2:0]            awsize; logic [1:0]  awburst; logic       awvalid;
    logic [DATA_WIDTH-1:0] wdata;  logic [STRB_WIDTH-1:0] wstrb;
    logic                  wlast;  logic        wvalid;  logic       bready;
    logic [AXI_ID_W-1:0]   arid;   logic [63:0] araddr;  logic [7:0] arlen;
    logic [2:0]            arsize; logic [1:0]  arburst; logic       arvalid;
    logic                  rready;
  } axi_mm_m2s_t;

  typedef struct packed {
    logic                  awready; logic wready;
    logic [AXI_ID_W-1:0]   bid;     logic [1:0] bresp;   logic bvalid;
    logic                  arready;
    logic [AXI_ID_W-1:0]   rid;     logic [DATA_WIDTH-1:0] rdata;
    logic [1:0]            rresp;   logic rlast;         logic rvalid;
  } axi_mm_s2m_t;

  // ---- AXI4-Stream device bus (data one way, tready back) ----
  typedef struct packed {
    logic [DATA_WIDTH-1:0] tdata; logic [STRB_WIDTH-1:0] tkeep;
    logic                  tlast; logic [31:0] tuser;    logic tvalid;
  } axis_data_t;

  // ---- Channel control/status (CSR <-> channel) ----
  typedef enum logic [2:0] { ST_IDLE    = 3'd0,
                             ST_FETCH   = 3'd1,  // fetching descriptors
                             ST_RUN     = 3'd2,
                             ST_SUSPEND = 3'd3,  // PRI page-fault wait
                             ST_STOP    = 3'd4,
                             ST_ERROR   = 3'd5
                           } ch_state_e;

  typedef struct packed {
    logic                enable;       // CH_EN
    logic                lle;          // CONTROL1.LLE (1=linked-list, 0=register mode)
    ch_dir_e             dir;          // fixed by channel index
    dev_mode_e           mode;         // ext STREAM_CTRL: AXI-MM vs AXI-Stream
    logic [63:0]         llp_base;     // LLP_HI:LLP_LO — linked-list head pointer
    logic [63:0]         sar;          // register-mode source address
    logic [63:0]         dar;          // register-mode dest address
    logic [31:0]         xfer_size;    // register-mode TRANSFER_SIZE (bytes)
    logic [2:0]          tc;           // CONTROL1.TC traffic class
    logic                ro;           // CONTROL1.RO relaxed-ordering attr
    logic                ns;           // CONTROL1.NS no-snoop attr
    logic                lie;          // local interrupt enable
    logic                rie;          // remote interrupt enable
    logic [15:0]         prefetch;     // PREFETCH depth
    logic                watermark_en; // WATERMARK_EN
    logic [7:0]          func_num;     // FUNC_NUM (SR-IOV/PF)
    logic [15:0]         stream_user;  // STREAM_CTRL[31:16] TUSER seed
    logic [PASID_W-1:0]  pasid;        // ext PASID
    logic                pasid_en;
    logic                pasid_priv;
    logic [3:0]          qos;          // QOS arbiter weight
    logic                doorbell;     // 1-cycle start pulse
  } ch_ctrl_t;

  typedef struct packed {
    ch_state_e           state;
    err_code_e           err;
    logic [63:0]         bytes_xferred;  // residue = size - this
    logic                done_irq;       // transfer/element completion event
    logic                abort_irq;      // abort/error event
    logic                watermark_irq;  // mid-transfer watermark event
  } ch_stat_t;

  // Per-channel MSI(-X) message config (CSR -> PAA IMWr generator)
  typedef struct packed {
    logic [63:0]         stop_addr;   // done/stop IMWr address
    logic [63:0]         wmk_addr;    // watermark IMWr address
    logic [63:0]         abort_addr;  // abort IMWr address
    logic [31:0]         msgd;        // MSI message data
  } ch_msi_t;

endpackage : dma_pkg
