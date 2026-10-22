/* verilator lint_off UNUSEDPARAM */
// ============================================================================
// sim/axi_ram.sv — Behavioral AXI4-MM slave RAM model (512-bit datapath).
// Used in directed simulation to model on-card DDR / local device SRAM.
// ============================================================================
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */

module axi_ram #(
  parameter int DATA_WIDTH = dma_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH = 64,
  parameter int ID_WIDTH   = dma_pkg::AXI_ID_W,
  parameter int STRB_WIDTH = DATA_WIDTH / 8,
  parameter int MEM_SIZE   = 1024 * 1024 // 1 MB addressable model
) (
  input  logic                    clk,
  input  logic                    rst_n,
  input  dma_pkg::axi_mm_m2s_t    s_axi_m2s,
  output dma_pkg::axi_mm_s2m_t    s_axi_s2m
);
  import dma_pkg::*;

  // 1MB byte-addressable flat memory array
  logic [7:0] mem [MEM_SIZE];

  // Reference package parameters so standalone -Wall lint passes without requiring command-line waivers
  localparam int unsigned _LINT_TOUCH = int'(STRADDLE_EN) + NUM_RD_CH + NUM_WR_CH + MAX_TAGS
                                      + ATC_ENTRIES + PRI_CONTEXTS + REORDER_BYTES
                                      + int'(AXIS_EN) + int'(ATS_EN) + int'(PASID_EN) + int'(PRI_EN)
                                      + int'(P2P_EN) + int'(ECC_EN)
                                      + CTRL_CB + CTRL_TCB + CTRL_LLP + CTRL_LIE + CTRL_RIE;

  // Write channel state
  typedef enum logic [1:0] { W_IDLE, W_BURST, W_RESP } w_state_e;
  w_state_e w_state;
  logic [ADDR_WIDTH-1:0] w_addr;
  logic [7:0]            w_len;
  logic [ID_WIDTH-1:0]   w_id;

  // Read channel state
  typedef enum logic [1:0] { R_IDLE, R_BURST } r_state_e;
  r_state_e r_state;
  logic [ADDR_WIDTH-1:0] r_addr;
  logic [7:0]            r_len;
  logic [ID_WIDTH-1:0]   r_id;

  // Combinational default tie-offs
  assign s_axi_s2m.awready = (w_state == W_IDLE);
  assign s_axi_s2m.wready  = (w_state == W_BURST);
  assign s_axi_s2m.bvalid  = (w_state == W_RESP);
  assign s_axi_s2m.bresp   = 2'b00; // OKAY
  assign s_axi_s2m.bid     = w_id;

  assign s_axi_s2m.arready = (r_state == R_IDLE);
  assign s_axi_s2m.rvalid  = (r_state == R_BURST);
  assign s_axi_s2m.rresp   = 2'b00; // OKAY
  assign s_axi_s2m.rid     = r_id;
  assign s_axi_s2m.rlast   = (r_state == R_BURST) && (r_len == 0);

  // Read data output assembly
  always_comb begin
    s_axi_s2m.rdata = '0;
    if (r_state == R_BURST) begin
      for (int i = 0; i < STRB_WIDTH; i++) begin
        s_axi_s2m.rdata[8*i +: 8] = mem[(int'(r_addr) + i) % MEM_SIZE];
      end
    end
  end

  // Sequential write engine
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w_state <= W_IDLE;
      w_addr  <= '0;
      w_len   <= '0;
      w_id    <= '0;
    end else begin
      case (w_state)
        W_IDLE: begin
          if (s_axi_m2s.awvalid && s_axi_s2m.awready) begin
            w_addr  <= s_axi_m2s.awaddr;
            w_len   <= s_axi_m2s.awlen;
            w_id    <= s_axi_m2s.awid;
            w_state <= W_BURST;
          end
        end
        W_BURST: begin
          if (s_axi_m2s.wvalid && s_axi_s2m.wready) begin
            for (int i = 0; i < STRB_WIDTH; i++) begin
              if (s_axi_m2s.wstrb[i]) begin
                mem[(int'(w_addr) + i) % MEM_SIZE] <= s_axi_m2s.wdata[8*i +: 8];
              end
            end
            w_addr <= w_addr + ADDR_WIDTH'(STRB_WIDTH);
            if (s_axi_m2s.wlast || w_len == 0) begin
              w_state <= W_RESP;
            end else begin
              w_len <= w_len - 1'b1;
            end
          end
        end
        W_RESP: begin
          if (s_axi_m2s.bready && s_axi_s2m.bvalid) begin
            w_state <= W_IDLE;
          end
        end
        default: w_state <= W_IDLE;
      endcase
    end
  end

  // Sequential read engine
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_state <= R_IDLE;
      r_addr  <= '0;
      r_len   <= '0;
      r_id    <= '0;
    end else begin
      case (r_state)
        R_IDLE: begin
          if (s_axi_m2s.arvalid && s_axi_s2m.arready) begin
            r_addr  <= s_axi_m2s.araddr;
            r_len   <= s_axi_m2s.arlen;
            r_id    <= s_axi_m2s.arid;
            r_state <= R_BURST;
          end
        end
        R_BURST: begin
          if (s_axi_s2m.rvalid && s_axi_m2s.rready) begin
            r_addr <= r_addr + ADDR_WIDTH'(STRB_WIDTH);
            if (r_len == 0) begin
              r_state <= R_IDLE;
            end else begin
              r_len <= r_len - 1'b1;
            end
          end
        end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // Helper tasks for testbench verification
  task automatic write_byte(input logic [ADDR_WIDTH-1:0] a, input logic [7:0] d);
    mem[int'(a) % MEM_SIZE] = d;
  endtask

  function automatic logic [7:0] read_byte(input logic [ADDR_WIDTH-1:0] a);
    return mem[int'(a) % MEM_SIZE];
  endfunction

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on WIDTHEXPAND */
