# Milestone M4 ATS, PASID Datapath & Address Translation Cache (ATC) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Implement the Milestone M4 Address Translation Services (ATS), PASID datapath, and 64-entry on-chip Address Translation Cache (ATC / Device TLB) in `rtl/blocks/ats_pri_subsys.sv`, integrated with top-level CSR controls and verified by a directed testbench (`make sim-m4`) with all 7 tests passing.

**Architecture:** A centralized 64-entry fully associative ATC device TLB sits on the unified requester path in `ats_pri_subsys.sv`. It intercepts untranslated PCIe memory requests, performs 1-cycle tag lookups against `{pasid_en, pasid, vpn}`, issues ATS Translation Requests over `ats_pri_if` to the host IOMMU on misses, holds untranslated requests in a translation skid buffer, populates the ATC on translation completion, substitutes physical addresses with `AT=10b`, and enforces page permissions and invalidations.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Verilator 5.032+ (lint and simulation), standard GNU Make, bash.

**Spec:** [`docs/superpowers/specs/2026-10-23-milestone-m4-ats-pasid-atc-design.md`](file:///home/user/RTL-VIBECODE/DMA/dma_cl/docs/superpowers/specs/2026-10-23-milestone-m4-ats-pasid-atc-design.md)

## Global Constraints

- Native WSL2 / Linux toolchain only: Use standard Linux binaries (`/usr/bin/verilator`, `make`, `git`), never Windows `.exe` tools or `/mnt/c/`.
- IP-Agnostic Core: Zero proprietary IP dependencies (no Synopsys DWC shims in synthesizable RTL).
- Data Width: Primary `DATA_WIDTH = 512` bits (64 bytes/beat) at 500 MHz datapath clock.
- Page Granularity: Standard 4KB page translations (lower 12 bits `addr[11:0]` preserved).
- Complete Channel Isolation: Stalled or faulting channels must not block or starve active sibling channels (spec NFR-1).
- Verification Rule: Evidence before assertions always (`make lint`, `make lint-if`, `make sim-csr`, `make sim-m2`, `make sim-m3`, `make sim-m4`).

## Review Focus

1. **ATC Invalidation Racing In-Flight Translation:** If an invalidation arrives while an ATS translation request is in flight for that page, the entry must not be left valid with stale data.
2. **Permission Violation on Multi-Beat Write Burst:** If a Write channel attempts an `MWr` to a page with `w_perm = 0`, the engine must abort cleanly without corrupting the device AXI-MM bus or issuing partial beats to PCIe.
3. **Cross-PASID Alias Isolation:** Two channels using the same virtual address with different PASIDs must never alias or hit each other's cache entry.
4. **4KB Page Boundary Crossing with ATS Translation:** A transfer crossing a 4KB boundary will have segment 1 translated to Page A and segment 2 translated to Page B. The ATC lookup must translate each segment's VPN independently.
5. **ATS Translation Request Timeout:** If the host IOMMU fails to respond to an ATS request within the timeout window, the channel must cleanly abort with `ERR_ATS_FAIL` and not deadlock `dn_req` or other channels.

---

### Task 1: Package Definitions & ATC Data Types

**Files:**
- Modify: `rtl/dma_pkg.sv:140-160`
- Test: `make lint`

**Interfaces:**
- Consumes: Existing `dma_pkg::ats_rsp_t`, `dma_pkg::at_e`, `dma_pkg::PASID_W`.
- Produces: `ATC_ENTRIES`, `atc_entry_t`, `atc_inval_t`, `atc_status_t`.

- [x] **Step 1: Inspect `rtl/dma_pkg.sv` and write the failing lint check for new types**

Add a temporary reference to `atc_entry_t` in `sim/lint_top.sv` to verify lint fails:
```systemverilog
dma_pkg::atc_entry_t test_atc;
```

Run: `make lint-if`  
Expected: FAIL with "atc_entry_t is not a type".

- [x] **Step 2: Add ATC parameters and types to `rtl/dma_pkg.sv`**

Add the following to `rtl/dma_pkg.sv` right after `ats_rsp_t`:
```systemverilog
  // ATC (Device TLB) configuration and structures
  localparam int unsigned ATC_ENTRIES = 64;

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
```

- [x] **Step 3: Run lint to verify cleanliness**

Run: `make lint && make lint-if`  
Expected: PASS (0 errors, 0 warnings).

- [x] **Step 4: Commit**

```bash
git add rtl/dma_pkg.sv sim/lint_top.sv
git commit -m "feat(pkg): add ATC device TLB entry structure and parameters"
```

---

### Task 2: CSR Block ATS & ATC Control/Status Export

**Files:**
- Modify: `rtl/blocks/csr_block.sv:25-35, 100-110, 280-290, 370-380`
- Test: `make sim-csr`

**Interfaces:**
- Consumes: `dma_pkg::atc_status_t`.
- Produces: `ats_ctrl_o` (`logic [31:0]`), `atc_inval_o` (`logic [31:0]`), `atc_inval_pulse_o` (`logic`).

- [x] **Step 1: Add ATS/ATC control and status ports to `csr_block.sv`**

In `rtl/blocks/csr_block.sv`, add ports to module header:
```systemverilog
  // ATS & ATC subsystem controls
  output logic [31:0]       ats_ctrl_o,
  output logic [31:0]       atc_inval_o,
  output logic              atc_inval_pulse_o,
  input  dma_pkg::atc_status_t atc_stat_i,
```

- [x] **Step 2: Connect register storage to outputs and hook readback**

In `rtl/blocks/csr_block.sv`:
1. Generate 1-cycle `atc_inval_pulse_o` when `do_write && in_ext && (eoff == E_ATC_INVAL) && pwdata[0]`:
```systemverilog
  logic atc_inval_pulse_r;
  assign atc_inval_pulse_o = atc_inval_pulse_r;
  assign ats_ctrl_o        = ats_ctrl_r;
  assign atc_inval_o       = atc_inval_r;
```
2. In sequential block:
```systemverilog
  atc_inval_pulse_r <= do_write && in_ext && (eoff == E_ATC_INVAL) && pwdata[0];
```
3. In read multiplexer:
```systemverilog
  else if (eoff == E_ATC_STATUS) rdata = {atc_stat_i.miss_count, atc_stat_i.hit_count};
```

- [x] **Step 3: Update `sim/tb_csr.sv` to test `E_ATS_CTRL`, `E_ATC_INVAL`, and `E_ATC_STATUS`**

Add APB write and read checks in `sim/tb_csr.sv` for offsets `20'h9300`, `20'h9304`, `20'h9308`.

- [x] **Step 4: Run simulation and lint**

Run: `make lint && make sim-csr`  
Expected: `[tb_csr] ALL PASS` (0 errors, 0 warnings).

- [x] **Step 5: Commit**

```bash
git add rtl/blocks/csr_block.sv sim/tb_csr.sv
git commit -m "feat(csr): export ATS control, ATC invalidation strobe, and ATC status readback"
```

---

### Task 3: Address Translation Cache (ATC) & Invalidation Engine

**Files:**
- Modify: `rtl/blocks/ats_pri_subsys.sv`
- Test: `make lint`

**Interfaces:**
- Consumes: `dma_pkg::atc_entry_t`, `dma_pkg::ATC_ENTRIES`.
- Produces: Combinatorial ATC tag lookup function/signals (`atc_hit`, `atc_match_entry`, `atc_match_idx`).

- [x] **Step 1: Declare 64-entry ATC memory array and replacement pointer**

In `rtl/blocks/ats_pri_subsys.sv`:
```systemverilog
  atc_entry_t atc_mem [ATC_ENTRIES];
  logic [$clog2(ATC_ENTRIES)-1:0] replace_ptr;
  logic [15:0] hit_cnt_r, miss_cnt_r;

  assign atc_stat.hit_count  = hit_cnt_r;
  assign atc_stat.miss_count = miss_cnt_r;
```

- [x] **Step 2: Implement combinatorial tag lookup and permission check**

```systemverilog
  wire [51:0] req_vpn = up_req.req.addr[63:12];
  logic [ATC_ENTRIES-1:0] hit_mask;
  atc_entry_t match_entry;
  logic atc_hit;

  always_comb begin
    atc_hit = 1'b0;
    match_entry = '0;
    for (int i = 0; i < ATC_ENTRIES; i++) begin
      hit_mask[i] = atc_mem[i].valid &&
                    (atc_mem[i].vpn == req_vpn) &&
                    (!atc_mem[i].pasid_en || (atc_mem[i].pasid == up_req.req.pasid));
      if (hit_mask[i]) begin
        atc_hit = 1'b1;
        match_entry = atc_mem[i];
      end
    end
  end

  wire perm_ok = up_req.req.is_read ? match_entry.r_perm : match_entry.w_perm;
```

- [x] **Step 3: Implement single-cycle Invalidation Engine**

Handle both software CSR writes (`atc_inval_pulse`) and host IOMMU messages (`ats_dn`):
```systemverilog
  wire do_inval = atc_inval_pulse || ats_dn_inval_valid;
  wire inval_global = atc_inval_pulse ? atc_inval_reg[1] : ats_dn_inval_global;
  wire [19:0] inval_pasid = atc_inval_pulse ? atc_inval_reg[19:4] : ats_dn_inval_pasid;
  wire [51:0] inval_vpn   = atc_inval_pulse ? {40'b0, atc_inval_reg[31:20]} : ats_dn_inval_vpn;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      for (int i = 0; i < ATC_ENTRIES; i++) atc_mem[i] <= '0;
      replace_ptr <= '0;
      hit_cnt_r   <= '0;
      miss_cnt_r  <= '0;
    end else if (do_inval) begin
      for (int i = 0; i < ATC_ENTRIES; i++) begin
        if (inval_global) begin
          atc_mem[i].valid <= 1'b0;
        end else if (atc_mem[i].valid && (atc_mem[i].vpn == inval_vpn)) begin
          atc_mem[i].valid <= 1'b0;
        end
      end
    end
  end
```

- [x] **Step 4: Verify with `make lint`**

Run: `make lint`  
Expected: PASS (0 errors, 0 warnings).

- [x] **Step 5: Commit**

```bash
git add rtl/blocks/ats_pri_subsys.sv
git commit -m "feat(ats): implement 64-entry ATC device TLB and invalidation engine"
```

---

### Task 4: ATS Translation Agent FSM & Skid Buffer

**Files:**
- Modify: `rtl/blocks/ats_pri_subsys.sv`
- Test: `make lint`

**Interfaces:**
- Consumes: `ats_pri_if` signals, `up_req`, `dn_req_ready`, `ats_dn`.
- Produces: `dn_req` (with `AT_TRANSLATED` and PA), `ats_up.ats_req_*`, `up_req_ready`.

- [x] **Step 1: Declare Translation Agent FSM states and Skid Buffer**

In `rtl/blocks/ats_pri_subsys.sv`:
```systemverilog
  typedef enum logic [2:0] {
    T_PASSTHROUGH,
    T_IDLE,
    T_REQ,
    T_WAIT,
    T_EMIT,
    T_FAULT
  } xlate_state_e;

  xlate_state_e state;
  pcie_req_bus_t skid_req;
  logic [7:0]    cur_req_id;
  logic [3:0]    fault_channel;
  logic          fault_active;
```

- [x] **Step 2: Implement translation request generation, fill, and emission**

1. In `T_IDLE`:
   - If `!ats_en`: bypass transparently to `dn_req`.
   - If `ats_en && up_req.valid`:
     - If `atc_hit && perm_ok`:
       - Translate: `dn_req = up_req; dn_req.req.addr = {match_entry.ppn, up_req.req.addr[11:0]}; dn_req.req.at = AT_TRANSLATED;`
       - Increment `hit_cnt_r <= hit_cnt_r + 1'b1;`
       - If `dn_req_ready`: hand off directly in 1 cycle.
     - If `atc_hit && !perm_ok`:
       - Increment `miss_cnt_r <= miss_cnt_r + 1'b1;`
       - Transition to `T_FAULT`.
     - If `!atc_hit`:
       - Latch `skid_req <= up_req;`
       - Increment `miss_cnt_r <= miss_cnt_r + 1'b1;`
       - Transition to `T_REQ`.
2. In `T_REQ`:
   - Assert `ats_up.ats_req_valid <= 1'b1;`
   - Drive `ats_up.ats_req_addr <= {skid_req.req.addr[63:12], 12'b0};`
   - Drive `ats_up.ats_req_pasid <= skid_req.req.pasid;`
   - Drive `ats_up.ats_req_pasid_en <= skid_req.req.pasid_en;`
   - Drive `ats_up.ats_req_is_write <= !skid_req.req.is_read;`
   - Drive `ats_up.ats_req_id <= cur_req_id;`
   - When `ats_dn.ats_req_ready`: enter `T_WAIT`.
3. In `T_WAIT`:
   - When `ats_dn.ats_rsp_valid && (ats_dn.ats_rsp_id == cur_req_id)`:
     - If `ats_dn.ats_rsp.fail || ats_dn.ats_rsp.non_present`: transition to `T_FAULT`.
     - Else:
       - Populate ATC: `atc_mem[replace_ptr] <= '{valid: 1'b1, pasid_en: skid_req.req.pasid_en, pasid: skid_req.req.pasid, vpn: skid_req.req.addr[63:12], ppn: ats_dn.ats_rsp.trans_addr[63:12], r_perm: ats_dn.ats_rsp.r, w_perm: ats_dn.ats_rsp.w, priv: ats_dn.ats_rsp.priv};`
       - Increment `replace_ptr <= replace_ptr + 1'b1;`
       - Update `skid_req.req.addr <= {ats_dn.ats_rsp.trans_addr[63:12], skid_req.req.addr[11:0]};`
       - Update `skid_req.req.at <= AT_TRANSLATED;`
       - Transition to `T_EMIT`.
4. In `T_EMIT`:
   - Assert `dn_req <= skid_req;`
   - When `dn_req_ready`: return to `T_IDLE`.

- [x] **Step 3: Implement Fault Abortion**

In `T_FAULT`:
- Assert channel error strobe with `ERR_ATS_FAIL` (`4'h8`).
- Free skid buffer and return to `T_IDLE`.

- [x] **Step 4: Run lint to verify cleanliness**

Run: `make lint`  
Expected: PASS (0 errors, 0 warnings).

- [x] **Step 5: Commit**

```bash
git add rtl/blocks/ats_pri_subsys.sv
git commit -m "feat(ats): implement ATS translation agent FSM and request skid buffer"
```

---

### Task 5: Top-Level Integration & PASID Wiring

**Files:**
- Modify: `rtl/dma_top.sv:60-70, 120-145`
- Test: `make lint && make lint-if`

**Interfaces:**
- Consumes: `csr_block` exports (`ats_ctrl_o`, `atc_inval_o`, `atc_inval_pulse_o`), `ats_pri_subsys` ports.
- Produces: Fully integrated `dma_top` with external `ats_up` and `ats_dn` buses.

- [x] **Step 1: Expose ATS/PRI interfaces on `dma_top`**

Ensure `ats_up` and `ats_dn` ports are exposed at `dma_top`:
```systemverilog
  output dma_pkg::ats_pri_up_t ats_up,
  input  dma_pkg::ats_pri_dn_t ats_dn,
```

- [x] **Step 2: Connect `csr_block` and `ats_pri_subsys` in `dma_top.sv`**

Wire `ats_ctrl`, `atc_inval`, `atc_inval_pulse`, and `atc_stat`:
```systemverilog
  logic [31:0] ats_ctrl, atc_inval;
  logic        atc_inval_pulse;
  atc_status_t atc_stat;

  csr_block #(.NUM_RD_CH(NUM_RD_CH), .NUM_WR_CH(NUM_WR_CH), .APB_AW(APB_AW)) u_csr (
    ...
    .ats_ctrl_o        (ats_ctrl),
    .atc_inval_o       (atc_inval),
    .atc_inval_pulse_o (atc_inval_pulse),
    .atc_stat_i        (atc_stat),
    ...
  );

  ats_pri_subsys u_ats (
    .core_clk        (core_clk),
    .core_rst_n      (core_rst_n),
    .up_req          (arb_req),
    .up_req_ready    (arb_req_ready),
    .up_cpl          (arb_cpl),
    .up_cpl_ready    (arb_cpl_ready),
    .dn_req          (host_req),
    .dn_req_ready    (host_req_ready),
    .dn_cpl          (host_cpl),
    .dn_cpl_ready    (host_cpl_ready),
    .ats_up          (ats_up),
    .ats_dn          (ats_dn),
    .ats_ctrl_reg    (ats_ctrl),
    .atc_inval_reg   (atc_inval),
    .atc_inval_pulse (atc_inval_pulse),
    .atc_stat        (atc_stat)
  );
```

- [x] **Step 3: Run complete lint check**

Run: `make lint && make lint-if`  
Expected: PASS (0 errors, 0 warnings).

- [x] **Step 4: Commit**

```bash
git add rtl/dma_top.sv
git commit -m "feat(top): wire ATS/ATC control and status registers between CSR and subsystem"
```

---

### Task 6: Directed Verification Suite & Makefile Target

**Files:**
- Create: `sim/tb_m4.sv`
- Modify: `Makefile:25-35`
- Test: `make sim-m4`

**Interfaces:**
- Consumes: `dma_top`, `sim/axi_ram.sv`, `dma_pkg::*`.
- Produces: Milestone M4 directed test suite verifying Tests 1 through 7.

- [x] **Step 1: Create `sim/tb_m4.sv` scaffolding and IOMMU BFM**

Implement the IOMMU model in `tb_m4.sv`:
```systemverilog
  // IOMMU Page Table mapping entry
  typedef struct {
    logic        valid;
    logic [19:0] pasid;
    logic [51:0] vpn;
    logic [51:0] ppn;
    logic        r_perm;
    logic        w_perm;
    logic        non_present;
    logic        fail;
  } iommu_page_t;

  iommu_page_t iommu_pt [$];
```
Implement the ATS translation request handler:
- Matches incoming `ats_up.ats_req_addr` and `ats_up.ats_req_pasid` against `iommu_pt`.
- Returns `ats_dn.ats_rsp_valid = 1'b1` with physical address, permissions, and status.

- [x] **Step 2: Implement Tests 1 through 7 in `tb_m4.sv`**

1. **Test 1: ATS Disabled Passthrough (`ats_en=0`)**
   - Untranslated baseline transfer passes directly with `AT=00b`.
2. **Test 2: ATC Miss & Translation Fill**
   - Read request misses ATC -> triggers ATS request to IOMMU BFM -> ATC fills -> emitted to PCIe with `AT=10b` and translated PA -> data verified in device AXI RAM.
3. **Test 3: ATC Hit Steady-State Streaming**
   - Sequential transfers to the same page hit ATC -> 0-cycle extra latency -> verify high throughput.
4. **Test 4: Translated Write (`MWr`) with PASID**
   - C2H Write channel issues transfer with `pasid_en=1` -> ATC translates VA to PA -> verify host memory received data at physical destination.
5. **Test 5: Multi-Channel PASID Isolation**
   - Ch0 (`PASID=0x1`) and Ch1 (`PASID=0x2`) both access VA `0x0000_A000` -> map to different PAs (`0x1000` vs `0x2000`) -> verify no cross-talk.
6. **Test 6: ATC Invalidation via CSR & Host Message**
   - Populate entry -> flush via `ATC_INVAL` CSR -> subsequent access causes ATC miss and re-translation -> miss counter increments.
7. **Test 7: Permission Violation & Fault Abortion**
   - Attempt write to read-only page (`w_perm=0`) -> engine aborts -> channel status reports `ERR_ATS_FAIL` (`4'h8`) -> done IRQ does not assert.

- [x] **Step 3: Add `sim-m4` target to `Makefile`**

In `Makefile`:
```makefile
.PHONY: lint lint-if lint-strict sim-csr sim-m2 sim-m3 sim-m4 synth-csr clean

# Milestone M4 directed simulation (ATS, PASID, ATC)
sim-m4:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m4.sv --top-module tb_m4 -o tb_m4
	@./obj_dir/tb_m4
```

- [x] **Step 4: Run `make sim-m4` and verify all tests pass**

Run: `make sim-m4`  
Expected: `[tb_m4] ALL TESTS PASSED (0 errors)`.

- [x] **Step 5: Run full regression suite**

Run: `make lint && make lint-if && make sim-csr && make sim-m2 && make sim-m3 && make sim-m4`  
Expected: All targets succeed cleanly with 0 errors and 0 warnings.

- [x] **Step 6: Commit**

```bash
git add sim/tb_m4.sv Makefile
git commit -m "test(m4): add comprehensive Milestone M4 ATS and ATC verification suite"
```
