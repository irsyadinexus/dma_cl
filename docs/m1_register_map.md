# M1 — Register Map (HDMA-native, BAR0)

**Primary map:** HDMA-native. Anchored to the upstream `dw-edma` driver's HDMA
model (`dw-hdma-v0-regs.h`) so the driver binds. **All offsets/reset values/bit
positions below are representative and marked ⚑RECONCILE — lock against the
Synopsys DWC_pcie HDMA databook (revision TBD) and `dw-hdma-v0-regs.h` before RTL.**

Direction convention (HDMA): **RD engine = Read channels = H2C** (host→device);
**WR engine = Write channels = C2H** (device→host).

---

## 1. BAR0 address decode ⚑RECONCILE

| Region | Base | Size | Contents |
|---|---|---|---|
| Common | `0x0_0000` | 4 KB | ID/version, engine enables, global int aggregation |
| RD (H2C) channels | `0x0_1000` | `NUM_RD_CH × 0x200` | per-channel block (§3) |
| WR (C2H) channels | `0x0_5000` | `NUM_WR_CH × 0x200` | per-channel block (§3) |
| **Vendor extension** | `0x0_9000` | 4 KB | stream/PASID/ATS/PRI/P2P (§5) — invisible to stock `dw-edma` |
| MSI-X table | `0x0_A000` | — | vectors (or in DWC controller space) ⚑RECONCILE |
| MSI-X PBA | `0x0_B000` | — | pending bits |

Per-channel stride = `0x200` ⚑RECONCILE. Channel *n* block base = region base + `n×0x200`.

---

## 2. Common region (`0x0_0000`) ⚑RECONCILE

| Offset | Name | Acc | Reset | Notes |
|---|---|---|---|---|
| 0x000 | `ID_VERSION` | RO | impl | magic, HDMA major/minor, `NUM_RD_CH`,`NUM_WR_CH`,`DATA_WIDTH` |
| 0x004 | `CAPABILITIES` | RO | impl | bit0 AXIS, bit1 ATS, bit2 PASID, bit3 PRI, bit4 P2P, bit5 ECC |
| 0x008 | `RD_ENGINE_EN` | RW | 0 | bit0 enable Read(H2C) engine |
| 0x00C | `WR_ENGINE_EN` | RW | 0 | bit0 enable Write(C2H) engine |
| 0x010 | `RD_INT_STATUS` | RO | 0 | per-channel done/abort aggregate (bit n = ch n) |
| 0x014 | `RD_INT_CLEAR` | W1C | 0 | clear per-channel |
| 0x018 | `WR_INT_STATUS` | RO | 0 | per-channel done/abort aggregate |
| 0x01C | `WR_INT_CLEAR` | W1C | 0 | clear per-channel |
| 0x020 | `RD_ERR_STATUS` | RO | 0 | per-channel error present (see §4 err codes) |
| 0x024 | `WR_ERR_STATUS` | RO | 0 | per-channel error present |
| 0x028 | `SOFT_RESET` | RW | 0 | bit0 drain+reset datapath |

---

## 3. Per-channel register block (RD and WR identical layout) ⚑RECONCILE

Mirrors `struct dw_hdma_v0_ch_regs`. Offsets relative to the channel base.

| Off | Name | Acc | Reset | Notes |
|---|---|---|---|---|
| 0x00 | `CH_EN` | RW | 0 | bit0 channel enable |
| 0x04 | `DOORBELL` | WO | – | write `start`(bit0) to launch from `LLP` |
| 0x08 | `PREFETCH` | RW | impl | descriptor prefetch depth |
| 0x0C | `HANDSHAKE` | RW | 0 | HW-handshake mode (reserved for our use) |
| 0x10 | `LLP_LO` | RW | 0 | linked-list head pointer [31:0] |
| 0x14 | `LLP_HI` | RW | 0 | linked-list head pointer [63:32] |
| 0x18 | `CYCLE_SYNC` | RW | 0 | initial cycle-bit state (cyclic mode) |
| 0x1C | `TRANSFER_SIZE` | RW | 0 | register-mode byte count (LL mode: RO status) |
| 0x20 | `SAR_LO` | RW | 0 | register-mode source [31:0] |
| 0x24 | `SAR_HI` | RW | 0 | register-mode source [63:32] |
| 0x28 | `DAR_LO` | RW | 0 | register-mode dest [31:0] |
| 0x2C | `DAR_HI` | RW | 0 | register-mode dest [63:32] |
| 0x30 | `WATERMARK_EN` | RW | 0 | mid-transfer watermark IRQ enable |
| 0x34 | `CONTROL1` | RW | 0 | see §3.1 |
| 0x38 | `FUNC_NUM` | RW | 0 | PCIe function number (SR-IOV/PF) ⚑RECONCILE |
| 0x3C | `QOS` | RW | impl | arbiter weight for this channel |
| 0x40 | `CH_STATUS` | RO | 0 | state (idle/running/stopped/error), `err_code` (§4) |
| 0x44 | `XFERRED_LO` | RO | 0 | bytes moved [31:0] (residue = size − xferred) |
| 0x48 | `XFERRED_HI` | RO | 0 | bytes moved [63:32] |
| 0x50 | `INT_SETUP` | RW | 0 | done/abort/watermark IRQ enables + local/remote select |
| 0x54 | `INT_CLEAR` | W1C | 0 | clear this channel's int bits |
| 0x58 | `INT_STATUS` | RO | 0 | done/abort/watermark pending |
| 0x60 | `MSI_STOP_LO/HI` | RW | 0 | IMWr addr for "stop/done" MSI |
| 0x68 | `MSI_WMK_LO/HI` | RW | 0 | IMWr addr for watermark MSI |
| 0x70 | `MSI_ABORT_LO/HI` | RW | 0 | IMWr addr for abort MSI |
| 0x78 | `MSI_MSGD` | RW | 0 | MSI message data |

### 3.1 `CONTROL1` bitfields ⚑RECONCILE
| Bit | Name | Notes |
|---|---|---|
| 0 | `CB` | cycle bit (matches head element) |
| 1 | `TCB` | toggle cycle bit (cyclic) |
| 2 | `LLE` | linked-list enable (0 = register mode) |
| 3 | `CCS` | consumer cycle state |
| 4 | `LIE` | local interrupt enable |
| 5 | `RIE` | remote interrupt enable |
| 8 | `RO` | request relaxed-ordering attr |
| 9 | `NS` | request no-snoop attr |
| 12:10 | `TC` | traffic class |

---

## 4. Error codes (`CH_STATUS.err_code`, `*_ERR_STATUS`)
Matches `dma_pkg::err_code_e`: `LL_FETCH, LL_FORMAT, CPL_TIMEOUT, CPL_UR, CPL_CA,
POISONED, ECC_UNCORR, ATS_FAIL, PRI_FAIL, PRI_TIMEOUT, 4KB_VIOL`. ⚑RECONCILE the
exact HDMA `*_ERR_STATUS` bit encoding (HDMA groups LL-error vs app-error).

---

## 5. Vendor extension region (`0x0_9000`) — non-standard, driver-addendum only

These are **outside** the HDMA-standard aperture so stock `dw-edma` ignores them.

| Off | Name | Acc | Notes |
|---|---|---|---|
| 0x000 | `EXT_ID` | RO | extension magic + version + databook-rev tag |
| 0x004 | `EXT_CAP` | RO | which extensions built (AXIS/ATS/PASID/PRI/P2P) |
| 0x010 + n×4 | `RD_CHn_STREAM_CTRL` | RW | bit0 mode (0=MM,1=AXI-Stream); [31:16] TUSER seed |
| 0x090 + n×4 | `WR_CHn_STREAM_CTRL` | RW | same for Write channels |
| 0x110 + n×8 | `RD_CHn_PASID` | RW | [19:0] PASID; bit20 PASID_EN; bit21 priv |
| 0x210 + n×8 | `WR_CHn_PASID` | RW | same for Write channels |
| 0x300 | `ATS_CTRL` | RW | bit0 ATS enable, bit1 use-translated-addr default |
| 0x304 | `ATC_INVAL` | RW | write PASID+addr → invalidate ATC entry; self-clear |
| 0x308 | `ATC_STATUS` | RO | occupancy, miss counters |
| 0x310 | `PRI_CTRL` | RW | bit0 PRI enable; [7:0] max concurrent groups; timeout cfg |
| 0x314 | `PRI_STATUS` | RO | active fault contexts, last PRG response code |
| 0x320 | `P2P_CTRL` | RW | per-channel P2P routing hints |

**Fallback if PRI disabled/unsupported by host:** `PRI_CTRL.bit0=0` → engine must
use pinned/pre-faulted memory (ATS-only); an ATC "not-present" then raises
`ERR_ATS_FAIL` instead of issuing a page request. (§ ATS/PRI micro-spec.)

---

## 6. Reconciliation checklist (before M2 RTL)
1. Lock every `⚑RECONCILE` offset/stride/reset against DWC_pcie HDMA databook rev **____**.
2. Diff against `dw-hdma-v0-regs.h` of the target kernel — field names + offsets must match for driver binding.
3. Confirm HDMA channel count option (up to 16+16) and per-channel block size.
4. Confirm MSI-X table location (our BAR vs DWC controller-managed).
5. Confirm `CONTROL1`/element `control` bit positions (HDMA vs eDMA differ).
6. Decide `FUNC_NUM`/SR-IOV scope (affects PASID + function mapping).
