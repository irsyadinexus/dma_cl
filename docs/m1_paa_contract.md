# M1 — DWC-PAA Signal Contract

The **PCIe Application Adapter (PAA)** is the only DWC-specific block. It maps the
vendor-neutral interfaces (`pcie_req_if`, `ats_pri_if`, `csr_apb_if`) to the
**Synopsys DWC_pcie native application (raw-TLP) interface**. ⚑RECONCILE all DWC
signal names/widths against the DWC_pcie databook + release config.

---

## 1. Responsibilities
1. **TX**: assemble MRd/MWr TLP headers from `pcie_req_t`; insert PASID prefix &
   `AT[1:0]`; obey DWC transmit credit/ready; **straddle-pack** small TLPs on the
   1024-bit datapath.
2. **RX**: parse completions (CplD) → `pcie_cpl_meta_t` + data; route to mover by tag.
3. **Completer**: deliver inbound BAR reads/writes to the CSR block (`csr_apb_if`);
   return completions.
4. **ATS/PRI**: carry translation requests/responses and page-request/PRG messages
   between `ats_pri_if` and the DWC controller.
5. **Interrupts**: turn channel done/abort into DWC MSI-X IMWr.

---

## 2. `pcie_req_t` → TLP header field mapping ⚑RECONCILE

| `pcie_req_t` field | TLP field |
|---|---|
| `is_read` | Fmt/Type = MRd (read) or MWr (write); 3DW/4DW by addr[63:32]≠0 |
| `addr[63:0]` | Address (4DW hdr if >4 GB) |
| `len_bytes` | Length (DW count) + first/last BE derivation |
| `tag[9:0]` | Tag (10-bit extended tags enabled in DWC config) |
| `pasid`,`pasid_en` | **PASID TLP prefix** (End-to-End prefix) |
| `at[1:0]` | AT field (00 untranslated / 10 translated) |
| `tc[2:0]` | Traffic Class |
| `no_snoop`,`relaxed` | Attr[1:0] |
| `first_be`,`last_be` | 1st/last DW byte enables |
| (const) | Requester ID = controller-assigned bus/dev/fn |

Completions returned by DWC → `pcie_cpl_meta_t`: `{tag, byte_count, lower_addr,
status(SC/UR/CA/CRS), last}` + `cpl_rdata`.

---

## 3. DWC native interface groups (representative) ⚑RECONCILE

DWC exposes a client/application interface for TLPs. Expected groups the PAA binds:

| DWC group (typical) | Direction | PAA use |
|---|---|---|
| Requester-side transmit (client req TLP) | PAA→DWC | issue MRd/MWr |
| Requester-side receive (completion TLP) | DWC→PAA | CplD in |
| Completer-side receive (BAR TLP) | DWC→PAA | CSR access in |
| Completer-side transmit (completion) | PAA→DWC | CSR read data out |
| Flow-control credit status | DWC→PAA | `fc_np_ok`/`fc_p_ok` |
| MSI/MSI-X trigger | PAA→DWC | done/abort IMWr |
| ATS/PRI message channel | both | translation & page requests |
| Link/AER status sideband | DWC→PAA | error classification |

> Exact DWC signal names (e.g. app-layer client bus, `radm_*`/`trgt_*`, credit
> buses) depend on the DWC_pcie configuration and databook rev — pin in M1.

---

## 4. Straddle framing (`STRADDLE_EN=1`, primary `DATA_WIDTH=512`)
At 512b, one beat = 64 B; a beat-aligned TLP wastes header+alignment slots (~63%
beat-efficiency at 64 B TLPs, ~85% @256 B, ~92% @512 B MPS). **Straddling is
required to hit x16 line rate at *any* width** — 512b@500 MHz meets raw capacity
(256 ≥ 252 Gb/s) but only realizes it when beats stay full. The PAA TX must:
- Pack >1 TLP (header+payload) per beat, bounded by min TLP size.
- Start TLPs mid-beat; emit start-of-TLP / end-of-TLP position markers per segment.
- On RX, de-straddle multiple completions/BAR TLPs arriving in one beat.
Size **separate RX (completions→device) and TX (device→writes)** paths — x16 is
full-duplex, ~252 Gb/s each way. ⚑RECONCILE whether the DWC config provides
straddle natively (and its app-interface width — 512b@500 vs 1024b@~250 for x16);
match it to avoid a width-conversion gearbox. If DWC presents 1024b, `DATA_WIDTH`
follows — the straddle logic is identical, just wider.

---

## 5. Ordering & credits
- Enforce PCIe producer/consumer ordering: posted writes push completions; a read
  must not pass a write to the same address unless `relaxed` set.
- Track NP credits for reads (`fc_np_ok`) — the mover throttles issue on credit;
  channel arbiter yields a credit-starved channel (no head-of-line block).

---

## 6. Clock/reset
PAA straddles `pcie_app_clk` (DWC, ~500 MHz) and `core_clk`. Async FIFOs on the
req/cpl/ATS/PRI boundaries; see spec §13. Reset drains in-flight TLPs before
datapath reset.

---

## 7. Reconciliation checklist
1. Obtain DWC_pcie databook rev **____** + the customer config (channels, tag
   width, PASID/ATS/PRI enables, MSI-X location, straddle support).
2. Map each `pcie_req_if`/`ats_pri_if` signal to concrete DWC app-layer signals.
3. Confirm PASID prefix + AT field support enabled in the DWC config.
4. Confirm extended-tag (10-bit) support and max outstanding NP.
5. Confirm ATS/PRI message plumbing exposed at app layer vs handled internally.
