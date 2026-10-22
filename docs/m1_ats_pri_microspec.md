# M1 — ATS / PASID / PRI Micro-Spec (full SVM)

Highest-complexity subsystem. Sits between the channel data movers and the PAA.
Every request address is translated before issue; misses request translation;
not-present pages trigger a PRI page request with **per-channel fault isolation**
so a stalled (faulting) channel never blocks others (spec NFR-1).

Interfaces: `ats_pri_if` (to PAA), plus per-request lookup port to the movers.

---

## 1. Block structure
```
mover req (IOVA,PASID,R/W)
        │  lookup
   ┌────▼─────────────────────────────────────────┐
   │ ATC (device TLB) : set-assoc, LRU, ECC        │  hit → translated PA, AT=10
   │  tag={PASID,VPN}  data={PPN,size,R/W/priv}     │
   └────┬───────────────────────┬──────────────────┘
      miss                    not-present
        │                        │
   ┌────▼──────────┐        ┌────▼───────────────────┐
   │ Xlate Agent   │        │ PRI Fault Engine        │
   │ ATS req/resp  │        │ suspend→page-req→resume │
   └────┬──────────┘        └────┬───────────────────┘
        └─────────► ats_pri_if ◄─┘  (to PAA / host IOMMU)
```

---

## 2. ATC — device TLB
- Organization: set-associative (`ATC_ENTRIES`, default 1024), LRU, **ECC** on tag+data.
- Entry: `{valid, PASID[19:0], VPN, PPN, size(pow2), R, W, priv}`.
- Lookup latency budget: 1–2 cycles; pipelined; feeds the mover's request issue.
- **Hit** → substitute translated PA, set `AT=AT_TRANSLATED`, issue.
- **Miss** → allocate a pending-translation slot, ask Xlate Agent, stall *only that
  request's* issue (other requests/channels proceed).
- Invalidation: `ATC_INVAL` CSR (PASID+addr) and host-initiated ATS invalidation
  messages via PAA → clear matching entries; in-flight requests to an invalidated
  entry complete or are re-translated ⚑RECONCILE policy.

---

## 3. Translation Agent (ATS)
- On ATC miss: issue **ATS Translation Request** via `ats_pri_if.ats_req_*`
  (IOVA, PASID, R/W, request-id).
- On **Translation Completion** (`ats_rsp`): 
  - `fail=1` → `ERR_ATS_FAIL`, abort the owning transfer.
  - `non_present=1` → hand off to **PRI Fault Engine** (§4).
  - else → fill ATC, wake the pending request.
- Outstanding translation requests tracked by `ats_req_id` (small pool); timeout → error.

---

## 4. PRI Fault Engine (page-fault suspend/resume)
State per fault context (`PRI_CONTEXTS`, default 64); ECC table.

**FSM**
```
IDLE ─(non_present)→ SUSPEND ─→ PAGE_REQ ─→ WAIT_PRG ─(success)→ RETRY_XLATE ─→ RESUME
                        │                        │(invalid/fail)→ ABORT
                        │                        │(timeout)─────→ ABORT (PRI_TIMEOUT)
```
1. **SUSPEND**: capture fault context = `{channel, dir, element ptr, byte offset,
   IOVA, PASID, R/W, tag(s) in flight}`. Quiesce *this transfer* at a clean
   boundary; **release shared resources** (tags/credits) so siblings run.
2. **PAGE_REQ**: allocate a **PRG index**, issue Page Request via `pri_req_*`.
3. **WAIT_PRG**: await `pri_rsp_*` matching the PRG index.
   - success → **RETRY_XLATE** (re-issue ATS; page now present) → **RESUME** from
     saved offset.
   - invalid/failure → **ABORT** with `ERR_PRI_FAIL`.
   - timeout (`PRI_CTRL` cfg) → **ABORT** with `ERR_PRI_TIMEOUT`.
4. **RESUME**: reload context, continue the element from `byte offset`.

**Isolation guarantee:** fault contexts, tag pools, and NP-credit accounting are
partitioned so a channel in WAIT_PRG (potentially very long — host page-in) does
not hold the shared requester or stall other channels. The arbiter skips
suspended channels.

---

## 5. PASID plumbing
- Per-channel `PASID`/`PASID_EN`/`priv` from CSR (`RD/WR_CHn_PASID`).
- Carried on every lookup (ATC tag), ATS request, and PRI request for that channel.
- PAA inserts the PASID TLP prefix on the resulting memory requests (see PAA §2).
- HDMA-native element PASID (if present) overrides channel value ⚑RECONCILE.

---

## 6. Interaction with data movers
- Request issue path: **mover → ATC lookup → (hit) issue | (miss) stall | (fault) suspend**.
- BDP impact: ATC-miss adds translation RTT; PRI adds page-in latency (huge).
  Therefore the reorder/outstanding buffers size for *ATC-hit* steady state, and
  faulting transfers are parked out-of-band (spec §4/§8).
- Ordering: a suspended write transfer must not let later writes to the same page
  pass it — enforce per-page fence on suspend ⚑RECONCILE.

---

## 7. PRI-disabled / ATS-only fallback
If `PRI_CTRL.en=0` (host lacks PRI or driver pins memory): ATC not-present →
`ERR_ATS_FAIL` (no page request). Driver must pre-fault/pin (`ATS-only` SVM or
classic pinned DMA).

---

## 8. Verification hooks (for V milestone)
- IOMMU/host BFM modelling: ATS translation (present/not-present/fail), PRI PRG
  responses (success/invalid/timeout), ATS invalidation messages.
- Directed cases: mid-element fault + resume; two channels, one faulting;
  invalidation racing an in-flight translated request; PRI timeout.

---

## 9. Reconciliation checklist
1. DWC ATS/PRI app-layer message format & flow (databook rev **____**).
2. PRG response code encoding (`pri_rsp_code`) ⚑RECONCILE.
3. Host IOMMU/SVA target (x86 VT-d vs ARM SMMUv3) → PASID width, invalidation semantics.
4. ATS invalidation completion requirements (must we ack? drain in-flight?).
5. Max concurrent page-request groups the host supports (`PRI_CONTEXTS` cap).
