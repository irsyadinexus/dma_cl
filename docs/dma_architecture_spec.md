# PCIe Gen4 Scatter-Gather DMA Engine — Architecture & Micro-Spec (v0.4)

**Status:** Draft for review — architecture/spec phase (no RTL yet)
**Owner:** irsyadimuh@gmail.com
**Date:** 2026-07-29

> v0.2 folds in the full interview: Synopsys DWC PCIe controller, real AXI4-MM
> device path, Linux `dmaengine` compatibility via an **eDMA/HDMA-compatible ABI**,
> **ATS + PASID + PRI (full SVM)**, and a **1024-bit @ ~500 MHz** datapath.
> v0.3: sharpened AXI-Stream framing (§3.2, §10.1 — dmaengine supports streams
> natively; ST delta is a HW mode bit, not a new API), recorded the QDMA/XDMA/mSGDMA
> alternatives weighed (§10.2), and confirmed **eDMA/HDMA + ST extension** with
> **driver strategy deferred to M1**.
> v0.4: corrected the datapath-sizing math (§4) — **512b @ 500 MHz = 256 Gb/s ≥
> 252 Gb/s meets x16 capacity**; width is not the gate. Flipped `DATA_WIDTH`
> **default to 512** (1024b = timing-relief alt), and marked **straddle enabled**
> (required to hit line rate regardless of width).

---

## 0. Locked design decisions

| Decision | Value | Consequence |
|---|---|---|
| Technology | ASIC / vendor-agnostic RTL core | Core is portable; **DWC PCIe** app-layer shim is the only vendor-specific block |
| PCIe controller | **Synopsys DesignWare (DWC) PCIe Gen4** | PAA targets DWC native application (raw-TLP) interface; controller provides iATU, ATS/PRI plumbing, MSI-X, AER |
| Language | SystemVerilog (RTL + TB) | SV interfaces/modports; UVM later |
| Role | PCIe **Endpoint**, multi-channel | Host-driven; eDMA-style **Read** (H2C) + **Write** (C2H) channels |
| Transfer model | Scatter-gather **linked-list** descriptors | **eDMA/HDMA-compatible** LL format so `dw-edma` driver works |
| Driver ABI | **eDMA/HDMA-compatible** register + descriptor layout | Reuse upstream Linux `dw-edma`/`dw-hdma` dmaengine driver; AXI-Stream is a flagged extension |
| Device datapath | **AXI4-MM** (real, on-card DDR) + AXI4-Stream (extension) | MM path is first-class; streaming needs a driver addendum |
| Address translation | **ATS + PASID + PRI (full SVM)** | New subsystem: device TLB (ATC), translation agent, PASID prefixes, **page-fault suspend/resume** |
| Link target | Gen4 **x16** (~252 Gb/s ≈ 31.5 GB/s TLP, per dir, full-duplex) | Deep outstanding-read pipeline; straddled datapath |
| Datapath | `DATA_WIDTH` param, **primary = 512b @ ~500 MHz** (= 256 Gb/s ≥ 252) | **Straddle enabled** (line rate needs it at any width); 1024b @ ~250–300 MHz = timing-relief alt; match DWC app-interface width |
| Deliverable (this phase) | Architecture + spec, register map, interface defs | RTL/verification/driver are later milestones |

---

## 1. Scope & requirements

### 1.1 Functional
- **FR-1** Move data **Host memory → device** (eDMA **Read** channel / H2C) and **device → Host memory** (eDMA **Write** channel / C2H) over PCIe Gen4.
- **FR-2** Device-side datapath: **AXI4-MM** (host ↔ on-card DDR/addressable local memory) as first-class; **AXI4-Stream** as a per-channel extension (host ↔ streaming accelerator/NIC).
- **FR-3** **Scatter-gather** via **eDMA/HDMA-compatible linked lists** in host memory; engine walks the list, executes elements, generates done/abort interrupts (IMWr/MSI-X).
- **FR-4** Multiple independent channels: up to **`NUM_RD_CH` Read + `NUM_WR_CH` Write** (default 4+4; eDMA supports up to 8+8, HDMA up to 16+16).
- **FR-5** 64-bit host addressing; host DRAM **and** peer-EP BAR targets (P2P — see §11).
- **FR-6** **ATS + PASID + PRI**: device TLB, translation requests, PASID-tagged SVM, and in-flight page-fault handling via Page Request Interface (§8).
- **FR-7** Software control through an **eDMA/HDMA-compatible register block** (BAR-mapped, `dw-edma` driveable).
- **FR-8** MSI-X interrupts via DWC controller (done/abort IMWr per channel + error vectors).
- **FR-9** Linux `dmaengine` compatibility: `prep_slave_sg`, `prep_dma_memcpy`, `prep_dma_cyclic`, residue reporting, `terminate_all` (§10).

### 1.2 Non-functional
- **NFR-1** Sustain ≥ 90% of Gen4 x16 usable BW on ≥64 KB transfers, per direction — **on translated (ATC-hit) traffic**. Page-fault paths isolated so one channel's fault can't stall others.
- **NFR-2** Synthesizable, vendor-neutral SV-2012 core; DWC specifics confined to the PAA shim.
- **NFR-3** ECC (SECDED) on all SRAMs (reorder, ATC, descriptor prefetch, PRI state), parametric.
- **NFR-4** Clean CDC: `pcie_app_clk` (DWC app ~500 MHz), `core_clk`, `axi_clk` (user), `csr_clk`.
- **NFR-5** Register map + descriptor ABI versioned; strictly track the DWC eDMA/HDMA databook revision used.

### 1.3 Explicit non-goals (this phase)
- No PCIe controller/PHY RTL (DWC IP; we define the app-layer contract, §3).
- No NVMe command/queue processing in HW (SSD = memory addresses; §11). Reserved hook only.
- No inline crypto/compression (hook points reserved).

---

## 2. Top-level block diagram

```
   Synopsys DWC PCIe Gen4 Controller (+PHY)   [external IP]
   - DL/TL, iATU, ATS/PRI plumbing, MSI-X, AER, config space
        ▲  native app (raw TLP)  │  ATS/PRI msgs
        │  MRd/MWr/Cpl, prefixes │
 ┌──────┴────────────────────────▼───────────────────────────────────────────┐
 │                        DMA ENGINE (this project)                           │
 │  ┌──────────────────────────────────────────────────────────────────────┐ │
 │  │   PCIe Application Adapter (PAA)  — DWC shim (portability boundary)    │ │
 │  │   Requester: MRd/MWr + PASID/AT prefixes | Completer: BAR | ATS/PRI IF │ │
 │  └───┬──────────────┬─────────────────┬──────────────────┬───────────────┘ │
 │      │ req/cpl      │ ATS req/rsp     │ PRI page-req/rsp  │ BAR CSR I/O     │
 │  ┌───▼──────┐   ┌───▼─────────────────▼───────┐    ┌──────▼──────────────┐  │
 │  │ Tag Mgr +│   │  ATS/PASID/PRI Subsystem     │    │ eDMA/HDMA-compatible│  │
 │  │ Reorder/ │◄─►│  ATC (device TLB) | Xlate    │    │ Register Block +    │  │
 │  │ Reassy   │   │  Agent | PRI fault engine    │    │ MSI-X shadow        │  │
 │  └───┬──────┘   └──────────────┬──────────────┘    └──────┬──────────────┘  │
 │      │ data           translated addr / stall/fault       │ ctrl/status     │
 │  ┌───▼───────────────────────────────────────────────────▼──────────────┐  │
 │  │                  Channel Arbiter / QoS (weighted RR)                   │  │
 │  └───┬───────────────┬───────────────┬───────────────┬───────────────────┘  │
 │   ┌──▼──┐         ┌──▼──┐         ┌──▼──┐         ┌──▼──┐  per channel       │
 │   │RdCh0│  ...    │RdChN│         │WrCh0│  ...    │WrChN│                    │
 │   │ LL  │  descriptor engine (walk eDMA/HDMA linked list, IMWr on done)     │
 │   │walk │  data mover: 4KB split, MPS/MRRS seg, ATC lookup, PRI on miss     │
 │   └──┬──┘         └─────┘         └──┬──┘         └─────┘                    │
 │      │ AXI4-MM (first-class)  or  AXI4-Stream (extension, per channel)      │
 └──────┼─────────────────────────────────────────────────────────────────────┘
        ▼
   User logic: on-card DDR controller (AXI4-MM)  /  accelerator (AXI-Stream)
```

---

## 3. External interface contract (DWC PCIe)

The **PAA** is the only DWC-specific block. It maps our internal vendor-neutral contract to the **DWC PCIe native application interface** (raw TLP), chosen over the AXI-bridge flavor because x16 line-rate needs full tag/credit/prefix control.

### 3.1 PAA ⇄ DWC controller
- **Requester TX:** assemble MRd/MWr TLP headers; insert **PASID TLP prefix** and set **AT[1:0]** field (00=untranslated, 10=translated) per request; honor DWC transmit credit/ready.
- **Completer RX/TX:** DWC delivers BAR-targeted reads/writes (our CSR + optional slave bridge); we return completions.
- **ATS interface:** issue **Translation Request** TLPs, receive **Translation Completion** (with translated addr + size + R/W perms) → fill ATC.
- **PRI interface:** issue **Page Request Messages**, receive **PRG Response (PRGR)** messages → wake suspended transfers. (DWC controller carries the messages; the state machine is ours.)
- **MSI-X:** transfers' done/abort generate **IMWr** (interrupt message write) — DWC integrated MSI-X. CSR keeps a shadow of vector config.
- **Config space / AER / iATU:** owned by the DWC controller; we consume link status via sideband.

**Internal requester contract (vendor-neutral, what the DMA core sees):**
```
req_valid/ready, req_is_read, req_addr[63:0], req_len_bytes, req_tag[9:0],
   req_pasid[19:0], req_pasid_en, req_at[1:0], req_attr(TC/NoSnoop/RO),
   req_wdata[DW-1:0], req_first_be, req_last_be
cpl_valid/ready, cpl_tag, cpl_rdata[DW-1:0], cpl_byte_count, cpl_lower_addr,
   cpl_status(SC/UR/CA/CRS), cpl_last
```

### 3.2 Device/user datapath (per channel)
- **AXI4-MM** (first-class): master, `WDATA/RDATA=DATA_WIDTH`, `AXI_ID_W`, INCR bursts ≤256 beats → on-card DDR / addressable local memory.
- **AXI4-Stream** (extension): master for Read/H2C, slave for Write/C2H; `TDATA=DATA_WIDTH`, `TKEEP/TLAST/TUSER`. The `dmaengine` *API* already supports streams natively (`slave_sg`/`cyclic` with the stream as the "device" endpoint) — the gap is purely that the **eDMA/HDMA descriptor carries both SAR and DAR**, i.e. assumes both ends are addressable. A stream-mode channel needs a HW **mode bit** (`CHx_STREAM_CTRL`) that says "ignore the local address, push/pull the FIFO," plus a driver flag to set it — **not** a new transfer type. See §10.1.

### 3.3 CSR: **AXI4-Lite / APB** slave on `csr_clk`; BAR0 → eDMA/HDMA register aperture (§9), BAR2/4 → optional user-memory slave bridge.

---

## 4. Throughput / datapath sizing (x16 Gen4)

**Link budget.** x16 Gen4 = 16 GT/s × 16 × (128/130) ≈ **252 Gb/s ≈ 31.5 GB/s** of TLP-layer bandwidth (headers+payload), **each direction, full-duplex**. After TLP-header/DLLP/ACK overhead, delivered **payload** is ~0.85–0.92× ⇒ **~215–230 Gb/s** per direction.

**Width — capacity is met at 512b; straddle/timing are the real drivers.**
`512 b × 500 MHz = 256 Gb/s = 32 GB/s ≥ 252 Gb/s`, so **`DATA_WIDTH = 512` @ ~500 MHz is the primary config** and clears raw x16 capacity (~1.5% margin). Width is *not* a raw-capacity gate here. What actually governs line rate:
- **Straddle (enabled, `STRADDLE_EN=1`).** At 64 B/beat a beat-aligned TLP wastes header+alignment slots — ~63% beat-efficiency for 64 B TLPs, ~85% @256 B, ~92% @512 B MPS. To reach line rate on realistic TLP sizes the PAA **must straddle**: pack >1 TLP/beat and start TLPs mid-beat (both RX de-straddle and TX pack). Required at *any* width; see PAA contract §4.
- **Margin.** 256 vs 252 = ~1.5% raw headroom — eaten by pipeline bubbles, CDC fill/drain, credit-return stalls, partial last beats. Straddle + adequate FIFO depth recover it; if short, push clock or widen.
- **Two datapaths, per direction.** Size **separate RX (completions→device) and TX (device→writes)** paths at 512b@500 each — full-duplex makes the directions independent; do *not* build one shared bus (would need 2×). Device-side (payload-only) carries ~215–230 Gb/s ⇒ *more* margin than the PCIe (TLP) side. (Note: a single on-card-DDR AXI port serving both directions at once must handle aggregate ≈450 Gb/s — a DDR-sizing concern, not the PCIe datapath.)
- **1024b @ ~250–300 MHz = timing-relief alternative.** Same/greater throughput at a slower, easier-to-close clock, with 2 TLPs/beat naturally. Use if 500 MHz timing on the wide DMA logic is hard, or if the DWC x16 config presents a 1024b app interface. `DATA_WIDTH` stays parameterized — **match whatever the DWC application interface presents** to avoid a width-conversion gearbox.
- **Outstanding-read (BDP) — the make-or-break number:** RTT ≈ 0.5–1 µs ⇒ ~**30 KB in flight per direction** to keep the pipe full. **With SVM/ATS the effective RTT grows** (ATC lookup; page-in on PRI is *very* long) — so the pipeline must **decouple faulting from non-faulting transfers per channel**.
  - Support **10-bit extended tags** (up to 1024 outstanding).
  - **Reorder/reassembly buffer ≥ 64 KB per active read channel** (ECC SRAM), `REORDER_DEPTH` parameter.
  - **ATC** sized so steady-state working set hits in-cache (misses cost translation RTT, faults cost page-in).

---

## 5. Descriptor model — eDMA/HDMA-compatible linked list

To let the upstream `dw-edma`/`dw-hdma` driver work, we adopt the **DWC linked-list** model. Exact bit encodings **locked against the Synopsys DWC_pcie / eDMA / HDMA databook in M1** — below is the structural contract.

### 5.1 Data element (eDMA legacy LL, 6 DWORDs / 24 B)
| DWORD | Field | Meaning |
|---|---|---|
| 0 | **Control** | `CB` cycle bit, `TCB` toggle-CB, `LLP` load-link-ptr, `LIE` local-IRQ-en, `RIE` remote-IRQ-en |
| 1 | **Transfer size** | bytes for this element |
| 2 | **SAR low** | source addr [31:0] |
| 3 | **SAR high** | source addr [63:32] |
| 4 | **DAR low** | dest addr [31:0] |
| 5 | **DAR high** | dest addr [63:32] |

**Link element:** Control (`CB`,`LLP=1`,`TCB`) + `LLP low/high` → next list chunk. **Cycle bit** gives lockless HW/SW ownership; **cyclic mode** toggles CB (maps to `prep_dma_cyclic`).
- **Direction convention (eDMA):** *Read* channel = remote(host)→local = **H2C**; *Write* channel = local→remote(host) = **C2H**.
- **HDMA native variant** (Gen4/5 high-perf) uses a different element layout + "native" register map with more channels — supported as the **primary** target (better for x16); eDMA legacy as fallback. Both served by `dw-edma`.
- **PASID/AT** are *not* in the legacy element → supplied **per-channel** via config register (§8/§9), or via HDMA native element fields where available.

---

## 6. Channel micro-architecture

Each channel = **LL descriptor engine + data mover**, direction-fixed (Read=H2C / Write=C2H) per eDMA.
- **Read/H2C mover:** issue MRd (segmented to MRRS & 4 KB, AT-tagged, PASID-prefixed) → completion reassembly (tag-indexed reorder, RCB-aware) → drive AXI4-write or AXI-Stream-master.
- **Write/C2H mover:** pull from AXI4-read or AXI-Stream-slave → packetize MWr (MPS & 4 KB seg) → posted writes.
- **Segmentation unit:** never cross **4 KB** boundary; obey **MPS/MRRS**; honor **RCB**.
- **ATC lookup in-path:** every request address translated via ATC before issue; miss → translation request; not-present → PRI fault path (§8).
- `NUM_RD_CH`/`NUM_WR_CH` parameterized; symmetric per-channel CSR bank.

---

## 7. Tag manager & completion reordering (throughput-critical)

- **Tag allocator:** free-list ≤1024 (10-bit ext tags). Per MRd: {tag → channel, buffer offset, expected bytes, translated addr}.
- **Reassembly:** completions arrive out-of-order & RCB-split; tag-indexed reorder buffer reconstructs contiguous data; byte-count scoreboard releases in order.
- **Completion timeout** per tag → error status + IRQ. Enforce PCIe posted/non-posted ordering (RO attr optional).

---

## 8. ATS / PASID / PRI subsystem (full SVM) — highest complexity

- **ATC (device TLB):** IOVA→PA translations w/ size + R/W/priv perms + PASID tag. Parameterized entries (`ATC_ENTRIES`), ECC, set-associative + LRU. Lookup on every request before issue.
- **Translation Agent:** on ATC miss, issue **ATS Translation Request** (via PAA/DWC); on **Translation Completion**, fill ATC and proceed. Requests thereafter use **AT=10 (translated)**.
- **PASID:** per-channel `PASID` + `PASID_EN` config register; PAA inserts the **PASID TLP prefix**. Enables per-process SVM. (HDMA native elements may carry PASID; legacy uses the channel register.)
- **PRI (Page Request Interface) — page-fault handling:**
  1. Translation Completion / ATS response indicates **page not present**.
  2. Engine **suspends the faulting transfer**, saving state (channel, current element, byte offset, tag) into a **PRI fault-context table**; frees non-fault resources so **other channels keep running**.
  3. Issue **Page Request Message** (PASID + address + R/W), allocate a **PRG group tag**.
  4. On **PRG Response (PRGR)** success → re-issue translation, **resume** transfer from saved offset. On failure/invalid → **abort** transfer, error status + IRQ.
  5. **PRI response timeout** → abort + fatal/non-fatal classification.
- **Isolation guarantee (NFR-1):** per-channel fault contexts + separate credit accounting so a stalled (page-faulting) channel never head-of-line-blocks others.
- **DWC role:** carries ATS/PRI TLPs & messages, integrates with system IOMMU; the **state machines/timeouts/replay are ours**.

---

## 9. Register map — eDMA/HDMA-compatible (BAR0)

Mirrors the **DWC eDMA/HDMA "unrolled" register map** so `dw-edma`/`dw-hdma` binds. Representative layout; **offsets/bitfields locked to the databook in M1.**

**Engine-level**
| Register | Notes |
|---|---|
| `DMA_CTRL` | #read/#write channels present |
| `DMA_WRITE_ENGINE_EN` / `DMA_READ_ENGINE_EN` | enable Write(C2H)/Read(H2C) engines |
| `DMA_WRITE_DOORBELL` / `DMA_READ_DOORBELL` | write channel# to start |
| `DMA_WRITE_INT_STATUS/MASK/CLEAR` | per-channel done/abort |
| `DMA_READ_INT_STATUS/MASK/CLEAR` | per-channel done/abort |
| `DMA_WRITE_ERR_STATUS` / `DMA_READ_ERR_STATUS` | LL error / app error / fetch error |
| `DMA_WRITE_DONE_IMWR_*` / `DMA_WRITE_ABORT_IMWR_*` | MSI-X message addr/data |
| `DMA_WRITE_CH_ARB_WEIGHT` / `DMA_READ_CH_ARB_WEIGHT` | QoS weights (§ arbiter) |

**Per-channel (unrolled, Read n / Write n)**
| Register | Notes |
|---|---|
| `CH_CONTROL1` | `CB`, `LLE` (LL enable), `CCS`, `LLP_EN`, `LIE`/`RIE`, `FUNC_NUM` |
| `CH_CONTROL2` | reserved/ext |
| `TRANSFER_SIZE` | (register mode) bytes |
| `SAR_LOW/HIGH`, `DAR_LOW/HIGH` | (register mode) addresses |
| `LLP_LOW/HIGH` | linked-list head pointer |

**Vendor-specific extension region (kept out of the eDMA-standard aperture)**
| Register | Notes |
|---|---|
| `CHx_STREAM_CTRL` | select AXI4-MM vs **AXI4-Stream** device mode; TUSER seed |
| `CHx_PASID` / `CHx_PASID_CTRL` | PASID value + enable (SVM) |
| `ATS_CTRL` / `ATC_INV` | ATC enable / invalidation |
| `PRI_CTRL` / `PRI_STATUS` | PRI enable, fault-context status, timeout cfg |
| `P2P_CTRL` | peer-to-peer routing hints |
| `ID_VERSION` / `CAPABILITIES` | our extensions + databook rev tag |

> The extension region is invisible to stock `dw-edma`; only the driver addendum touches it (AXI-Stream, SVM tuning, P2P).

---

## 10. Linux `dmaengine` mapping (dw-edma / dw-hdma)

| dmaengine API | HW behavior |
|---|---|
| channel = eDMA Read/Write channel | `DMA_DEV_TO_MEM` ↔ Read(H2C); `DMA_MEM_TO_DEV` ↔ Write(C2H) |
| `device_prep_slave_sg` | build LL data elements from scatterlist; set `LLP`, start via doorbell |
| `device_prep_dma_memcpy` | single/few-element LL |
| `device_prep_dma_cyclic` | cyclic LL via toggling `CB`/`TCB` |
| `device_tx_status` (residue) | remaining bytes from unconsumed elements / `TRANSFER_SIZE` |
| `device_terminate_all` | channel stop/abort + LL drain |
| ISR | done/abort IMWr → MSI-X → complete cookies |
| **SVM (ATS/PASID/PRI)** | driver programs `CHx_PASID`, enables ATS/PRI; binds to IOMMU SVA (`iommu_sva_bind_device`) |
| **AXI-Stream mode** | **driver addendum** sets `CHx_STREAM_CTRL` (not in stock dw-edma) |

**Positioning note:** DWC PCIe already ships eDMA/HDMA. This project is justified only by the **extensions** — AXI4-Stream device path, custom QoS/channel count, P2P, and owning the ATS/PRI datapath. The MM/SVM core stays eDMA-ABI-compatible so we inherit the mature driver; extensions ride the vendor region.

### 10.1 AXI-Stream: what "extension" actually costs

`dmaengine` is protocol-agnostic — it models **direction/transfer type**, not AXI-MM vs AXI-Stream:
- **AXI-MM ↔ AXI-MM** → `DMA_MEM_TO_MEM` / `device_prep_dma_memcpy`.
- **AXI-MM ↔ AXI-Stream** → `DMA_MEM_TO_DEV`/`DMA_DEV_TO_MEM` via `device_prep_slave_sg`/`device_prep_dma_cyclic`, with the **stream as the "device" endpoint**. This is exactly how `xilinx_dma` drives the Xilinx AXI DMA IP.

So streaming is a **first-class dmaengine capability already**. The only reason it's an "extension" here is the **eDMA/HDMA descriptor model is MM-centric** (both SAR and DAR present). The delta to add ST is therefore small and well-bounded:
1. HW: a per-channel `CHx_STREAM_CTRL` mode bit + datapath tap to the AXI-Stream port (skip the device-side address).
2. Driver: set that bit for ST channels; the client call (`slave_sg`, dir = `DEV_TO_MEM`/`MEM_TO_DEV`) is **identical** to the MM slave path.

### 10.2 Considered alternatives (native MM + ST) — recorded rationale

Evaluated ABIs that natively support both MM and Stream, and why we stayed with **eDMA/HDMA + ST extension**:
| Alternative | Native both? | Why not chosen (this project) |
|---|---|---|
| **Xilinx QDMA** (per-queue MM/ST, CMPT ring, SR-IOV) | Yes, best x16 fit | Queue/DPDK-oriented driver, not clean `dmaengine`; heavy driver port; Xilinx-tied ABI |
| **Xilinx XDMA** (configurable MM/ST) | Yes | Lower ceiling; still a Xilinx ABI/driver to port; less proven for full SVM |
| **Intel/Altera mSGDMA** (MM/ST dispatcher) | Yes | Avalon/Intel-oriented; no clean dmaengine provider |
| **eDMA/HDMA + ST extension** ✅ chosen | MM native, ST via mode bit | Reuses mature `dw-edma`; ST delta is small (§10.1); stays in DWC family |

**Driver strategy: decided at M1** (custom dmaengine provider vs. port QDMA/XDMA vs. DPDK-first) — HW ABI is frozen first, driver choice follows once interfaces are locked.

---

## 11. Storage / SSD path (memory-address model)
- **Default (host-mediated):** OS stages SSD data to host DRAM (SVM-mapped); engine DMAs those addresses.
- **P2P (optional):** element address points at a peer EP BAR (e.g. NVMe CMB); DWC/switch routes P2P. Engine stays storage-agnostic.
- **Reserved:** an NVMe queue block beside this engine could feed LLs later — not built.

---

## 12. Error handling
- PCIe: completion timeout, UR/CA, poisoned TLP, **ATS/PRI failure & timeout** → per-channel + global class (AER-friendly via DWC).
- Data: SRAM ECC (ATC/reorder/PRI/prefetch), LL fetch/format errors, 4 KB/size violations pre-issue.
- Recovery: per-channel abort (drain+status), engine disable, soft reset. Surfaces in eDMA `*_INT_STATUS`/`*_ERR_STATUS` + IMWr/MSI-X.

---

## 13. Clock/reset/DFT
- Domains: `pcie_app_clk` (DWC ~500 MHz), `core_clk`, `axi_clk`, `csr_clk`. Async FIFOs on data boundaries; 2-FF + handshake on control; doorbells gray-coded csr→core. Formal CDC lint required.
- Reset: async-assert/sync-deassert per domain; soft reset drains in-flight (incl. PRI contexts) then resets datapath.
- DFT: scan flops, MBIST on all SRAMs, ECC everywhere.

---

## 14. Parameters (top-level)

```systemverilog
parameter int DATA_WIDTH    = 512;   // primary x16 (512b@500MHz=256Gb/s>=252); 1024=timing-relief
parameter bit STRADDLE_EN   = 1;     // multi-TLP/beat pack+de-straddle (needed for line rate)
parameter int NUM_RD_CH     = 4;     // eDMA Read (H2C); up to 8 (eDMA)/16 (HDMA)
parameter int NUM_WR_CH     = 4;     // eDMA Write (C2H)
parameter int AXI_ID_W      = 8;
parameter int MAX_TAGS      = 1024;  // 10-bit extended tags
parameter int REORDER_DEPTH = 65536; // bytes per active read channel (ECC SRAM)
parameter int ATC_ENTRIES   = 1024;  // device TLB
parameter int PRI_CONTEXTS  = 64;    // concurrent page-fault contexts
parameter bit ECC_EN        = 1;
parameter bit AXIS_EN       = 1;     // AXI4-Stream extension
parameter bit ATS_EN        = 1;
parameter bit PASID_EN      = 1;
parameter bit PRI_EN        = 1;
parameter bit P2P_EN        = 1;
parameter int PCIE_VARIANT  = 1;     // 0=eDMA-legacy map, 1=HDMA-native map
```

---

## 15. Milestones

| M | Deliverable | Exit criteria |
|---|---|---|
| **M0 (now)** | This architecture spec (v0.2) | Reviewed & signed off |
| **M1** | Bit-level register map locked to DWC databook; eDMA/HDMA descriptor ABI; DWC-PAA signal contract; ATS/PRI micro-spec; SV `interface` skeletons | Interfaces frozen; `dw-edma` binding plan agreed |
| **M2** | RTL: DWC-PAA (raw-TLP) + eDMA-compatible CSR + 1 Read + 1 Write channel, register mode, DATA_WIDTH=512 | Directed sim moves data both dirs; `dw-edma` sees registers |
| **M3** | RTL: LL descriptor engine + tag mgr + reorder + multi-channel + arbiter | SG linked-list transfers pass |
| **M4** | ATS + PASID datapath + ATC | Translated traffic passes in sim (IOMMU BFM) |
| **M5** | **PRI** fault suspend/resume + isolation | Page-fault injected mid-transfer; other channels unblocked |
| **M6** | AXI4-Stream extension + P2P + ECC/errors | Feature-complete at 512b + driver addendum |
| **M7** | 1024b straddle datapath + BDP/SVM latency tuning | Sustains ≥90% x16 (ATC-hit) in sim |
| **V (parallel)** | SV/UVM TB: AXI + DWC-TLP + ATS/PRI (IOMMU) BFMs, scoreboard, coverage | Coverage goals met |

---

## 16. Open questions / risks (for M1)
1. **HDMA-native vs eDMA-legacy** as primary map? (Recommend **HDMA-native** for x16 — more channels/throughput; confirm your DWC config licenses HDMA.) `PCIE_VARIANT` covers both.
2. **DWC databook revision** — which DWC_pcie/eDMA/HDMA release? Locks exact offsets/bitfields/element format for M1.
3. **PRI fault policy** — max concurrent faults (`PRI_CONTEXTS`), timeout value, and behavior on PRI-disabled hosts (fall back to pinned-memory/ATS-only?).
4. **IOMMU/SVA stack** — Linux `iommu_sva` on which host arch (x86 VT-d / ARM SMMUv3)? Affects PASID width & PRGR semantics in the BFM.
5. **AXI-Stream driver** — extend `dw-edma` in-tree or ship an out-of-tree provider for the streaming path?
6. **On-card DDR** — width/depth of the AXI4-MM target (sizes AXI bursts & back-pressure model).
7. **MSI-X vector budget** — vectors available from the DWC MSI-X table (per-channel done+abort + error).

---

*Next after sign-off (M1): lock the register map bitfields against the chosen DWC databook, write the DWC-PAA signal spec + ATS/PRI micro-spec, and emit the SV `interface` skeletons to freeze interfaces.*
