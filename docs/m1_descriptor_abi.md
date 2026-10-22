# M1 — Descriptor ABI (HDMA-native linked list)

Defines the in-memory descriptor format the engine fetches and the software
(dmaengine driver) builds. HDMA-native. ⚑RECONCILE all bit positions and the
element layout against the DWC_pcie HDMA databook + `dw-hdma-v0-core.c`.

---

## 1. Data element (6 DWORDs / 24 bytes)

Little-endian; DWORD0 at the lowest address. (`dma_pkg::hdma_elem_t`.)

| DWORD | Byte off | Field | Meaning |
|---|---|---|---|
| 0 | 0x00 | `control` | see §2 |
| 1 | 0x04 | `size` | transfer bytes for this element (≤ 256 MB ⚑RECONCILE) |
| 2 | 0x08 | `sar_lo` | source addr [31:0] |
| 3 | 0x0C | `sar_hi` | source addr [63:32] |
| 4 | 0x10 | `dar_lo` | dest addr [31:0] |
| 5 | 0x14 | `dar_hi` | dest addr [63:32] |

- **Read channel (H2C):** `SAR` = host/IOVA source, `DAR` = device (on-card DDR) dest.
- **Write channel (C2H):** `SAR` = device source, `DAR` = host/IOVA dest.
- **AXI-Stream mode** (`CHn_STREAM_CTRL.mode=1`): the *device-side* address is
  **ignored** (stream has no address). H2C: `DAR` ignored, data pushed to the
  AXI-Stream master; `TUSER` seeded from `CHn_STREAM_CTRL[31:16]` + `TLAST` at
  element `size` end / EOP flag. C2H: `SAR` ignored, data pulled from the slave.

## 2. `control` DWORD bitfields ⚑RECONCILE

| Bit | Name | Meaning |
|---|---|---|
| 0 | `CB` | **Cycle Bit** — ownership toggle (see §4) |
| 1 | `TCB` | Toggle-CB marker (end of cyclic ring) |
| 2 | `LLP` | =1 → this is a **link element** (§3), not a data element |
| 3 | `LIE` | Local Interrupt Enable — raise done IRQ after this element |
| 4 | `RIE` | Remote Interrupt Enable — generate remote (host) IMWr |
| … | reserved | ⚑RECONCILE HDMA-specific bits (e.g. EOP/SOP for stream) |

## 3. Link element
When the engine reads an element with `control.LLP=1`, it treats DWORD2/3 as the
**next linked-list pointer** (lo/hi) and jumps. `control.CB` on the link element
participates in ownership. Enables lists longer than one contiguous chunk.

```
[data][data]...[data][LINK -> next chunk base]
```

## 4. Cycle-bit ownership (lockless producer/consumer)
- Engine consumes elements whose `CB` matches the channel's expected phase
  (`CONTROL1.CB` / `CYCLE_SYNC`). SW writes elements then flips `CB` last → HW
  sees a fully-formed element atomically. On wrap, expected phase toggles (`TCB`).
- **Cyclic mode** (`prep_dma_cyclic`): ring reused indefinitely; `CB`/`TCB`
  toggling drives period completions (audio-style).

## 5. dmaengine build mapping
| dmaengine op | Element construction |
|---|---|
| `prep_slave_sg` | one data element per `scatterlist` entry; last has `LIE`; set `LLP` head + doorbell |
| `prep_dma_memcpy` | 1..few data elements |
| `prep_dma_cyclic` | ring of elements, `CB`/`TCB` toggling, periodic `LIE` |
| residue (`tx_status`) | `size` sum − `XFERRED_LO/HI` (per-channel counter) |
| `terminate_all` | `CH_EN`=0 / abort → drain, status in `CH_STATUS` |

## 6. SVM / PASID
PASID is **per-channel** (legacy element has no PASID field): driver programs
`RD/WR_CHn_PASID` (value + `PASID_EN` + priv). The PAA inserts the PASID TLP
prefix on every request for that channel. HDMA-native *may* carry PASID/func in
the element — ⚑RECONCILE; if so, element field overrides channel register.

## 7. Segmentation rules the mover applies (transparent to SW)
- Never let a PCIe request cross a **4 KB** address boundary.
- Reads ≤ **MRRS**; writes ≤ **MPS**; reassemble read completions per **RCB**.
- Each element's `size` may fan out into many TLP-sized requests.

## 8. Alignment / limits ⚑RECONCILE
- Element array/link pointers: DWORD (4 B) aligned; recommend 32 B alignment.
- Max `size` per element and max elements per list: confirm vs databook.
- Address width: full 64-bit; IOVA when SVM enabled.
