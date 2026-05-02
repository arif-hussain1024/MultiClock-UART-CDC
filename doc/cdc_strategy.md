# CDC Strategy Document: Multi-Clock UART Controller

## 1. Clock Domain Overview

The design operates across two independent clock domains:

| Domain | Clock | Typical Freq | Purpose |
|--------|-------|-------------|---------|
| APB/System | `pclk` | 50 MHz | CPU register access, FIFO write (TX), FIFO read (RX) |
| Serial | `sclk` | 25-100 MHz | UART TX/RX shift registers, baud rate generation |

These clocks are **asynchronous** (no guaranteed phase/frequency relationship), making every signal crossing between them a potential metastability hazard.

---

## 2. CDC Crossing Inventory

### 2.1 Data Paths (Multi-bit)

| Signal | Width | Source | Dest | Method | Rationale |
|--------|-------|--------|------|--------|-----------|
| TX data | 8 bits | pclk | sclk | Async FIFO (gray-code pointers) | Multi-bit data requires a storage element with safe full/empty handshaking. Gray-code pointers guarantee single-bit transitions for safe synchronization. |
| RX data | 8 bits | sclk | pclk | Async FIFO (gray-code pointers) | Same rationale as TX path. |
| Baud divisor | 16 bits | pclk | sclk | Multi-bit 2-FF sync | Safe ONLY because software is required to configure the divisor while TX/RX are disabled. The value is static when consumed. |

### 2.2 Control Signals (Single-bit)

| Signal | Source | Dest | Method | Rationale |
|--------|--------|------|--------|-----------|
| `tx_enable` | pclk | sclk | 2-FF synchronizer | Level signal, changes infrequently. No pulse information to preserve. |
| `rx_enable` | pclk | sclk | 2-FF synchronizer | Same as tx_enable. |
| `parity_en` | pclk | sclk | 2-FF synchronizer | Configuration bit, stable before use. |
| `parity_type` | pclk | sclk | 2-FF synchronizer | Configuration bit, stable before use. |

### 2.3 Status Signals (Single-bit)

| Signal | Source | Dest | Method | Rationale |
|--------|--------|------|--------|-----------|
| TX FIFO empty | sclk | pclk | 2-FF synchronizer | Level signal indicating FIFO state. Slight latency (2 pclk cycles) is acceptable for status reporting. |
| RX FIFO full | sclk | pclk | 2-FF synchronizer | Level signal. Small synchronization delay may cause 1-2 extra writes before CPU sees full status; the FIFO's own full logic prevents actual overflow. |
| `parity_error` | sclk | pclk | 2-FF synchronizer | Pulse in sclk domain. May be missed if pulse is shorter than 2 pclk cycles. Acceptable because the sticky error register in pclk domain captures the event. |
| `frame_error` | sclk | pclk | 2-FF synchronizer | Same as parity_error. |
| `rx_data_valid` | sclk | pclk | 2-FF synchronizer | Used for overrun detection. |

### 2.4 External Asynchronous Input

| Signal | Source | Dest | Method | Rationale |
|--------|--------|------|--------|-----------|
| `uart_rx` | External pin | sclk | 2-FF synchronizer (reset to 1) | Asynchronous external input. Reset value is 1 (idle high) to prevent false start-bit detection during reset. |

---

## 3. Synchronization Methods

### 3.1 Double-Flop Synchronizer (`sync_2ff`)

```
Source Domain          Destination Domain
   ┌───┐                 ┌───┐    ┌───┐
d ─┤ ? ├─── async ───────┤ FF├────┤ FF├──── q (safe)
   └───┘                 └─┬─┘    └─┬─┘
                           │        │
                          meta    resolved
                     (may be       (settled)
                     metastable)
```

**When to use:** Single-bit signals or multi-bit buses that are guaranteed to be stable (not transitioning) when sampled. This covers level signals (enables, config bits) and slowly-changing status flags.

**Latency:** 2 destination clock cycles.

**Limitation:** Cannot safely synchronize multi-bit buses that change more than one bit simultaneously. Cannot reliably capture short pulses (pulse must be wider than 1 destination clock period).

### 3.2 Asynchronous FIFO with Gray-Code Pointers

```
Write Domain (pclk)              Read Domain (sclk)
┌──────────────────┐             ┌──────────────────┐
│  wptr_bin        │             │  rptr_bin         │
│    │             │             │    │              │
│    ▼             │             │    ▼              │
│  bin2gray ──────────sync_2ff────► comparison       │
│    │             │             │    │              │
│  wptr_gray       │             │  rempty           │
│                  │             │                   │
│  comparison ◄────sync_2ff──────── bin2gray         │
│    │             │             │    │              │
│  wfull           │             │  rptr_gray        │
└──────────────────┘             └──────────────────┘
                    ┌──────────┐
                    │  Dual-   │
             wdata ─┤  Port    ├─ rdata
                    │  RAM     │
                    └──────────┘
```

**When to use:** Transferring streaming data between clock domains. The FIFO decouples producer and consumer rates while maintaining data integrity.

**Gray-code property:** Adjacent values differ in exactly one bit. This means the synchronized pointer in the receiving domain is always either the true current value or one step behind -- never a corrupted intermediate value.

**Full/Empty detection:**
- **Empty:** Read-side gray pointer equals the synchronized write-side gray pointer. This is conservative (may report empty slightly late), which is safe.
- **Full:** Write-side gray pointer matches the synchronized read-side pointer with the two MSBs inverted. Also conservative (may report full slightly late), preventing overflow.

**FIFO depth constraint:** Must be a power of 2 for gray-code math to work correctly.

---

## 4. Design Constraints and Software Requirements

### 4.1 Baud Rate Configuration
The 16-bit baud divisor register crosses from pclk to sclk using a multi-bit 2-FF synchronizer. This is **only safe** under the following software protocol:

1. Disable TX and RX (clear CTRL[1:0])
2. Wait for any active transfer to complete
3. Write the new baud divisor to BAUDDIV register
4. Wait at least 3 sclk cycles (for synchronization to settle)
5. Re-enable TX and RX

Violating this protocol (changing divisor mid-transfer) will corrupt the baud rate in the serial domain.

### 4.2 Error Signal Timing
Parity and frame error pulses from the sclk domain may be narrower than the pclk period. The design uses sticky error bits in the APB register set to ensure no error event is lost, even if the synchronized pulse is too narrow to be sampled.

### 4.3 Overrun Detection Latency
RX overrun detection crosses the CDC boundary. Due to synchronization latency, up to 2 additional bytes may be received (and dropped) before the CPU observes the overrun flag. This is inherent to the asynchronous design and acceptable for UART applications.

---

## 5. Verification Strategy

### 5.1 SVA Assertions
- **Synchronizer correctness:** No signal from a foreign clock domain is used without passing through a 2-FF synchronizer. Verified by structural inspection and assertions on pointer paths.
- **Gray-code integrity:** Assertions verify that gray-code pointers change by exactly one bit (Hamming distance = 1) on every increment.
- **FIFO safety:** Assertions verify no write when full, no read when empty.
- **APB compliance:** Assertions verify setup/access phase handshake, stable address/control during transaction.

### 5.2 Directed Tests
- Back-to-back transfers (continuous FIFO throughput)
- FIFO full/empty boundary transitions
- Parity error injection
- Baud rate change between transfers
- Loopback (TX to RX) data integrity
- APB protocol error (invalid address)
- Overrun condition

### 5.3 Coverage Goals
- All FIFO occupancy levels (empty, partial, full)
- All error conditions triggered at least once
- All interrupt sources fired and cleared
- Both parity modes (even, odd) exercised

---

## 6. Known Limitations

1. **No handshake-based register transfer** for baud divisor. A handshake or req/ack protocol would be more robust but adds complexity. The documented software constraint is acceptable for this application.

2. **Pulse synchronization risk** for error signals. Very short error pulses (< 1 pclk period) could theoretically be missed by the 2-FF synchronizer. The sticky register mitigates this, but a toggle-based synchronizer would be more rigorous.

3. **No reset synchronization** between domains. In a production design, each domain's reset should be synchronized to its clock using a reset synchronizer (async assert, sync deassert). This design assumes both resets are asserted long enough for both domains to see them.
