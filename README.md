# Multi-Clock UART Controller with CDC Verification

**Author:** Arif Hussain

A SystemVerilog UART controller featuring asynchronous clock domain crossing, gray-code async FIFOs, APB3 register interface, and comprehensive SVA-based CDC verification.

## Architecture

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                     uart_top                            │
                    │                                                         │
   APB Bus          │  ┌──────────────┐    ┌───────────┐    ┌──────────┐     │
  (pclk domain)     │  │ apb_uart_regs│    │  TX Async  │    │ uart_tx  │     │──► uart_tx
  ──────────────────┼─►│              ├───►│   FIFO     ├───►│          │     │
  PSEL,PENABLE,     │  │  Registers   │    │ (gray-code)│    │ Shift Reg│     │
  PWRITE,PADDR,     │  │  + Status    │    └───────────┘    └──────────┘     │
  PWDATA ◄──────────┼──┤              │         CDC              │           │
  PRDATA,PREADY     │  │              │    ┌───────────┐    ┌──────────┐     │
                    │  │              │◄───┤  RX Async  │◄───┤ uart_rx  │     │◄── uart_rx
                    │  │              │    │   FIFO     │    │ 16x Over-│     │
                    │  └──────┬───────┘    │ (gray-code)│    │ sampling │     │
                    │         │            └───────────┘    └──────────┘     │
                    │         │                 CDC              │           │
                    │  ┌──────┴───────┐                   ┌──────────┐     │
                    │  │   uart_      │    2-FF sync      │ baud_gen │     │
  irq ◄─────────────┼──┤  interrupt   │◄──────────────────┤          │     │
                    │  └──────────────┘    (status/err)    └──────────┘     │
                    │     pclk domain         │           sclk domain      │
                    └─────────────────────────┼───────────────────────────────┘
                                              │
                                         CDC Boundary
```

## Register Map

| Offset | Name     | R/W  | Description |
|--------|----------|------|-------------|
| 0x00   | TXDATA   | WO   | TX data [7:0] - writes push to TX FIFO |
| 0x04   | RXDATA   | RO   | RX data [7:0] - reads pop from RX FIFO |
| 0x08   | BAUDDIV  | RW   | Baud rate divisor [15:0] |
| 0x0C   | CTRL     | RW   | [0]=TX_EN [1]=RX_EN [2]=PAR_EN [3]=PAR_TYPE |
| 0x10   | STATUS   | RO/W1C | [0]=TXE [1]=TXF [2]=RXE [3]=RXF [4]=PAR_ERR [5]=OVR [6]=FRM |
| 0x14   | INT_EN   | RW   | Interrupt enable mask [3:0] |
| 0x18   | INT_STAT | R/W1C | Interrupt status [3:0], write-1-to-clear |

## File Structure

```
MultiClock-UART-CDC/
├── rtl/
│   ├── sync_2ff.sv          # Double-flop synchronizer
│   ├── async_fifo.sv        # Async FIFO with gray-code pointers
│   ├── baud_gen.sv          # Programmable baud rate generator
│   ├── uart_tx.sv           # UART transmitter (parallel-to-serial)
│   ├── uart_rx.sv           # UART receiver (16x oversampling + majority vote)
│   ├── uart_interrupt.sv    # Interrupt controller (W1C)
│   ├── apb_uart_regs.sv     # APB3 slave register interface
│   └── uart_top.sv          # Top-level with all CDC wiring
├── verif/
│   ├── uart_sva_props.sv    # SVA assertions (CDC, FIFO, APB)
│   └── tb_uart_top.sv       # Directed testbench with 10 test cases
├── doc/
│   └── cdc_strategy.md      # CDC crossing documentation
├── sim/
│   └── Makefile             # Build targets for xsim, VCS, Xcelium, Yosys
└── README.md
```

## Quick Start

### Vivado xsim
```bash
cd sim
make xsim
```

### Icarus Verilog (no SVA)
```bash
cd sim
make iverilog
```

### VCS (full SVA + coverage)
```bash
cd sim
make vcs
```

### EDA Playground
Upload all files from `rtl/` and `verif/`, select VCS or Xcelium, enable SVA.

### Yosys synthesis check
```bash
cd sim
make synth_yosys
```

## Test Plan Summary

| # | Test | What It Verifies |
|---|------|-----------------|
| 1 | Configuration | APB register write/read, baud setup |
| 2 | Basic TX | Single byte transmission, serial output |
| 3 | Basic RX | Serial input reception, data readback |
| 4 | Loopback | TX-to-RX data integrity through full CDC path |
| 5 | Back-to-back TX | FIFO throughput, continuous streaming |
| 6 | Even parity | Parity bit generation |
| 7 | Parity error | Error injection and detection |
| 8 | APB error | PSLVERR on invalid address |
| 9 | Interrupt W1C | Interrupt status write-1-to-clear |
| 10 | Baud rate change | Reconfigure divisor between transfers |

## CDC Verification Checklist

- [x] All multi-bit data crosses via async FIFO (gray-code pointers)
- [x] All single-bit control/status signals cross via 2-FF synchronizer
- [x] External async input (uart_rx) synchronized before use
- [x] SVA: gray-code pointers change by Hamming distance = 1
- [x] SVA: no FIFO overflow or underflow
- [x] SVA: APB protocol compliance
- [x] Baud divisor CDC documented with software constraint
- [x] CDC crossing map documented in `doc/cdc_strategy.md`

## Resume Keywords

Serial protocol design, clock domain crossing, async FIFO, gray-code, double-flop synchronizer, APB bus protocol, interrupt handling, SVA assertions, CDC verification, FPGA synthesis, SystemVerilog, metastability, UART 16x oversampling, majority voting
