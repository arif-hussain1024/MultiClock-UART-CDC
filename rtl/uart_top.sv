// =============================================================================
// Module: uart_top
// Description: Top-level multi-clock UART controller with CDC.
//
// Clock Domains:
//   pclk  - APB/system clock (CPU side)
//   sclk  - Serial reference clock (UART TX/RX side)
//
// CDC Crossing Map (see doc/cdc_strategy.md for full rationale):
// ┌────────────────────┬─────────────┬─────────────┬─────────────────────────┐
// │ Signal             │ Source      │ Destination │ Method                  │
// ├────────────────────┼─────────────┼─────────────┼─────────────────────────┤
// │ TX data            │ pclk        │ sclk        │ Async FIFO (gray-code)  │
// │ RX data            │ sclk        │ pclk        │ Async FIFO (gray-code)  │
// │ TX FIFO empty flag │ sclk        │ pclk        │ 2-FF synchronizer       │
// │ TX FIFO full flag  │ pclk        │ pclk        │ Native (same domain)    │
// │ RX FIFO empty flag │ pclk        │ pclk        │ Native (same domain)    │
// │ RX FIFO full flag  │ sclk        │ pclk        │ 2-FF synchronizer       │
// │ baud_divisor[15:0] │ pclk        │ sclk        │ 2-FF sync (bus, stable) │
// │ tx_enable          │ pclk        │ sclk        │ 2-FF synchronizer       │
// │ rx_enable          │ pclk        │ sclk        │ 2-FF synchronizer       │
// │ parity_en          │ pclk        │ sclk        │ 2-FF synchronizer       │
// │ parity_type        │ pclk        │ sclk        │ 2-FF synchronizer       │
// │ parity_error       │ sclk        │ pclk        │ 2-FF synchronizer       │
// │ frame_error        │ sclk        │ pclk        │ 2-FF synchronizer       │
// │ rx_serial (ext pin)│ async       │ sclk        │ 2-FF synchronizer       │
// └────────────────────┴─────────────┴─────────────┴─────────────────────────┘
//
// Note on baud_divisor CDC: The 16-bit divisor is synchronized as a bus via
// a multi-bit 2-FF synchronizer. This is safe ONLY because the divisor is
// configured once and held stable before enabling TX/RX. Software must not
// change the divisor while transfers are active. An alternative would be a
// handshake-based register transfer, but for this use case the simpler
// approach is acceptable with the documented constraint.
// =============================================================================

module uart_top #(
    parameter int FIFO_DEPTH  = 8,
    parameter int DIV_WIDTH   = 16
)(
    // APB clock domain
    input  logic        pclk,
    input  logic        prst_n,

    // Serial clock domain
    input  logic        sclk,
    input  logic        srst_n,

    // APB3 slave interface
    input  logic        psel,
    input  logic        penable,
    input  logic        pwrite,
    input  logic [4:0]  paddr,
    input  logic [31:0] pwdata,
    output logic [31:0] prdata,
    output logic        pready,
    output logic        pslverr,

    // UART serial pins
    output logic        uart_tx,
    input  logic        uart_rx,

    // Interrupt output
    output logic        irq
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // APB register outputs (pclk domain)
    logic [15:0] baud_divisor_pclk;
    logic        tx_enable_pclk, rx_enable_pclk;
    logic        parity_en_pclk, parity_type_pclk;

    // Synchronized config signals (sclk domain)
    logic [15:0] baud_divisor_sclk;
    logic        tx_enable_sclk, rx_enable_sclk;
    logic        parity_en_sclk, parity_type_sclk;

    // TX FIFO signals
    logic [7:0]  tx_fifo_wdata;   // pclk domain
    logic        tx_fifo_wen;     // pclk domain
    logic        tx_fifo_full;    // pclk domain (write side full)
    logic [7:0]  tx_fifo_rdata;   // sclk domain
    logic        tx_fifo_ren;     // sclk domain
    logic        tx_fifo_empty_sclk; // sclk domain (read side empty)
    logic        tx_fifo_empty_pclk; // synchronized to pclk

    // RX FIFO signals
    logic [7:0]  rx_fifo_wdata;   // sclk domain
    logic        rx_fifo_wen;     // sclk domain
    logic        rx_fifo_full_sclk;  // sclk domain (write side full)
    logic        rx_fifo_full_pclk;  // synchronized to pclk
    logic [7:0]  rx_fifo_rdata;   // pclk domain
    logic        rx_fifo_ren;     // pclk domain
    logic        rx_fifo_empty;   // pclk domain (read side empty)

    // TX/RX signals (sclk domain)
    logic        tx_data_rd;
    logic        tx_active;
    logic [7:0]  rx_data_sclk;
    logic        rx_data_valid_sclk;
    logic        parity_error_sclk, frame_error_sclk;

    // Synchronized error signals (pclk domain)
    logic        parity_error_pclk, frame_error_pclk;

    // Overrun detection (pclk domain)
    logic        overrun_error;

    // Synchronized RX pin (sclk domain)
    logic        rx_serial_sync;

    // Interrupt signals
    logic [3:0]  int_en_reg;
    logic [3:0]  int_status;
    logic [3:0]  int_clear;
    logic        int_clear_valid;

    // =========================================================================
    // CDC: Config signals pclk -> sclk
    // =========================================================================

    // Baud divisor (16-bit bus - safe because stable before enable)
    sync_2ff #(.WIDTH(16)) u_sync_bauddiv (
        .clk   (sclk),
        .rst_n (srst_n),
        .d     (baud_divisor_pclk),
        .q     (baud_divisor_sclk)
    );

    // Single-bit control signals
    sync_2ff #(.WIDTH(1)) u_sync_tx_en (
        .clk(sclk), .rst_n(srst_n), .d(tx_enable_pclk), .q(tx_enable_sclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_rx_en (
        .clk(sclk), .rst_n(srst_n), .d(rx_enable_pclk), .q(rx_enable_sclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_par_en (
        .clk(sclk), .rst_n(srst_n), .d(parity_en_pclk), .q(parity_en_sclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_par_type (
        .clk(sclk), .rst_n(srst_n), .d(parity_type_pclk), .q(parity_type_sclk)
    );

    // =========================================================================
    // CDC: Status/error signals sclk -> pclk
    // =========================================================================

    sync_2ff #(.WIDTH(1)) u_sync_tx_empty (
        .clk(pclk), .rst_n(prst_n), .d(tx_fifo_empty_sclk), .q(tx_fifo_empty_pclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_rx_full (
        .clk(pclk), .rst_n(prst_n), .d(rx_fifo_full_sclk), .q(rx_fifo_full_pclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_par_err (
        .clk(pclk), .rst_n(prst_n), .d(parity_error_sclk), .q(parity_error_pclk)
    );
    sync_2ff #(.WIDTH(1)) u_sync_frm_err (
        .clk(pclk), .rst_n(prst_n), .d(frame_error_sclk), .q(frame_error_pclk)
    );

    // =========================================================================
    // CDC: External RX pin -> sclk domain
    // =========================================================================

    sync_2ff #(.WIDTH(1), .RESET_VAL(1'b1)) u_sync_rx_pin (
        .clk(sclk), .rst_n(srst_n), .d(uart_rx), .q(rx_serial_sync)
    );

    // =========================================================================
    // TX Async FIFO: pclk (write) -> sclk (read)
    // =========================================================================

    async_fifo #(
        .DEPTH (FIFO_DEPTH),
        .WIDTH (8)
    ) u_tx_fifo (
        .wclk   (pclk),
        .wrst_n (prst_n),
        .wen    (tx_fifo_wen),
        .wdata  (tx_fifo_wdata),
        .wfull  (tx_fifo_full),

        .rclk   (sclk),
        .rrst_n (srst_n),
        .ren    (tx_data_rd),
        .rdata  (tx_fifo_rdata),
        .rempty (tx_fifo_empty_sclk)
    );

    // =========================================================================
    // RX Async FIFO: sclk (write) -> pclk (read)
    // =========================================================================

    async_fifo #(
        .DEPTH (FIFO_DEPTH),
        .WIDTH (8)
    ) u_rx_fifo (
        .wclk   (sclk),
        .wrst_n (srst_n),
        .wen    (rx_fifo_wen),
        .wdata  (rx_fifo_wdata),
        .wfull  (rx_fifo_full_sclk),

        .rclk   (pclk),
        .rrst_n (prst_n),
        .ren    (rx_fifo_ren),
        .rdata  (rx_fifo_rdata),
        .rempty (rx_fifo_empty)
    );

    // RX FIFO write: push received data
    assign rx_fifo_wdata = rx_data_sclk;
    assign rx_fifo_wen   = rx_data_valid_sclk && !rx_fifo_full_sclk;

    // =========================================================================
    // Overrun detection (pclk domain)
    // =========================================================================
    // Overrun occurs when RX FIFO is full and new data arrives.
    // We detect this by observing the synchronized full flag and a synchronized
    // write-attempt signal.

    logic rx_data_valid_pclk;
    sync_2ff #(.WIDTH(1)) u_sync_rx_valid (
        .clk(pclk), .rst_n(prst_n), .d(rx_data_valid_sclk), .q(rx_data_valid_pclk)
    );

    always_ff @(posedge pclk or negedge prst_n) begin
        if (!prst_n)
            overrun_error <= 1'b0;
        else
            overrun_error <= rx_fifo_full_pclk && rx_data_valid_pclk;
    end

    // =========================================================================
    // Baud rate generator (sclk domain)
    // =========================================================================

    logic tick_16x;

    baud_gen #(
        .DIV_WIDTH (DIV_WIDTH)
    ) u_baud_gen (
        .sclk     (sclk),
        .srst_n   (srst_n),
        .divisor  (baud_divisor_sclk),
        .enable   (tx_enable_sclk | rx_enable_sclk),
        .tick_16x (tick_16x)
    );

    // =========================================================================
    // UART Transmitter (sclk domain)
    // =========================================================================

    uart_tx u_uart_tx (
        .sclk          (sclk),
        .srst_n        (srst_n),
        .tick_16x      (tick_16x),
        .tx_data       (tx_fifo_rdata),
        .tx_data_valid (!tx_fifo_empty_sclk),
        .tx_data_rd    (tx_data_rd),
        .tx_enable     (tx_enable_sclk),
        .parity_en     (parity_en_sclk),
        .parity_type   (parity_type_sclk),
        .tx_serial     (uart_tx),
        .tx_active     (tx_active)
    );

    // =========================================================================
    // UART Receiver (sclk domain)
    // =========================================================================

    uart_rx u_uart_rx (
        .sclk          (sclk),
        .srst_n        (srst_n),
        .tick_16x      (tick_16x),
        .rx_data       (rx_data_sclk),
        .rx_data_valid (rx_data_valid_sclk),
        .rx_enable     (rx_enable_sclk),
        .parity_en     (parity_en_sclk),
        .parity_type   (parity_type_sclk),
        .parity_error  (parity_error_sclk),
        .frame_error   (frame_error_sclk),
        .rx_serial     (rx_serial_sync)
    );

    // =========================================================================
    // APB Register Interface (pclk domain)
    // =========================================================================

    apb_uart_regs u_apb_regs (
        .pclk              (pclk),
        .prst_n            (prst_n),
        .psel              (psel),
        .penable           (penable),
        .pwrite            (pwrite),
        .paddr             (paddr),
        .pwdata            (pwdata),
        .prdata            (prdata),
        .pready            (pready),
        .pslverr           (pslverr),

        .tx_fifo_wdata     (tx_fifo_wdata),
        .tx_fifo_wen       (tx_fifo_wen),
        .tx_fifo_full      (tx_fifo_full),
        .tx_fifo_empty     (tx_fifo_empty_pclk),

        .rx_fifo_rdata     (rx_fifo_rdata),
        .rx_fifo_ren       (rx_fifo_ren),
        .rx_fifo_full      (rx_fifo_full_pclk),
        .rx_fifo_empty     (rx_fifo_empty),

        .baud_divisor      (baud_divisor_pclk),
        .tx_enable         (tx_enable_pclk),
        .rx_enable         (rx_enable_pclk),
        .parity_en         (parity_en_pclk),
        .parity_type       (parity_type_pclk),

        .parity_error_sync (parity_error_pclk),
        .frame_error_sync  (frame_error_pclk),
        .overrun_error     (overrun_error),

        .int_en_reg        (int_en_reg),
        .int_status        (int_status),
        .int_clear         (int_clear),
        .int_clear_valid   (int_clear_valid)
    );

    // =========================================================================
    // Interrupt Controller (pclk domain)
    // =========================================================================

    uart_interrupt u_irq (
        .pclk            (pclk),
        .prst_n          (prst_n),
        .irq_tx_empty    (tx_fifo_empty_pclk),
        .irq_rx_full     (rx_fifo_full_pclk),
        .irq_parity_err  (parity_error_pclk),
        .irq_overrun     (overrun_error),
        .int_en          (int_en_reg),
        .int_clear       (int_clear),
        .int_clear_valid (int_clear_valid),
        .int_status      (int_status),
        .irq_out         (irq)
    );

endmodule
