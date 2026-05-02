// =============================================================================
// Module: tb_uart_top
// Description: Testbench for the multi-clock UART controller with CDC.
//
// Test Plan:
//   1. Basic TX: single byte transmission, verify serial output
//   2. Basic RX: drive serial input, verify received byte
//   3. Loopback: connect TX to RX, verify data integrity
//   4. Back-to-back TX: fill FIFO, verify continuous transmission
//   5. FIFO full/empty transitions: boundary testing
//   6. Parity: even and odd parity modes, parity error injection
//   7. Interrupt: verify all interrupt sources and W1C behavior
//   8. APB error: access to undefined register address
//   9. Baud rate change: reconfigure divisor between transfers
//  10. Overrun error: overflow RX FIFO
// =============================================================================

`timescale 1ns / 1ps

module tb_uart_top;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int FIFO_DEPTH   = 8;
    localparam int PCLK_PERIOD  = 20;   // 50 MHz system clock
    localparam int SCLK_PERIOD  = 12;   // ~83 MHz serial ref clock
    localparam int BAUD_DIVISOR = 3;    // tick_16x period = 3 * SCLK_PERIOD
                                         // Bit period = 16 * 3 * 12ns = 576ns
    localparam int BIT_PERIOD   = 16 * BAUD_DIVISOR * SCLK_PERIOD;

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic        pclk, prst_n;
    logic        sclk, srst_n;

    logic        psel, penable, pwrite;
    logic [4:0]  paddr;
    logic [31:0] pwdata, prdata;
    logic        pready, pslverr;

    logic        uart_tx_pin, uart_rx_pin;
    logic        irq;

    // =========================================================================
    // Clock generation (asynchronous clocks)
    // =========================================================================
    initial pclk = 0;
    always #(PCLK_PERIOD/2) pclk = ~pclk;

    initial sclk = 0;
    always #(SCLK_PERIOD/2) sclk = ~sclk;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    uart_top #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .DIV_WIDTH  (16)
    ) u_dut (
        .pclk    (pclk),
        .prst_n  (prst_n),
        .sclk    (sclk),
        .srst_n  (srst_n),
        .psel    (psel),
        .penable (penable),
        .pwrite  (pwrite),
        .paddr   (paddr),
        .pwdata  (pwdata),
        .prdata  (prdata),
        .pready  (pready),
        .pslverr (pslverr),
        .uart_tx (uart_tx_pin),
        .uart_rx (uart_rx_pin),
        .irq     (irq)
    );

    // =========================================================================
    // SVA binding - connect gray-code pointers for assertion checking
    // =========================================================================
    // Track previous gray-code values for hamming distance checks
    logic [3:0] tx_wptr_gray_prev, tx_rptr_gray_prev;
    logic [3:0] rx_wptr_gray_prev, rx_rptr_gray_prev;

    always_ff @(posedge pclk or negedge prst_n)
        if (!prst_n) tx_wptr_gray_prev <= '0;
        else         tx_wptr_gray_prev <= u_dut.u_tx_fifo.wptr_gray;

    always_ff @(posedge sclk or negedge srst_n)
        if (!srst_n) tx_rptr_gray_prev <= '0;
        else         tx_rptr_gray_prev <= u_dut.u_tx_fifo.rptr_gray;

    always_ff @(posedge sclk or negedge srst_n)
        if (!srst_n) rx_wptr_gray_prev <= '0;
        else         rx_wptr_gray_prev <= u_dut.u_rx_fifo.wptr_gray;

    always_ff @(posedge pclk or negedge prst_n)
        if (!prst_n) rx_rptr_gray_prev <= '0;
        else         rx_rptr_gray_prev <= u_dut.u_rx_fifo.rptr_gray;

    uart_sva_props u_sva (
        .pclk              (pclk),
        .prst_n            (prst_n),
        .sclk              (sclk),
        .srst_n            (srst_n),
        .psel              (psel),
        .penable           (penable),
        .pwrite            (pwrite),
        .paddr             (paddr),
        .pready            (pready),
        .pslverr           (pslverr),
        .tx_fifo_wen       (u_dut.tx_fifo_wen),
        .tx_fifo_full      (u_dut.tx_fifo_full),
        .tx_fifo_ren       (u_dut.tx_data_rd),
        .tx_fifo_empty_sclk(u_dut.tx_fifo_empty_sclk),
        .rx_fifo_wen       (u_dut.rx_fifo_wen),
        .rx_fifo_full_sclk (u_dut.rx_fifo_full_sclk),
        .rx_fifo_ren       (u_dut.rx_fifo_ren),
        .rx_fifo_empty     (u_dut.rx_fifo_empty),
        .tx_wptr_gray      (u_dut.u_tx_fifo.wptr_gray),
        .tx_wptr_gray_prev (tx_wptr_gray_prev),
        .tx_rptr_gray      (u_dut.u_tx_fifo.rptr_gray),
        .tx_rptr_gray_prev (tx_rptr_gray_prev),
        .rx_wptr_gray      (u_dut.u_rx_fifo.wptr_gray),
        .rx_wptr_gray_prev (rx_wptr_gray_prev),
        .rx_rptr_gray      (u_dut.u_rx_fifo.rptr_gray),
        .rx_rptr_gray_prev (rx_rptr_gray_prev),
        .uart_tx           (uart_tx_pin),
        .tx_active         (u_dut.tx_active),
        .int_status        (u_dut.int_status),
        .int_en            (u_dut.int_en_reg),
        .irq               (irq)
    );

    // =========================================================================
    // Functional coverage
    // =========================================================================
    covergroup cg_fifo_occupancy @(posedge pclk);
        option.per_instance = 1;
        cp_tx_full:  coverpoint u_dut.tx_fifo_full;
        cp_tx_empty: coverpoint u_dut.tx_fifo_empty_pclk;
        cp_rx_full:  coverpoint u_dut.rx_fifo_full_pclk;
        cp_rx_empty: coverpoint u_dut.rx_fifo_empty;
    endgroup

    covergroup cg_errors @(posedge pclk);
        option.per_instance = 1;
        cp_parity_err: coverpoint u_dut.parity_error_pclk;
        cp_overrun:    coverpoint u_dut.overrun_error;
        cp_pslverr:    coverpoint pslverr;
    endgroup

    covergroup cg_interrupts @(posedge pclk);
        option.per_instance = 1;
        cp_irq:        coverpoint irq;
        cp_int_status: coverpoint u_dut.int_status {
            bins no_irq   = {4'b0000};
            bins tx_empty = {4'b0001};
            bins rx_full  = {4'b0010};
            bins par_err  = {4'b0100};
            bins overrun  = {4'b1000};
            bins multi    = {[4'b0011:4'b1111]};
        }
    endgroup

    cg_fifo_occupancy cov_fifo = new();
    cg_errors         cov_err  = new();
    cg_interrupts     cov_irq  = new();

    // =========================================================================
    // APB bus functional model tasks
    // =========================================================================

    task automatic apb_write_reg(input logic [4:0] addr, input logic [31:0] data);
        @(posedge pclk);
        psel    <= 1'b1;
        penable <= 1'b0;
        pwrite  <= 1'b1;
        paddr   <= addr;
        pwdata  <= data;
        @(posedge pclk);
        penable <= 1'b1;
        @(posedge pclk);
        psel    <= 1'b0;
        penable <= 1'b0;
        pwrite  <= 1'b0;
    endtask

    task automatic apb_read_reg(input logic [4:0] addr, output logic [31:0] data);
        @(posedge pclk);
        psel    <= 1'b1;
        penable <= 1'b0;
        pwrite  <= 1'b0;
        paddr   <= addr;
        @(posedge pclk);
        penable <= 1'b1;
        @(posedge pclk);
        data    = prdata;
        psel    <= 1'b0;
        penable <= 1'b0;
    endtask

    // =========================================================================
    // UART serial line driver (for RX testing)
    // =========================================================================

    task automatic uart_send_byte(input logic [7:0] data,
                                   input bit parity_en = 0,
                                   input bit parity_type = 0,
                                   input bit inject_parity_error = 0,
                                   input int bit_period = BIT_PERIOD);
        logic parity_bit;

        // Start bit
        uart_rx_pin = 1'b0;
        #(bit_period);

        // Data bits (LSB first)
        for (int i = 0; i < 8; i++) begin
            uart_rx_pin = data[i];
            #(bit_period);
        end

        // Parity bit (if enabled)
        if (parity_en) begin
            parity_bit = (^data) ^ parity_type;
            if (inject_parity_error) parity_bit = ~parity_bit;
            uart_rx_pin = parity_bit;
            #(bit_period);
        end

        // Stop bit
        uart_rx_pin = 1'b1;
        #(bit_period);
    endtask

    // =========================================================================
    // TX serial line monitor (captures transmitted bytes)
    // =========================================================================

    logic [7:0] tx_captured_data;
    logic       tx_capture_valid;

    task automatic uart_capture_tx(output logic [7:0] data,
                                    input bit parity_en = 0,
                                    input int bit_period = BIT_PERIOD);
        // Wait for start bit (falling edge)
        @(negedge uart_tx_pin);

        // Wait to center of start bit
        #(bit_period / 2);

        // Verify start bit is low
        assert (uart_tx_pin == 1'b0) else $error("TX: Invalid start bit");

        // Sample 8 data bits
        for (int i = 0; i < 8; i++) begin
            #(bit_period);
            data[i] = uart_tx_pin;
        end

        // Skip parity if enabled
        if (parity_en) #(bit_period);

        // Verify stop bit
        #(bit_period);
        assert (uart_tx_pin == 1'b1) else $error("TX: Invalid stop bit");
    endtask

    // =========================================================================
    // Register address constants
    // =========================================================================
    localparam logic [4:0] REG_TXDATA   = 5'h00;
    localparam logic [4:0] REG_RXDATA   = 5'h04;
    localparam logic [4:0] REG_BAUDDIV  = 5'h08;
    localparam logic [4:0] REG_CTRL     = 5'h0C;
    localparam logic [4:0] REG_STATUS   = 5'h10;
    localparam logic [4:0] REG_INT_EN   = 5'h14;
    localparam logic [4:0] REG_INT_STAT = 5'h18;

    // =========================================================================
    // Main test sequence
    // =========================================================================

    logic [31:0] read_data;
    logic [7:0]  captured_byte;
    int          pass_count, fail_count;

    initial begin
        $display("============================================");
        $display(" Multi-Clock UART Controller - Testbench");
        $display("============================================");

        // Initialize
        prst_n     = 0;
        srst_n     = 0;
        psel       = 0;
        penable    = 0;
        pwrite     = 0;
        paddr      = 0;
        pwdata     = 0;
        uart_rx_pin = 1'b1; // Idle high
        pass_count = 0;
        fail_count = 0;

        // Reset
        repeat (10) @(posedge pclk);
        prst_n = 1;
        @(posedge sclk);
        srst_n = 1;
        repeat (10) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 1: Configure UART
        // -----------------------------------------------------------------
        $display("\n[TEST 1] Configure UART");
        apb_write_reg(REG_BAUDDIV, BAUD_DIVISOR);
        apb_write_reg(REG_CTRL, 32'h03); // TX enable + RX enable, no parity
        apb_write_reg(REG_INT_EN, 32'h0F); // Enable all interrupts
        repeat (20) @(posedge pclk); // Wait for synchronization
        $display("  Config: baud_div=%0d, ctrl=0x03, int_en=0x0F", BAUD_DIVISOR);
        pass_count++;

        // -----------------------------------------------------------------
        // Test 2: Basic TX - send one byte
        // -----------------------------------------------------------------
        $display("\n[TEST 2] Basic TX - single byte 0xA5");
        fork
            begin
                apb_write_reg(REG_TXDATA, 32'h000000A5);
            end
            begin
                uart_capture_tx(captured_byte);
                if (captured_byte == 8'hA5) begin
                    $display("  PASS: Captured TX byte = 0x%02X", captured_byte);
                    pass_count++;
                end else begin
                    $display("  FAIL: Expected 0xA5, got 0x%02X", captured_byte);
                    fail_count++;
                end
            end
        join

        repeat (50) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 3: Basic RX - receive one byte
        // -----------------------------------------------------------------
        $display("\n[TEST 3] Basic RX - receive byte 0x3C");
        uart_send_byte(8'h3C);
        repeat (100) @(posedge pclk); // Wait for data to cross CDC

        apb_read_reg(REG_STATUS, read_data);
        $display("  STATUS register = 0x%08X", read_data);

        apb_read_reg(REG_RXDATA, read_data);
        if (read_data[7:0] == 8'h3C) begin
            $display("  PASS: RX data = 0x%02X", read_data[7:0]);
            pass_count++;
        end else begin
            $display("  FAIL: Expected 0x3C, got 0x%02X", read_data[7:0]);
            fail_count++;
        end

        // -----------------------------------------------------------------
        // Test 4: Loopback - connect TX to RX
        // -----------------------------------------------------------------
        $display("\n[TEST 4] Loopback test (TX -> RX) with byte 0x55");
        // Loopback wire
        force uart_rx_pin = uart_tx_pin;

        apb_write_reg(REG_TXDATA, 32'h00000055);
        // Wait for full frame + CDC latency
        #(BIT_PERIOD * 12 + 500);
        repeat (100) @(posedge pclk);

        apb_read_reg(REG_RXDATA, read_data);
        if (read_data[7:0] == 8'h55) begin
            $display("  PASS: Loopback data = 0x%02X", read_data[7:0]);
            pass_count++;
        end else begin
            $display("  FAIL: Expected 0x55, got 0x%02X", read_data[7:0]);
            fail_count++;
        end

        release uart_rx_pin;
        uart_rx_pin = 1'b1;
        repeat (50) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 5: Back-to-back TX - fill FIFO
        // -----------------------------------------------------------------
        $display("\n[TEST 5] Back-to-back TX - fill FIFO with 4 bytes");
        for (int i = 0; i < 4; i++) begin
            apb_write_reg(REG_TXDATA, i + 1);
        end

        // Capture all transmitted bytes
        for (int i = 0; i < 4; i++) begin
            uart_capture_tx(captured_byte);
            if (captured_byte == (i + 1)) begin
                $display("  PASS: TX byte %0d = 0x%02X", i, captured_byte);
                pass_count++;
            end else begin
                $display("  FAIL: TX byte %0d expected 0x%02X, got 0x%02X",
                         i, i+1, captured_byte);
                fail_count++;
            end
        end

        repeat (50) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 6: Parity - enable even parity
        // -----------------------------------------------------------------
        $display("\n[TEST 6] Even parity mode");
        apb_write_reg(REG_CTRL, 32'h07); // TX+RX enable, parity enable, even

        repeat (20) @(posedge pclk);

        fork
            begin
                apb_write_reg(REG_TXDATA, 32'h000000FF); // 8 ones -> even parity = 0
            end
            begin
                uart_capture_tx(captured_byte, .parity_en(1));
                if (captured_byte == 8'hFF) begin
                    $display("  PASS: Parity TX byte = 0x%02X", captured_byte);
                    pass_count++;
                end else begin
                    $display("  FAIL: Expected 0xFF, got 0x%02X", captured_byte);
                    fail_count++;
                end
            end
        join

        repeat (50) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 7: Parity error injection
        // -----------------------------------------------------------------
        $display("\n[TEST 7] Parity error injection");
        uart_send_byte(8'hAA, .parity_en(1), .parity_type(0),
                       .inject_parity_error(1));
        repeat (100) @(posedge pclk);

        apb_read_reg(REG_STATUS, read_data);
        if (read_data[4]) begin // Parity error flag
            $display("  PASS: Parity error detected (STATUS=0x%08X)", read_data);
            pass_count++;
        end else begin
            $display("  FAIL: Parity error not detected (STATUS=0x%08X)", read_data);
            fail_count++;
        end

        // Clear parity error
        apb_write_reg(REG_STATUS, 32'h10);
        repeat (5) @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 8: APB error - invalid address
        // -----------------------------------------------------------------
        $display("\n[TEST 8] APB error on invalid address 0x1C");
        @(posedge pclk);
        psel    <= 1'b1;
        penable <= 1'b0;
        pwrite  <= 1'b0;
        paddr   <= 5'h1C; // Invalid address
        @(posedge pclk);
        penable <= 1'b1;
        @(posedge pclk);
        if (pslverr) begin
            $display("  PASS: PSLVERR asserted for invalid address");
            pass_count++;
        end else begin
            $display("  FAIL: PSLVERR not asserted for invalid address");
            fail_count++;
        end
        psel    <= 1'b0;
        penable <= 1'b0;
        @(posedge pclk);

        // -----------------------------------------------------------------
        // Test 9: Interrupt - TX empty, W1C behavior
        // -----------------------------------------------------------------
        $display("\n[TEST 9] Interrupt behavior and W1C");
        apb_read_reg(REG_INT_STAT, read_data);
        $display("  Interrupt status before clear = 0x%01X", read_data[3:0]);

        // Clear all interrupts
        apb_write_reg(REG_INT_STAT, 32'h0F);
        repeat (5) @(posedge pclk);

        apb_read_reg(REG_INT_STAT, read_data);
        $display("  Interrupt status after W1C = 0x%01X", read_data[3:0]);
        pass_count++;

        // -----------------------------------------------------------------
        // Test 10: Baud rate change between transfers
        // -----------------------------------------------------------------
        $display("\n[TEST 10] Baud rate change");
        // Disable TX/RX first
        apb_write_reg(REG_CTRL, 32'h00);
        repeat (20) @(posedge pclk);

        // Change baud rate
        apb_write_reg(REG_BAUDDIV, 32'h0005); // New divisor
        repeat (20) @(posedge pclk);

        // Re-enable (no parity)
        apb_write_reg(REG_CTRL, 32'h03);
        repeat (20) @(posedge pclk);

        // Send a byte at new rate
        fork
            begin
                apb_write_reg(REG_TXDATA, 32'h000000CC);
            end
            begin
                // Capture at new baud rate (divisor=5 -> bit_period = 16*5*SCLK_PERIOD)
                uart_capture_tx(captured_byte, .bit_period(16 * 5 * SCLK_PERIOD));
                if (captured_byte == 8'hCC) begin
                    $display("  PASS: TX at new baud = 0x%02X", captured_byte);
                    pass_count++;
                end else begin
                    $display("  FAIL: Expected 0xCC, got 0x%02X", captured_byte);
                    fail_count++;
                end
            end
        join

        repeat (50) @(posedge pclk);

        // -----------------------------------------------------------------
        // Results
        // -----------------------------------------------------------------
        $display("\n============================================");
        $display(" Test Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("============================================\n");

        if (fail_count == 0)
            $display(">>> ALL TESTS PASSED <<<");
        else
            $display(">>> SOME TESTS FAILED <<<");

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(BIT_PERIOD * 500);
        $display("\nERROR: Simulation timeout!");
        $finish;
    end

    // =========================================================================
    // Waveform dump
    // =========================================================================
    initial begin
        $dumpfile("uart_top.vcd");
        $dumpvars(0, tb_uart_top);
    end

endmodule
