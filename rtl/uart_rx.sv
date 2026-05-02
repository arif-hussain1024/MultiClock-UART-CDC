// =============================================================================
// Module: uart_rx
// Description: UART receiver with 16x oversampling and majority voting.
//              Operates in the serial clock domain (sclk).
//
//              Sampling strategy:
//              - Start bit detection: waits for falling edge on rx_serial
//              - Aligns to bit center by counting to 7 (half bit period)
//              - Samples at ticks 7, 8, 9 of each subsequent bit period
//              - Majority vote (2-of-3) determines the received bit value
//              - Provides noise immunity against single-sample glitches
//
//              Frame format: START(1) + DATA(8) + PARITY(0/1) + STOP(1)
// =============================================================================

module uart_rx (
    input  logic       sclk,
    input  logic       srst_n,

    // Baud timing
    input  logic       tick_16x,     // 16x baud rate tick

    // Data interface (to async FIFO write port, in sclk domain)
    output logic [7:0] rx_data,
    output logic       rx_data_valid, // Pulse when full byte received

    // Configuration (pre-synchronized to sclk domain)
    input  logic       rx_enable,
    input  logic       parity_en,
    input  logic       parity_type,   // 0=even, 1=odd

    // Error flags
    output logic       parity_error,  // Pulse on parity mismatch
    output logic       frame_error,   // Pulse on missing stop bit

    // Serial input
    input  logic       rx_serial
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE   = 3'd0,
        S_START  = 3'd1,
        S_DATA   = 3'd2,
        S_PARITY = 3'd3,
        S_STOP   = 3'd4
    } rx_state_t;

    rx_state_t state, next_state;

    // -------------------------------------------------------------------------
    // Internal signals
    // -------------------------------------------------------------------------
    logic [3:0] tick_cnt;       // 0-15 within bit period
    logic [2:0] bit_idx;        // Current data bit (0-7)
    logic [7:0] shift_reg;      // Serial-to-parallel shift register
    logic       rx_sync;        // Synchronized RX input (already in sclk domain,
                                 // but registered for edge detection)

    // Majority voting samples
    logic       sample_7, sample_8, sample_9;
    logic       voted_bit;

    // -------------------------------------------------------------------------
    // Input registration (for clean edge detection within sclk domain)
    // Note: If rx_serial comes from an external pin, it should be
    //       synchronized via sync_2ff in the parent module before arriving here.
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            rx_sync <= 1'b1;
        else
            rx_sync <= rx_serial;
    end

    // -------------------------------------------------------------------------
    // Majority voting: 2-of-3 vote on samples at ticks 7, 8, 9
    // -------------------------------------------------------------------------
    assign voted_bit = (sample_7 & sample_8) |
                       (sample_8 & sample_9) |
                       (sample_7 & sample_9);

    // Capture samples at tick positions 7, 8, 9
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n) begin
            sample_7 <= 1'b1;
            sample_8 <= 1'b1;
            sample_9 <= 1'b1;
        end else if (tick_16x && state != S_IDLE) begin
            if (tick_cnt == 4'd7) sample_7 <= rx_sync;
            if (tick_cnt == 4'd8) sample_8 <= rx_sync;
            if (tick_cnt == 4'd9) sample_9 <= rx_sync;
        end
    end

    // -------------------------------------------------------------------------
    // Tick counter
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            tick_cnt <= '0;
        else if (state == S_IDLE) begin
            // Reset on start bit falling edge detection
            if (rx_enable && !rx_sync && tick_16x)
                tick_cnt <= '0;
        end else if (tick_16x)
            tick_cnt <= tick_cnt + 1'b1;
    end

    // -------------------------------------------------------------------------
    // State register
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // -------------------------------------------------------------------------
    // Next state logic
    // -------------------------------------------------------------------------
    logic tick_mid; // Middle of bit period (after majority vote is ready)
    assign tick_mid = (tick_cnt == 4'd10) && tick_16x; // Sample at tick 10 (after 7,8,9 captured)

    always_comb begin
        next_state = state;
        case (state)
            S_IDLE: begin
                // Detect falling edge of start bit
                if (rx_enable && !rx_sync && tick_16x)
                    next_state = S_START;
            end
            S_START: begin
                // Verify start bit is still low at mid-point
                if (tick_mid) begin
                    if (!voted_bit)
                        next_state = S_DATA;   // Valid start bit
                    else
                        next_state = S_IDLE;   // False start, abort
                end
            end
            S_DATA: begin
                if (tick_mid && bit_idx == 3'd7)
                    next_state = parity_en ? S_PARITY : S_STOP;
            end
            S_PARITY: begin
                if (tick_mid)
                    next_state = S_STOP;
            end
            S_STOP: begin
                if (tick_mid)
                    next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // Bit index counter
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            bit_idx <= '0;
        else if (state == S_DATA && tick_mid)
            bit_idx <= bit_idx + 1'b1;
        else if (state != S_DATA)
            bit_idx <= '0;
    end

    // -------------------------------------------------------------------------
    // Shift register: capture data bits (LSB first)
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            shift_reg <= '0;
        else if (state == S_DATA && tick_mid)
            shift_reg[bit_idx] <= voted_bit;
    end

    // -------------------------------------------------------------------------
    // Output data and valid
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n) begin
            rx_data       <= '0;
            rx_data_valid <= 1'b0;
        end else if (state == S_STOP && tick_mid && voted_bit) begin
            // Valid stop bit received - output the data
            rx_data       <= shift_reg;
            rx_data_valid <= 1'b1;
        end else begin
            rx_data_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Error detection
    // -------------------------------------------------------------------------
    logic received_parity;

    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            received_parity <= 1'b0;
        else if (state == S_PARITY && tick_mid)
            received_parity <= voted_bit;
    end

    // Parity error: check when stop bit arrives
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            parity_error <= 1'b0;
        else if (state == S_STOP && tick_mid && parity_en) begin
            // Expected parity = XOR of data bits ^ parity_type
            parity_error <= (received_parity != ((^shift_reg) ^ parity_type));
        end else
            parity_error <= 1'b0;
    end

    // Frame error: stop bit should be high
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            frame_error <= 1'b0;
        else if (state == S_STOP && tick_mid)
            frame_error <= !voted_bit; // Stop bit should be 1
        else
            frame_error <= 1'b0;
    end

endmodule
