// =============================================================================
// Module: uart_tx
// Description: UART transmitter with parallel-to-serial shift register.
//              Operates in the serial clock domain (sclk).
//              Frame format: START(1) + DATA(8) + PARITY(0/1) + STOP(1)
//
//              Uses tick_16x from baud_gen; counts 16 ticks per bit period
//              to match the standard 16x oversampling architecture.
// =============================================================================

module uart_tx (
    input  logic       sclk,
    input  logic       srst_n,

    // Baud timing
    input  logic       tick_16x,     // 16x baud rate tick

    // Data interface (from async FIFO read port, already in sclk domain)
    input  logic [7:0] tx_data,
    input  logic       tx_data_valid, // Data available in FIFO
    output logic       tx_data_rd,    // Read strobe to FIFO

    // Configuration (pre-synchronized to sclk domain)
    input  logic       tx_enable,
    input  logic       parity_en,
    input  logic       parity_type,   // 0=even, 1=odd

    // Serial output
    output logic       tx_serial,
    output logic       tx_active      // High while transmitting a frame
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
    } tx_state_t;

    tx_state_t state, next_state;

    // -------------------------------------------------------------------------
    // Internal signals
    // -------------------------------------------------------------------------
    logic [3:0] tick_cnt;      // Counts 0-15 within each bit period
    logic [2:0] bit_idx;       // Current data bit index (0-7)
    logic [7:0] shift_reg;     // Parallel-to-serial shift register
    logic       parity_bit;    // Computed parity
    logic       tick_done;     // Asserted when tick_cnt reaches 15

    assign tick_done = (tick_cnt == 4'd15) && tick_16x;

    // -------------------------------------------------------------------------
    // Tick counter: counts 16 ticks of tick_16x per bit period
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            tick_cnt <= '0;
        else if (state == S_IDLE)
            tick_cnt <= '0;
        else if (tick_16x)
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
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (tx_enable && tx_data_valid)
                    next_state = S_START;
            end
            S_START: begin
                if (tick_done)
                    next_state = S_DATA;
            end
            S_DATA: begin
                if (tick_done && bit_idx == 3'd7)
                    next_state = parity_en ? S_PARITY : S_STOP;
            end
            S_PARITY: begin
                if (tick_done)
                    next_state = S_STOP;
            end
            S_STOP: begin
                if (tick_done)
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
        else if (state == S_DATA && tick_done)
            bit_idx <= bit_idx + 1'b1;
        else if (state != S_DATA)
            bit_idx <= '0;
    end

    // -------------------------------------------------------------------------
    // Shift register: load on IDLE->START transition
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            shift_reg <= 8'hFF;
        else if (state == S_IDLE && tx_enable && tx_data_valid)
            shift_reg <= tx_data;
    end

    // -------------------------------------------------------------------------
    // Parity computation
    // -------------------------------------------------------------------------
    assign parity_bit = (^shift_reg) ^ parity_type; // even=0, odd=1

    // -------------------------------------------------------------------------
    // FIFO read strobe: pulse when we latch data from FIFO
    // -------------------------------------------------------------------------
    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n)
            tx_data_rd <= 1'b0;
        else
            tx_data_rd <= (state == S_IDLE) && tx_enable && tx_data_valid;
    end

    // -------------------------------------------------------------------------
    // Serial output
    // -------------------------------------------------------------------------
    always_comb begin
        case (state)
            S_IDLE:   tx_serial = 1'b1;            // Line idle high
            S_START:  tx_serial = 1'b0;            // Start bit = 0
            S_DATA:   tx_serial = shift_reg[bit_idx]; // LSB first
            S_PARITY: tx_serial = parity_bit;
            S_STOP:   tx_serial = 1'b1;            // Stop bit = 1
            default:  tx_serial = 1'b1;
        endcase
    end

    assign tx_active = (state != S_IDLE);

endmodule
