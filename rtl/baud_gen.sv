// =============================================================================
// Module: baud_gen
// Description: Programmable baud rate generator for the serial clock domain.
//              Divides the serial reference clock (sclk) by the programmed
//              divisor value to produce a 16x baud-rate tick.
//
//              Baud rate = sclk_freq / (divisor * 16)
//              Example: sclk=50MHz, divisor=27 -> 115200 baud (approx)
//
// CDC Note: The divisor value originates from the APB register in the pclk
//           domain and must be synchronized to sclk before use here.
//           The parent module handles this synchronization.
// =============================================================================

module baud_gen #(
    parameter int DIV_WIDTH = 16
)(
    input  logic                  sclk,
    input  logic                  srst_n,
    input  logic [DIV_WIDTH-1:0]  divisor,   // Pre-synchronized from pclk domain
    input  logic                  enable,    // Pre-synchronized from pclk domain
    output logic                  tick_16x   // 16x baud rate tick
);

    logic [DIV_WIDTH-1:0] counter;

    always_ff @(posedge sclk or negedge srst_n) begin
        if (!srst_n) begin
            counter  <= '0;
            tick_16x <= 1'b0;
        end else if (!enable || divisor == '0) begin
            counter  <= '0;
            tick_16x <= 1'b0;
        end else if (counter >= (divisor - 1'b1)) begin
            counter  <= '0;
            tick_16x <= 1'b1;
        end else begin
            counter  <= counter + 1'b1;
            tick_16x <= 1'b0;
        end
    end

endmodule
