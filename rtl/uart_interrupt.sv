// =============================================================================
// Module: uart_interrupt
// Description: Interrupt generation and management for the UART controller.
//              Operates in the APB/system clock domain (pclk).
//
//              Interrupt sources (active-high pulses from status logic):
//                [0] TX FIFO empty
//                [1] RX FIFO full
//                [2] RX parity error
//                [3] RX overrun error
//
//              Interrupt enable register: masks individual sources
//              Interrupt status register: write-1-to-clear sticky bits
// =============================================================================

module uart_interrupt (
    input  logic       pclk,
    input  logic       prst_n,

    // Raw interrupt source pulses (synchronized to pclk domain)
    input  logic       irq_tx_empty,     // TX FIFO became empty
    input  logic       irq_rx_full,      // RX FIFO became full
    input  logic       irq_parity_err,   // Parity error detected
    input  logic       irq_overrun,      // RX overrun detected

    // Register interface
    input  logic [3:0] int_en,           // Interrupt enable bits
    input  logic [3:0] int_clear,        // Write-1-to-clear from APB write
    input  logic       int_clear_valid,  // APB write strobe for int status reg

    // Outputs
    output logic [3:0] int_status,       // Sticky interrupt status (readable)
    output logic       irq_out           // Combined interrupt output
);

    // -------------------------------------------------------------------------
    // Raw interrupt event capture (OR with existing status)
    // -------------------------------------------------------------------------
    logic [3:0] irq_sources;
    assign irq_sources = {irq_overrun, irq_parity_err, irq_rx_full, irq_tx_empty};

    // -------------------------------------------------------------------------
    // Sticky interrupt status with write-1-to-clear
    // -------------------------------------------------------------------------
    always_ff @(posedge pclk or negedge prst_n) begin
        if (!prst_n) begin
            int_status <= '0;
        end else begin
            for (int i = 0; i < 4; i++) begin
                if (int_clear_valid && int_clear[i])
                    int_status[i] <= irq_sources[i]; // Clear but re-set if simultaneous
                else
                    int_status[i] <= int_status[i] | irq_sources[i];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Combined interrupt output (masked by enable)
    // -------------------------------------------------------------------------
    assign irq_out = |(int_status & int_en);

endmodule
