// =============================================================================
// Module: apb_uart_regs
// Description: APB3 slave interface for CPU-side register access.
//              Operates entirely in the pclk domain.
//
// Register Map (byte-addressed, 32-bit aligned):
//   0x00 TXDATA   [WO] bits [7:0]  = TX data (writes push to TX FIFO)
//   0x04 RXDATA   [RO] bits [7:0]  = RX data (reads pop from RX FIFO)
//   0x08 BAUDDIV  [RW] bits [15:0] = Baud rate divisor
//   0x0C CTRL     [RW] bit 0 = TX enable
//                       bit 1 = RX enable
//                       bit 2 = Parity enable
//                       bit 3 = Parity type (0=even, 1=odd)
//   0x10 STATUS   [RO] bit 0 = TX FIFO empty
//                       bit 1 = TX FIFO full
//                       bit 2 = RX FIFO empty
//                       bit 3 = RX FIFO full
//                       bit 4 = Parity error (sticky)
//                       bit 5 = Overrun error (sticky)
//                       bit 6 = Frame error (sticky)
//   0x14 INT_EN   [RW] bits [3:0] = Interrupt enable mask
//   0x18 INT_STAT [R/W1C] bits [3:0] = Interrupt status (write-1-to-clear)
//
// APB3 compliance: PREADY always asserted (zero wait state),
//                  PSLVERR on access to undefined addresses.
// =============================================================================

module apb_uart_regs (
    // APB3 interface
    input  logic        pclk,
    input  logic        prst_n,
    input  logic        psel,
    input  logic        penable,
    input  logic        pwrite,
    input  logic [4:0]  paddr,      // Byte address, bits [4:2] select register
    input  logic [31:0] pwdata,
    output logic [31:0] prdata,
    output logic        pready,
    output logic        pslverr,

    // TX FIFO interface (pclk domain - write side of TX async FIFO)
    output logic [7:0]  tx_fifo_wdata,
    output logic        tx_fifo_wen,
    input  logic        tx_fifo_full,
    input  logic        tx_fifo_empty,

    // RX FIFO interface (pclk domain - read side of RX async FIFO)
    input  logic [7:0]  rx_fifo_rdata,
    output logic        rx_fifo_ren,
    input  logic        rx_fifo_full,
    input  logic        rx_fifo_empty,

    // Configuration outputs (to be synchronized to sclk domain by parent)
    output logic [15:0] baud_divisor,
    output logic        tx_enable,
    output logic        rx_enable,
    output logic        parity_en,
    output logic        parity_type,

    // Status inputs (synchronized from sclk domain)
    input  logic        parity_error_sync,   // Synchronized from sclk
    input  logic        frame_error_sync,    // Synchronized from sclk
    input  logic        overrun_error,       // Detected in pclk domain

    // Interrupt interface
    output logic [3:0]  int_en_reg,
    input  logic [3:0]  int_status,
    output logic [3:0]  int_clear,
    output logic        int_clear_valid
);

    // -------------------------------------------------------------------------
    // Address decode
    // -------------------------------------------------------------------------
    localparam logic [4:0] ADDR_TXDATA   = 5'h00;
    localparam logic [4:0] ADDR_RXDATA   = 5'h04;
    localparam logic [4:0] ADDR_BAUDDIV  = 5'h08;
    localparam logic [4:0] ADDR_CTRL     = 5'h0C;
    localparam logic [4:0] ADDR_STATUS   = 5'h10;
    localparam logic [4:0] ADDR_INT_EN   = 5'h14;
    localparam logic [4:0] ADDR_INT_STAT = 5'h18;

    logic valid_addr;
    assign valid_addr = (paddr == ADDR_TXDATA)   || (paddr == ADDR_RXDATA)  ||
                        (paddr == ADDR_BAUDDIV)  || (paddr == ADDR_CTRL)    ||
                        (paddr == ADDR_STATUS)   || (paddr == ADDR_INT_EN)  ||
                        (paddr == ADDR_INT_STAT);

    // -------------------------------------------------------------------------
    // APB handshake: zero wait-state, error on invalid address
    // -------------------------------------------------------------------------
    assign pready  = 1'b1;
    assign pslverr = psel && penable && !valid_addr;

    // -------------------------------------------------------------------------
    // Write phase detection
    // -------------------------------------------------------------------------
    logic apb_write;
    logic apb_read;
    assign apb_write = psel && penable && pwrite && valid_addr;
    assign apb_read  = psel && penable && !pwrite && valid_addr;

    // -------------------------------------------------------------------------
    // Configuration registers
    // -------------------------------------------------------------------------
    logic [15:0] r_bauddiv;
    logic [3:0]  r_ctrl;     // {parity_type, parity_en, rx_enable, tx_enable}
    logic [3:0]  r_int_en;

    // Sticky error status bits (set by pulse, cleared by writing STATUS register)
    logic        r_parity_err_sticky;
    logic        r_frame_err_sticky;
    logic        r_overrun_err_sticky;

    always_ff @(posedge pclk or negedge prst_n) begin
        if (!prst_n) begin
            r_bauddiv  <= 16'd1;   // Default divisor
            r_ctrl     <= 4'b0000; // Disabled
            r_int_en   <= 4'b0000;
        end else if (apb_write) begin
            case (paddr)
                ADDR_BAUDDIV: r_bauddiv <= pwdata[15:0];
                ADDR_CTRL:    r_ctrl    <= pwdata[3:0];
                ADDR_INT_EN:  r_int_en  <= pwdata[3:0];
                default: ;
            endcase
        end
    end

    // Sticky error bits
    always_ff @(posedge pclk or negedge prst_n) begin
        if (!prst_n) begin
            r_parity_err_sticky  <= 1'b0;
            r_frame_err_sticky   <= 1'b0;
            r_overrun_err_sticky <= 1'b0;
        end else begin
            // Set on error pulse
            if (parity_error_sync) r_parity_err_sticky  <= 1'b1;
            if (frame_error_sync)  r_frame_err_sticky   <= 1'b1;
            if (overrun_error)     r_overrun_err_sticky <= 1'b1;

            // Clear on STATUS register write
            if (apb_write && paddr == ADDR_STATUS) begin
                if (pwdata[4]) r_parity_err_sticky  <= 1'b0;
                if (pwdata[5]) r_overrun_err_sticky <= 1'b0;
                if (pwdata[6]) r_frame_err_sticky   <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Configuration output mapping
    // -------------------------------------------------------------------------
    assign baud_divisor = r_bauddiv;
    assign tx_enable    = r_ctrl[0];
    assign rx_enable    = r_ctrl[1];
    assign parity_en    = r_ctrl[2];
    assign parity_type  = r_ctrl[3];

    assign int_en_reg   = r_int_en;

    // -------------------------------------------------------------------------
    // TX FIFO write: push on write to TXDATA register
    // -------------------------------------------------------------------------
    assign tx_fifo_wdata = pwdata[7:0];
    assign tx_fifo_wen   = apb_write && (paddr == ADDR_TXDATA) && !tx_fifo_full;

    // -------------------------------------------------------------------------
    // RX FIFO read: pop on read from RXDATA register
    // -------------------------------------------------------------------------
    assign rx_fifo_ren = apb_read && (paddr == ADDR_RXDATA) && !rx_fifo_empty;

    // -------------------------------------------------------------------------
    // Interrupt status write-1-to-clear
    // -------------------------------------------------------------------------
    assign int_clear       = pwdata[3:0];
    assign int_clear_valid = apb_write && (paddr == ADDR_INT_STAT);

    // -------------------------------------------------------------------------
    // Read data mux
    // -------------------------------------------------------------------------
    always_comb begin
        prdata = 32'h0;
        case (paddr)
            ADDR_TXDATA:   prdata = 32'h0; // Write-only
            ADDR_RXDATA:   prdata = {24'h0, rx_fifo_rdata};
            ADDR_BAUDDIV:  prdata = {16'h0, r_bauddiv};
            ADDR_CTRL:     prdata = {28'h0, r_ctrl};
            ADDR_STATUS:   prdata = {25'h0,
                                     r_frame_err_sticky,
                                     r_overrun_err_sticky,
                                     r_parity_err_sticky,
                                     rx_fifo_full,
                                     rx_fifo_empty,
                                     tx_fifo_full,
                                     tx_fifo_empty};
            ADDR_INT_EN:   prdata = {28'h0, r_int_en};
            ADDR_INT_STAT: prdata = {28'h0, int_status};
            default:       prdata = 32'h0;
        endcase
    end

endmodule
