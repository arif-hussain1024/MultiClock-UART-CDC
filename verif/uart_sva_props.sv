// =============================================================================
// Module: uart_sva_props
// Description: SystemVerilog Assertions for CDC, async FIFO, and APB protocol
//              verification. Bound to the DUT hierarchy in the testbench.
//
// Assertion Categories:
//   1. Synchronizer correctness - no unsynchronized CDC usage
//   2. Async FIFO properties - no overflow, no underflow, gray-code integrity
//   3. APB protocol compliance
//   4. UART frame integrity
//   5. Interrupt logic
// =============================================================================

module uart_sva_props (
    // APB domain
    input logic        pclk,
    input logic        prst_n,

    // Serial domain
    input logic        sclk,
    input logic        srst_n,

    // APB signals
    input logic        psel,
    input logic        penable,
    input logic        pwrite,
    input logic [4:0]  paddr,
    input logic        pready,
    input logic        pslverr,

    // TX FIFO signals
    input logic        tx_fifo_wen,
    input logic        tx_fifo_full,
    input logic        tx_fifo_ren,
    input logic        tx_fifo_empty_sclk,

    // RX FIFO signals
    input logic        rx_fifo_wen,
    input logic        rx_fifo_full_sclk,
    input logic        rx_fifo_ren,
    input logic        rx_fifo_empty,

    // TX FIFO gray-code pointers (for gray-code assertions)
    input logic [3:0]  tx_wptr_gray,
    input logic [3:0]  tx_wptr_gray_prev,
    input logic [3:0]  tx_rptr_gray,
    input logic [3:0]  tx_rptr_gray_prev,

    // RX FIFO gray-code pointers
    input logic [3:0]  rx_wptr_gray,
    input logic [3:0]  rx_wptr_gray_prev,
    input logic [3:0]  rx_rptr_gray,
    input logic [3:0]  rx_rptr_gray_prev,

    // UART serial
    input logic        uart_tx,
    input logic        tx_active,

    // Interrupt
    input logic [3:0]  int_status,
    input logic [3:0]  int_en,
    input logic        irq
);

    // =========================================================================
    // 1. APB PROTOCOL COMPLIANCE ASSERTIONS
    // =========================================================================

    // APB: PENABLE must follow PSEL (setup -> access phase)
    property apb_setup_access;
        @(posedge pclk) disable iff (!prst_n)
        (psel && !penable) |=> (psel && penable);
    endproperty
    assert property (apb_setup_access)
        else $error("APB: PENABLE did not assert in access phase after setup");

    // APB: PREADY must be asserted during access phase
    property apb_pready_during_access;
        @(posedge pclk) disable iff (!prst_n)
        (psel && penable) |-> pready;
    endproperty
    assert property (apb_pready_during_access)
        else $error("APB: PREADY not asserted during access phase");

    // APB: Address must be stable between setup and access phases
    property apb_addr_stable;
        @(posedge pclk) disable iff (!prst_n)
        (psel && !penable) |=> $stable(paddr);
    endproperty
    assert property (apb_addr_stable)
        else $error("APB: PADDR changed between setup and access phases");

    // APB: PWRITE must be stable between setup and access phases
    property apb_pwrite_stable;
        @(posedge pclk) disable iff (!prst_n)
        (psel && !penable) |=> $stable(pwrite);
    endproperty
    assert property (apb_pwrite_stable)
        else $error("APB: PWRITE changed between setup and access phases");

    // APB: PSLVERR only during access phase with PSEL
    property apb_pslverr_valid;
        @(posedge pclk) disable iff (!prst_n)
        pslverr |-> (psel && penable);
    endproperty
    assert property (apb_pslverr_valid)
        else $error("APB: PSLVERR asserted outside valid access phase");

    // =========================================================================
    // 2. ASYNC FIFO ASSERTIONS
    // =========================================================================

    // TX FIFO: No write when full (overflow protection)
    property tx_fifo_no_overflow;
        @(posedge pclk) disable iff (!prst_n)
        tx_fifo_full |-> !tx_fifo_wen;
    endproperty
    assert property (tx_fifo_no_overflow)
        else $error("TX FIFO: Write attempted while full (overflow!)");

    // TX FIFO: No read when empty (underflow protection)
    property tx_fifo_no_underflow;
        @(posedge sclk) disable iff (!srst_n)
        tx_fifo_empty_sclk |-> !tx_fifo_ren;
    endproperty
    assert property (tx_fifo_no_underflow)
        else $error("TX FIFO: Read attempted while empty (underflow!)");

    // RX FIFO: No write when full
    property rx_fifo_no_overflow;
        @(posedge sclk) disable iff (!srst_n)
        rx_fifo_full_sclk |-> !rx_fifo_wen;
    endproperty
    assert property (rx_fifo_no_overflow)
        else $error("RX FIFO: Write attempted while full (overflow!)");

    // RX FIFO: No read when empty
    property rx_fifo_no_underflow;
        @(posedge pclk) disable iff (!prst_n)
        rx_fifo_empty |-> !rx_fifo_ren;
    endproperty
    assert property (rx_fifo_no_underflow)
        else $error("RX FIFO: Read attempted while empty (underflow!)");

    // =========================================================================
    // 3. GRAY-CODE POINTER ASSERTIONS
    //    Gray-code pointers must change by exactly one bit per increment.
    //    This is critical for CDC safety - multi-bit transitions would cause
    //    incorrect full/empty detection in the receiving domain.
    // =========================================================================

    // Function to count bit differences
    function automatic int unsigned hamming_dist(input logic [3:0] a, b);
        logic [3:0] diff;
        diff = a ^ b;
        return $countones(diff);
    endfunction

    // TX write pointer: gray-code increment changes exactly 1 bit
    property tx_wptr_gray_onehot_change;
        @(posedge pclk) disable iff (!prst_n)
        (tx_wptr_gray != tx_wptr_gray_prev) |->
            (hamming_dist(tx_wptr_gray, tx_wptr_gray_prev) == 1);
    endproperty
    assert property (tx_wptr_gray_onehot_change)
        else $error("TX FIFO: Gray-code write pointer changed >1 bit!");

    // TX read pointer: gray-code increment changes exactly 1 bit
    property tx_rptr_gray_onehot_change;
        @(posedge sclk) disable iff (!srst_n)
        (tx_rptr_gray != tx_rptr_gray_prev) |->
            (hamming_dist(tx_rptr_gray, tx_rptr_gray_prev) == 1);
    endproperty
    assert property (tx_rptr_gray_onehot_change)
        else $error("TX FIFO: Gray-code read pointer changed >1 bit!");

    // RX write pointer: gray-code increment changes exactly 1 bit
    property rx_wptr_gray_onehot_change;
        @(posedge sclk) disable iff (!srst_n)
        (rx_wptr_gray != rx_wptr_gray_prev) |->
            (hamming_dist(rx_wptr_gray, rx_wptr_gray_prev) == 1);
    endproperty
    assert property (rx_wptr_gray_onehot_change)
        else $error("RX FIFO: Gray-code write pointer changed >1 bit!");

    // RX read pointer: gray-code increment changes exactly 1 bit
    property rx_rptr_gray_onehot_change;
        @(posedge pclk) disable iff (!prst_n)
        (rx_rptr_gray != rx_rptr_gray_prev) |->
            (hamming_dist(rx_rptr_gray, rx_rptr_gray_prev) == 1);
    endproperty
    assert property (rx_rptr_gray_onehot_change)
        else $error("RX FIFO: Gray-code read pointer changed >1 bit!");

    // =========================================================================
    // 4. UART FRAME ASSERTIONS
    // =========================================================================

    // TX line must be idle (high) when not actively transmitting
    property tx_idle_high;
        @(posedge sclk) disable iff (!srst_n)
        !tx_active |-> uart_tx;
    endproperty
    assert property (tx_idle_high)
        else $error("UART TX: Line not idle-high when inactive");

    // =========================================================================
    // 5. INTERRUPT LOGIC ASSERTIONS
    // =========================================================================

    // IRQ output must equal OR of (status AND enable)
    property irq_correct;
        @(posedge pclk) disable iff (!prst_n)
        irq == |(int_status & int_en);
    endproperty
    assert property (irq_correct)
        else $error("IRQ: Output does not match (status & enable)");

    // =========================================================================
    // COVERAGE
    // =========================================================================

    // FIFO occupancy coverage
    cover property (@(posedge pclk) disable iff (!prst_n) tx_fifo_full);
    cover property (@(posedge pclk) disable iff (!prst_n) tx_fifo_empty_sclk);
    cover property (@(posedge pclk) disable iff (!prst_n) rx_fifo_full_sclk);
    cover property (@(posedge pclk) disable iff (!prst_n) rx_fifo_empty);

    // Error condition coverage
    cover property (@(posedge pclk) disable iff (!prst_n) int_status[2]); // parity error
    cover property (@(posedge pclk) disable iff (!prst_n) int_status[3]); // overrun

    // Interrupt coverage
    cover property (@(posedge pclk) disable iff (!prst_n) irq);

    // Back-to-back TX transfer coverage
    cover property (@(posedge sclk) disable iff (!srst_n)
        tx_fifo_ren ##1 !tx_fifo_empty_sclk);

endmodule
