// =============================================================================
// Module: async_fifo
// Description: Asynchronous FIFO with gray-code read/write pointers for safe
//              data transfer across clock domains.
//
// CDC Strategy:
//   - Write pointer (binary) is converted to gray-code in the write domain,
//     then synchronized to the read domain via a 2-FF synchronizer.
//   - Read pointer (binary) is converted to gray-code in the read domain,
//     then synchronized to the write domain via a 2-FF synchronizer.
//   - Gray-code ensures only one bit changes per pointer increment,
//     preventing erroneous full/empty detection from multi-bit transitions.
//   - DEPTH must be a power of 2 for gray-code to work correctly.
//
// Full condition:  wptr_gray == {~rptr_gray_sync[ADDR_W:ADDR_W-1],
//                                 rptr_gray_sync[ADDR_W-2:0]}
// Empty condition: rptr_gray == wptr_gray_sync
// =============================================================================

module async_fifo #(
    parameter int DEPTH = 8,        // Must be power of 2
    parameter int WIDTH = 8         // Data width
)(
    // Write port (source clock domain)
    input  logic             wclk,
    input  logic             wrst_n,
    input  logic             wen,
    input  logic [WIDTH-1:0] wdata,
    output logic             wfull,

    // Read port (destination clock domain)
    input  logic             rclk,
    input  logic             rrst_n,
    input  logic             ren,
    output logic [WIDTH-1:0] rdata,
    output logic             rempty
);

    // Address width (extra bit for full/empty distinction)
    localparam int ADDR_W = $clog2(DEPTH);
    localparam int PTR_W  = ADDR_W + 1; // Extra MSB for wrap-around detection

    // -------------------------------------------------------------------------
    // Memory
    // -------------------------------------------------------------------------
    logic [WIDTH-1:0] mem [DEPTH];

    // -------------------------------------------------------------------------
    // Write domain signals
    // -------------------------------------------------------------------------
    logic [PTR_W-1:0] wptr_bin, wptr_bin_next;
    logic [PTR_W-1:0] wptr_gray, wptr_gray_next;
    logic [PTR_W-1:0] rptr_gray_sync; // Read pointer synchronized to write domain

    // -------------------------------------------------------------------------
    // Read domain signals
    // -------------------------------------------------------------------------
    logic [PTR_W-1:0] rptr_bin, rptr_bin_next;
    logic [PTR_W-1:0] rptr_gray, rptr_gray_next;
    logic [PTR_W-1:0] wptr_gray_sync; // Write pointer synchronized to read domain

    // =========================================================================
    // Binary to Gray-code conversion
    // =========================================================================
    function automatic logic [PTR_W-1:0] bin2gray(input logic [PTR_W-1:0] bin);
        return bin ^ (bin >> 1);
    endfunction

    // =========================================================================
    // Write domain logic
    // =========================================================================
    assign wptr_bin_next  = wptr_bin + (wen & ~wfull);
    assign wptr_gray_next = bin2gray(wptr_bin_next);

    always_ff @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin  <= '0;
            wptr_gray <= '0;
        end else begin
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
        end
    end

    // Write to memory
    always_ff @(posedge wclk) begin
        if (wen && !wfull)
            mem[wptr_bin[ADDR_W-1:0]] <= wdata;
    end

    // Full detection: compare write gray pointer with synchronized read gray pointer
    // Full when gray pointers match except the two MSBs are inverted
    assign wfull = (wptr_gray_next == {~rptr_gray_sync[PTR_W-1:PTR_W-2],
                                        rptr_gray_sync[PTR_W-3:0]});

    // =========================================================================
    // Read domain logic
    // =========================================================================
    assign rptr_bin_next  = rptr_bin + (ren & ~rempty);
    assign rptr_gray_next = bin2gray(rptr_bin_next);

    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin  <= '0;
            rptr_gray <= '0;
        end else begin
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
        end
    end

    // Read from memory (combinational read)
    assign rdata = mem[rptr_bin[ADDR_W-1:0]];

    // Empty detection: read gray pointer matches synchronized write gray pointer
    assign rempty = (rptr_gray_next == wptr_gray_sync);

    // =========================================================================
    // Gray-code pointer synchronizers (CDC crossings)
    // =========================================================================

    // Sync write pointer (gray) to read domain
    sync_2ff #(.WIDTH(PTR_W)) u_sync_wptr (
        .clk   (rclk),
        .rst_n (rrst_n),
        .d     (wptr_gray),
        .q     (wptr_gray_sync)
    );

    // Sync read pointer (gray) to write domain
    sync_2ff #(.WIDTH(PTR_W)) u_sync_rptr (
        .clk   (wclk),
        .rst_n (wrst_n),
        .d     (rptr_gray),
        .q     (rptr_gray_sync)
    );

endmodule
