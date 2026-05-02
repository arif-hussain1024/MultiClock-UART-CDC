// =============================================================================
// Module: sync_2ff
// Description: Parameterized double-flop synchronizer for single-bit or
//              multi-bit signal crossing between asynchronous clock domains.
//              NOT safe for multi-bit buses unless signals are gray-coded
//              or otherwise guaranteed to change only one bit at a time.
//
// CDC Strategy: Two back-to-back flip-flops reduce metastability probability
//               to negligible levels (MTBF >> system lifetime for typical
//               clock frequencies and FPGA/ASIC technologies).
// =============================================================================

module sync_2ff #(
    parameter int WIDTH     = 1,
    parameter logic [WIDTH-1:0] RESET_VAL = '0
)(
    input  logic             clk,      // Destination clock
    input  logic             rst_n,    // Destination async reset (active low)
    input  logic [WIDTH-1:0] d,        // Input from source domain
    output logic [WIDTH-1:0] q         // Synchronized output in destination domain
);

    // Synthesis attributes to prevent optimization and ensure proper placement
    (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] sync_stage [1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_stage[0] <= RESET_VAL;
            sync_stage[1] <= RESET_VAL;
        end else begin
            sync_stage[0] <= d;            // May go metastable
            sync_stage[1] <= sync_stage[0]; // Resolves metastability
        end
    end

    assign q = sync_stage[1];

endmodule
