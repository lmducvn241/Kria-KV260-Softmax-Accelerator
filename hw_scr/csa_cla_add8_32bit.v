`timescale 1ns / 1ps
// ============================================================
// csa_cla_add8_32bit.v
// ============================================================

module csa_cla_add8_32bit (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] a,       // input 0
    input  wire [31:0] b,       // input 1
    input  wire [31:0] c,       // input 2
    input  wire [31:0] d,       // input 3
    input  wire [31:0] e,       // input 4
    input  wire [31:0] f,       // input 5
    input  wire [31:0] g,       // input 6
    input  wire [31:0] h,       // input 7
    output wire [33:0] sum_out, // 34-bit result
    output wire        overflow // high if result > 32 bits
);

    // Stage A: Two parallel 4-input CSA+CLA adders
    wire [32:0] sum_abcd;
    wire [32:0] sum_efgh;
    wire        ovf_abcd, ovf_efgh;

    csa_cla_add4_32bit u_add4_0 (
        .a(a), .b(b), .c(c), .d(d),
        .sum_out(sum_abcd), .overflow(ovf_abcd)
    );

    csa_cla_add4_32bit u_add4_1 (
        .a(e), .b(f), .c(g), .d(h),
        .sum_out(sum_efgh), .overflow(ovf_efgh)
    );

    // Pipeline register: break critical path between add4 outputs and final CLA
    reg [32:0] sum_abcd_r, sum_efgh_r;

    always @(posedge clk) begin
        if (rst) begin
            sum_abcd_r <= 33'd0;
            sum_efgh_r <= 33'd0;
        end else begin
            sum_abcd_r <= sum_abcd;
            sum_efgh_r <= sum_efgh;
        end
    end

    // Stage B: Final CLA addition of two 33-bit registered values
    wire [34:0] sum_final = {1'b0, sum_abcd_r} + {1'b0, sum_efgh_r};

    assign sum_out  = sum_final[33:0];
    assign overflow = sum_final[34]; 

endmodule