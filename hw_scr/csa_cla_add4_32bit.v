`timescale 1ns / 1ps
// ============================================================
// csa_cla_add4_32bit.v
// ============================================================

module csa_cla_add4_32bit (
    input  wire [31:0] a,       // input 0
    input  wire [31:0] b,       // input 1
    input  wire [31:0] c,       // input 2
    input  wire [31:0] d,       // input 3
    output wire [32:0] sum_out, // 33-bit result
    output wire        overflow // high if result
);

    wire [31:0] s1 = a ^ b ^ c;
    wire [31:0] c1_raw = (a & b) | (b & c) | (a & c);
    wire [32:0] c1 = {c1_raw, 1'b0};
    wire [32:0] s1_ext = {1'b0, s1};
    wire [32:0] d_ext  = {1'b0, d};
    wire [32:0] s2 = s1_ext ^ c1 ^ d_ext;
    wire [32:0] c2_raw = (s1_ext & c1) | (c1 & d_ext) | (s1_ext & d_ext);
    wire [33:0] c2 = {c2_raw, 1'b0}; 
    wire [33:0] s2_ext = {1'b0, s2};
    wire [33:0] result = s2_ext + c2;
    assign sum_out  = result[32:0];
    assign overflow = result[33]; 

endmodule
