`timescale 1ns/1ps
// ============================================================
// pwl_reciprocal_v2.v — Configurable PWL Reciprocal with BRAM ROMs
// ============================================================

module pwl_reciprocal_v2 #(
    parameter SEG_DEPTH  = 64,     
    parameter ADDR_WIDTH = (SEG_DEPTH == 256) ? 8 : (SEG_DEPTH == 128) ? 7 : 6,
    parameter DATA_WIDTH = 16
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    input  wire [DATA_WIDTH-1:0] m_in,       // normalized mantissa (Q1.15)
    output reg  [DATA_WIDTH-1:0] recip_out,
    output reg                   ready
);

    // Sync reset
    reg rst_n_sync1, rst_n_sync2;
    always @(posedge clk) begin
        rst_n_sync1 <= rst_n;
        rst_n_sync2 <= rst_n_sync1;
    end
    wire rst = ~rst_n_sync2;

    localparam integer DELTA_WIDTH = DATA_WIDTH - ADDR_WIDTH - 1;

    // ========================================
    // Stage S0: Extract segment index and delta
    // ========================================
    reg [ADDR_WIDTH-1:0]  seg_idx_s0;
    reg [DELTA_WIDTH-1:0] delta_s0;

    always @(posedge clk) begin
        if (rst) begin
            seg_idx_s0 <= {ADDR_WIDTH{1'b0}};
            delta_s0   <= {DELTA_WIDTH{1'b0}};
        end else begin
            seg_idx_s0 <= m_in[DATA_WIDTH-2 : DATA_WIDTH-ADDR_WIDTH-1];
            delta_s0   <= m_in[DELTA_WIDTH-1 : 0];
        end
    end

    // ========================================
    // Stage S1: BRAM ROM read (y0 and C)
    // ========================================

    // y0 ROM: SEG_DEPTH × 16-bit (BRAM)
    (* ram_style = "block" *) reg [15:0] y0_rom [0:SEG_DEPTH-1];
    // C ROM: SEG_DEPTH × 32-bit (BRAM)
    (* ram_style = "block" *) reg signed [31:0] C_rom [0:SEG_DEPTH-1];

    generate
        if (SEG_DEPTH == 64) begin : gen_rom_64
            initial begin
                y0_rom[ 0]=16'h8000; y0_rom[ 1]=16'h7E08; y0_rom[ 2]=16'h7C1F; y0_rom[ 3]=16'h7A45;
                y0_rom[ 4]=16'h7878; y0_rom[ 5]=16'h76BA; y0_rom[ 6]=16'h7507; y0_rom[ 7]=16'h7361;
                y0_rom[ 8]=16'h71C7; y0_rom[ 9]=16'h7038; y0_rom[10]=16'h6EB4; y0_rom[11]=16'h6D3A;
                y0_rom[12]=16'h6BCA; y0_rom[13]=16'h6A64; y0_rom[14]=16'h6907; y0_rom[15]=16'h67B2;
                y0_rom[16]=16'h6666; y0_rom[17]=16'h6523; y0_rom[18]=16'h63E7; y0_rom[19]=16'h62B3;
                y0_rom[20]=16'h6186; y0_rom[21]=16'h6060; y0_rom[22]=16'h5F41; y0_rom[23]=16'h5E29;
                y0_rom[24]=16'h5D17; y0_rom[25]=16'h5C0C; y0_rom[26]=16'h5B06; y0_rom[27]=16'h5A06;
                y0_rom[28]=16'h590B; y0_rom[29]=16'h5816; y0_rom[30]=16'h5726; y0_rom[31]=16'h563B;
                y0_rom[32]=16'h5555; y0_rom[33]=16'h5474; y0_rom[34]=16'h5398; y0_rom[35]=16'h52BF;
                y0_rom[36]=16'h51EC; y0_rom[37]=16'h511C; y0_rom[38]=16'h5050; y0_rom[39]=16'h4F89;
                y0_rom[40]=16'h4EC5; y0_rom[41]=16'h4E05; y0_rom[42]=16'h4D48; y0_rom[43]=16'h4C90;
                y0_rom[44]=16'h4BDA; y0_rom[45]=16'h4B28; y0_rom[46]=16'h4A79; y0_rom[47]=16'h49CD;
                y0_rom[48]=16'h4925; y0_rom[49]=16'h487F; y0_rom[50]=16'h47DC; y0_rom[51]=16'h473C;
                y0_rom[52]=16'h469F; y0_rom[53]=16'h4604; y0_rom[54]=16'h456C; y0_rom[55]=16'h44D7;
                y0_rom[56]=16'h4444; y0_rom[57]=16'h43B4; y0_rom[58]=16'h4326; y0_rom[59]=16'h429A;
                y0_rom[60]=16'h4211; y0_rom[61]=16'h4189; y0_rom[62]=16'h4104; y0_rom[63]=16'h4081;

                C_rom[ 0]=32'hF81F81F8; C_rom[ 1]=32'hF85C9D10; C_rom[ 2]=32'hF896FBB7; C_rom[ 3]=32'hF8CEC723;
                C_rom[ 4]=32'hF904258A; C_rom[ 5]=32'hF9373A69; C_rom[ 6]=32'hF96826BC; C_rom[ 7]=32'hF9970937;
                C_rom[ 8]=32'hF9C3FE71; C_rom[ 9]=32'hF9EF2114; C_rom[10]=32'hFA188A02; C_rom[11]=32'hFA40507C;
                C_rom[12]=32'hFA668A3D; C_rom[13]=32'hFA8B4B9D; C_rom[14]=32'hFAAEA7AD; C_rom[15]=32'hFAD0B049;
                C_rom[16]=32'hFAF17634; C_rom[17]=32'hFB11092C; C_rom[18]=32'hFB2F77FD; C_rom[19]=32'hFB4CD08F;
                C_rom[20]=32'hFB691FFB; C_rom[21]=32'hFB847296; C_rom[22]=32'hFB9ED400; C_rom[23]=32'hFBB84F2E;
                C_rom[24]=32'hFBD0EE7B; C_rom[25]=32'hFBE8BBAB; C_rom[26]=32'hFBFFBFFC; C_rom[27]=32'hFC160429;
                C_rom[28]=32'hFC2B9075; C_rom[29]=32'hFC406CB4; C_rom[30]=32'hFC54A04F; C_rom[31]=32'hFC68324D;
                C_rom[32]=32'hFC7B2959; C_rom[33]=32'hFC8D8BC5; C_rom[34]=32'hFC9F5F92; C_rom[35]=32'hFCB0AA76;
                C_rom[36]=32'hFCC171DB; C_rom[37]=32'hFCD1BAEB; C_rom[38]=32'hFCE18A8D; C_rom[39]=32'hFCF0E56D;
                C_rom[40]=32'hFCFFCFFD; C_rom[41]=32'hFD0E4E7B; C_rom[42]=32'hFD1C64F0; C_rom[43]=32'hFD2A1737;
                C_rom[44]=32'hFD3768FE; C_rom[45]=32'hFD445DC6; C_rom[46]=32'hFD50F8EA; C_rom[47]=32'hFD5D3D9C;
                C_rom[48]=32'hFD692EEE; C_rom[49]=32'hFD74CFCA; C_rom[50]=32'hFD8022FE; C_rom[51]=32'hFD8B2B37;
                C_rom[52]=32'hFD95EB06; C_rom[53]=32'hFDA064DF; C_rom[54]=32'hFDAA9B1C; C_rom[55]=32'hFDB48FFE;
                C_rom[56]=32'hFDBE45AD; C_rom[57]=32'hFDC7BE3D; C_rom[58]=32'hFDD0FBAB; C_rom[59]=32'hFDD9FFDE;
                C_rom[60]=32'hFDE2CCAB; C_rom[61]=32'hFDEB63D5; C_rom[62]=32'hFDF3C70C; C_rom[63]=32'hFDFBF7F0;
            end
        end else if (SEG_DEPTH == 128) begin : gen_rom_128
            initial begin
                $readmemh("y0_rom_128.mem", y0_rom);
                $readmemh("C_rom_128.mem",  C_rom);
            end
        end else begin : gen_rom_256
            initial begin
                $readmemh("y0_rom_256.mem", y0_rom);
                $readmemh("C_rom_256.mem",  C_rom);
            end
        end
    endgenerate

    reg [15:0]         y0_s1;
    reg signed [31:0]  C_s1;
    reg [DELTA_WIDTH-1:0] delta_s1;
	
    always @(posedge clk) begin
        if (rst) begin
            y0_s1    <= 16'd0;
            C_s1     <= 32'sd0;
            delta_s1 <= {DELTA_WIDTH{1'b0}};
        end else begin
            y0_s1    <= y0_rom[seg_idx_s0];
            C_s1     <= C_rom[seg_idx_s0];
            delta_s1 <= delta_s0;
        end
    end

    // ========================================
    // Stage S1b: Register BRAM outputs
    // ========================================
    reg [15:0]           y0_s1b;
    reg signed [31:0]    C_s1b;
    reg [DELTA_WIDTH-1:0] delta_s1b;

    always @(posedge clk) begin
        if (rst) begin
            y0_s1b    <= 16'd0;
            C_s1b     <= 32'sd0;
            delta_s1b <= {DELTA_WIDTH{1'b0}};
        end else begin
            y0_s1b    <= y0_s1;
            C_s1b     <= C_s1;
            delta_s1b <= delta_s1;
        end
    end

    // ========================================
    // Stages S2-S5: Shift-add multiply C × delta
    // ========================================
    wire signed [47:0] pp [0:DELTA_WIDTH-1];
    genvar k;
    generate
        for (k = 0; k < DELTA_WIDTH; k = k + 1) begin : GEN_PP
            assign pp[k] = delta_s1b[k] ? ({{16{C_s1b[31]}}, C_s1b} <<< k) : 48'sd0;
        end
    endgenerate

    // 4-stage adder tree adapted for variable DELTA_WIDTH
    // Level 1: pair-wise add
    reg signed [47:0] l1_0, l1_1, l1_2, l1_3, l1_4;
    // Level 2
    reg signed [47:0] l2_0, l2_1, l2_2;
    // Level 3
    reg signed [47:0] l3_0, l3_1;
    // Level 4
    reg signed [47:0] mult_sum;

    always @(posedge clk) begin
        if (rst) begin
            l1_0<=0; l1_1<=0; l1_2<=0; l1_3<=0; l1_4<=0;
            l2_0<=0; l2_1<=0; l2_2<=0;
            l3_0<=0; l3_1<=0;
            mult_sum<=0;
        end else begin
            // Level 1
            l1_0 <= pp[0] + pp[1];
            l1_1 <= pp[2] + pp[3];
            l1_2 <= (DELTA_WIDTH > 4) ? pp[4] + pp[5] : pp[4];
            l1_3 <= (DELTA_WIDTH > 6) ? pp[6] + pp[7] : ((DELTA_WIDTH > 6) ? pp[6] : 48'sd0);
            l1_4 <= (DELTA_WIDTH > 8) ? pp[8] : 48'sd0;
            // Level 2
            l2_0 <= l1_0 + l1_1;
            l2_1 <= l1_2 + l1_3;
            l2_2 <= l1_4;
            // Level 3
            l3_0 <= l2_0 + l2_1;
            l3_1 <= l2_2;
            // Level 4
            mult_sum <= l3_0 + l3_1;
        end
    end

    // ========================================
    // Stage S6: Output — y0 + correction
    // ========================================

    // Rounding >> 27 with round-to-nearest
    localparam signed [47:0] ROUND_27 = 48'sd67108864;
    wire signed [47:0] mult_sum_rnd;
    assign mult_sum_rnd = mult_sum + (mult_sum[47] ? -ROUND_27 : ROUND_27);

    wire signed [31:0] corr_q115;
    assign corr_q115 = mult_sum_rnd >>> 27;

    // Delay y0 to align with correction
    reg [15:0] y0_d [0:3];
    integer di;
    always @(posedge clk) begin
        if (rst) begin
            for (di = 0; di < 4; di = di + 1) y0_d[di] <= 16'd0;
        end else begin
            y0_d[0] <= y0_s1b;
            for (di = 1; di < 4; di = di + 1) y0_d[di] <= y0_d[di-1];
        end
    end

    // recip = y0 + corr clamped to [0, 65535]
    wire signed [17:0] recip_signed;
    assign recip_signed = $signed({2'b00, y0_d[3]}) + $signed(corr_q115[17:0]);

    wire [15:0] recip_clamped =
        (recip_signed < 0)           ? 16'd0 :
        (recip_signed > 18'sd65535)  ? 16'hFFFF :
                                       recip_signed[15:0];

    reg [7:0] vpipe;
    always @(posedge clk) begin
        if (rst) vpipe <= 8'd0;
        else     vpipe <= {vpipe[6:0], start};
    end

    // Output
    always @(posedge clk) begin
        if (rst) begin
            recip_out <= 16'd0;
            ready     <= 1'b0;
        end else begin
            recip_out <= recip_clamped;
            ready     <= vpipe[7];
        end
    end

endmodule
