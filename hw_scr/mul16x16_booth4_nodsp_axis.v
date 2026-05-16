`timescale 1ns / 1ps
// ============================================================
// mul16x16_booth4_nodsp_axis.v
// ============================================================

module mul16x16_booth4_nodsp_axis #(
  parameter integer DATA_WIDTH = 16
)(
  input  wire              clk,
  input  wire              rst,    

  input  wire [DATA_WIDTH-1:0] in_exp_q15,     // unsigned Q1.15
  input  wire [DATA_WIDTH-1:0] in_recip_q15,   // unsigned Q1.15
  input  wire [4:0]            in_shift,
  input  wire                  in_last,
  input  wire                  in_valid,
  output wire                  in_ready,

  output reg  [DATA_WIDTH-1:0] out_prob_q15,
  output reg                   out_last,
  output reg                   out_valid,
  input  wire                  out_ready
);

  // Stallable output register
  wire ce = (~out_valid) || out_ready;
  assign in_ready = ce;

  // Pipeline latency = 9
  localparam integer LAT = 9;

  reg [LAT-1:0] vpipe;
  reg [LAT-1:0] lpipe;
  reg [4:0]     sh_pipe [0:LAT-1];

  integer i;

  // ========================================
  // Stage 0a: Latch operands + Booth recode
  // ========================================
  reg [15:0] a0;
  reg [16:0] b_ext;

  always @(posedge clk) begin
    if (rst) begin
      a0    <= 16'd0;
      b_ext <= 17'd0;
    end else if (ce) begin
      a0    <= in_exp_q15;
      b_ext <= {in_recip_q15, 1'b0};
    end
  end

  wire signed [17:0] a_x1  = $signed({2'b00, a0});
  wire signed [17:0] a_nx1 = -$signed({2'b00, a0});
  wire signed [18:0] a_x2  = $signed({2'b00, a0, 1'b0});
  wire signed [18:0] a_nx2 = -$signed({2'b00, a0, 1'b0});

  wire [2:0] booth_group [0:7];
  wire signed [33:0] pp_sel_comb [0:7];

  genvar g;
  generate
    for (g = 0; g < 8; g = g + 1) begin : GEN_BOOTH
      assign booth_group[g] = b_ext[2*g+2 : 2*g];

      reg signed [33:0] pp_sel_r;
      always @(*) begin
        case (booth_group[g])
          3'b000:  pp_sel_r = 34'sd0;
          3'b001:  pp_sel_r = {{16{a_x1[17]}},  a_x1};
          3'b010:  pp_sel_r = {{16{a_x1[17]}},  a_x1};
          3'b011:  pp_sel_r = {{15{a_x2[18]}},  a_x2};
          3'b100:  pp_sel_r = {{15{a_nx2[18]}}, a_nx2};
          3'b101:  pp_sel_r = {{16{a_nx1[17]}}, a_nx1};
          3'b110:  pp_sel_r = {{16{a_nx1[17]}}, a_nx1};
          3'b111:  pp_sel_r = 34'sd0;
          default: pp_sel_r = 34'sd0;
        endcase
      end
      assign pp_sel_comb[g] = pp_sel_r;
    end
  endgenerate

  reg signed [33:0] pp_sel_reg [0:7];
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 8; i = i + 1) pp_sel_reg[i] <= 34'sd0;
    end else if (ce) begin
      for (i = 0; i < 8; i = i + 1) pp_sel_reg[i] <= pp_sel_comb[i];
    end
  end

  // ========================================
  // Stage 0b: Positional shift + register
  // ========================================
  wire signed [47:0] pp_shifted [0:7];
  generate
    for (g = 0; g < 8; g = g + 1) begin : GEN_SHIFT
      assign pp_shifted[g] = ({{14{pp_sel_reg[g][33]}}, pp_sel_reg[g]}) <<< (2*g);
    end
  endgenerate

  reg signed [47:0] pp_reg [0:7];
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 8; i = i + 1) pp_reg[i] <= 48'sd0;
    end else if (ce) begin
      for (i = 0; i < 8; i = i + 1) pp_reg[i] <= pp_shifted[i];
    end
  end

  // ========================================
  // Stage 1a: Wallace Tree CSA L1 only (8 to 6)
  // ========================================
  wire [47:0] l1_sa = pp_reg[0] ^ pp_reg[1] ^ pp_reg[2];
  wire [47:0] l1_ca = ((pp_reg[0] & pp_reg[1]) | (pp_reg[1] & pp_reg[2]) | (pp_reg[0] & pp_reg[2])) << 1;

  wire [47:0] l1_sb = pp_reg[3] ^ pp_reg[4] ^ pp_reg[5];
  wire [47:0] l1_cb = ((pp_reg[3] & pp_reg[4]) | (pp_reg[4] & pp_reg[5]) | (pp_reg[3] & pp_reg[5])) << 1;

  reg [47:0] l1_sa_r, l1_ca_r, l1_sb_r, l1_cb_r;
  reg [47:0] pp6_r, pp7_r;

  always @(posedge clk) begin
    if (rst) begin
      l1_sa_r <= 48'd0; l1_ca_r <= 48'd0;
      l1_sb_r <= 48'd0; l1_cb_r <= 48'd0;
      pp6_r   <= 48'd0; pp7_r   <= 48'd0;
    end else if (ce) begin
      l1_sa_r <= l1_sa; l1_ca_r <= l1_ca;
      l1_sb_r <= l1_sb; l1_cb_r <= l1_cb;
      pp6_r   <= pp_reg[6]; pp7_r <= pp_reg[7];
    end
  end

  // ========================================
  // Stage 1b: Wallace Tree CSA L2 (6 to 4)
  // ========================================
  wire [47:0] l2_sc = l1_sa_r ^ l1_ca_r ^ l1_sb_r;
  wire [47:0] l2_cc = ((l1_sa_r & l1_ca_r) | (l1_ca_r & l1_sb_r) | (l1_sa_r & l1_sb_r)) << 1;

  wire [47:0] l2_sd = l1_cb_r ^ pp6_r ^ pp7_r;
  wire [47:0] l2_cd = ((l1_cb_r & pp6_r) | (pp6_r & pp7_r) | (l1_cb_r & pp7_r)) << 1;

  reg [47:0] l2_sc_r, l2_cc_r, l2_sd_r, l2_cd_r;

  always @(posedge clk) begin
    if (rst) begin
      l2_sc_r <= 48'd0; l2_cc_r <= 48'd0;
      l2_sd_r <= 48'd0; l2_cd_r <= 48'd0;
    end else if (ce) begin
      l2_sc_r <= l2_sc; l2_cc_r <= l2_cc;
      l2_sd_r <= l2_sd; l2_cd_r <= l2_cd;
    end
  end

  // ========================================
  // Stage 1c: Wallace Tree CSA L3 only (4 to 3)
  // ========================================
  wire [47:0] l3_se = l2_sc_r ^ l2_cc_r ^ l2_sd_r;
  wire [47:0] l3_ce = ((l2_sc_r & l2_cc_r) | (l2_cc_r & l2_sd_r) | (l2_sc_r & l2_sd_r)) << 1;

  reg [47:0] l3_se_r, l3_ce_r, l2_cd_r2;

  always @(posedge clk) begin
    if (rst) begin
      l3_se_r  <= 48'd0;
      l3_ce_r  <= 48'd0;
      l2_cd_r2 <= 48'd0;
    end else if (ce) begin
      l3_se_r  <= l3_se;
      l3_ce_r  <= l3_ce;
      l2_cd_r2 <= l2_cd_r;
    end
  end

  // ========================================
  // Stage 1d: Wallace Tree CSA L4 (3 to 2)
  // ========================================
  wire [47:0] l4_sf = l3_se_r ^ l3_ce_r ^ l2_cd_r2;
  wire [47:0] l4_cf = ((l3_se_r & l3_ce_r) | (l3_ce_r & l2_cd_r2) | (l3_se_r & l2_cd_r2)) << 1;

  reg [47:0] ws_reg, wc_reg;

  always @(posedge clk) begin
    if (rst) begin
      ws_reg <= 48'd0;
      wc_reg <= 48'd0;
    end else if (ce) begin
      ws_reg <= l4_sf;
      wc_reg <= l4_cf;
    end
  end

  // ========================================
  // Stage 2a: Final CPA addition (registered)
  // ========================================
  reg [31:0] prod_reg;

  always @(posedge clk) begin
    if (rst)
      prod_reg <= 32'd0;
    else if (ce)
      prod_reg <= ws_reg[31:0] + wc_reg[31:0];
  end

  // ========================================
  // Stage 2b: Round + extract (registered)
  // ========================================
  localparam [31:0] ROUND_15 = 32'd16384;
  wire [31:0] prod_rnd = prod_reg + ROUND_15;
  reg [15:0] prob_pre_shift;

  always @(posedge clk) begin
    if (rst)
      prob_pre_shift <= 16'd0;
    else if (ce)
      prob_pre_shift <= prod_rnd[30:15];
  end

  // ========================================
  // Stage 2c: Norm-shift + output
  // ========================================
  wire [15:0] prob_norm = prob_pre_shift >> sh_pipe[LAT-1];

  // ========================================
  // Control pipeline
  // ========================================
  always @(posedge clk) begin
    if (rst) begin
      vpipe <= 0;
      lpipe <= 0;
      for (i = 0; i < LAT; i = i + 1) sh_pipe[i] <= 0;
    end else if (ce) begin
      vpipe <= {vpipe[LAT-2:0], in_valid};
      lpipe <= {lpipe[LAT-2:0], (in_valid && in_last)};
      sh_pipe[0] <= in_shift;
      for (i = 1; i < LAT; i = i + 1) sh_pipe[i] <= sh_pipe[i-1];
    end
  end

  // ========================================
  // Output register
  // ========================================
  always @(posedge clk) begin
    if (rst) begin
      out_valid    <= 1'b0;
      out_prob_q15 <= 16'd0;
      out_last     <= 1'b0;
    end else if (ce) begin
      out_valid    <= vpipe[LAT-1];
      out_prob_q15 <= prob_norm;
      out_last     <= lpipe[LAT-1];
    end
  end

endmodule
