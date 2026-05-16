`timescale 1ns / 1ps
// ============================================================
// exp2_base2_v2.v — Base-2 Exponential with BRAM ROM + Taylor LSB
// ============================================================

module exp2_base2_v2 #(
    parameter EXP_MODE = 0   // 0=ROM-only, 1=ROM+Taylor-1
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        ce,        // clock enable (freeze pipeline when 0)
    input  wire [15:0] d_q412,    // Q4.12 unsigned difference (max - x[i])
    output reg  [15:0] y_q115     // Q1.15 unsigned result (exp(-d))
);

    // ========================================
    // Stage S1a: Level 1 — pair-wise additions
    // ========================================
    wire [28:0] d_ext = {13'd0, d_q412};
    wire [28:0] t0 = (d_ext << 12) + (d_ext << 10);   
    wire [28:0] t1 = (d_ext <<  9) + (d_ext <<  8);   
    wire [28:0] t2 = (d_ext <<  4) + (d_ext <<  2);   
    wire [28:0] t3 = d_ext;                            
    (* use_dsp = "no" *)
    reg [28:0] t0_r, t1_r, t2_r, t3_r;
    always @(posedge clk) begin
        if (rst) begin
            t0_r <= 29'd0;
            t1_r <= 29'd0;
            t2_r <= 29'd0;
            t3_r <= 29'd0;
        end else if (ce) begin
            t0_r <= t0;
            t1_r <= t1;
            t2_r <= t2;
            t3_r <= t3;
        end
    end

    // ========================================
    // Stage S1b: Level 2 — reduce 4 to 2
    // ========================================
    (* use_dsp = "no" *)
    reg [28:0] t4_r, t5_r;
    always @(posedge clk) begin
        if (rst) begin
            t4_r <= 29'd0;
            t5_r <= 29'd0;
        end else if (ce) begin
            t4_r <= t0_r + t1_r;
            t5_r <= t2_r + t3_r;
        end
    end

    // ========================================
    // Stage S1c: Level 3 — final sum to z
    // ========================================
    (* use_dsp = "no", DONT_TOUCH = "yes" *)
    reg [28:0] z_s1;
    always @(posedge clk) begin
        if (rst)
            z_s1 <= 29'd0;
        else if (ce)
            z_s1 <= t4_r + t5_r;
    end

    // ========================================
    // Stage S2: Extract n, f, ROM lookup (BRAM)
    // ========================================
    wire [4:0] n_extract = z_s1[28:24];
    wire [7:0] f_extract = z_s1[23:16];
    (* ram_style = "block" *) reg [15:0] exp_rom [0:255];

    // ROM initialization
    initial begin
        exp_rom[  0] = 16'd32768; exp_rom[  1] = 16'd32679; exp_rom[  2] = 16'd32591; exp_rom[  3] = 16'd32503;
        exp_rom[  4] = 16'd32415; exp_rom[  5] = 16'd32327; exp_rom[  6] = 16'd32240; exp_rom[  7] = 16'd32153;
        exp_rom[  8] = 16'd32066; exp_rom[  9] = 16'd31979; exp_rom[ 10] = 16'd31893; exp_rom[ 11] = 16'd31806;
        exp_rom[ 12] = 16'd31720; exp_rom[ 13] = 16'd31635; exp_rom[ 14] = 16'd31549; exp_rom[ 15] = 16'd31464;
        exp_rom[ 16] = 16'd31379; exp_rom[ 17] = 16'd31294; exp_rom[ 18] = 16'd31209; exp_rom[ 19] = 16'd31125;
        exp_rom[ 20] = 16'd31041; exp_rom[ 21] = 16'd30957; exp_rom[ 22] = 16'd30873; exp_rom[ 23] = 16'd30790;
        exp_rom[ 24] = 16'd30706; exp_rom[ 25] = 16'd30623; exp_rom[ 26] = 16'd30541; exp_rom[ 27] = 16'd30458;
        exp_rom[ 28] = 16'd30376; exp_rom[ 29] = 16'd30293; exp_rom[ 30] = 16'd30212; exp_rom[ 31] = 16'd30130;
        exp_rom[ 32] = 16'd30048; exp_rom[ 33] = 16'd29967; exp_rom[ 34] = 16'd29886; exp_rom[ 35] = 16'd29805;
        exp_rom[ 36] = 16'd29725; exp_rom[ 37] = 16'd29644; exp_rom[ 38] = 16'd29564; exp_rom[ 39] = 16'd29484;
        exp_rom[ 40] = 16'd29405; exp_rom[ 41] = 16'd29325; exp_rom[ 42] = 16'd29246; exp_rom[ 43] = 16'd29167;
        exp_rom[ 44] = 16'd29088; exp_rom[ 45] = 16'd29009; exp_rom[ 46] = 16'd28931; exp_rom[ 47] = 16'd28852;
        exp_rom[ 48] = 16'd28774; exp_rom[ 49] = 16'd28697; exp_rom[ 50] = 16'd28619; exp_rom[ 51] = 16'd28542;
        exp_rom[ 52] = 16'd28464; exp_rom[ 53] = 16'd28388; exp_rom[ 54] = 16'd28311; exp_rom[ 55] = 16'd28234;
        exp_rom[ 56] = 16'd28158; exp_rom[ 57] = 16'd28082; exp_rom[ 58] = 16'd28006; exp_rom[ 59] = 16'd27930;
        exp_rom[ 60] = 16'd27855; exp_rom[ 61] = 16'd27779; exp_rom[ 62] = 16'd27704; exp_rom[ 63] = 16'd27629;
        exp_rom[ 64] = 16'd27554; exp_rom[ 65] = 16'd27480; exp_rom[ 66] = 16'd27406; exp_rom[ 67] = 16'd27332;
        exp_rom[ 68] = 16'd27258; exp_rom[ 69] = 16'd27184; exp_rom[ 70] = 16'd27110; exp_rom[ 71] = 16'd27037;
        exp_rom[ 72] = 16'd26964; exp_rom[ 73] = 16'd26891; exp_rom[ 74] = 16'd26818; exp_rom[ 75] = 16'd26746;
        exp_rom[ 76] = 16'd26674; exp_rom[ 77] = 16'd26601; exp_rom[ 78] = 16'd26530; exp_rom[ 79] = 16'd26458;
        exp_rom[ 80] = 16'd26386; exp_rom[ 81] = 16'd26315; exp_rom[ 82] = 16'd26244; exp_rom[ 83] = 16'd26173;
        exp_rom[ 84] = 16'd26102; exp_rom[ 85] = 16'd26031; exp_rom[ 86] = 16'd25961; exp_rom[ 87] = 16'd25891;
        exp_rom[ 88] = 16'd25821; exp_rom[ 89] = 16'd25751; exp_rom[ 90] = 16'd25681; exp_rom[ 91] = 16'd25612;
        exp_rom[ 92] = 16'd25543; exp_rom[ 93] = 16'd25474; exp_rom[ 94] = 16'd25405; exp_rom[ 95] = 16'd25336;
        exp_rom[ 96] = 16'd25268; exp_rom[ 97] = 16'd25199; exp_rom[ 98] = 16'd25131; exp_rom[ 99] = 16'd25063;
        exp_rom[100] = 16'd24995; exp_rom[101] = 16'd24928; exp_rom[102] = 16'd24860; exp_rom[103] = 16'd24793;
        exp_rom[104] = 16'd24726; exp_rom[105] = 16'd24659; exp_rom[106] = 16'd24593; exp_rom[107] = 16'd24526;
        exp_rom[108] = 16'd24460; exp_rom[109] = 16'd24394; exp_rom[110] = 16'd24328; exp_rom[111] = 16'd24262;
        exp_rom[112] = 16'd24196; exp_rom[113] = 16'd24131; exp_rom[114] = 16'd24066; exp_rom[115] = 16'd24001;
        exp_rom[116] = 16'd23936; exp_rom[117] = 16'd23871; exp_rom[118] = 16'd23806; exp_rom[119] = 16'd23742;
        exp_rom[120] = 16'd23678; exp_rom[121] = 16'd23614; exp_rom[122] = 16'd23550; exp_rom[123] = 16'd23486;
        exp_rom[124] = 16'd23423; exp_rom[125] = 16'd23359; exp_rom[126] = 16'd23296; exp_rom[127] = 16'd23233;
        exp_rom[128] = 16'd23170; exp_rom[129] = 16'd23108; exp_rom[130] = 16'd23045; exp_rom[131] = 16'd22983;
        exp_rom[132] = 16'd22921; exp_rom[133] = 16'd22859; exp_rom[134] = 16'd22797; exp_rom[135] = 16'd22735;
        exp_rom[136] = 16'd22674; exp_rom[137] = 16'd22613; exp_rom[138] = 16'd22552; exp_rom[139] = 16'd22491;
        exp_rom[140] = 16'd22430; exp_rom[141] = 16'd22369; exp_rom[142] = 16'd22309; exp_rom[143] = 16'd22248;
        exp_rom[144] = 16'd22188; exp_rom[145] = 16'd22128; exp_rom[146] = 16'd22068; exp_rom[147] = 16'd22009;
        exp_rom[148] = 16'd21949; exp_rom[149] = 16'd21890; exp_rom[150] = 16'd21831; exp_rom[151] = 16'd21772;
        exp_rom[152] = 16'd21713; exp_rom[153] = 16'd21654; exp_rom[154] = 16'd21595; exp_rom[155] = 16'd21537;
        exp_rom[156] = 16'd21479; exp_rom[157] = 16'd21421; exp_rom[158] = 16'd21363; exp_rom[159] = 16'd21305;
        exp_rom[160] = 16'd21247; exp_rom[161] = 16'd21190; exp_rom[162] = 16'd21133; exp_rom[163] = 16'd21076;
        exp_rom[164] = 16'd21019; exp_rom[165] = 16'd20962; exp_rom[166] = 16'd20905; exp_rom[167] = 16'd20849;
        exp_rom[168] = 16'd20792; exp_rom[169] = 16'd20736; exp_rom[170] = 16'd20680; exp_rom[171] = 16'd20624;
        exp_rom[172] = 16'd20568; exp_rom[173] = 16'd20513; exp_rom[174] = 16'd20457; exp_rom[175] = 16'd20402;
        exp_rom[176] = 16'd20347; exp_rom[177] = 16'd20292; exp_rom[178] = 16'd20237; exp_rom[179] = 16'd20182;
        exp_rom[180] = 16'd20127; exp_rom[181] = 16'd20073; exp_rom[182] = 16'd20019; exp_rom[183] = 16'd19965;
        exp_rom[184] = 16'd19911; exp_rom[185] = 16'd19857; exp_rom[186] = 16'd19803; exp_rom[187] = 16'd19750;
        exp_rom[188] = 16'd19696; exp_rom[189] = 16'd19643; exp_rom[190] = 16'd19590; exp_rom[191] = 16'd19537;
        exp_rom[192] = 16'd19484; exp_rom[193] = 16'd19431; exp_rom[194] = 16'd19379; exp_rom[195] = 16'd19326;
        exp_rom[196] = 16'd19274; exp_rom[197] = 16'd19222; exp_rom[198] = 16'd19170; exp_rom[199] = 16'd19118;
        exp_rom[200] = 16'd19066; exp_rom[201] = 16'd19015; exp_rom[202] = 16'd18963; exp_rom[203] = 16'd18912;
        exp_rom[204] = 16'd18861; exp_rom[205] = 16'd18810; exp_rom[206] = 16'd18759; exp_rom[207] = 16'd18708;
        exp_rom[208] = 16'd18658; exp_rom[209] = 16'd18607; exp_rom[210] = 16'd18557; exp_rom[211] = 16'd18507;
        exp_rom[212] = 16'd18457; exp_rom[213] = 16'd18407; exp_rom[214] = 16'd18357; exp_rom[215] = 16'd18308;
        exp_rom[216] = 16'd18258; exp_rom[217] = 16'd18209; exp_rom[218] = 16'd18160; exp_rom[219] = 16'd18110;
        exp_rom[220] = 16'd18061; exp_rom[221] = 16'd18013; exp_rom[222] = 16'd17964; exp_rom[223] = 16'd17915;
        exp_rom[224] = 16'd17867; exp_rom[225] = 16'd17819; exp_rom[226] = 16'd17770; exp_rom[227] = 16'd17722;
        exp_rom[228] = 16'd17674; exp_rom[229] = 16'd17627; exp_rom[230] = 16'd17579; exp_rom[231] = 16'd17531;
        exp_rom[232] = 16'd17484; exp_rom[233] = 16'd17437; exp_rom[234] = 16'd17390; exp_rom[235] = 16'd17343;
        exp_rom[236] = 16'd17296; exp_rom[237] = 16'd17249; exp_rom[238] = 16'd17202; exp_rom[239] = 16'd17156;
        exp_rom[240] = 16'd17109; exp_rom[241] = 16'd17063; exp_rom[242] = 16'd17017; exp_rom[243] = 16'd16971;
        exp_rom[244] = 16'd16925; exp_rom[245] = 16'd16879; exp_rom[246] = 16'd16834; exp_rom[247] = 16'd16788;
        exp_rom[248] = 16'd16743; exp_rom[249] = 16'd16697; exp_rom[250] = 16'd16652; exp_rom[251] = 16'd16607;
        exp_rom[252] = 16'd16562; exp_rom[253] = 16'd16518; exp_rom[254] = 16'd16473; exp_rom[255] = 16'd16428;
    end

    reg [15:0] lut_val_s2;   
    reg [4:0]  n_s2;
    reg [15:0] epsilon_s2;  

    always @(posedge clk) begin
        if (rst) begin
            lut_val_s2 <= 16'd0;
            n_s2       <= 5'd0;
            epsilon_s2 <= 16'd0;
        end else if (ce) begin
            n_s2       <= n_extract;
            lut_val_s2 <= exp_rom[f_extract];  // BRAM read, 1-cycle latency
            epsilon_s2 <= z_s1[15:0];          // save sub-ROM bits
        end
    end

    // ========================================
    // Stage S2b: Taylor-1 correction (EXP_MODE=1 only)
    // ========================================

    generate
        if (EXP_MODE == 1) begin : gen_taylor
            // Taylor-1 correction stage (1 extra cycle)           
            wire [7:0]  eps8 = epsilon_s2[15:8];
            (* use_dsp = "no" *)
            wire [23:0] prod_re = lut_val_s2 * eps8;      // ROM × eps8, max 24-bit
            (* use_dsp = "no" *)
            wire [31:0] prod_rek = prod_re * 8'd177;       // × 177 (ln2×256), 32-bit
            wire [15:0] correction = prod_rek[31:24];       // >> 24

            reg [15:0] corrected_val;
            reg [4:0]  n_s2b;

            always @(posedge clk) begin
                if (rst) begin
                    corrected_val <= 16'd0;
                    n_s2b         <= 5'd0;
                end else if (ce) begin
                    // Subtract correction (Taylor: multiply by 1-δ)
                    if (correction > lut_val_s2)
                        corrected_val <= 16'd0;
                    else
                        corrected_val <= lut_val_s2 - correction;
                    n_s2b <= n_s2;
                end
            end

            // Stage S3: Output — barrel shift + clamp
            always @(posedge clk) begin
                if (rst)
                    y_q115 <= 16'd0;
                else if (ce) begin
                    if (n_s2b >= 5'd16)
                        y_q115 <= 16'd0;
                    else
                        y_q115 <= corrected_val >> n_s2b;
                end
            end

        end else begin : gen_rom_only
            // EXP_MODE=0: ROM-only
            // Stage S2b: Register BRAM output
            reg [15:0] lut_val_s2b;
            reg [4:0]  n_s2b;
            always @(posedge clk) begin
                if (rst) begin
                    lut_val_s2b <= 16'd0;
                    n_s2b       <= 5'd0;
                end else if (ce) begin
                    lut_val_s2b <= lut_val_s2;
                    n_s2b       <= n_s2;
                end
            end

            // Stage S3: Output — barrel shift + clamp
            always @(posedge clk) begin
                if (rst)
                    y_q115 <= 16'd0;
                else if (ce) begin
                    if (n_s2b >= 5'd16)
                        y_q115 <= 16'd0;
                    else
                        y_q115 <= lut_val_s2b >> n_s2b;
                end
            end

        end
    endgenerate

endmodule
