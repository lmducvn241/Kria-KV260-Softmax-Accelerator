`timescale 1ns/1ps
// ============================================================
// softmax_ip_v6.v — Softmax IP v6.0: Block-Scan Lookahead
// ============================================================

module softmax_ip_v6 #(
    parameter integer ELEM_WIDTH  = 16,
    parameter integer IN_FRAC     = 12,
    parameter integer Z_FRAC      = 15,
    parameter integer SUM_WIDTH   = 48,
    parameter integer EXP_MODE    = 0,     // 0=ROM-only, 1=Taylor-1
    parameter integer SEG_DEPTH   = 64,    // PWL segments: 64, 128, 256
    parameter integer BLOCK_SIZE  = 16,    // Lookahead block size (beats)
    parameter integer FIFO_DEPTH  = 32,    // Scan FIFO depth (2×BLOCK_SIZE)
    parameter integer DEBUG       = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // Control interface (from AXI-Lite slave)
    input  wire        start_p1,
    input  wire        start_p2,       // legacy mode
    input  wire        clear_error,
    input  wire [31:0] k_config,
    input  wire        auto_p2_en,     // auto-transition P1→P2

    output reg         busy,
    output reg         p1_done,
    output reg         p2_done,
    output reg         error,

    // AXIS input  — 128-bit packed: 8×16-bit
    input  wire [8*ELEM_WIDTH-1:0]   s_axis_tdata,
    input  wire [8*ELEM_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // AXIS output — 128-bit packed: 8×16-bit
    output reg [8*ELEM_WIDTH-1:0]    m_axis_tdata,
    output reg [8*ELEM_WIDTH/8-1:0]  m_axis_tkeep,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast,

    // Results
    output wire [31:0]               argmax_idx_o,
    output wire [ELEM_WIDTH-1:0]     argmax_val_o,

    // Performance counters (always enabled)
    output reg [63:0]                perf_total,
    output reg [63:0]                perf_p1,
    output reg [63:0]                perf_p2,
    output reg [63:0]                perf_stall,
    output reg [63:0]                perf_compute,
    output reg [63:0]                perf_dma_wait,
    input  wire                      clear_perf,

    // Scan FIFO status (debug)
    output wire [5:0]                fifo_count
);

    // =========================
    // SYNC RESET
    // =========================
    reg rst_n_s1, rst_n_s2;
    always @(posedge clk) begin
        rst_n_s1 <= rst_n;
        rst_n_s2 <= rst_n_s1;
    end
    wire rst = ~rst_n_s2;

    // =========================
    // FSM STATES
    // =========================
    localparam [3:0]
        ST_IDLE        = 4'd0,
        ST_SCAN_FILL   = 4'd1,   
        ST_PASS1       = 4'd2,   // process from FIFO + refill
        ST_P1_BLK_RSC  = 4'd3,   
        ST_P1_DRAIN    = 4'd4,
        ST_RECIP       = 4'd5,
        ST_WAIT_P2     = 4'd6,
        ST_PASS2       = 4'd7,
        ST_DONE        = 4'd8;

    reg [3:0] state;
    reg s_axis_tready_r;

    // =========================
    // UNPACK INPUT (tkeep masking)
    // =========================
    wire [ELEM_WIDTH-1:0] in_elem [0:7];
    wire                  in_valid [0:7];
    wire signed [ELEM_WIDTH-1:0] in_masked [0:7];

    genvar g;
    generate
        for (g = 0; g < 8; g = g + 1) begin : unpack_in
            assign in_elem[g]  = s_axis_tdata[g*ELEM_WIDTH +: ELEM_WIDTH];
            assign in_valid[g] = s_axis_tkeep[g*2] & s_axis_tkeep[g*2+1];
            assign in_masked[g] = in_valid[g] ? in_elem[g] : 16'h8000;
        end
    endgenerate

    // =========================
    // COUNTERS
    // =========================
    reg [31:0] beat_cnt;
    reg [31:0] n_elems;
    reg [31:0] n_beats;

    // =========================
    // MAX TRACKING
    // =========================
    reg signed [ELEM_WIDTH-1:0] max_val_reg;    // block max for exp2 computation
    reg [31:0]                  max_idx_reg;    // argmax index output
    reg                         max_inited;
    reg signed [ELEM_WIDTH-1:0] argmax_max_val;
    assign argmax_idx_o = max_idx_reg;
    assign argmax_val_o = max_val_reg;
    integer j;

    // =========================
    // SCAN FIFO INSTANCE
    // =========================
    // Writes from AXI-Stream, reads to P1 pipeline.
    // Outputs block_max every BLOCK_SIZE beats.

    reg  fifo_wr_en;
    wire fifo_rd_en;
    wire [127:0]       fifo_rd_data;
    wire [15:0]        fifo_rd_tkeep;
    wire               fifo_rd_tlast;
    wire               fifo_rd_valid;
    wire signed [15:0] fifo_block_max;
    wire               fifo_block_max_valid;
    wire               fifo_block_max_first;
    wire               fifo_empty, fifo_full;
    reg                fifo_flush;

    scan_fifo #(
        .DATA_W(128),
        .ELEM_W(ELEM_WIDTH),
        .LANES(8),
        .BLOCK_SIZE(BLOCK_SIZE),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) u_scan_fifo (
        .clk(clk), .rst(rst),
        .wr_en(fifo_wr_en),
        .wr_data(s_axis_tdata),
        .wr_tkeep(s_axis_tkeep),
        .wr_tlast(s_axis_tlast),
        .rd_en(fifo_rd_en),
        .rd_data(fifo_rd_data),
        .rd_tkeep(fifo_rd_tkeep),
        .rd_tlast(fifo_rd_tlast),
        .rd_valid(fifo_rd_valid),
        .block_max(fifo_block_max),
        .block_max_valid(fifo_block_max_valid),
        .block_max_first(fifo_block_max_first),
        .count(fifo_count),
        .empty(fifo_empty),
        .full(fifo_full),
        .flush(fifo_flush)
    );

    // FIFO write: accept from AXI-Stream when actual handshake occurs
    always @* begin
        fifo_wr_en = s_axis_tvalid && s_axis_tready
                   && (state == ST_SCAN_FILL || state == ST_PASS1);
    end

    // FIFO read: pull data for P1 pipeline processing
    reg fifo_rd_en_r;
    assign fifo_rd_en = fifo_rd_en_r;

    // Unpack FIFO read data for the pipeline
    wire signed [ELEM_WIDTH-1:0] fifo_elem [0:7];
    generate
        for (g = 0; g < 8; g = g + 1) begin : unpack_fifo
            assign fifo_elem[g] = fifo_rd_data[g*ELEM_WIDTH +: ELEM_WIDTH];
        end
    endgenerate

    // =========================
    // BLOCK MAX LATCH (2-slot buffer for pipelined scanner)
    // =========================
    // Slot 1 (primary): pending_block_*
    // Slot 2 (overflow): overflow_block_*
    reg signed [ELEM_WIDTH-1:0] pending_block_max;
    reg                         pending_block_valid;
    reg                         pending_block_first;
    reg                         block_needs_rescale;

    // Overflow slot for back-to-back block_max_valid pulses
    reg signed [ELEM_WIDTH-1:0] overflow_block_max;
    reg                         overflow_block_valid;
    reg                         overflow_block_first;

    // FSM signals when pending block info has been processed
    reg pending_consumed;

    always @(posedge clk) begin
        if (rst || fifo_flush) begin
            pending_block_valid  <= 1'b0;
            pending_block_first  <= 1'b0;
            pending_block_max    <= 16'sh8000;
            block_needs_rescale  <= 1'b0;
            overflow_block_valid <= 1'b0;
            overflow_block_first <= 1'b0;
            overflow_block_max   <= 16'sh8000;
        end else begin
            // ---- Consume logic: promote overflow when primary is consumed ----
            if (pending_consumed) begin
                if (overflow_block_valid) begin
                    // Promote overflow to primary
                    pending_block_max   <= overflow_block_max;
                    pending_block_valid <= 1'b1;
                    pending_block_first <= overflow_block_first;
                    block_needs_rescale <= !overflow_block_first
                                        && ($signed(overflow_block_max) > $signed(max_val_reg));
                    overflow_block_valid <= 1'b0;
                end else begin
                    // No overflow — just clear primary
                    pending_block_valid <= 1'b0;
                    block_needs_rescale <= 1'b0;
                end
            end

            // ---- Capture new block_max_valid ----
            if (fifo_block_max_valid) begin
                if (!pending_block_valid || pending_consumed) begin
                    // Primary slot free — capture directly
                    pending_block_max   <= fifo_block_max;
                    pending_block_valid <= 1'b1;
                    pending_block_first <= fifo_block_max_first;
                    block_needs_rescale <= !fifo_block_max_first
                                        && ($signed(fifo_block_max) > $signed(max_val_reg));
                end else begin
                    // Primary occupied — store in overflow
                    overflow_block_max   <= fifo_block_max;
                    overflow_block_valid <= 1'b1;
                    overflow_block_first <= fifo_block_max_first;
                end
            end
        end
    end

    // =========================
    // MAX TREE (8→1) — Process FIFO data
    // =========================
    reg [31:0] p1_process_cnt;  // beat counter for FIFO reads
    wire [31:0] fifo_elem_idx_base = {p1_process_cnt[28:0], 3'd0};

    // Stage A: 8→4
    wire signed [ELEM_WIDTH-1:0] m_s1_v_comb [0:3];
    wire [31:0]                  m_s1_i_comb [0:3];

    assign m_s1_v_comb[0] = ($signed(fifo_elem[0]) >= $signed(fifo_elem[1])) ? fifo_elem[0] : fifo_elem[1];
    assign m_s1_i_comb[0] = ($signed(fifo_elem[0]) >= $signed(fifo_elem[1])) ? fifo_elem_idx_base : (fifo_elem_idx_base + 32'd1);
    assign m_s1_v_comb[1] = ($signed(fifo_elem[2]) >= $signed(fifo_elem[3])) ? fifo_elem[2] : fifo_elem[3];
    assign m_s1_i_comb[1] = ($signed(fifo_elem[2]) >= $signed(fifo_elem[3])) ? (fifo_elem_idx_base+32'd2) : (fifo_elem_idx_base+32'd3);
    assign m_s1_v_comb[2] = ($signed(fifo_elem[4]) >= $signed(fifo_elem[5])) ? fifo_elem[4] : fifo_elem[5];
    assign m_s1_i_comb[2] = ($signed(fifo_elem[4]) >= $signed(fifo_elem[5])) ? (fifo_elem_idx_base+32'd4) : (fifo_elem_idx_base+32'd5);
    assign m_s1_v_comb[3] = ($signed(fifo_elem[6]) >= $signed(fifo_elem[7])) ? fifo_elem[6] : fifo_elem[7];
    assign m_s1_i_comb[3] = ($signed(fifo_elem[6]) >= $signed(fifo_elem[7])) ? (fifo_elem_idx_base+32'd6) : (fifo_elem_idx_base+32'd7);

    reg signed [ELEM_WIDTH-1:0] m_s1_v_r [0:3];
    reg [31:0]                  m_s1_i_r [0:3];
    reg signed [ELEM_WIDTH-1:0] data_pipe1 [0:7];
    reg [15:0]                  tkeep_pipe1;
    reg                         tlast_pipe1;
    reg                         valid_pipe1;

    always @(posedge clk) begin
        if (rst) begin
            valid_pipe1 <= 1'b0;
            for (j = 0; j < 4; j = j + 1) begin
                m_s1_v_r[j] <= 16'h8000;
                m_s1_i_r[j] <= 32'd0;
            end
        end else begin
            valid_pipe1 <= fifo_rd_en && fifo_rd_valid && (state == ST_PASS1);
            for (j = 0; j < 4; j = j + 1) begin
                m_s1_v_r[j[1:0]] <= m_s1_v_comb[j[1:0]];
                m_s1_i_r[j[1:0]] <= m_s1_i_comb[j[1:0]];
            end
            for (j = 0; j < 8; j = j + 1)
                data_pipe1[j[2:0]] <= fifo_elem[j[2:0]];
            tkeep_pipe1 <= fifo_rd_tkeep;
            tlast_pipe1 <= fifo_rd_tlast;
        end
    end

    // Stage B: 4→1
    wire signed [ELEM_WIDTH-1:0] m_s2_v [0:1];
    wire [31:0]                  m_s2_i [0:1];
    wire signed [ELEM_WIDTH-1:0] beat_max_comb;
    wire [31:0]                  beat_max_idx_comb;

    assign m_s2_v[0] = ($signed(m_s1_v_r[0]) >= $signed(m_s1_v_r[1])) ? m_s1_v_r[0] : m_s1_v_r[1];
    assign m_s2_i[0] = ($signed(m_s1_v_r[0]) >= $signed(m_s1_v_r[1])) ? m_s1_i_r[0] : m_s1_i_r[1];
    assign m_s2_v[1] = ($signed(m_s1_v_r[2]) >= $signed(m_s1_v_r[3])) ? m_s1_v_r[2] : m_s1_v_r[3];
    assign m_s2_i[1] = ($signed(m_s1_v_r[2]) >= $signed(m_s1_v_r[3])) ? m_s1_i_r[2] : m_s1_i_r[3];

    assign beat_max_comb     = ($signed(m_s2_v[0]) >= $signed(m_s2_v[1])) ? m_s2_v[0] : m_s2_v[1];
    assign beat_max_idx_comb = ($signed(m_s2_v[0]) >= $signed(m_s2_v[1])) ? m_s2_i[0] : m_s2_i[1];

    reg signed [ELEM_WIDTH-1:0] beat_max_r;
    reg [31:0]                  beat_max_idx_r;
    reg signed [ELEM_WIDTH-1:0] data_pipe2 [0:7];
    reg [15:0]                  tkeep_pipe2;
    reg                         tlast_pipe2;
    reg                         beat_valid_r;

    always @(posedge clk) begin
        if (rst) begin
            beat_valid_r   <= 1'b0;
            beat_max_r     <= 16'h8000;
            beat_max_idx_r <= 32'd0;
        end else begin
            beat_valid_r   <= valid_pipe1;
            beat_max_r     <= beat_max_comb;
            beat_max_idx_r <= beat_max_idx_comb;
            for (j = 0; j < 8; j = j + 1)
                data_pipe2[j[2:0]] <= data_pipe1[j[2:0]];
            tkeep_pipe2 <= tkeep_pipe1;
            tlast_pipe2 <= tlast_pipe1;
        end
    end

    // Argmax tracking — registered comparison stage.
    reg        argmax_update_r;      // 1-cycle delayed when beat_max won
    reg [31:0] argmax_capture_idx_r; // captured index for delayed update
    reg signed [ELEM_WIDTH-1:0] argmax_capture_val_r; // captured value

    always @(posedge clk) begin
        if (rst) begin
            argmax_update_r      <= 1'b0;
            argmax_capture_idx_r <= 32'd0;
            argmax_capture_val_r <= 16'sh8000;
            argmax_max_val       <= 16'sh8000;
        end else begin
            // Reset running max on new operation
            if (state == ST_IDLE && start_p1)
                argmax_max_val <= 16'sh8000;
            // Update running max in this block
            else if (beat_valid_r && (state == ST_PASS1) &&
                     ($signed(beat_max_r) > $signed(argmax_max_val)))
                argmax_max_val <= beat_max_r;
            // Registered comparison flag
            argmax_update_r <= beat_valid_r && (state == ST_PASS1) &&
                               ($signed(beat_max_r) > $signed(argmax_max_val));
            argmax_capture_idx_r <= beat_max_idx_r;
            argmax_capture_val_r <= beat_max_r;
        end
    end

    // =========================
    // 8× EXP2 INSTANCES
    // =========================
    reg  [15:0] exp2_in [0:7];
    wire [15:0] exp2_out [0:7];
    wire mults_ready;

    // Scanner holdoff: prevent scanner from getting >1 block ahead of reader
    // When pending_block_valid=1 during PASS1, pause AXI input so scanner will pause.
    wire scan_holdoff = (state == ST_PASS1) && pending_block_valid;

    // Combinational tready
    wire p2_active = (state == ST_PASS2);
    assign s_axis_tready = s_axis_tready_r
                         & (!p2_active | mults_ready)
                         & !fifo_full
                         & !scan_holdoff;

    wire exp2_ce = (!p2_active) || mults_ready;

    generate
        for (g = 0; g < 8; g = g + 1) begin : gen_exp2
            exp2_base2_v2 #(.EXP_MODE(EXP_MODE)) u_exp2 (
                .clk(clk),
                .rst(rst),
                .ce(exp2_ce),
                .d_q412(exp2_in[g]),
                .y_q115(exp2_out[g])
            );
        end
    endgenerate

    reg        rescaling;
    reg [4:0]  rescale_cnt;

    // =========================
    // EXP2 FEED VALID
    // =========================
    reg exp2_feed_valid_r;
    always @* begin
        exp2_feed_valid_r = (state == ST_PASS1 && beat_valid_r)
                         || (state == ST_PASS2 && s_axis_tvalid && s_axis_tready);
    end

    // Exp2 valid pipeline (5 exp2 stages + 1 CSA = 6 total)
    localparam EXP2_LATENCY = 6;
    localparam EXP2_PIPE_W  = EXP2_LATENCY + 1;

    reg [EXP2_PIPE_W-1:0] exp2_valid_pipe;
    always @(posedge clk) begin
        if (rst) exp2_valid_pipe <= {EXP2_PIPE_W{1'b0}};
        else     exp2_valid_pipe <= {exp2_valid_pipe[EXP2_PIPE_W-2:0], exp2_feed_valid_r};
    end
    wire exp2_out_valid = exp2_valid_pipe[EXP2_PIPE_W-1];

    // =========================
    // CSA+CLA 8-INPUT ADDER
    // =========================
    wire [33:0] exp8_sum_out;
    wire        exp8_overflow;
    csa_cla_add8_32bit u_exp8_add (
        .clk(clk), .rst(rst),
        .a({16'd0, exp2_out[0]}), .b({16'd0, exp2_out[1]}),
        .c({16'd0, exp2_out[2]}), .d({16'd0, exp2_out[3]}),
        .e({16'd0, exp2_out[4]}), .f({16'd0, exp2_out[5]}),
        .g({16'd0, exp2_out[6]}), .h({16'd0, exp2_out[7]}),
        .sum_out(exp8_sum_out), .overflow(exp8_overflow)
    );

    reg csa_out_valid;
    always @(posedge clk) begin
        if (rst) csa_out_valid <= 1'b0;
        else     csa_out_valid <= exp2_out_valid;
    end

    // =========================
    // 48-BIT SUM_EXP ACCUMULATOR
    // =========================
    reg [SUM_WIDTH-1:0] sum_exp_reg;

    // =========================
    // RESCALE SIGNALS
    // =========================
    reg [15:0] rescale_delta;
    wire [15:0] rescale_exp_out;
    reg [15:0] rescale_factor;
    reg signed [ELEM_WIDTH-1:0] new_max_val;
    reg [31:0]                  new_max_idx;

    (* use_dsp = "no" *) reg [23:0] rsc_8x16_lo_a, rsc_8x16_lo_b;
    (* use_dsp = "no" *) reg [23:0] rsc_8x16_mid_a, rsc_8x16_mid_b;
    (* use_dsp = "no" *) reg [23:0] rsc_8x16_hi_a, rsc_8x16_hi_b;
    (* use_dsp = "no" *) reg [31:0] rsc_pp_lo;
    (* use_dsp = "no" *) reg [31:0] rsc_pp_mid;
    (* use_dsp = "no" *) reg [31:0] rsc_pp_hi;
    reg [47:0] rsc_partial;

    // =========================
    // LZD + RECIPROCAL
    // =========================
    reg [2:0]  recip_phase;
    reg [5:0]  lzd_pos;
    reg [2:0]  lzd_coarse;
    reg [7:0]  lzd_fine_byte;
    reg [15:0] recip_mantissa;
    reg [5:0]  norm_shift_reg;
    reg        start_recip;
    wire       ready_recip;
    wire [15:0] recip_raw;

    pwl_reciprocal_v2 #(.SEG_DEPTH(SEG_DEPTH)) u_recip (
        .clk(clk), .rst_n(rst_n),
        .start(start_recip),
        .m_in(recip_mantissa),
        .recip_out(recip_raw),
        .ready(ready_recip)
    );

    reg [15:0] recip_val;

    // =========================
    // PASS 2 PIPELINE (Stream mode only)
    // =========================
    reg p2_feed_valid;

    // Booth multipliers: 8 instances
    wire [15:0] mul_out_prob [0:7];
    wire        mul_out_valid [0:7];
    wire        mul_out_last [0:7];
    wire        mul_in_ready [0:7];

    wire mults_valid = mul_out_valid[0] & mul_out_valid[1] & mul_out_valid[2] & mul_out_valid[3]
                     & mul_out_valid[4] & mul_out_valid[5] & mul_out_valid[6] & mul_out_valid[7];
    assign mults_ready = mul_in_ready[0] & mul_in_ready[1] & mul_in_ready[2] & mul_in_ready[3]
                       & mul_in_ready[4] & mul_in_ready[5] & mul_in_ready[6] & mul_in_ready[7];

    wire out_accept = (!m_axis_tvalid) || m_axis_tready;

    wire [15:0] p2_exp_src [0:7];
    generate
        for (g = 0; g < 8; g = g + 1) begin : mux_p2_exp
            assign p2_exp_src[g] = exp2_out[g];
        end
    endgenerate

    // P2 valid/last pipeline
    reg [EXP2_LATENCY:0] p2_exp_valid_pipe;
    reg [EXP2_LATENCY:0] p2_last_pipe;

    wire p2_feed_new = (state == ST_PASS2 && s_axis_tvalid && s_axis_tready);
    wire p2_feed_last_w = (state == ST_PASS2 && s_axis_tvalid && s_axis_tready && s_axis_tlast);

    always @(posedge clk) begin
        if (rst) begin
            p2_exp_valid_pipe <= {(EXP2_LATENCY+1){1'b0}};
            p2_last_pipe      <= {(EXP2_LATENCY+1){1'b0}};
        end else if (mults_ready) begin
            p2_exp_valid_pipe <= {p2_exp_valid_pipe[EXP2_LATENCY-1:0], p2_feed_new};
            p2_last_pipe      <= {p2_last_pipe[EXP2_LATENCY-1:0], p2_feed_last_w};
        end
    end
    wire p2_exp_ready = p2_exp_valid_pipe[EXP2_LATENCY];
    wire p2_last_aligned = p2_last_pipe[EXP2_LATENCY];

    wire booth_feed_valid = p2_exp_ready;
    wire booth_feed_last  = p2_last_aligned;

    // P2 beat counter
    reg [31:0] p2_beat_cnt;
    reg        p2_feed_last;

    // Booth instantiation
    generate
        for (g = 0; g < 8; g = g + 1) begin : gen_muls
            mul16x16_booth4_nodsp_axis #(.DATA_WIDTH(ELEM_WIDTH)) u_mul (
                .clk(clk), .rst(rst),
                .in_exp_q15(p2_exp_src[g]),
                .in_recip_q15(recip_val),
                .in_shift(norm_shift_reg[4:0]),
                .in_last(booth_feed_last),
                .in_valid(booth_feed_valid),
                .in_ready(mul_in_ready[g]),
                .out_prob_q15(mul_out_prob[g]),
                .out_last(mul_out_last[g]),
                .out_valid(mul_out_valid[g]),
                .out_ready(out_accept)
            );
        end
    endgenerate

    // =========================
    // TKEEP/TLAST DELAY CHAINS
    // =========================
    localparam P2_PIPE_DEPTH = 16;
    reg [15:0] tkeep_dly [0:P2_PIPE_DEPTH-1];
    reg        tlast_dly [0:P2_PIPE_DEPTH-1];

    wire pipe_advance_p2 = (state == ST_PASS2 && s_axis_tvalid && s_axis_tready);

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < P2_PIPE_DEPTH; i = i + 1) begin
                tkeep_dly[i] <= 16'd0;
                tlast_dly[i] <= 1'b0;
            end
        end else if (pipe_advance_p2) begin
            tkeep_dly[0] <= s_axis_tkeep;
            tlast_dly[0] <= s_axis_tlast;
            for (i = 1; i < P2_PIPE_DEPTH; i = i + 1) begin
                tkeep_dly[i] <= tkeep_dly[i-1];
                tlast_dly[i] <= tlast_dly[i-1];
            end
        end
    end

    // =========================
    // DRAIN + OUTPUT
    // =========================
    reg [3:0] drain_cnt;
    reg [31:0] out_beat_cnt;

    // Q1.15 clamp
    wire [15:0] prob_clamped [0:7];
    generate
        for (g = 0; g < 8; g = g + 1) begin : gen_clamp
            assign prob_clamped[g] = (mul_out_prob[g][15]) ? 16'h7FFF : mul_out_prob[g];
        end
    endgenerate

    // =========================
    // Block processing tracking
    // =========================
    reg [4:0]  p1_block_beat_cnt;   // beats READ from FIFO in current block
    reg        p1_block_pause;      // pause FIFO reads at block boundary
    reg        p1_last_seen;        // Last beat seen from FIFO
    reg        p1_scan_done;        // All data pushed into FIFO

    // =========================
    // MAIN FSM
    // =========================

    always @(posedge clk) begin
        if (rst) begin
            state         <= ST_IDLE;
            busy          <= 1'b0;
            p1_done       <= 1'b0;
            p2_done       <= 1'b0;
            error         <= 1'b0;
            s_axis_tready_r <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= {8*ELEM_WIDTH{1'b0}};
            m_axis_tkeep  <= 16'd0;
            m_axis_tlast  <= 1'b0;
            beat_cnt      <= 32'd0;
            n_elems       <= 32'd0;
            n_beats       <= 32'd0;
            max_val_reg    <= 16'h8000;
            max_idx_reg    <= 32'd0;
            max_inited     <= 1'b0;
            sum_exp_reg   <= {SUM_WIDTH{1'b0}};
            rescaling     <= 1'b0;
            rescale_cnt   <= 5'd0;
            drain_cnt     <= 4'd0;
            start_recip   <= 1'b0;
            recip_val     <= 16'd0;
            norm_shift_reg<= 6'd0;
            lzd_pos       <= 6'd0;
            lzd_coarse    <= 3'd0;
            recip_phase   <= 3'd0;
            p2_beat_cnt   <= 32'd0;
            p2_feed_last  <= 1'b0;
            out_beat_cnt  <= 32'd0;
            fifo_flush    <= 1'b0;
            fifo_rd_en_r  <= 1'b0;
            p1_process_cnt<= 32'd0;
            p1_block_beat_cnt <= 5'd0;
            p1_block_pause <= 1'b0;
            p1_last_seen  <= 1'b0;
            p1_scan_done  <= 1'b0;
            pending_consumed <= 1'b0;
            perf_total    <= 64'd0;
            perf_p1       <= 64'd0;
            perf_p2       <= 64'd0;
            perf_stall    <= 64'd0;
            perf_compute  <= 64'd0;
            perf_dma_wait <= 64'd0;
            rsc_pp_lo     <= 32'd0;
            rsc_pp_mid    <= 32'd0;
            rsc_pp_hi     <= 32'd0;
            for (j = 0; j < 8; j = j + 1) begin
                exp2_in[j]   <= 16'd0;
            end
        end else begin

            if (clear_error) error <= 1'b0;

            if (clear_perf) begin
                perf_total   <= 64'd0;
                perf_p1      <= 64'd0;
                perf_p2      <= 64'd0;
                perf_stall   <= 64'd0;
                perf_compute <= 64'd0;
                perf_dma_wait<= 64'd0;
            end

            // Performance counters
            if (busy) perf_total <= perf_total + 64'd1;
            if (state == ST_SCAN_FILL || state == ST_PASS1
             || state == ST_P1_BLK_RSC || state == ST_P1_DRAIN)
                perf_p1 <= perf_p1 + 64'd1;
            if (state == ST_PASS2)
                perf_p2 <= perf_p2 + 64'd1;
            if (state == ST_P1_BLK_RSC)
                perf_stall <= perf_stall + 64'd1;
            if (state == ST_WAIT_P2)
                perf_dma_wait <= perf_dma_wait + 64'd1;
            if (((state == ST_PASS1) && fifo_rd_en && fifo_rd_valid) ||
                (state == ST_P1_BLK_RSC) ||
                (state == ST_P1_DRAIN) ||
                (state == ST_RECIP) ||
                ((state == ST_PASS2) && ((s_axis_tvalid && s_axis_tready) || mults_valid)) ||
                (state == ST_DONE))
                perf_compute <= perf_compute + 64'd1;

            case (state)

            // =====================================================
            // ST_IDLE
            // =====================================================
            ST_IDLE: begin
                fifo_flush <= 1'b0;
                if (start_p1) begin
                    state         <= ST_SCAN_FILL;
                    busy          <= 1'b1;
                    p1_done       <= 1'b0;
                    p2_done       <= 1'b0;
                    beat_cnt      <= 32'd0;
                    n_beats       <= (k_config + 32'd7) >> 3;
                    n_elems       <= k_config;
                    max_val_reg   <= 16'h8000;
                    max_idx_reg   <= 32'd0;
                    max_inited    <= 1'b0;
                    sum_exp_reg   <= {SUM_WIDTH{1'b0}};
                    s_axis_tready_r <= 1'b1;
                    fifo_flush    <= 1'b1;  // clear FIFO
                    fifo_rd_en_r  <= 1'b0;
                    p1_process_cnt<= 32'd0;
                    p1_block_beat_cnt <= 5'd0;
                    p1_block_pause <= 1'b0;
                    p1_last_seen  <= 1'b0;
                    p1_scan_done  <= 1'b0;
                    pending_consumed <= 1'b0;
                    rescaling     <= 1'b0;
                end
            end

            // =====================================================
            // ST_SCAN_FILL: Pre-fill FIFO with first B beats
            // =====================================================
            ST_SCAN_FILL: begin
                fifo_flush <= 1'b0;  // release flush after 1 cycle
                pending_consumed <= 1'b0;
                // Wait for first block max from scanner
                if (fifo_block_max_valid && fifo_block_max_first) begin
                    max_val_reg <= fifo_block_max;
                    max_inited  <= 1'b1;
                    state       <= ST_PASS1;
                    fifo_rd_en_r <= 1'b1;
                    p1_block_beat_cnt <= 5'd0;  
                    p1_block_pause <= 1'b0;
                    pending_consumed <= 1'b1;
                end
            end

            // =====================================================
            // ST_PASS1: Process from FIFO + refill from AXI-Stream
            // =====================================================
            ST_PASS1: begin
                pending_consumed <= 1'b0;
                // ---- Process FIFO read data through pipeline ----
                if (beat_valid_r) begin
                    // Compute exp2(max - x) — max
                    for (j = 0; j < 8; j = j + 1)
                        exp2_in[j] <= max_val_reg - data_pipe2[j];
                    beat_cnt <= beat_cnt + 32'd1;
                    if (tlast_pipe2) begin
                        // Last beat from FIFO
                        p1_last_seen <= 1'b1;
                        fifo_rd_en_r <= 1'b0;
                        s_axis_tready_r <= 1'b0;
                        state <= ST_P1_DRAIN;
                        drain_cnt <= 4'd0;
                    end
                end

                // ---- Argmax update (1-cycle delayed from comparison) ----
                if (argmax_update_r) begin
                    max_idx_reg    <= argmax_capture_idx_r;
                end

                // ---- FIFO read control WITH block boundary pause ----
                if (!p1_block_pause) begin
                    if (fifo_rd_valid && !fifo_empty) begin
                        fifo_rd_en_r <= 1'b1;
                        p1_process_cnt <= p1_process_cnt + 32'd1;
                        p1_block_beat_cnt <= p1_block_beat_cnt + 5'd1;
                        // Pause after reading the last beat of a block
                        if (p1_block_beat_cnt + 5'd1 >= BLOCK_SIZE) begin
                            p1_block_pause <= 1'b1;
                            fifo_rd_en_r   <= 1'b0;
                        end
                    end else begin
                        fifo_rd_en_r <= 1'b0;
                    end
                end else begin
                    // Paused at block boundary
                    fifo_rd_en_r <= 1'b0;
                end

                // ---- Block boundary rescale check ----
                if (p1_block_pause && !beat_valid_r && !valid_pipe1 && !csa_out_valid
                    && (exp2_valid_pipe == {EXP2_PIPE_W{1'b0}})) begin
                    if (pending_block_valid && block_needs_rescale) begin
                        // Need rescale
                        s_axis_tready_r <= 1'b0;
                        rescale_delta   <= pending_block_max - max_val_reg;
                        new_max_val     <= pending_block_max;
                        rescaling       <= 1'b1;
                        rescale_cnt     <= 5'd0;
                        state           <= ST_P1_BLK_RSC;
                        p1_block_beat_cnt <= 5'd0;
                        p1_block_pause <= 1'b0;
                        pending_consumed <= 1'b1;
                    end else if (pending_block_valid && !block_needs_rescale) begin
                        // No rescale needed
                        p1_block_pause <= 1'b0;
                        p1_block_beat_cnt <= 5'd0;
                        fifo_rd_en_r <= 1'b1;
                        pending_consumed <= 1'b1;
                    end
                end

                // ---- Sum accumulation ----
                if (csa_out_valid) begin
                    sum_exp_reg <= sum_exp_reg + {{(SUM_WIDTH-34){1'b0}}, exp8_sum_out};
                end

                // ---- Track when all data has been pushed to FIFO ----
                if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                    p1_scan_done <= 1'b1;
                    s_axis_tready_r <= 1'b0;
                end
            end

            // =====================================================
            // ST_P1_BLK_RSC: Block-boundary rescale
            // =====================================================
            ST_P1_BLK_RSC: begin
                pending_consumed <= 1'b0;
                rescale_cnt <= rescale_cnt + 5'd1;
                case (rescale_cnt)
                5'd0: begin
                    exp2_in[0] <= rescale_delta;
                    for (j = 1; j < 8; j = j + 1)
                        exp2_in[j] <= 16'hFFFF;
                end
                5'd7: begin
                    rescale_factor <= exp2_out[0];
                end
                5'd10: begin
                    rsc_8x16_lo_a  <= sum_exp_reg[7:0]   * rescale_factor;
                    rsc_8x16_lo_b  <= sum_exp_reg[15:8]  * rescale_factor;
                    rsc_8x16_mid_a <= sum_exp_reg[23:16] * rescale_factor;
                    rsc_8x16_mid_b <= sum_exp_reg[31:24] * rescale_factor;
                    rsc_8x16_hi_a  <= sum_exp_reg[39:32] * rescale_factor;
                    rsc_8x16_hi_b  <= sum_exp_reg[47:40] * rescale_factor;
                end
                5'd11: begin
                    rsc_pp_lo  <= {8'd0, rsc_8x16_lo_b,  8'd0} + {8'd0, rsc_8x16_lo_a};
                    rsc_pp_mid <= {8'd0, rsc_8x16_mid_b, 8'd0} + {8'd0, rsc_8x16_mid_a};
                    rsc_pp_hi  <= {8'd0, rsc_8x16_hi_b,  8'd0} + {8'd0, rsc_8x16_hi_a};
                end
                5'd12: begin
                    rsc_partial <= {16'd0, rsc_pp_lo} + {rsc_pp_mid, 16'd0};
                end
                5'd13: begin
                    rsc_partial <= {rsc_pp_hi, 32'd0} + {16'd0, rsc_partial};
                end
                5'd14: begin
                    sum_exp_reg <= rsc_partial >> Z_FRAC;
                end
                5'd15: begin
                    // Rescale done update max and resume
                    max_val_reg <= new_max_val;
                    rescaling   <= 1'b0;
                    fifo_rd_en_r <= 1'b1;
                    s_axis_tready_r <= !p1_scan_done;
                    state       <= ST_PASS1;
                    p1_block_beat_cnt <= 5'd0;
                    p1_block_pause <= 1'b0;
                end
                endcase
                if (csa_out_valid && rescale_cnt < 5'd10)
                    sum_exp_reg <= sum_exp_reg + {{(SUM_WIDTH-34){1'b0}}, exp8_sum_out};
                // Catch late argmax update from last block's pipeline
                if (argmax_update_r) begin
                    max_idx_reg    <= argmax_capture_idx_r;
                end
            end

            // =====================================================
            // ST_P1_DRAIN: Wait for pipeline flush
            // =====================================================
            ST_P1_DRAIN: begin
                drain_cnt <= drain_cnt + 4'd1;
                if (csa_out_valid)
                    sum_exp_reg <= sum_exp_reg + {{(SUM_WIDTH-34){1'b0}}, exp8_sum_out};
                if (argmax_update_r) begin
                    max_idx_reg    <= argmax_capture_idx_r;
                end
                if (drain_cnt == 4'd8) begin
                    state       <= ST_RECIP;
                    recip_phase <= 3'd0;
                end
            end

            // =====================================================
            // ST_RECIP: LZD + Reciprocal
            // =====================================================
            ST_RECIP: begin
                case (recip_phase)
                3'd0: begin
                    if (sum_exp_reg[47:40] != 8'd0)      lzd_coarse <= 3'd5;
                    else if (sum_exp_reg[39:32] != 8'd0)  lzd_coarse <= 3'd4;
                    else if (sum_exp_reg[31:24] != 8'd0)  lzd_coarse <= 3'd3;
                    else if (sum_exp_reg[23:16] != 8'd0)  lzd_coarse <= 3'd2;
                    else if (sum_exp_reg[15:8]  != 8'd0)  lzd_coarse <= 3'd1;
                    else                                   lzd_coarse <= 3'd0;
                    recip_phase <= 3'd1;
                end
                3'd1: begin
                    case (lzd_coarse)
                        3'd5: lzd_fine_byte = sum_exp_reg[47:40];
                        3'd4: lzd_fine_byte = sum_exp_reg[39:32];
                        3'd3: lzd_fine_byte = sum_exp_reg[31:24];
                        3'd2: lzd_fine_byte = sum_exp_reg[23:16];
                        3'd1: lzd_fine_byte = sum_exp_reg[15:8];
                        default: lzd_fine_byte = sum_exp_reg[7:0];
                    endcase

                    casez (lzd_fine_byte)
                        8'b1???????: lzd_pos <= {lzd_coarse, 3'd7};
                        8'b01??????: lzd_pos <= {lzd_coarse, 3'd6};
                        8'b001?????: lzd_pos <= {lzd_coarse, 3'd5};
                        8'b0001????: lzd_pos <= {lzd_coarse, 3'd4};
                        8'b00001???: lzd_pos <= {lzd_coarse, 3'd3};
                        8'b000001??: lzd_pos <= {lzd_coarse, 3'd2};
                        8'b0000001?: lzd_pos <= {lzd_coarse, 3'd1};
                        8'b00000001: lzd_pos <= {lzd_coarse, 3'd0};
                        default:     lzd_pos <= 6'd15;
                    endcase
                    recip_phase <= 3'd2;
                end
                3'd2: begin
                    if (lzd_pos >= 6'd15) begin
                        recip_mantissa <= sum_exp_reg >> (lzd_pos - 6'd15);
                        norm_shift_reg <= lzd_pos - 6'd15;
                    end else begin
                        recip_mantissa <= sum_exp_reg << (6'd15 - lzd_pos);
                        norm_shift_reg <= 6'd0;
                    end
                    start_recip    <= 1'b1;
                    recip_phase    <= 3'd3;
                end
                3'd3: begin
                    start_recip <= 1'b0;
                    if (ready_recip) begin
                        recip_val <= recip_raw;
                        p1_done   <= 1'b1;
                        if (auto_p2_en) begin
                            // AUTO: Go directly to Pass 2
                            state           <= ST_PASS2;
                            s_axis_tready_r <= 1'b1;
                            p2_beat_cnt     <= 32'd0;
                            out_beat_cnt    <= 32'd0;
                            m_axis_tvalid   <= 1'b0;
                        end else begin
                            state <= ST_WAIT_P2;
                        end
                    end
                end
                endcase
            end

            // =====================================================
            // ST_WAIT_P2: Legacy SW-gated transition
            // =====================================================
            ST_WAIT_P2: begin
                s_axis_tready_r <= 1'b0;
                if (start_p2) begin
                    state           <= ST_PASS2;
                    s_axis_tready_r <= 1'b1;
                    p1_done         <= 1'b0;
                    p2_beat_cnt     <= 32'd0;
                    out_beat_cnt    <= 32'd0;
                    m_axis_tvalid   <= 1'b0;
                end
            end

            // =====================================================
            // ST_PASS2: Stream mode — Data from DMA, exp2, Booth, output
            // =====================================================
            ST_PASS2: begin
                s_axis_tready_r <= mults_ready && out_accept;
                if (s_axis_tvalid && s_axis_tready) begin
                    for (j = 0; j < 8; j = j + 1)
                        exp2_in[j] <= max_val_reg - in_masked[j];
                    p2_beat_cnt <= p2_beat_cnt + 32'd1;
                    p2_feed_last <= (p2_beat_cnt + 32'd1 >= n_beats);
                    if (s_axis_tlast)
                        s_axis_tready_r <= 1'b0;
                end
                if (mults_valid && out_accept) begin
                    m_axis_tvalid <= 1'b1;
                    m_axis_tdata  <= {prob_clamped[7], prob_clamped[6], prob_clamped[5], prob_clamped[4],
                                     prob_clamped[3], prob_clamped[2], prob_clamped[1], prob_clamped[0]};
                    m_axis_tlast  <= mul_out_last[0];
                    m_axis_tkeep  <= tkeep_dly[P2_PIPE_DEPTH-1];
                    out_beat_cnt <= out_beat_cnt + 32'd1;
                    if (mul_out_last[0])
                        state <= ST_DONE;
                end else if (out_accept) begin
                    m_axis_tvalid <= 1'b0;
                end
            end

            // =====================================================
            // ST_DONE
            // =====================================================
            ST_DONE: begin
                s_axis_tready_r <= 1'b0;
                if (!m_axis_tvalid || m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    busy    <= 1'b0;
                    p2_done <= 1'b1;
                    state   <= ST_IDLE;
                end
            end

            endcase
        end
    end

endmodule
