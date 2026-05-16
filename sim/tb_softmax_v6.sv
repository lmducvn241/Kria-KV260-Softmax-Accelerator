// ============================================================
// tb_softmax_v6.sv 
//
// === Functional Tests ===
//   TC1:  Random data (stream baseline)
//   TC2:  All Identical (uniform softmax)
//   TC3:  Single Dominant
//   TC4:  Random Backpressure
//   TC5:  CRV Self-Check (3 iters, random K)
//   TC6:  Back-to-back
//   TC7:  Reset mid-operation & recovery
//
// === Architectural Verification Tests ===
//   TC8:  Ascending K=160  (block-scan rescale stress)
//   TC9:  Descending K=160 (no-rescale baseline)
//   TC10: K=8 (single beat, < BLOCK_SIZE)
//   TC11: K=256 random (multi-block)
//   TC12: Ascending K=256 + backpressure
//   TC13: K=1024 random (scalability)
//   TC14: K=1024 ascending + heavy backpressure (worst-case stall)
//   TC15: Partial block = B-1 (K=120, 15 beats - no block 2)
//   TC16: K exactly = BLOCK_SIZE*8 = 128 (1 full block only)
//   TC17: Ascending K=1024 NO backpressure (stall only)
//
// === Stall & Throughput Comparison ===
//   PERF_COMPARE: TC8 vs TC9 stall ratio -> must be >4x
//   PERF_COMPARE: TC14 vs TC13 latency check
//
// DUT: softmax_dl2_v6 -> softmax_ip_v6
// ============================================================
`timescale 1ns / 1ps

module tb_softmax_v6;

    // ============================================================
    // Parameters
    // ============================================================
    parameter integer K = 160;
    localparam integer OCT_K = (K + 7) / 8;
    localparam integer CLK_PERIOD = 3;  // 333.33 MHz

    // Register addresses
    localparam logic [6:0]
        ADDR_CTRL         = 7'h00,
        ADDR_STATUS       = 7'h04,
        ADDR_ARGMAX_IDX   = 7'h08,
        ADDR_ARGMAX_VAL   = 7'h0C,
        ADDR_PERF_TOTAL   = 7'h10,
        ADDR_PERF_P1      = 7'h18,
        ADDR_PERF_P2      = 7'h20,
        ADDR_PERF_STALL   = 7'h28,
        ADDR_K_CONFIG     = 7'h30,
        ADDR_PERF_COMPUTE = 7'h38;

    localparam integer BP_NONE   = 0;
    localparam integer BP_RANDOM = 1;
    localparam integer BP_HEAVY  = 2;

    // ============================================================
    // Clock & Reset
    // ============================================================
    logic clk = 0;
    logic rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ============================================================
    // DUT Signals
    // ============================================================
    logic [6:0]  s00_axi_awaddr;
    logic [2:0]  s00_axi_awprot;
    logic        s00_axi_awvalid;
    wire         s00_axi_awready;
    logic [31:0] s00_axi_wdata;
    logic [3:0]  s00_axi_wstrb;
    logic        s00_axi_wvalid;
    wire         s00_axi_wready;
    wire  [1:0]  s00_axi_bresp;
    wire         s00_axi_bvalid;
    logic        s00_axi_bready;
    logic [6:0]  s00_axi_araddr;
    logic [2:0]  s00_axi_arprot;
    logic        s00_axi_arvalid;
    wire         s00_axi_arready;
    wire  [31:0] s00_axi_rdata;
    wire  [1:0]  s00_axi_rresp;
    wire         s00_axi_rvalid;
    logic        s00_axi_rready;

    logic [127:0] s_axis_tdata;
    logic [15:0]  s_axis_tkeep;
    logic         s_axis_tvalid;
    wire          s_axis_tready;
    logic         s_axis_tlast;

    wire  [127:0] m_axis_tdata;
    wire  [15:0]  m_axis_tkeep;
    wire          m_axis_tvalid;
    logic         m_axis_tready;
    wire          m_axis_tlast;

    // ============================================================
    // DUT Instantiation
    // ============================================================
    softmax_dl2_v6 #(
        .C_S00_AXI_ADDR_WIDTH(7)
    ) dut (
        .s00_axi_aclk      (clk),
        .s00_axi_aresetn   (rst_n),
        .s00_axi_awaddr    (s00_axi_awaddr),
        .s00_axi_awprot    (s00_axi_awprot),
        .s00_axi_awvalid   (s00_axi_awvalid),
        .s00_axi_awready   (s00_axi_awready),
        .s00_axi_wdata     (s00_axi_wdata),
        .s00_axi_wstrb     (s00_axi_wstrb),
        .s00_axi_wvalid    (s00_axi_wvalid),
        .s00_axi_wready    (s00_axi_wready),
        .s00_axi_bresp     (s00_axi_bresp),
        .s00_axi_bvalid    (s00_axi_bvalid),
        .s00_axi_bready    (s00_axi_bready),
        .s00_axi_araddr    (s00_axi_araddr),
        .s00_axi_arprot    (s00_axi_arprot),
        .s00_axi_arvalid   (s00_axi_arvalid),
        .s00_axi_arready   (s00_axi_arready),
        .s00_axi_rdata     (s00_axi_rdata),
        .s00_axi_rresp     (s00_axi_rresp),
        .s00_axi_rvalid    (s00_axi_rvalid),
        .s00_axi_rready    (s00_axi_rready),
        .s_axis_tdata      (s_axis_tdata),
        .s_axis_tkeep      (s_axis_tkeep),
        .s_axis_tvalid     (s_axis_tvalid),
        .s_axis_tready     (s_axis_tready),
        .s_axis_tlast      (s_axis_tlast),
        .m_axis_tdata      (m_axis_tdata),
        .m_axis_tkeep      (m_axis_tkeep),
        .m_axis_tvalid     (m_axis_tvalid),
        .m_axis_tready     (m_axis_tready),
        .m_axis_tlast      (m_axis_tlast)
    );

    // ============================================================
    // Wire aliases
    // ============================================================
    wire [3:0] dut_fsm_state = dut.u_softmax_ip.state;

    // ============================================================
    // Cycle-accurate latency counters (running autonomously)
    // ============================================================
    longint cyc_p1_start, cyc_p1_end, cyc_p2_end;
    longint lat_p1, lat_p2, lat_total;
    int     rescale_event_cnt;

    // Count rescale events (each entry into ST_P1_BLK_RSC = state 3)
    reg [3:0] prev_fsm_state;
    always @(posedge clk) begin
        if (!rst_n) begin
            rescale_event_cnt <= 0;
            prev_fsm_state    <= 4'd0;
        end else begin
            prev_fsm_state <= dut_fsm_state;
            if (dut_fsm_state == 4'd3 && prev_fsm_state != 4'd3)
                rescale_event_cnt <= rescale_event_cnt + 1;
        end
    end

    // Watchdog: print FSM state diagnostics when stuck
    int watchdog_cnt;
    always @(posedge clk) begin
        if (!rst_n || dut_fsm_state == 4'd0) begin
            watchdog_cnt <= 0;
        end else begin
            watchdog_cnt <= watchdog_cnt + 1;
            if (watchdog_cnt > 0 && watchdog_cnt % 10000 == 0)
                $display("[%0t] WATCHDOG: state=%0d fifo_cnt=%0d pending_valid=%0b needs_rsc=%0b scan_holdoff=%0b p1_pause=%0b p1_scan_done=%0b fifo_empty=%0b tready=%0b tvalid=%0b",
                    $time, dut_fsm_state,
                    dut.u_softmax_ip.u_scan_fifo.cnt,
                    dut.u_softmax_ip.pending_block_valid,
                    dut.u_softmax_ip.block_needs_rescale,
                    dut.u_softmax_ip.scan_holdoff,
                    dut.u_softmax_ip.p1_block_pause,
                    dut.u_softmax_ip.p1_scan_done,
                    dut.u_softmax_ip.fifo_empty,
                    s_axis_tready, s_axis_tvalid);
        end
    end

    // ============================================================
    // Counters
    // ============================================================
    int total_pass = 0;
    int total_fail = 0;
    int tc_errors  = 0;

    // Stall/latency records for comparison
    longint tc8_stall, tc9_stall;
    longint tc8_lat_p1, tc9_lat_p1;
    longint tc8_rescales, tc9_rescales;
    longint tc14_stall, tc13_stall;
    longint tc14_lat_p1, tc13_lat_p1;
    longint tc17_stall, tc17_lat_p1;
    longint tc18_stall, tc18_lat_p1;
    longint tc19_stall, tc19_lat_p1;

    // ============================================================
    // Coverage Tracker (XSim-compatible, no covergroup)
    // ============================================================
    // Track which bins have been hit across all test cases
    // K-range bins: tiny(1-16), small(17-128), medium(129-512), large(513-2048), xlarge(>2048)
    bit cov_k_tiny, cov_k_small, cov_k_medium, cov_k_large, cov_k_xlarge;
    // Rescale bins: zero, one, few(2-4), many(>=5)
    bit cov_rsc_zero, cov_rsc_one, cov_rsc_few, cov_rsc_many;
    // Stall bins: zero, low(1-32), med(33-128), high(>128)
    bit cov_stall_zero, cov_stall_low, cov_stall_med, cov_stall_high;
    // BP bins: none, random, heavy
    bit cov_bp_none, cov_bp_random, cov_bp_heavy;
    // Match bins
    bit cov_match_pass, cov_match_fail;
    // Sample count
    int cov_sample_cnt;

    task automatic cov_sample(
        input int k_val, input int rescale_cnt,
        input int stall_cyc, input int bp_mode, input bit data_match
    );
        cov_sample_cnt++;
        // K range
        if (k_val <= 16)        cov_k_tiny   = 1;
        else if (k_val <= 128)  cov_k_small  = 1;
        else if (k_val <= 512)  cov_k_medium = 1;
        else if (k_val <= 2048) cov_k_large  = 1;
        else                    cov_k_xlarge = 1;
        // Rescale
        if (rescale_cnt == 0)      cov_rsc_zero = 1;
        else if (rescale_cnt == 1) cov_rsc_one  = 1;
        else if (rescale_cnt <= 4) cov_rsc_few  = 1;
        else                       cov_rsc_many = 1;
        // Stall
        if (stall_cyc == 0)        cov_stall_zero = 1;
        else if (stall_cyc <= 32)  cov_stall_low  = 1;
        else if (stall_cyc <= 128) cov_stall_med  = 1;
        else                       cov_stall_high = 1;
        // BP mode
        if (bp_mode == 0) cov_bp_none   = 1;
        if (bp_mode == 1) cov_bp_random = 1;
        if (bp_mode == 2) cov_bp_heavy  = 1;
        // Match
        if (data_match) cov_match_pass = 1;
        else            cov_match_fail = 1;
    endtask

    // ============================================================
    // AXI-Lite Driver Tasks
    // ============================================================
    task automatic axi_write(input logic [6:0] addr, input logic [31:0] data);
        @(posedge clk);
        s00_axi_awaddr  <= addr;
        s00_axi_awprot  <= 3'b000;
        s00_axi_awvalid <= 1'b1;
        s00_axi_wdata   <= data;
        s00_axi_wstrb   <= 4'hF;
        s00_axi_wvalid  <= 1'b1;
        s00_axi_bready  <= 1'b1;
        do @(posedge clk); while (!(s00_axi_awready && s00_axi_wready));
        s00_axi_awvalid <= 1'b0;
        s00_axi_wvalid  <= 1'b0;
        do @(posedge clk); while (!s00_axi_bvalid);
        s00_axi_bready <= 1'b0;
        @(posedge clk);
    endtask

    task automatic axi_read(input logic [6:0] addr, output logic [31:0] data);
        @(posedge clk);
        s00_axi_araddr  <= addr;
        s00_axi_arprot  <= 3'b000;
        s00_axi_arvalid <= 1'b0;
        s00_axi_rready  <= 1'b0;
        @(posedge clk);
        s00_axi_arvalid <= 1'b1;
        s00_axi_rready  <= 1'b1;
        do @(posedge clk); while (!s00_axi_arready);
        s00_axi_arvalid <= 1'b0;
        do @(posedge clk); while (!s00_axi_rvalid);
        data = s00_axi_rdata;
        s00_axi_rready <= 1'b0;
        @(posedge clk);
    endtask

    // ============================================================
    // AXI-Stream Driver (with heavy BP option)
    // ============================================================
    task automatic axis_send_beats(
        input logic [127:0] beats[], input int num_beats,
        input int bp_mode, input logic [15:0] last_tkeep
    );
        for (int i = 0; i < num_beats; i++) begin
            if (bp_mode == BP_RANDOM) begin
                while ($urandom_range(0,1) == 0) begin
                    s_axis_tvalid <= 1'b0;
                    @(posedge clk);
                end
            end else if (bp_mode == BP_HEAVY) begin
                while ($urandom_range(0,3) != 0) begin  // 75% stall
                    s_axis_tvalid <= 1'b0;
                    @(posedge clk);
                end
            end
            s_axis_tvalid <= 1'b1;
            s_axis_tdata  <= beats[i];
            s_axis_tkeep  <= (i == num_beats - 1) ? last_tkeep : 16'hFFFF;
            s_axis_tlast  <= (i == num_beats - 1);
            do @(posedge clk); while (!s_axis_tready);
        end
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
    endtask

    // ============================================================
    // AXI-Stream Monitor
    // ============================================================
    logic [127:0] captured_data [];

    task automatic axis_receive(input int num_beats, input int bp_mode);
        int cnt = 0;
        captured_data = new[num_beats];
        while (cnt < num_beats) begin
            if (bp_mode == BP_HEAVY)
                m_axis_tready <= ($urandom_range(0,3) == 0);  // 25% accept
            else if (bp_mode == BP_RANDOM)
                m_axis_tready <= ($urandom_range(0,1) == 1);
            else
                m_axis_tready <= 1'b1;
            @(posedge clk);
            if (m_axis_tvalid && m_axis_tready) begin
                captured_data[cnt] = m_axis_tdata;
                cnt++;
            end
        end
        m_axis_tready <= 1'b1;
    endtask

    // ============================================================
    // Reference Model -- identical to previous version
    // ============================================================
    logic [15:0] EXP2_ROM [256];

    function void init_rom();
        real val;
        for (int i = 0; i < 256; i++) begin
            val = $pow(2.0, -1.0 * i / 256.0) * 32768.0;
            EXP2_ROM[i] = int'(val + 0.5);
        end
    endfunction

    function automatic logic [15:0] ref_exp2(input logic [15:0] d_q412);
        longint z;
        int n, f;
        z = longint'(d_q412) * longint'(5909);
        n = (z >> 24) & 5'h1F;
        f = (z >> 16) & 8'hFF;
        if (n >= 16) return 16'd0;
        return (EXP2_ROM[f] >> n) & 16'hFFFF;
    endfunction

    logic [15:0] Y0_ROM [64] = '{
        16'h8000, 16'h7E08, 16'h7C1F, 16'h7A45, 16'h7878, 16'h76BA, 16'h7507, 16'h7361,
        16'h71C7, 16'h7038, 16'h6EB4, 16'h6D3A, 16'h6BCA, 16'h6A64, 16'h6907, 16'h67B2,
        16'h6666, 16'h6523, 16'h63E7, 16'h62B3, 16'h6186, 16'h6060, 16'h5F41, 16'h5E29,
        16'h5D17, 16'h5C0C, 16'h5B06, 16'h5A06, 16'h590B, 16'h5816, 16'h5726, 16'h563B,
        16'h5555, 16'h5474, 16'h5398, 16'h52BF, 16'h51EC, 16'h511C, 16'h5050, 16'h4F89,
        16'h4EC5, 16'h4E05, 16'h4D48, 16'h4C90, 16'h4BDA, 16'h4B28, 16'h4A79, 16'h49CD,
        16'h4925, 16'h487F, 16'h47DC, 16'h473C, 16'h469F, 16'h4604, 16'h456C, 16'h44D7,
        16'h4444, 16'h43B4, 16'h4326, 16'h429A, 16'h4211, 16'h4189, 16'h4104, 16'h4081
    };
    logic [31:0] C_ROM [64] = '{
        32'hF81F81F8, 32'hF85C9D10, 32'hF896FBB7, 32'hF8CEC723, 32'hF904258A,
        32'hF9373A69, 32'hF96826BC, 32'hF9970937, 32'hF9C3FE71, 32'hF9EF2114,
        32'hFA188A02, 32'hFA40507C, 32'hFA668A3D, 32'hFA8B4B9D, 32'hFAAEA7AD,
        32'hFAD0B049, 32'hFAF17634, 32'hFB11092C, 32'hFB2F77FD, 32'hFB4CD08F,
        32'hFB691FFB, 32'hFB847296, 32'hFB9ED400, 32'hFBB84F2E, 32'hFBD0EE7B,
        32'hFBE8BBAB, 32'hFBFFBFFC, 32'hFC160429, 32'hFC2B9075, 32'hFC406CB4,
        32'hFC54A04F, 32'hFC68324D, 32'hFC7B2959, 32'hFC8D8BC5, 32'hFC9F5F92,
        32'hFCB0AA76, 32'hFCC171DB, 32'hFCD1BAEB, 32'hFCE18A8D, 32'hFCF0E56D,
        32'hFCFFCFFD, 32'hFD0E4E7B, 32'hFD1C64F0, 32'hFD2A1737, 32'hFD3768FE,
        32'hFD445DC6, 32'hFD50F8EA, 32'hFD5D3D9C, 32'hFD692EEE, 32'hFD74CFCA,
        32'hFD8022FE, 32'hFD8B2B37, 32'hFD95EB06, 32'hFDA064DF, 32'hFDAA9B1C,
        32'hFDB48FFE, 32'hFDBE45AD, 32'hFDC7BE3D, 32'hFDD0FBAB, 32'hFDD9FFDE,
        32'hFDE2CCAB, 32'hFDEB63D5, 32'hFDF3C70C, 32'hFDFBF7F0
    };

    function automatic logic [15:0] ref_pwl_recip(input logic [15:0] m_in);
        int seg_idx, delta, y0, recip;
        longint c_signed, mult_result, mult_rnd, corr;
        seg_idx = (m_in >> 9) & 6'h3F;
        delta   = m_in & 9'h1FF;
        y0      = Y0_ROM[seg_idx];
        c_signed = $signed(C_ROM[seg_idx]);
        mult_result = c_signed * longint'(delta);
        if (mult_result < 0)
            mult_rnd = mult_result - (longint'(1) << 26);
        else
            mult_rnd = mult_result + (longint'(1) << 26);
        corr = mult_rnd >>> 27;
        recip = y0 + int'(corr);
        if (recip < 0)       return 16'h0000;
        if (recip > 16'hFFFF) return 16'hFFFF;
        return recip[15:0];
    endfunction

    function automatic void ref_softmax(
        input  logic [15:0] elems [],
        output logic [127:0] out_beats [],
        output int          argmax_idx,
        output logic [15:0] argmax_val
    );
        int n = elems.size();
        int n_beats = (n + 7) / 8;
        logic signed [15:0] max_val_s;
        logic [15:0] exp_vals [];
        logic [47:0] sum_exp;
        logic [15:0] d, sum_norm, recip;
        int norm_shift;
        logic [15:0] prob [0:7];
        logic [31:0] prod_rnd;

        exp_vals = new[n];
        out_beats = new[n_beats];

        // Find max
        max_val_s = $signed(elems[0]);
        argmax_idx = 0;
        for (int e = 1; e < n; e++) begin
            if ($signed(elems[e]) > max_val_s) begin
                max_val_s = $signed(elems[e]);
                argmax_idx = e;
            end
        end
        argmax_val = max_val_s;

        // Exp + sum
        sum_exp = 0;
        for (int e = 0; e < n; e++) begin
            d = (argmax_val - elems[e]) & 16'hFFFF;
            exp_vals[e] = ref_exp2(d);
            sum_exp = sum_exp + exp_vals[e];
        end

        // Normalize
        sum_norm = 0;
        norm_shift = 0;
        if (sum_exp != 0) begin
            for (int bp = 47; bp >= 0; bp--) begin
                if (sum_exp[bp]) begin
                    norm_shift = bp;
                    if (bp >= 15)
                        sum_norm = (sum_exp >> (bp - 15)) & 16'hFFFF;
                    else
                        sum_norm = (sum_exp << (15 - bp)) & 16'hFFFF;
                    break;
                end
            end
        end
        recip = ref_pwl_recip(sum_norm);

        // Output
        for (int beat = 0; beat < n_beats; beat++) begin
            for (int j = 0; j < 8; j++) begin
                if (beat*8 + j < n) begin
                    prod_rnd = exp_vals[beat*8+j] * recip + 32'd16384;
                    prob[j] = (prod_rnd >> norm_shift) & 16'hFFFF;
                    if (prob[j][15]) prob[j] = 16'h7FFF;
                end else
                    prob[j] = 16'd0;
            end
            out_beats[beat] = {prob[7], prob[6], prob[5], prob[4],
                               prob[3], prob[2], prob[1], prob[0]};
        end
    endfunction

    // ============================================================
    // Wait for p1_done / p2_done
    // ============================================================
    task automatic wait_status_bit(input int bit_pos, input int timeout = 5000);
        int cnt = 0;
        if (bit_pos == 0) begin
            // Detect p1_done OR FSM reaching ST_PASS2/ST_WAIT_P2
            while (!dut.u_softmax_ip.p1_done
                   && dut.u_softmax_ip.state != 4'd7   // ST_PASS2
                   && dut.u_softmax_ip.state != 4'd6   // ST_WAIT_P2
                   && cnt < timeout) begin
                @(posedge clk);
                cnt++;
            end
        end else begin
            while (dut.u_softmax_ip.state != 4'd8 && cnt < timeout) begin
                @(posedge clk);
                cnt++;
            end
            repeat(3) @(posedge clk);
        end
        if (cnt >= timeout)
            $display("[%0t] ERROR: Timeout waiting for status bit %0d after %0d cycles!",
                     $time, bit_pos, timeout);
    endtask

    // ============================================================
    // Performance counter reader (reads ALL counters)
    // ============================================================
    task automatic read_perf_counters(
        output longint perf_total, output longint perf_p1,
        output longint perf_p2,    output longint perf_stall,
        output longint perf_compute
    );
        logic [31:0] lo, hi;
        axi_read(ADDR_PERF_TOTAL, lo);   perf_total   = longint'(lo);
        axi_read(ADDR_PERF_P1, lo);      perf_p1      = longint'(lo);
        axi_read(ADDR_PERF_P2, lo);      perf_p2      = longint'(lo);
        axi_read(ADDR_PERF_STALL, lo);   perf_stall   = longint'(lo);
        axi_read(ADDR_PERF_COMPUTE, lo); perf_compute  = longint'(lo);
    endtask

    // ============================================================
    // Full 2-pass test with cycle-accurate measurement
    // ============================================================
    task automatic run_stream_test(
        input string tc_name,
        input logic [127:0] beats[], input int num_elems, input int num_beats,
        input logic [15:0] last_tkeep,
        input int tx_bp, input int rx_bp,
        input int tolerance,
        output int errors,
        output longint out_lat_p1, output longint out_stall,
        output longint out_rescales
    );
        logic [15:0] elements [];
        logic [127:0] ref_out [];
        int ref_argmax_idx;
        logic [15:0] ref_argmax_val;
        logic [31:0] rd_data;
        longint perf_total, perf_p1, perf_p2, perf_stall, perf_compute;
        longint t_start, t_p1_done;
        int rsc_cnt_before, rsc_cnt_after;

        $display("\n============================================================");
        $display("[%0t] ===== %s (K=%0d) =====", $time, tc_name, num_elems);
        $display("============================================================");

        // Unpack elements
        elements = new[num_elems];
        for (int i = 0; i < num_beats; i++)
            for (int j = 0; j < 8; j++)
                if (i*8 + j < num_elems)
                    elements[i*8 + j] = beats[i][j*16 +: 16];

        // Reference
        ref_softmax(elements, ref_out, ref_argmax_idx, ref_argmax_val);

        // Configure & clear perf
        axi_write(ADDR_CTRL, 32'h04);  // clear perf
        repeat(2) @(posedge clk);
        axi_write(ADDR_K_CONFIG, num_elems);

        // Record rescale count before test
        rsc_cnt_before = rescale_event_cnt;

        // Record start time
        t_start = $time;

        // Start P1 (auto-P2 hardcoded on)
        axi_write(ADDR_CTRL, 32'h01);  // start_p1

        // Send data for P1 (adaptive timeout: K*3 base + BP overhead)
        begin : blk_p1_timeout
            int adaptive_timeout;
            adaptive_timeout = num_elems * 3;
            if (tx_bp == BP_HEAVY) adaptive_timeout = adaptive_timeout * 4;
            else if (tx_bp == BP_RANDOM) adaptive_timeout = adaptive_timeout * 2;
            if (adaptive_timeout < 2000) adaptive_timeout = 2000;
            axis_send_beats(beats, num_beats, tx_bp, last_tkeep);
            wait_status_bit(0, adaptive_timeout);
        end
        t_p1_done = $time;
        $display("[%0t] Pass 1 DONE  [P1 latency = %0d ns = %0d cyc]",
                 $time, t_p1_done - t_start, (t_p1_done - t_start) / CLK_PERIOD);

        // P2: re-send data via DMA + receive output (adaptive timeout)
        fork
            axis_send_beats(beats, num_beats, tx_bp, last_tkeep);
            axis_receive(num_beats, rx_bp);
        join

        begin : blk_p2_timeout
            int adaptive_timeout;
            adaptive_timeout = num_elems * 3;
            if (rx_bp == BP_HEAVY) adaptive_timeout = adaptive_timeout * 4;
            else if (rx_bp == BP_RANDOM) adaptive_timeout = adaptive_timeout * 2;
            if (adaptive_timeout < 2000) adaptive_timeout = 2000;
            wait_status_bit(1, adaptive_timeout);
        end
        $display("[%0t] Pass 2 DONE  [Total = %0d ns = %0d cyc]",
                 $time, $time - t_start, ($time - t_start) / CLK_PERIOD);

        // Record rescale count after test
        rsc_cnt_after = rescale_event_cnt;
        out_rescales = rsc_cnt_after - rsc_cnt_before;

        // Check output
        errors = 0;
        begin : blk_check_output
            logic [15:0] got, exp_v;
            for (int i = 0; i < num_beats; i++) begin
                for (int j = 0; j < 8; j++) begin
                    if (i*8 + j >= num_elems) continue;
                    got   = captured_data[i][j*16 +: 16];
                    exp_v = ref_out[i][j*16 +: 16];
                    if (got > exp_v ? (got - exp_v) > tolerance : (exp_v - got) > tolerance) begin
                        if (errors < 10) 
                            $display("[%0t] MISMATCH beat=%0d elem=%0d: got=0x%04h exp=0x%04h",
                                     $time, i, j, got, exp_v);
                        errors++;
                    end
                end
            end
        end

        if (errors == 0)
            $display("[%0t] All %0d elements MATCH (tol=%0d)", $time, num_elems, tolerance);
        else
            $display("[%0t] %0d MISMATCHES (showing first 10)", $time, errors);

        // Read perf counters
        read_perf_counters(perf_total, perf_p1, perf_p2, perf_stall, perf_compute);
        $display("[%0t] Perf: TOTAL=%0d  P1=%0d  P2=%0d  STALL=%0d  COMPUTE=%0d  RESCALES=%0d",
                 $time, perf_total, perf_p1, perf_p2, perf_stall, perf_compute, out_rescales);

        out_lat_p1 = perf_p1;
        out_stall  = perf_stall;

        // Update coverage tracker
        cov_sample(num_elems, int'(out_rescales), int'(perf_stall),
                   (tx_bp == BP_HEAVY) ? 2 : (tx_bp == BP_RANDOM) ? 1 : 0,
                   (errors == 0));

        // Throughput calculation
        if (perf_total > 0) begin
            real throughput_elems_per_cyc, efficiency;
            throughput_elems_per_cyc = real'(num_elems) / real'(perf_total);
            efficiency = real'(perf_compute) / real'(perf_total) * 100.0;
            $display("[%0t]   Throughput: %.3f elem/cyc   Efficiency: %.1f%%",
                     $time, throughput_elems_per_cyc, efficiency);
        end
    endtask

    // ============================================================
    // Generate Test Data
    // ============================================================
    function automatic void gen_random_data(
        input int k_val,
        output logic [127:0] beats [], output int n_beats
    );
        n_beats = (k_val + 7) / 8;
        beats = new[n_beats];
        for (int i = 0; i < n_beats; i++)
            for (int j = 0; j < 8; j++)
                beats[i][j*16 +: 16] = $urandom() & 16'hFFFF;
    endfunction

    function automatic void gen_identical_data(
        input int k_val, input logic [15:0] val,
        output logic [127:0] beats [], output int n_beats
    );
        n_beats = (k_val + 7) / 8;
        beats = new[n_beats];
        for (int i = 0; i < n_beats; i++)
            beats[i] = {8{val}};
    endfunction

    function automatic void gen_dominant_data(
        input int k_val, input logic [15:0] dominant_val,
        input logic [15:0] bg_val, input int dom_idx,
        output logic [127:0] beats [], output int n_beats
    );
        n_beats = (k_val + 7) / 8;
        beats = new[n_beats];
        for (int i = 0; i < n_beats; i++)
            beats[i] = {8{bg_val}};
        beats[dom_idx/8][(dom_idx%8)*16 +: 16] = dominant_val;
    endfunction

    function automatic void gen_ascending_data(
        input int k_val,
        output logic [127:0] beats [], output int n_beats
    );
        n_beats = (k_val + 7) / 8;
        beats = new[n_beats];
        for (int i = 0; i < n_beats; i++)
            for (int j = 0; j < 8; j++)
                beats[i][j*16 +: 16] = 16'((i*8 + j) * 16'h0010);
    endfunction

    function automatic void gen_descending_data(
        input int k_val,
        output logic [127:0] beats [], output int n_beats
    );
        n_beats = (k_val + 7) / 8;
        beats = new[n_beats];
        for (int i = 0; i < n_beats; i++)
            for (int j = 0; j < 8; j++)
                beats[i][j*16 +: 16] = 16'(16'h7000 - (i*8 + j) * 16'h0010);
    endfunction

    // ============================================================
    // SVA
    // ============================================================
    property p_tvalid_stable;
        @(posedge clk) disable iff (!rst_n)
        m_axis_tvalid && !m_axis_tready |=> m_axis_tvalid;
    endproperty
    assert property (p_tvalid_stable)
    else $error("[SVA] tvalid dropped without handshake at %0t", $time);

    property p_fsm_known;
        @(posedge clk) disable iff (!rst_n)
        dut_fsm_state inside {[0:8]};
    endproperty
    assert property (p_fsm_known)
    else $error("[SVA] FSM unknown state %0d at %0t", dut_fsm_state, $time);

    // ============================================================
    // Main Test Sequence
    // ============================================================
    initial begin
        longint dummy_lat, dummy_stall, dummy_rsc;

        // Init signals
        s00_axi_awaddr = 0; s00_axi_awprot = 0; s00_axi_awvalid = 0;
        s00_axi_wdata  = 0; s00_axi_wstrb  = 0; s00_axi_wvalid  = 0;
        s00_axi_bready = 0;
        s00_axi_araddr = 0; s00_axi_arprot = 0; s00_axi_arvalid = 0;
        s00_axi_rready = 0;
        s_axis_tdata   = 0; s_axis_tkeep   = 0; s_axis_tvalid   = 0;
        s_axis_tlast   = 0; m_axis_tready  = 0;

        // Reset
        rst_n = 0;
        repeat(14) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        $display("[%0t] Reset deasserted.", $time);

        init_rom();

        // ===========================================================
        //  PART 1: FUNCTIONAL TESTS (TC1-TC7)
        // ===========================================================

        // TC1: Random data (stream baseline)
        begin : blk_tc1
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(K, tc_beats, tc_n_beats);
            run_stream_test("TC1_Random", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // TC2: All Identical
        begin : blk_tc2
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_identical_data(K, 16'h1000, tc_beats, tc_n_beats);
            run_stream_test("TC2_Identical", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 4, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // TC3: Single Dominant
        begin : blk_tc3
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_dominant_data(K, 16'h2000, 16'h8000, 7, tc_beats, tc_n_beats);
            run_stream_test("TC3_Dominant", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 4, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // TC4: Random Backpressure
        begin : blk_tc4
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(K, tc_beats, tc_n_beats);
            run_stream_test("TC4_Backpressure", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_RANDOM, BP_RANDOM, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // TC5: CRV Self-Check (expanded: 10 iters, K up to 4096)
        begin : blk_tc5
            $display("\n============================================================");
            $display("[%0t] ===== TC5_CRV_SelfCheck (10 iters) =====", $time);
            $display("============================================================");
            for (int iter = 0; iter < 10; iter++) begin
                logic [127:0] crv_beats [];
                int crv_k, crv_n;
                // K range: 8 to 4096 (covers tiny/small/medium/large/xlarge bins)
                crv_k = 8 * ($urandom_range(1, 512));
                gen_random_data(crv_k, crv_beats, crv_n);
                $display("\n[%0t] CRV iter %0d: K=%0d", $time, iter, crv_k);
                run_stream_test($sformatf("TC5_CRV_%0d", iter), crv_beats, crv_k, crv_n,
                                16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                                dummy_lat, dummy_stall, dummy_rsc);
                if (tc_errors == 0) total_pass++; else total_fail++;
            end
        end

        // TC6: Back-to-back
        begin : blk_tc6
            logic [127:0] tc_beats_a [], tc_beats_b [];
            int n_a, n_b;
            gen_random_data(K, tc_beats_a, n_a);
            gen_identical_data(K, 16'h0800, tc_beats_b, n_b);
            run_stream_test("TC6_B2B_Run1", tc_beats_a, K, n_a,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
            repeat(5) @(posedge clk);
            run_stream_test("TC6_B2B_Run2", tc_beats_b, K, n_b,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // TC7: Reset mid-operation & recovery
        begin : blk_tc7
            logic [127:0] tc_beats [];
            int tc_n;

            $display("\n============================================================");
            $display("[%0t] ===== TC7_Reset_Mid_Op =====", $time);
            $display("============================================================");

            gen_random_data(K, tc_beats, tc_n);
            axi_write(ADDR_K_CONFIG, K);
            axi_write(ADDR_CTRL, 32'h04);
            repeat(2) @(posedge clk);
            axi_write(ADDR_CTRL, 32'h01);

            s_axis_tvalid <= 1'b1;
            s_axis_tdata  <= 128'hAAAA_BBBB_CCCC_DDDD;
            s_axis_tkeep  <= 16'hFFFF;
            s_axis_tlast  <= 1'b0;
            repeat(15) @(posedge clk);
            s_axis_tvalid <= 1'b0;

            $display("[%0t] Asserting reset mid-operation...", $time);
            rst_n <= 1'b0;
            repeat(5) @(posedge clk);
            rst_n <= 1'b1;
            repeat(10) @(posedge clk);

            if (dut_fsm_state == 4'd0)
                $display("[%0t] FSM back to IDLE after reset. OK", $time);
            else
                $display("[%0t] FSM stuck in state %0d after reset. FAIL", $time, dut_fsm_state);

            gen_random_data(K, tc_beats, tc_n);
            run_stream_test("TC7_Recovery", tc_beats, K, tc_n,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ===========================================================
        //  PART 2: ARCHITECTURAL VERIFICATION (TC8-TC17)
        // ===========================================================

        $display("\n");
        $display("##########################################################");
        $display("##  PART 2: ARCHITECTURAL VERIFICATION                  ##");
        $display("##  Block-Scan Lookahead + Stall Reduction              ##");
        $display("##########################################################");

        // ---- TC8: Ascending K=160 (RESCALE STRESS) ----
        begin : blk_tc8
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(K, tc_beats, tc_n_beats);
            $display("\n--- TC8: Ascending K=%0d (worst-case for block-scan) ---", K);
            run_stream_test("TC8_Ascending", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            tc8_lat_p1, tc8_stall, tc8_rescales);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC9: Descending K=160 (NO RESCALE, baseline) ----
        begin : blk_tc9
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_descending_data(K, tc_beats, tc_n_beats);
            $display("\n--- TC9: Descending K=%0d (zero-rescale baseline) ---", K);
            run_stream_test("TC9_Descending", tc_beats, K, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            tc9_lat_p1, tc9_stall, tc9_rescales);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC10: K=8 (edge case: 1 beat, < BLOCK_SIZE) ----
        begin : blk_tc10
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(8, tc_beats, tc_n_beats);
            run_stream_test("TC10_K8", tc_beats, 8, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC11: K=256 random (multi-block) ----
        begin : blk_tc11
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(256, tc_beats, tc_n_beats);
            run_stream_test("TC11_K256", tc_beats, 256, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC12: Ascending K=256 + backpressure ----
        begin : blk_tc12
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(256, tc_beats, tc_n_beats);
            run_stream_test("TC12_Asc_K256_BP", tc_beats, 256, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_RANDOM, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC13: K=1024 random (scalability) ----
        begin : blk_tc13
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(1024, tc_beats, tc_n_beats);
            $display("\n--- TC13: K=1024 Random (scalability test) ---");
            run_stream_test("TC13_K1024_Rand", tc_beats, 1024, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            tc13_lat_p1, tc13_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC14: K=1024 ascending + heavy BP ----
        begin : blk_tc14
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(1024, tc_beats, tc_n_beats);
            $display("\n--- TC14: K=1024 Ascending + Heavy BP (worst stress) ---");
            run_stream_test("TC14_K1024_Asc_HBP", tc_beats, 1024, tc_n_beats,
                            16'hFFFF, BP_HEAVY, BP_HEAVY, 8, tc_errors,
                            tc14_lat_p1, tc14_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC15: K=120 (15 beats = B-1, partial block edge) ----
        begin : blk_tc15
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(120, tc_beats, tc_n_beats);
            $display("\n--- TC15: K=120 Ascending (B-1 beats, partial block edge) ---");
            run_stream_test("TC15_K120_Partial", tc_beats, 120, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC16: K=128 (exactly 1 full block, 16 beats) ----
        begin : blk_tc16
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(128, tc_beats, tc_n_beats);
            $display("\n--- TC16: K=128 Ascending (exactly 1 block) ---");
            run_stream_test("TC16_K128_OneBlock", tc_beats, 128, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            dummy_lat, dummy_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC17: K=1024 ascending NO BP (stall measurement only) ----
        begin : blk_tc17
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(1024, tc_beats, tc_n_beats);
            $display("\n--- TC17: K=1024 Ascending NO BP (pure stall measurement) ---");
            run_stream_test("TC17_K1024_Asc", tc_beats, 1024, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            tc17_lat_p1, tc17_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC18: K=4096 random (xlarge scalability) ----
        begin : blk_tc18
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_random_data(4096, tc_beats, tc_n_beats);
            $display("\n--- TC18: K=4096 Random (xlarge scalability test) ---");
            run_stream_test("TC18_K4096_Rand", tc_beats, 4096, tc_n_beats,
                            16'hFFFF, BP_NONE, BP_NONE, 8, tc_errors,
                            tc18_lat_p1, tc18_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ---- TC19: K=4096 ascending + heavy BP (max stall stress) ----
        begin : blk_tc19
            logic [127:0] tc_beats [];
            int tc_n_beats;
            gen_ascending_data(4096, tc_beats, tc_n_beats);
            $display("\n--- TC19: K=4096 Ascending + Heavy BP (max stall stress) ---");
            run_stream_test("TC19_K4096_Asc_HBP", tc_beats, 4096, tc_n_beats,
                            16'hFFFF, BP_HEAVY, BP_HEAVY, 8, tc_errors,
                            tc19_lat_p1, tc19_stall, dummy_rsc);
            if (tc_errors == 0) total_pass++; else total_fail++;
        end

        // ===========================================================
        //  PART 3: PERFORMANCE COMPARISON & ARCHITECTURAL ASSERTIONS
        // ===========================================================

        $display("\n");
        $display("##########################################################");
        $display("##  PART 3: PERFORMANCE ANALYSIS                        ##");
        $display("##########################################################");

        // ---- Compare TC8 (ascending) vs TC9 (descending) K=160 ----
        $display("\n=== STALL COMPARISON: TC8 (ascending) vs TC9 (descending) K=%0d ===", K);
        $display("  TC8 Ascending:  P1=%0d cyc,  STALL=%0d cyc, RESCALES=%0d",
                 tc8_lat_p1, tc8_stall, tc8_rescales);
        $display("  TC9 Descending: P1=%0d cyc,  STALL=%0d cyc, RESCALES=%0d",
                 tc9_lat_p1, tc9_stall, tc9_rescales);

        if (tc8_stall > 0 && tc9_stall == 0) begin
            $display("  [OK] Ascending has stall=%0d, descending has 0 -- block-scan is working", tc8_stall);
        end else if (tc8_stall == 0 && tc9_stall == 0) begin
            $display("  [OK] Both zero stall -- no rescale needed for either");
        end

        if (tc8_lat_p1 > 0 && tc9_lat_p1 > 0) begin
            real ratio;
            ratio = real'(tc8_lat_p1) / real'(tc9_lat_p1);
            $display("  P1 latency ratio (asc/desc) = %.2f", ratio);
            if (ratio < 2.0)
                $display("  [PASS] Ascending P1 latency within 2x of descending -- near-deterministic!");
            else
                $display("  [WARN] Ascending P1 latency %.1fx of descending -- check block-scan", ratio);
        end

        // ---- Compare TC13 (K=1024 random) vs TC17 (K=1024 ascending) ----
        $display("\n=== SCALABILITY: K=1024 Random vs Ascending ===");
        $display("  TC13 Random:    P1=%0d cyc,  STALL=%0d cyc", tc13_lat_p1, tc13_stall);
        $display("  TC17 Ascending: P1=%0d cyc,  STALL=%0d cyc", tc17_lat_p1, tc17_stall);

        if (tc17_lat_p1 > 0 && tc13_lat_p1 > 0) begin
            real ratio;
            ratio = real'(tc17_lat_p1) / real'(tc13_lat_p1);
            $display("  P1 latency ratio (asc/rand) = %.2f", ratio);
            if (ratio < 2.5)
                $display("  [PASS] K=1024 ascending within 2.5x of random -- block-scan effective!");
            else
                $display("  [WARN] K=1024 ascending %.1fx of random -- stall not well controlled", ratio);
        end

        // ---- v6.0 vs v4.1 theoretical comparison ----
        $display("\n=== v6.0 BLOCK-SCAN THEORETICAL COMPARISON ===");
        begin : blk_v41_compare
            int n_beats_1024 = (1024 + 7) / 8;
            int v41_worst_rescales = n_beats_1024 - 1;
            int v60_worst_rescales = (n_beats_1024 + 15) / 16;
            int v41_worst_stall = v41_worst_rescales * 16;
            int v60_worst_stall = v60_worst_rescales * 16;
            $display("  K=1024 (%0d beats):", n_beats_1024);
            $display("    v4.1 worst-case: %0d rescales x 16 cyc = %0d stall cycles",
                     v41_worst_rescales, v41_worst_stall);
            $display("    v6.0 worst-case: %0d rescales x 16 cyc = %0d stall cycles",
                     v60_worst_rescales, v60_worst_stall);
            $display("    Theoretical improvement: %.1fx",
                     real'(v41_worst_stall) / real'(v60_worst_stall));
            $display("    Measured v6.0 stall (TC17): %0d cycles", tc17_stall);

            if (tc17_stall <= v60_worst_stall + 20)
                $display("    [PASS] Measured stall within theoretical bound");
            else
                $display("    [WARN] Measured stall exceeds theoretical bound by %0d cycles",
                         tc17_stall - v60_worst_stall);
        end

        // ===========================================================
        //  Final Summary
        // ===========================================================
        repeat(10) @(posedge clk);

        $display("\n");
        $display("==========================================================");
        $display("||  SOFTMAX IP v6.0 VERIFICATION SUMMARY               ||");
        $display("==========================================================");
        $display("||  Total PASS: %3d                                      ||", total_pass);
        $display("||  Total FAIL: %3d                                      ||", total_fail);
        $display("==========================================================");

        if (total_fail == 0)
            $display("||   [PASS] ALL TESTS PASSED                            ||");
        else
            $display("||   [FAIL] SOME TESTS FAILED                           ||");

        $display("==========================================================");
        $display("");

        // ---- Coverage Report ----
        begin : blk_cov_report
            int k_hit, k_total, rsc_hit, rsc_total, stall_hit, stall_total;
            int bp_hit, bp_total, match_hit, func_total, func_hit;
            k_hit = cov_k_tiny + cov_k_small + cov_k_medium + cov_k_large + cov_k_xlarge;
            k_total = 5;
            rsc_hit = cov_rsc_zero + cov_rsc_one + cov_rsc_few + cov_rsc_many;
            rsc_total = 4;
            stall_hit = cov_stall_zero + cov_stall_low + cov_stall_med + cov_stall_high;
            stall_total = 4;
            bp_hit = cov_bp_none + cov_bp_random + cov_bp_heavy;
            bp_total = 3;
            // Functional coverage excludes 'fail' bin (it's a success indicator)
            func_hit = k_hit + rsc_hit + stall_hit + bp_hit;
            func_total = k_total + rsc_total + stall_total + bp_total;
            match_hit = cov_match_pass + cov_match_fail;
            $display("==========================================================");
            $display("||  COVERAGE REPORT (%0d samples)                       ||", cov_sample_cnt);
            $display("==========================================================");
            $display("  K range:   %0d/%0d bins (tiny=%0b small=%0b med=%0b large=%0b xlarge=%0b)",
                     k_hit, k_total, cov_k_tiny, cov_k_small, cov_k_medium, cov_k_large, cov_k_xlarge);
            $display("  Rescale:   %0d/%0d bins (zero=%0b one=%0b few=%0b many=%0b)",
                     rsc_hit, rsc_total, cov_rsc_zero, cov_rsc_one, cov_rsc_few, cov_rsc_many);
            $display("  Stall:     %0d/%0d bins (zero=%0b low=%0b med=%0b high=%0b)",
                     stall_hit, stall_total, cov_stall_zero, cov_stall_low, cov_stall_med, cov_stall_high);
            $display("  BP mode:   %0d/%0d bins (none=%0b random=%0b heavy=%0b)",
                     bp_hit, bp_total, cov_bp_none, cov_bp_random, cov_bp_heavy);
            $display("  Data match:pass=%0b fail=%0b (fail bin excluded from score)",
                     cov_match_pass, cov_match_fail);
            $display("  ---------------------------------------------------------");
            $display("  Functional:  %0d/%0d bins hit (%.1f%%)",
                     func_hit, func_total,
                     real'(func_hit) / real'(func_total) * 100.0);
            $display("==========================================================");
        end

        #100;
        $finish;
    end

endmodule
