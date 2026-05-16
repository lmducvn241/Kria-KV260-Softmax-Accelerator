`timescale 1ns/1ps
// ============================================================
// scan_fifo.v — Block-Scan Lookahead FIFO for Softmax IP v6.0
// ============================================================

module scan_fifo #(
    parameter integer DATA_W     = 128,   // 8 × 16-bit packed
    parameter integer ELEM_W     = 16,    // per-element width
    parameter integer LANES      = 8,     // elements per beat
    parameter integer BLOCK_SIZE = 16,    // beats per block (B)
    parameter integer FIFO_DEPTH = 32     // 2×B for double-buffering
) (
    input  wire                  clk,
    input  wire                  rst,

    // Write port (from AXI-Stream)
    input  wire                  wr_en,
    input  wire [DATA_W-1:0]     wr_data,
    input  wire [LANES*2-1:0]    wr_tkeep,
    input  wire                  wr_tlast,

    // Read port (to P1 pipeline)
    input  wire                  rd_en,
    output wire [DATA_W-1:0]     rd_data,
    output wire [LANES*2-1:0]    rd_tkeep,
    output wire                  rd_tlast,
    output wire                  rd_valid,

    // Block max output
    output reg signed [ELEM_W-1:0] block_max,      
    output reg                     block_max_valid, 
    output reg                     block_max_first, 

    // Status
    output wire [5:0]            count,
    output wire                  empty,
    output wire                  full,

    // Control
    input  wire                  flush  
);

    localparam ADDR_W = $clog2(FIFO_DEPTH);

    // ============================
    // FIFO Storage (distributed RAM)
    // ============================
    (* ram_style = "distributed" *)
    reg [DATA_W-1:0]      mem_data  [0:FIFO_DEPTH-1];
    (* ram_style = "distributed" *)
    reg [LANES*2-1:0]     mem_tkeep [0:FIFO_DEPTH-1];
    (* ram_style = "distributed" *)
    reg                   mem_tlast [0:FIFO_DEPTH-1];
    reg [ADDR_W-1:0] wr_ptr;
    reg [ADDR_W-1:0] rd_ptr;
    reg [5:0]         cnt;

    assign count = cnt;
    assign empty = (cnt == 0);
    assign full  = (cnt >= FIFO_DEPTH);
    assign rd_valid = (cnt > 0);

    // Read port
    assign rd_data  = mem_data[rd_ptr];
    assign rd_tkeep = mem_tkeep[rd_ptr];
    assign rd_tlast = mem_tlast[rd_ptr];

    // ============================
    // Write/Read Logic
    // ============================
    always @(posedge clk) begin
        if (rst || flush) begin
            wr_ptr <= {ADDR_W{1'b0}};
            rd_ptr <= {ADDR_W{1'b0}};
            cnt    <= 6'd0;
        end else begin
            // Write
            if (wr_en && !full) begin
                mem_data[wr_ptr]  <= wr_data;
                mem_tkeep[wr_ptr] <= wr_tkeep;
                mem_tlast[wr_ptr] <= wr_tlast;
                wr_ptr <= (wr_ptr == FIFO_DEPTH-1) ? {ADDR_W{1'b0}} : wr_ptr + 1;
            end
            // Read
            if (rd_en && !empty) begin
                rd_ptr <= (rd_ptr == FIFO_DEPTH-1) ? {ADDR_W{1'b0}} : rd_ptr + 1;
            end
            // Count
            case ({wr_en && !full, rd_en && !empty})
                2'b10:   cnt <= cnt + 6'd1;
                2'b01:   cnt <= cnt - 6'd1;
                default: cnt <= cnt;
            endcase
        end
    end

    // ============================
    // Block Max Scanner (3-Stage Pipeline)
    // ============================
    reg [4:0] scan_beat_cnt;  // beats in current scan block
    reg       scan_first_block;
    reg signed [ELEM_W-1:0] scan_running_max;

    // ---- Combinational: Unpack 8 lanes ----
    wire signed [ELEM_W-1:0] wr_elem [0:LANES-1];
    wire                     wr_elem_valid [0:LANES-1];
    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : unpack_wr
            assign wr_elem[g]       = wr_data[g*ELEM_W +: ELEM_W];
            assign wr_elem_valid[g] = wr_tkeep[g*2] & wr_tkeep[g*2+1];
        end
    endgenerate

    // ---- Combinational: 8→4 pairwise max (Stage 1a input) ----
    wire signed [ELEM_W-1:0] cmp_s1 [0:3];
    assign cmp_s1[0] = ($signed(wr_elem[0]) >= $signed(wr_elem[1])) ? wr_elem[0] : wr_elem[1];
    assign cmp_s1[1] = ($signed(wr_elem[2]) >= $signed(wr_elem[3])) ? wr_elem[2] : wr_elem[3];
    assign cmp_s1[2] = ($signed(wr_elem[4]) >= $signed(wr_elem[5])) ? wr_elem[4] : wr_elem[5];
    assign cmp_s1[3] = ($signed(wr_elem[6]) >= $signed(wr_elem[7])) ? wr_elem[6] : wr_elem[7];

    // ============================
    // Stage 1a: Register pairwise winners + metadata
    // ============================
    reg signed [ELEM_W-1:0] cmp_s1_r [0:3];
    reg                     s1a_valid;
    reg                     s1a_first_beat;
    reg                     s1a_block_done;

    always @(posedge clk) begin
        if (rst || flush) begin
            s1a_valid <= 1'b0;
        end else begin
            s1a_valid <= 1'b0;
            if (wr_en && !full) begin
                cmp_s1_r[0]   <= cmp_s1[0];
                cmp_s1_r[1]   <= cmp_s1[1];
                cmp_s1_r[2]   <= cmp_s1[2];
                cmp_s1_r[3]   <= cmp_s1[3];
                s1a_valid     <= 1'b1;
                s1a_first_beat <= (scan_beat_cnt == 5'd0);
                s1a_block_done <= (scan_beat_cnt + 5'd1 >= BLOCK_SIZE) || wr_tlast;
            end
        end
    end

    always @(posedge clk) begin
        if (rst || flush) begin
            scan_beat_cnt <= 5'd0;
        end else if (wr_en && !full) begin
            if (scan_beat_cnt + 5'd1 >= BLOCK_SIZE || wr_tlast)
                scan_beat_cnt <= 5'd0;
            else
                scan_beat_cnt <= scan_beat_cnt + 5'd1;
        end
    end

    // ---- Combinational: 4 to 2 to 1 from registered cmp_s1 (Stage 1b input) ----
    wire signed [ELEM_W-1:0] cmp_s2_w [0:1];
    assign cmp_s2_w[0] = ($signed(cmp_s1_r[0]) >= $signed(cmp_s1_r[1])) ? cmp_s1_r[0] : cmp_s1_r[1];
    assign cmp_s2_w[1] = ($signed(cmp_s1_r[2]) >= $signed(cmp_s1_r[3])) ? cmp_s1_r[2] : cmp_s1_r[3];

    wire signed [ELEM_W-1:0] wr_beat_max;
    assign wr_beat_max = ($signed(cmp_s2_w[0]) >= $signed(cmp_s2_w[1])) ? cmp_s2_w[0] : cmp_s2_w[1];

    // ============================
    // Stage 1b: Register final beat max + forward metadata
    // ============================
    reg signed [ELEM_W-1:0] beat_max_s1;
    reg                     s1_valid;
    reg                     s1_first_beat;
    reg                     s1_block_done;

    always @(posedge clk) begin
        if (rst || flush) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= 1'b0;
            if (s1a_valid) begin
                beat_max_s1   <= wr_beat_max;
                s1_valid      <= 1'b1;
                s1_first_beat <= s1a_first_beat;
                s1_block_done <= s1a_block_done;
            end
        end
    end

    // ============================
    // Stage 2: Running max + block max output
    // ============================
    always @(posedge clk) begin
        if (rst || flush) begin
            scan_running_max <= 16'sh8000;  // min signed
            scan_first_block <= 1'b1;
            block_max        <= 16'sh8000;
            block_max_valid  <= 1'b0;
            block_max_first  <= 1'b0;
        end else begin
            block_max_valid <= 1'b0;  // default: pulse
            block_max_first <= 1'b0;
            if (s1_valid) begin
                if (s1_first_beat)
                    scan_running_max <= beat_max_s1;
                else if ($signed(beat_max_s1) > $signed(scan_running_max))
                    scan_running_max <= beat_max_s1;
                if (s1_block_done) begin
                    if (s1_first_beat)
                        block_max <= beat_max_s1;  // single-beat block
                    else if ($signed(beat_max_s1) > $signed(scan_running_max))
                        block_max <= beat_max_s1;   // last beat is max
                    else
                        block_max <= scan_running_max;
                    block_max_valid  <= 1'b1;
                    block_max_first  <= scan_first_block;
                    scan_first_block <= 1'b0;
                    scan_running_max <= 16'sh8000;
                end
            end
        end
    end

endmodule
