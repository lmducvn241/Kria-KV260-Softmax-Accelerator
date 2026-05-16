`timescale 1 ns / 1 ps
// ============================================================
// softmax_dl2_slave_lite_v1_0_S00_AXI.v
// ============================================================
module softmax_dl2_slave_lite_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
) (
    // AXI-Lite
    input  wire S_AXI_ACLK,
    input  wire S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [2:0] S_AXI_AWPROT,
    input  wire S_AXI_AWVALID,
    output wire S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire S_AXI_WVALID,
    output wire S_AXI_WREADY,
    output wire [1:0] S_AXI_BRESP,
    output wire S_AXI_BVALID,
    input  wire S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [2:0] S_AXI_ARPROT,
    input  wire S_AXI_ARVALID,
    output wire S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [1:0] S_AXI_RRESP,
    output wire S_AXI_RVALID,
    input  wire S_AXI_RREADY,

    // Control interface
    output wire        start_p1_pulse,
    output wire        start_p2_pulse,
    output wire        clear_perf_pulse,
    output wire        clear_error_pulse,
    output wire [31:0] k_config_o,

    // Status interface
    input  wire        ip_busy,
    input  wire        ip_p1_done,
    input  wire        ip_p2_done,
    input  wire        ip_error,

    // Performance counters
    input  wire [63:0] perf_total_i,
    input  wire [63:0] perf_p1_i,
    input  wire [63:0] perf_p2_i,
    input  wire [63:0] perf_stall_i,
    input  wire [63:0] perf_compute_i,

    // Results
    input  wire [31:0] argmax_idx_i,
    input  wire [15:0] argmax_val_i
);

    localparam integer ADDR_LSB = 2;
    localparam integer OPT_MEM_ADDR_BITS = 3;

    // Write channel
    reg axi_awready;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg axi_wready;
    reg [1:0] axi_bresp;
    reg axi_bvalid;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;

    wire write_addr_hs = S_AXI_AWVALID & S_AXI_AWREADY;
    wire write_data_hs = S_AXI_WVALID  & S_AXI_WREADY;
    wire write_hs      = write_addr_hs & write_data_hs;

    // CTRL register (0x00) self-clearing bits
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg0;

    // K_CONFIG register (0x30)
    reg [31:0] k_config_reg;
    assign k_config_o = k_config_reg;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            axi_awaddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            axi_wready  <= 1'b0;
        end else begin
            axi_awready <= (~axi_awready) & S_AXI_AWVALID & S_AXI_WVALID;
            axi_wready  <= (~axi_wready)  & S_AXI_WVALID  & S_AXI_AWVALID;
            if (S_AXI_AWVALID & S_AXI_WVALID)
                axi_awaddr <= S_AXI_AWADDR;
        end
    end

    // Register write
    integer i;
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            slv_reg0     <= {C_S_AXI_DATA_WIDTH{1'b0}};
            k_config_reg <= 32'd1024;  // default K
        end else if (write_hs) begin
            case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                4'h0: begin  // CTRL register
                    for (i=0; i<C_S_AXI_DATA_WIDTH/8; i=i+1)
                        if (S_AXI_WSTRB[i]) slv_reg0[(i*8)+:8] <= S_AXI_WDATA[(i*8)+:8];
                    slv_reg0[0] <= 1'b0;
                    slv_reg0[1] <= 1'b0;
                    slv_reg0[2] <= 1'b0;
                    slv_reg0[3] <= 1'b0;
                end
                4'hC: begin  // K_CONFIG register (0x30)
                    for (i=0; i<4; i=i+1)
                        if (S_AXI_WSTRB[i]) k_config_reg[(i*8)+:8] <= S_AXI_WDATA[(i*8)+:8];
                end
            endcase
        end else begin
            slv_reg0[0] <= 1'b0;
            slv_reg0[1] <= 1'b0;
            slv_reg0[2] <= 1'b0;
            slv_reg0[3] <= 1'b0;
        end
    end

    // B channel
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin
            if (write_hs & ~axi_bvalid) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00;
            end else if (axi_bvalid & S_AXI_BREADY) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    // Read channel
    reg axi_arready;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg [1:0] axi_rresp;
    reg axi_rvalid;

    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_araddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            axi_rvalid  <= 1'b0;
            axi_rresp   <= 2'b00;
            axi_rdata   <= {C_S_AXI_DATA_WIDTH{1'b0}};
        end else begin
            axi_arready <= (~axi_arready) & S_AXI_ARVALID;
            if (~axi_arready & S_AXI_ARVALID)
                axi_araddr <= S_AXI_ARADDR;

            if (axi_arready & S_AXI_ARVALID & ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
                case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                    4'h0: axi_rdata <= slv_reg0;                                         // 0x00 CTRL
                    4'h1: axi_rdata <= {28'd0, ip_error, ip_busy, ip_p2_done, ip_p1_done}; // 0x04 STATUS: b0=p1_done b1=p2_done b2=busy b3=error
                    4'h2: axi_rdata <= argmax_idx_i;                                     // 0x08
                    4'h3: axi_rdata <= {16'd0, argmax_val_i};                            // 0x0C
                    4'h4: axi_rdata <= perf_total_i[31:0];                               // 0x10
                    4'h5: axi_rdata <= perf_total_i[63:32];                              // 0x14
                    4'h6: axi_rdata <= perf_p1_i[31:0];                                  // 0x18
                    4'h7: axi_rdata <= perf_p1_i[63:32];                                 // 0x1C
                    4'h8: axi_rdata <= perf_p2_i[31:0];                                  // 0x20
                    4'h9: axi_rdata <= perf_p2_i[63:32];                                 // 0x24
                    4'hA: axi_rdata <= perf_stall_i[31:0];                               // 0x28
                    4'hB: axi_rdata <= perf_stall_i[63:32];                              // 0x2C
                    4'hC: axi_rdata <= k_config_reg;                                     // 0x30
                    4'hD: axi_rdata <= perf_compute_i[31:0];                              // 0x34
                    4'hE: axi_rdata <= perf_compute_i[63:32];                             // 0x38
                    default: axi_rdata <= 32'd0;
                endcase
            end else if (axi_rvalid & S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    // Pulse generators
    reg start_p1_r, start_p2_r, clear_perf_r, clear_error_r;
    assign start_p1_pulse    = start_p1_r;
    assign start_p2_pulse    = start_p2_r;
    assign clear_perf_pulse  = clear_perf_r;
    assign clear_error_pulse = clear_error_r;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            start_p1_r    <= 1'b0;
            start_p2_r    <= 1'b0;
            clear_perf_r  <= 1'b0;
            clear_error_r <= 1'b0;
        end else begin
            start_p1_r    <= (write_hs &&
                              (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h0) &&
                              (S_AXI_WDATA[0] == 1'b1));
            start_p2_r    <= (write_hs &&
                              (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h0) &&
                              (S_AXI_WDATA[1] == 1'b1));
            clear_perf_r  <= (write_hs &&
                              (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h0) &&
                              (S_AXI_WDATA[2] == 1'b1));
            clear_error_r <= (write_hs &&
                              (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h0) &&
                              (S_AXI_WDATA[3] == 1'b1));
        end
    end

endmodule
