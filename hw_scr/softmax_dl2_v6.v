`timescale 1 ns / 1 ps
// ============================================================
// softmax_dl2_v6.v — v6.0 Top-level wrapper
// ============================================================
module softmax_dl2_v6 #(
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 7,
    parameter integer IN_ELEM_WIDTH  = 16,
    parameter integer OUT_ELEM_WIDTH = 16,
    parameter integer IN_FRAC  = 12,
    parameter integer Z_FRAC   = 15,
    parameter integer EXP_MODE = 0,
    parameter integer SEG_DEPTH = 64,
    parameter integer BLOCK_SIZE = 16,
    parameter integer FIFO_DEPTH = 32
) (
    // AXI-Lite
    input  wire s00_axi_aclk,
    input  wire s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
    input  wire [2:0] s00_axi_awprot,
    input  wire s00_axi_awvalid,
    output wire s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire s00_axi_wvalid,
    output wire s00_axi_wready,
    output wire [1:0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input  wire s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
    input  wire [2:0] s00_axi_arprot,
    input  wire s00_axi_arvalid,
    output wire s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_rdata,
    output wire [1:0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input  wire s00_axi_rready,

    // AXIS Slave
    input  wire [8*IN_ELEM_WIDTH-1:0]   s_axis_tdata,
    input  wire [8*IN_ELEM_WIDTH/8-1:0] s_axis_tkeep,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire                         s_axis_tlast,

    // AXIS Master
    output wire [8*OUT_ELEM_WIDTH-1:0]   m_axis_tdata,
    output wire [8*OUT_ELEM_WIDTH/8-1:0] m_axis_tkeep,
    output wire                          m_axis_tvalid,
    input  wire                          m_axis_tready,
    output wire                          m_axis_tlast
);

    wire clk = s00_axi_aclk;
    wire rst_n = s00_axi_aresetn;

    // ========================================
    // AXI-Lite regfile
    // ========================================
    wire        start_p1_w, start_p2_w;
    wire        clear_perf_w, clear_error_w;
    wire [31:0] k_config_w;
    wire        ip_busy_w, ip_p1_done_w, ip_p2_done_w, ip_error_w;
    wire [31:0] argmax_idx_w;
    wire [15:0] argmax_val_w;
    wire [63:0] perf_total_w, perf_p1_w, perf_p2_w, perf_stall_w;
    wire [63:0] perf_compute_w;

    softmax_dl2_slave_lite_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(6)
    ) u_regfile (
        .S_AXI_ACLK        (clk),
        .S_AXI_ARESETN     (rst_n),
        .S_AXI_AWADDR      (s00_axi_awaddr[5:0]),
        .S_AXI_AWPROT      (s00_axi_awprot),
        .S_AXI_AWVALID     (s00_axi_awvalid),
        .S_AXI_AWREADY     (s00_axi_awready),
        .S_AXI_WDATA       (s00_axi_wdata),
        .S_AXI_WSTRB       (s00_axi_wstrb),
        .S_AXI_WVALID      (s00_axi_wvalid),
        .S_AXI_WREADY      (s00_axi_wready),
        .S_AXI_BRESP       (s00_axi_bresp),
        .S_AXI_BVALID      (s00_axi_bvalid),
        .S_AXI_BREADY      (s00_axi_bready),
        .S_AXI_ARADDR      (s00_axi_araddr[5:0]),
        .S_AXI_ARPROT      (s00_axi_arprot),
        .S_AXI_ARVALID     (s00_axi_arvalid),
        .S_AXI_ARREADY     (s00_axi_arready),
        .S_AXI_RDATA       (s00_axi_rdata),
        .S_AXI_RRESP       (s00_axi_rresp),
        .S_AXI_RVALID      (s00_axi_rvalid),
        .S_AXI_RREADY      (s00_axi_rready),
        .start_p1_pulse     (start_p1_w),
        .start_p2_pulse     (start_p2_w),
        .clear_perf_pulse   (clear_perf_w),
        .clear_error_pulse  (clear_error_w),
        .k_config_o         (k_config_w),
        .ip_busy            (ip_busy_w),
        .ip_p1_done         (ip_p1_done_w),
        .ip_p2_done         (ip_p2_done_w),
        .ip_error           (ip_error_w),
        .perf_total_i       (perf_total_w),
        .perf_p1_i          (perf_p1_w),
        .perf_p2_i          (perf_p2_w),
        .perf_stall_i       (perf_stall_w),
        .perf_compute_i     (perf_compute_w),
        .argmax_idx_i       (argmax_idx_w),
        .argmax_val_i       (argmax_val_w)
    );

    // ========================================
    // FIFO count (debug)
    // ========================================
    wire [5:0] fifo_count_w;

    // ========================================
    // Softmax IP v6.0
    // ========================================
    softmax_ip_v6 #(
        .ELEM_WIDTH (IN_ELEM_WIDTH),
        .IN_FRAC    (IN_FRAC),
        .Z_FRAC     (Z_FRAC),
        .SUM_WIDTH  (48),
        .EXP_MODE   (EXP_MODE),
        .SEG_DEPTH  (SEG_DEPTH),
        .BLOCK_SIZE (BLOCK_SIZE),
        .FIFO_DEPTH (FIFO_DEPTH),
        .DEBUG      (0)
    ) u_softmax_ip (
        .clk           (clk),
        .rst_n         (rst_n),

        .start_p1      (start_p1_w),
        .start_p2      (start_p2_w),
        .clear_error   (clear_error_w),
        .k_config      (k_config_w),
        .auto_p2_en    (1'b1),          // Always auto-transition

        .busy          (ip_busy_w),
        .p1_done       (ip_p1_done_w),
        .p2_done       (ip_p2_done_w),
        .error         (ip_error_w),

        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tkeep  (s_axis_tkeep),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (s_axis_tready),
        .s_axis_tlast  (s_axis_tlast),

        .m_axis_tdata  (m_axis_tdata),
        .m_axis_tkeep  (m_axis_tkeep),
        .m_axis_tvalid (m_axis_tvalid),
        .m_axis_tready (m_axis_tready),
        .m_axis_tlast  (m_axis_tlast),

        .argmax_idx_o  (argmax_idx_w),
        .argmax_val_o  (argmax_val_w),

        .perf_total    (perf_total_w),
        .perf_p1       (perf_p1_w),
        .perf_p2       (perf_p2_w),
        .perf_stall    (perf_stall_w),
        .perf_compute  (perf_compute_w),
        .perf_dma_wait (),
        .clear_perf    (clear_perf_w),

        .fifo_count    (fifo_count_w)
    );

endmodule
