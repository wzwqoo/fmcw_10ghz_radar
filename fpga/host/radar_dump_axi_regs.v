`timescale 1ns/1ps
// ============================================================================
// radar_dump_axi_regs.v
//
// AXI4-Lite register slave: MicroBlaze <-> radar_frame_dump_streamer control.
//
// *** Proof-of-concept model ***
// The peak / mean / SNR / peak-velocity decision is done in SOFTWARE
// (fusion_control.c) by scanning the region-C summed |X|^2 map that is already
// in DDR — no hardware peak tracker.  This block only:
//   * tells software when a frame's C map is ready and which buffer it is, and
//   * lets software start a one-shot A/B/C/D dump and freeze (halt) the radar
//     pipeline so the chosen frame is not overwritten while it streams.
//
// Register map (32-bit words):
//  Off   Name           Dir  Bits
//  0x00  CTRL           W    [0] start_dump (use DUMP_SEL/FID regs)
//                            [1] clear_newframe (ack the frame-ready flag)
//                            [2] clear_done
//                            [3] set_halt    (freeze radar pipeline)
//                            [4] clear_halt  (resume)
//  0x04  DUMP_SEL       RW   [0] frame_buf_sel  ([1] reserved)
//  0x08  DUMP_FRAME_ID  RW   32-bit tag echoed into the UDP header
//  0x0C  STATUS         R    [0]busy [1]done_sticky [2]new_frame [3]halt
//                            [8]cur_set
// ============================================================================
`default_nettype none

module radar_dump_axi_regs #(
    parameter C_S_AXI_ADDR_WIDTH = 7,
    parameter C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                            s_axi_aclk,
    input  wire                            s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire                            s_axi_awvalid,
    output reg                             s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                            s_axi_wvalid,
    output reg                             s_axi_wready,
    output reg  [1:0]                      s_axi_bresp,
    output reg                             s_axi_bvalid,
    input  wire                            s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire                            s_axi_arvalid,
    output reg                             s_axi_arready,
    output reg  [C_S_AXI_DATA_WIDTH-1:0]   s_axi_rdata,
    output reg  [1:0]                      s_axi_rresp,
    output reg                             s_axi_rvalid,
    input  wire                            s_axi_rready,

    // ── PL frame-ready report (from rd_map_collector_summed.frame_complete) ─
    input  wire        frame_complete,   // 1-cycle pulse: a frame's B/C/D bundle is in DDR
    input  wire        frame_set,        // which B/C/D set it landed in (0/1)

    // ── To radar_frame_dump_streamer ──────────────────────────────────────
    output reg         dump_start,       // 1-cycle pulse
    output reg         dump_frame_buf_sel,
    output reg  [31:0] dump_frame_id,
    input  wire        dump_busy,
    input  wire        dump_done,

    // ── Pipeline freeze (early stop) ──────────────────────────────────────
    output reg         halt              // level: gate the radar frame-start
);
    localparam [1:0] REG_CTRL   = 2'h0;
    localparam [1:0] REG_SEL    = 2'h1;
    localparam [1:0] REG_FID    = 2'h2;
    localparam [1:0] REG_STATUS = 2'h3;

    wire clk   = s_axi_aclk;
    wire rst_n = s_axi_aresetn;
    wire unused_wstrb = |s_axi_wstrb;

    reg [1:0]  reg_sel;        // [0]=frame_buf_sel  ([1] reserved)
    reg [31:0] reg_fid;

    reg        cur_set, new_frame, done_sticky;

    wire [31:0] status_word = {23'd0, cur_set,
                               4'd0, halt, new_frame, done_sticky, dump_busy};

    reg aw_seen, w_seen;
    reg [C_S_AXI_ADDR_WIDTH-1:0] aw_addr_lat;
    reg [C_S_AXI_DATA_WIDTH-1:0] w_data_lat;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0;
            s_axi_bresp   <= 2'b00; s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0; s_axi_rdata  <= 32'd0;
            s_axi_rresp   <= 2'b00; s_axi_rvalid <= 1'b0;
            aw_seen <= 1'b0; w_seen <= 1'b0;
            aw_addr_lat <= 0; w_data_lat <= 0;
            reg_sel <= 2'd0; reg_fid <= 32'd0;
            cur_set <= 1'b0; new_frame <= 1'b0;
            done_sticky <= 1'b0; halt <= 1'b0;
            dump_start <= 1'b0; dump_frame_buf_sel <= 1'b0;
            dump_frame_id <= 32'd0;
        end else begin
            dump_start    <= 1'b0;       // default: pulse low
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_arready <= 1'b0;

            // PL frame-ready latch
            if (frame_complete) begin
                cur_set   <= frame_set;
                new_frame <= 1'b1;
            end

            if (dump_done) done_sticky <= 1'b1;

            // AXI write
            if (s_axi_awvalid && !aw_seen) begin
                s_axi_awready <= 1'b1; aw_addr_lat <= s_axi_awaddr; aw_seen <= 1'b1;
            end
            if (s_axi_wvalid && !w_seen) begin
                s_axi_wready <= 1'b1; w_data_lat <= s_axi_wdata; w_seen <= 1'b1;
            end
            if (aw_seen && w_seen && !s_axi_bvalid) begin
                aw_seen <= 1'b0; w_seen <= 1'b0;
                case (aw_addr_lat[3:2])
                    REG_CTRL: begin
                        if (w_data_lat[1]) new_frame   <= 1'b0;
                        if (w_data_lat[2]) done_sticky <= 1'b0;
                        if (w_data_lat[3]) halt        <= 1'b1;
                        if (w_data_lat[4]) halt        <= 1'b0;
                        if (w_data_lat[0] && !dump_busy) begin       // start_dump
                            dump_start         <= 1'b1;
                            dump_frame_buf_sel <= reg_sel[0];
                            dump_frame_id      <= reg_fid;
                        end
                    end
                    REG_SEL: reg_sel <= w_data_lat[1:0];
                    REG_FID: reg_fid <= w_data_lat;
                    default: ;
                endcase
                s_axi_bresp  <= 2'b00; s_axi_bvalid <= 1'b1;
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;

            // AXI read (single outstanding)
            if (s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1; s_axi_rresp <= 2'b00; s_axi_rvalid <= 1'b1;
                case (s_axi_araddr[3:2])
                    REG_SEL:    s_axi_rdata <= {30'd0, reg_sel};
                    REG_FID:    s_axi_rdata <= reg_fid;
                    REG_STATUS: s_axi_rdata <= status_word;
                    default:    s_axi_rdata <= 32'd0;
                endcase
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
        end
    end

endmodule

`default_nettype wire
