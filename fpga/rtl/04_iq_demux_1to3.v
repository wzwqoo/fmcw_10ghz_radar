`timescale 1ns/1ps
`default_nettype none

// *** 3-RX VARIANT: demuxes the TDM I/Q stream into 3 per-RX channels. ***
module iq_demux_1to3 #(
    parameter IQ_W = 16,
    parameter DATA_W = 32
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire [IQ_W-1:0]      i_tdm_data,
    input  wire [IQ_W-1:0]      q_tdm_data,
    input  wire [1:0]           i_tdm_ch_id,
    input  wire                 i_tdm_valid,
    input  wire                 q_tdm_valid,

    output reg  [DATA_W-1:0]    ch0_tdata,
    output reg                  ch0_tvalid,
    output reg  [DATA_W-1:0]    ch1_tdata,
    output reg                  ch1_tvalid,
    output reg  [DATA_W-1:0]    ch2_tdata,
    output reg                  ch2_tvalid
);

    wire iq_valid = i_tdm_valid & q_tdm_valid;
    wire [DATA_W-1:0] iq_word;

    generate
        if (DATA_W == 2*IQ_W) begin : g_no_pad
            assign iq_word = {q_tdm_data, i_tdm_data};
        end else begin : g_pad
            assign iq_word = {{(DATA_W-(2*IQ_W)){1'b0}}, q_tdm_data, i_tdm_data};
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch0_tdata  <= {DATA_W{1'b0}};
            ch1_tdata  <= {DATA_W{1'b0}};
            ch2_tdata  <= {DATA_W{1'b0}};
            ch0_tvalid <= 1'b0;
            ch1_tvalid <= 1'b0;
            ch2_tvalid <= 1'b0;
        end else begin
            ch0_tvalid <= 1'b0;
            ch1_tvalid <= 1'b0;
            ch2_tvalid <= 1'b0;

            if (iq_valid) begin
                case (i_tdm_ch_id)
                    2'd0: begin ch0_tdata <= iq_word; ch0_tvalid <= 1'b1; end
                    2'd1: begin ch1_tdata <= iq_word; ch1_tvalid <= 1'b1; end
                    2'd2: begin ch2_tdata <= iq_word; ch2_tvalid <= 1'b1; end
                    default: ;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
