`timescale 1ns / 1ps

module single_freq_output(
    input clk, rst, en,
    input [15:0] freq,  // 除以25的频率
    input [15:0] vpp,   // 峰峰值Q2.14 无符号定点幅度，单位V

    output reg signed [13:0] sin_out
    );

    // 例化dds
    wire signed [13:0] dds_out;
    dds_new#(
        .phase ( 0 )
    )u_dds_new(
        .clk  ( clk  ),
        .freq ( freq ),
        .dds_o  ( dds_out  )
    );

    // 幅度变换
    always @(posedge clk or negedge rst) begin
        if (!rst) sin_out <= 0;
        else if (!en) sin_out <= 0;
        else sin_out <= dds_out*$signed({1'b0, vpp})/61307;
    end

endmodule
