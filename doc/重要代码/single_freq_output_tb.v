`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/01/2025 02:45:38 PM
// Design Name: 
// Module Name: single_freq_output_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module single_freq_output_tb();
    reg clk, rst, en;
    reg [15:0] freq, vpp;

    wire [13:0] sin_out;

    single_freq_output u_single_freq_output(
        .clk  ( clk  ),
        .rst  ( rst  ),
        .en   ( en   ),
        .freq ( freq ),
        .vpp  ( vpp  ),
        .sin_out  ( sin_out  )
    );

    initial begin
        clk = 0;
        rst = 0;
        en = 1;
        freq = 50;
        vpp = 50;

        #100
        rst = 1;
    end 

    always #10 clk=~clk;
    always #10000 freq = freq+10;
    always #10000 vpp = vpp+10;

endmodule
