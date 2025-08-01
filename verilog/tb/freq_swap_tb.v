`timescale 1ns / 1ps

module freq_swap_tb();
    reg clk, rst, start;
    reg signed [9:0] d_in;
    reg [7:0] raddr;

    wire done;
    wire signed [9:0] d_out;
    wire [11:0] amp_read_out, phase_read_out;

    single_freq_swap u_single_freq_swap(
        .clk          ( clk          ),
        .rst          ( rst          ),
        .start        ( start        ),
        .d_in         ( d_in         ),
        .raddr        ( raddr        ),

        .done         ( done         ),
        .d_out        ( d_out        ),
        .amp_read_out ( amp_read_out ),
        .phase_read_out  ( phase_read_out  )
    );

    initial begin
        clk=0;
        start=0;
        rst=0;
        raddr=0;

        #100
        rst=1;

        #1000
        start = 1;
        #20
        start = 0;
    end

    always @(*) d_in=d_out/2;

    always #10 clk=~clk;



endmodule
