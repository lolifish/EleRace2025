`timescale 1ns / 1ps

module single_freq_test_unit_tb();

    reg clk, rst, start;
    reg signed [9:0] d_in;

    wire signed [9:0] d_out;
    wire done;
    wire [11:0] amp, phase;

    single_freq_test_unit u_single_freq_test_unit(
        .clk                    ( clk                    ),
        .rst                    ( rst                    ),
        .start                  ( start                  ),
        .freq                   ( 5                   ),
        .d_in                   ( d_in                   ),
        .wait_response_delay_us ( 10000                    ),
        .d_out                  ( d_out                  ),
        .done                   ( done                   ),
        .amp                    ( amp                    ),
        .phase                  ( phase                  )
    );

    initial begin
        clk=0;
        start=0;
        rst=0;

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
