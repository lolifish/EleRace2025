
module iir_core_tb();

    reg clk, rst, ctrl;
    reg [13:0] freq;
    reg signed [9:0] in_data;
    reg signed [9:0] I_o, Q_o;

    wire signed [9:0] I, Q, out_data;

    single_freq_synth_core u_single_freq_synth_core(
        .clk     ( clk     ),
        .rst     ( rst     ),
        .ctrl    ( ctrl    ),
        .freq    ( freq    ),
        .in_data ( in_data ),
        .I_o     ( I_o     ),
        .Q_o     ( Q_o     ),

        .I       ( I       ),
        .Q       ( Q       ),
        .out_data  ( out_data  )
    );

    initial begin
        clk=0;
        ctrl=0;
        freq=10;
        I_o=0;
        Q_o=0;
        rst=0;
        #100
        rst=1;
    end

    always @(*) in_data=out_data;

    always #10 clk=~clk;
    always #10000 freq=freq+10;

endmodule
