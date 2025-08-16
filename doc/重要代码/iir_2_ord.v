module iir_2_ord(
input clk, rst,
input signed [9:0]in_data,
output wire signed [9:0]out_data
);

wire signed [9:0]mid_sig;

iir iir0(
.clk(clk),
.rst(rst),
.in_data(in_data),
.out_data(mid_sig)
);

iir iir1(
.clk(clk),
.rst(rst),
.in_data(mid_sig),
.out_data(out_data)
);

endmodule