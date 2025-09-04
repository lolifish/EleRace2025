
module harmonic_rebuild(
input clk,
input prog,
input rst,
input en,
input [3:0]num,
input [13:0]Base_freq,
input signed [9:0]I_in,
input signed [9:0]Q_in,
input wire [9:0]in_data,
output reg [13:0]out_data
);

reg [13:0]freq[0:10];
reg signed [9:0]I_o[0:10];
reg signed [9:0]Q_o[0:10];

integer j;

initial begin
    for(j=0;j<11;j=j+1)begin
        freq[j]=0;
        I_o[j]=0;
        Q_o[j]=0;
    end
end

genvar i;

wire signed [13:0]dout[0:10];

generate

for(i=0;i<11;i=i+1)begin
single_freq_synth_core core(
.clk(clk),
.rst(rst),
.ctrl(1),
.freq(freq[i]),
.in_data(in_data),
.I_o(I_o[i]),
.Q_o(Q_o[i]),
.I(),
.Q(),
.out_data(dout[i])
); 
end
endgenerate

always@(posedge clk)begin
    if(prog&&(num<11))begin
        freq[num]<=Base_freq*(num+1);
        I_o[num]<=I_in;
        Q_o[num]<=Q_in;
    end
end

reg signed [14:0]add[0:3];
wire signed [14:0]add_w[0:3];

assign add_w[0] = dout[0]+dout[1]+dout[2];
assign add_w[1] = dout[3]+dout[4]+dout[5]+dout[6];
assign add_w[2] = dout[7]+dout[8]+dout[9]+dout[10];
assign add_w[3] = add[0]+add[1]+add[2];

always@(posedge clk)begin
    add[0]<=(add_w[0]>-8190 && add_w[0]<8192)?add_w[0]:add_w[0][14]?-8190:8191; 
    add[1]<=(add_w[1]>-8190 && add_w[1]<8192)?add_w[1]:add_w[1][14]?-8190:8191; 
    add[2]<=(add_w[2]>-8190 && add_w[2]<8192)?add_w[2]:add_w[2][14]?-8190:8191;
    add[3]<=(add_w[3]>-8190 && add_w[3]<8192)?add_w[3]:add_w[3][14]?-8190:8191;
    out_data<=en?add[3]:0;
end


endmodule

