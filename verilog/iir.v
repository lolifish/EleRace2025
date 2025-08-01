module iir #(
parameter N = 14,
parameter M = 2
)(
input clk, rst,
input signed [9:0]in_data,
output wire signed [9:0]out_data
);

reg signed [23:0]cache[0:1];
reg signed [23:0]sum;
wire signed [9:0]cut = cache[0]>>>14; 
always@(posedge clk or negedge rst)begin
    if (!rst) begin
        cache[0] <= 0; cache[1] <= 0;
        sum <= 0;
    end else begin
        cache[0]<=cache[0]-cut+in_data;
        cache[1]<=cache[0];
        sum<=(cache[0]+cache[1])/M;
    end
end

assign out_data = sum[23:14];

endmodule