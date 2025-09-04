module spi(
input wire clk,
input wire SCK,
input wire CS,
input wire MOSI,
output reg MISO=0,
output wire done,
input wire [15:0]send_data,
output reg [15:0]recv_data=0
);

reg [4:0]cnt=0;

reg [1:0]CS_r=0;
reg [1:0]MOSI_r=0;
reg [1:0]SCK_r=0;
reg flag=0;

always@(posedge clk)begin
    CS_r<={CS_r[0],CS};
    MOSI_r<={MOSI_r[0],MOSI};
    SCK_r<={SCK_r[0],SCK};
end

wire SCK_up = SCK_r == 2'b01;
wire SCK_dn = SCK_r == 2'b10;
assign done = (CS_r == 2'b01)&&flag;

reg [15:0]send_buf=0;

always@(posedge clk)begin
    if(CS_r[1] || cnt == 16)begin
        cnt<=0;
        MISO<=0;
        send_buf<=send_data;
    end else if(SCK_dn)begin
        cnt<=cnt+1;
        MISO<=send_buf[15];
        send_buf<={send_buf[14:0],1'b0};
    end
    MISO<=send_buf[15];
end

reg [15:0]recv_buf=0;
reg [4:0]bit_cnt=0;

always@(posedge clk)begin
    if(!CS_r[1])begin
        if(SCK_up&&(!flag))begin
            recv_buf<={recv_buf[15:0],MOSI_r[1]};
            bit_cnt<=bit_cnt+1;
            if(bit_cnt==15)
                flag<=1;
        end 
    end else begin
        if(flag==1)begin
            flag<=0;
            recv_data<=recv_buf;
        end
        bit_cnt<=0;
        recv_buf<=0;
    end
        
end

endmodule