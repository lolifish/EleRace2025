`timescale 1ns/1ns

module tb;

reg clk=1;
reg signed [9:0]ad=0;
wire [13:0]da;
wire nsysclk;
wire ddsclk = ~nsysclk;
reg key=0;
reg SCK =0;
reg CS  =1;
wire MOSI;
wire MISO;
reg [15:0]send_data=0;
wire [15:0]recv_data;
reg [14:0]vpp = 30000;

top #(
.DEBUG(1)
)uut(
.clk    (clk   ),
.ad     (ad    ),
.da     (da    ),
.ad_clk (),
.da_clk (nsysclk),
.PD     (),
.CTRL   (),
.key    (key   ),
.SCK    (SCK   ),
.CS     (CS    ),
.MOSI   (MOSI  ),
.MISO   (MISO  )
);

spi spi_m(
.clk        (clk      ),
.SCK        (SCK      ),
.CS         (CS       ),
.MOSI       (MISO     ),
.MISO       (MOSI     ),
.send_data  (send_data),
.recv_data  (recv_data)
);

wire signed [9:0]dds_to;
wire signed [9:0]squa = (dds_to > 300)?250:-250;

dds_p dds_fortest(
.clk    (ddsclk),
.freq   (10),
.phase  (0),
.dds_o  (dds_to)
);

//always@(posedge clk) ad <= {~da[13],da[12:4]};

always@(posedge clk) ad <= squa;

always #10 clk=~clk;

integer i=0,j=0;

initial begin
      
    #1000;
    key=1;
    
    #2000140;
    send_data = 16'h4;
    #20;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #160;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #160;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #140;
    send_data = 16'hF004;
    #20;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
for(j=0;j<11;j=j+1)begin
    #140;
    send_data = {j[3:0],2'b0,10'hff};
    #20;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #140;
    send_data = {j[3:0],2'b0,10'h00};
    #20;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
 end 
    #160;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #160;
    CS=0;
    for(i=0;i<16;i=i+1)begin
          #160;
          SCK=1;
          #160;
          SCK=0;
    end
    #160;
    CS=1;
    #1000;
    #20000000;
    $finish;

//    #140;
//    send_data = 16'h2;
//    #20;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #160;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #200000000;
//    for(i=0;i<100;i=i+1)begin
//        #160;
//        CS=0;
//        for(i=0;i<16;i=i+1)begin
//              #160;
//              SCK=1;
//              #160;
//              SCK=0;
//        end
//        #160;
//        CS=1;
//    end

//    send_data = 16'h1;
//    #20;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #140;
//    send_data = {1'b1,vpp};
//    #20;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #140;
//    send_data = 10;
//    #20;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #140;
//    send_data = 16'h0;
//    #20;
//    CS=0;
//    for(i=0;i<16;i=i+1)begin
//          #160;
//          SCK=1;
//          #160;
//          SCK=0;
//    end
//    #160;
//    CS=1;
//    #1000;
    $finish;
end



endmodule