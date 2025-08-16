`timescale 1ns / 1ps

module tb_spi_slave;

    reg clk, rst;
    reg sclk, cs, mosi;
    reg [15:0] data_trans;

    wire miso, done;
    wire [15:0] data_recv;

    // DUT: Device Under Test
    spi_slave uut (
        .clk(clk),
        .rst(rst),
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .data_trans(data_trans),
        .miso(miso),
        .done(done),
        .data_recv(data_recv)
    );

    // 系统时钟 50MHz
    initial clk = 0;
    always #10 clk = ~clk;

    // SPI 信号模拟
    reg [15:0] mosi_data = 16'hA5C3; // 主机要发给从机的数据
    integer i;

    initial begin
        // 初始化
        rst = 0;
        cs = 1;
        sclk = 0;
        mosi = 0;
        data_trans = 16'h3C5A; // 从机要发给主机的数据

        #80;
        rst = 1;

        // 开始通信
        #80;
        cs = 0; // 拉低片选使能通信

        for (i = 0; i < 16; i = i + 1) begin
            // 在 SCLK 上升沿采样 MOSI
            mosi = mosi_data[15 - i];

            #50 sclk = 1; // 上升沿，主设备发送数据
            #50 sclk = 0; // 下降沿，从设备准备发送下一位
        end

        // 通信结束
        #80;
        cs = 1;

        // 检查结果
        #50;
        $display("Data received by slave: %h", data_recv);
        $display("Done signal: %b", done);

        $finish;
    end

endmodule
