

module freq_get(
input clk,
input wire signed [9:0]in_data,
input wire signed [9:0]th,
output wire [13:0]freq
);


reg comp=0;

always@(posedge clk)begin
    if(comp==0&&in_data>th)begin
        comp<=1;
    end else if(comp&&in_data<-th)begin
        comp<=0;
    end
end

wire [13:0]freq1;


freq_getf fdc1(
.clk     (clk ),
.signal  (comp),
.freq    (freq1)
);

assign freq = freq1;

endmodule


module freq_getf (
    input clk,              // FPGA系统时钟50MHz
    input signal,           // 输入待测信号
    output reg [13:0] freq =0 // 输出频率（单位Hz）
);
    // 定义变量
    reg [23:0] count=0;          // 用于计数输入信号的周期数
    reg [23:0] time_counter=0;   // 用于计时
    reg signal_d=0;                // 输入信号的延迟信号（用于边沿检测）
    reg signal_rising_edge=0;      // 记录信号的上升沿
    reg [23:0] period=0;         // 测量的周期数
    reg [23:0] prev_time=0;      // 上一次计时的值
    
    // 定义测量周期的时间窗（20ms）
    localparam TIME_WINDOW = 5000000;  // 50MHz时钟下20ms的计数值

    always @(posedge clk) begin
        // 记录信号的上升沿
        signal_d <= signal;
        signal_rising_edge <= (signal & ~signal_d);  // 上升沿检测
        // 时间计数器
        if (time_counter < TIME_WINDOW) begin
            time_counter <= time_counter + 1;
        end else begin
            // 在20ms时刻清零，开始新的测量
            time_counter <= 0;
            period <= count;  // 保存周期数
            count <= 0;       // 重置计数器
        end

        // 信号计数
        if (signal_rising_edge) begin
            count <= count + 1;
        end
    end

    // 计算频率
    always @(posedge clk) begin
        if (time_counter == 0 && period > 0) begin
            // 计算频率: 频率 = 周期数 / 计时窗口时间
            // 由于时间窗口为20ms，50MHz的时钟周期为20ms / 50MHz = 1秒
            freq <= (period+2)/10;
        end
    end

endmodule

