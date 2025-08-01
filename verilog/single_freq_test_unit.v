`timescale 1ns / 1ps

module single_freq_test_unit(
    input clk, rst, start,
    input [13:0]freq,
    input signed [9:0] d_in,
    input [15:0] wait_response_delay_us,

    output signed [9:0] d_out,
    output reg done,
    output reg [11:0] amp, phase
    );
    reg signed [9:0] locked_I, locked_Q;

    // 例化一个iir核心用作相关扫频
    wire signed [9:0] I, Q;
    single_freq_synth_core u_single_freq_synth_core(
        .clk     ( clk     ),
        .rst     ( rst     ),
        .ctrl    ( 0       ),
        .freq    ( freq    ),
        .in_data ( d_in    ),
        .I_o     ( 0       ),
        .Q_o     ( 0       ),
        .I       ( I       ),
        .Q       ( Q       ),
        .out_data  ( d_out  )
    );

    // 计时器延迟，用于等待响应建立
    reg delay_start, delay_over, delay_busy;
    reg [31:0] delay_cnt;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            delay_cnt <= 0;
            delay_over <= 0;
            delay_busy <= 0;
        end else if (delay_over) begin
            delay_cnt <= 0;
            delay_over <= 0;
            delay_busy <= 0;
        end else begin
            if (delay_start) delay_busy<=1;
            if (delay_busy) begin
                delay_cnt <= delay_cnt+1;
                if (delay_cnt >= wait_response_delay_us*50) delay_over<=1;
            end
        end
    end

    // 两个cordic，用于计算arctan和square root
    reg cordic_start;
    wire cordic1_done, cordic2_done;
    // arctan，输入10位，输出Q3.9有符号弧度制[-pi, pi]
    wire [31:0] cordic_arctan_input;
    wire [11:0]  cordic_arctan_output;
    assign cordic_arctan_input = {{6{locked_Q[9]}}, locked_Q , {6{locked_I[9]}}, locked_I};
    cordic_arctan cordic_arctan0 (
        .aclk(clk),                                        // input wire aclk
        .s_axis_cartesian_tvalid(cordic_start),  // input wire s_axis_cartesian_tvalid
        .s_axis_cartesian_tdata(cordic_arctan_input),    // input wire [31 : 0] s_axis_cartesian_tdata
        .m_axis_dout_tvalid(cordic1_done),            // output wire m_axis_dout_tvalid
        .m_axis_dout_tdata(cordic_arctan_output)               // output wire [15 : 0] m_axis_dout_tdata
    );
    // sqaure root，输入21位整型，输出11位整型
    wire [23:0] cordic_square_root_input;
    wire [10:0] cordic_square_root_output;
    assign cordic_square_root_input = locked_I*locked_I + locked_Q*locked_Q;
    cordic_square_root cordic_square_root0 (
        .aclk(clk),                                        // input wire aclk
        .s_axis_cartesian_tvalid(cordic_start),  // input wire s_axis_cartesian_tvalid
        .s_axis_cartesian_tdata(cordic_square_root_input),    // input wire [23 : 0] s_axis_cartesian_tdata
        .m_axis_dout_tvalid(cordic2_done),            // output wire m_axis_dout_tvalid
        .m_axis_dout_tdata(cordic_square_root_output)              // output wire [15 : 0] m_axis_dout_tdata
    );

    // 同步两个cordic的完成
    reg cordic1_done_reg, cordic2_done_reg, cordic_all_done;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            cordic1_done_reg <= 0;
            cordic2_done_reg <= 0;
            cordic_all_done <= 0;
        end else begin
            if (cordic_all_done) begin
                cordic1_done_reg <= 0;
                cordic2_done_reg <= 0;
                cordic_all_done <= 0;
            end else if (cordic1_done_reg & cordic2_done_reg) cordic_all_done <= 1;
            else begin
                if (cordic1_done) cordic1_done_reg<=1;
                if (cordic2_done) cordic2_done_reg<=1;
            end
        end
    end

    // 扫频控制状态机
    // 状态定义
    localparam IDLE             = 0;
    localparam WAIT_START       = 1;
    localparam SYS_START        = 2;
    localparam WAIT_DELAY       = 3;
    localparam LOCK_IQ          = 4;
    localparam START_CORDIC     = 5;
    localparam WAIT_CORDIC      = 6;
    localparam SYS_DONE         = 7;
    reg [2:0] state, next_state;

    // 下一状态
    always @(posedge clk or negedge rst)
        if (!rst) state <= IDLE;
        else state <= next_state;
    
    // 状态转移
    always @(*) case (state)
        IDLE: next_state=WAIT_START;
        WAIT_START: if (start) next_state=SYS_START;
        SYS_START: next_state = WAIT_DELAY;
        WAIT_DELAY: if (delay_over) next_state = LOCK_IQ;
        LOCK_IQ: next_state = START_CORDIC;
        START_CORDIC: next_state = WAIT_CORDIC;
        WAIT_CORDIC: if (cordic_all_done) next_state=SYS_DONE;
        SYS_DONE:   next_state=IDLE;
        default: next_state = IDLE;
    endcase

    // 状态输出
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            delay_start <= 0; cordic_start <= 0;
            locked_I <= 0; locked_Q <= 0;
            done <= 0;
            amp <= 0; phase <= 0;
        end else case (state)
            IDLE: begin
                delay_start <= 0; cordic_start <= 0;
                locked_I <= 0; locked_Q <= 0;
                done <= 0;
            end

            WAIT_START: ;
            SYS_START: delay_start <= 1;
            WAIT_DELAY: delay_start <= 0;
            LOCK_IQ: begin
                locked_I <= I;
                locked_Q <= Q;
            end
            START_CORDIC: cordic_start <= 1;
            WAIT_CORDIC: cordic_start <= 0;
            SYS_DONE:   begin
                done <= 1;
                amp <= cordic_square_root_output;
                phase <= cordic_arctan_output;
            end
            default: begin
                delay_start <= 0; cordic_start <= 0;
                locked_I <= 0; locked_Q <= 0;
                done <= 0;
            end
        endcase
    end

endmodule
