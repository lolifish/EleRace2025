`timescale 1ns / 1ps

module single_freq_swap(
    input clk, rst, start,
    input signed [9:0] d_in,
    input [7:0] raddr,

    output reg done,
    output wire signed [9:0] d_out,
    output reg [11:0] amp_read_out, //Q12.0
    output reg [11:0] phase_read_out//Q3.9 [-pi, pi]
    );
    integer i;
    // 记录各个频点的幅度相位特性
    reg [11:0] amp_list [0:32];
    reg [11:0] phase_list [0:32];

    // 地址读取
    always @(*) begin
        amp_read_out = amp_list[raddr];
        phase_read_out = phase_list[raddr];
    end
    
    // 扫频器
    reg single_unit_start;
    reg [15:0] wait_response_delay_us;
    reg [13:0] control_freq;

    wire [11:0] amp_out, phase_out;
    wire single_unit_done;
    
    single_freq_test_unit u_single_freq_test_unit(
        .clk                    ( clk                    ),
        .rst                    ( rst                    ),
        .start                  ( single_unit_start      ),
        .freq                   ( control_freq           ),
        .d_in                   ( d_in                   ),
        .wait_response_delay_us ( wait_response_delay_us ),
        .d_out                  ( d_out                  ),
        .done                   ( single_unit_done       ),
        .amp                    ( amp_out                ),
        .phase                  ( phase_out              )
    );

    // 扫描频率表
    reg [31:0] freq_table [0:32];
    initial begin
        freq_table[0] = 500;
        freq_table[1] = 600;
        freq_table[2] = 700;
        freq_table[3] = 800;
        freq_table[4] = 900;

        freq_table[5] = 1000;
        freq_table[6] = 2000;
        freq_table[7] = 3000;
        freq_table[8] = 4000;
        freq_table[9] = 5000;
        freq_table[10] = 6000;
        freq_table[11] = 7000;
        freq_table[12] = 8000;
        freq_table[13] = 9000;

        freq_table[14] = 10000;
        freq_table[15] = 20000;
        freq_table[16] = 30000;
        freq_table[17] = 40000;
        freq_table[18] = 50000;
        freq_table[19] = 60000;
        freq_table[20] = 70000;
        freq_table[21] = 80000;
        freq_table[22] = 90000;

        freq_table[23] = 100000;
        freq_table[24] = 200000;
        freq_table[25] = 300000;
        freq_table[26] = 400000;
        freq_table[27] = 500000;
        freq_table[28] = 600000;
        freq_table[29] = 700000;
        freq_table[30] = 800000;
        freq_table[31] = 900000;
        freq_table[32] = 1000000;
    end

    reg [7:0] step;
    // 状态机
    // 状态定义
    localparam IDLE         = 1;
    localparam WAIT_START   = 2;
    localparam UNIT_START   = 3;
    localparam WAIT_UNIT    = 4;
    localparam NEXT_STEP    = 5;
    localparam DONE         = 6;
    reg [3:0] state, next_state;

    // 下一状态
    always @(posedge clk or negedge rst)
        if (!rst) state <= IDLE;
        else state <= next_state;
    
    // 状态转移
    always @(*) case (state)
        IDLE: next_state = WAIT_START;
        WAIT_START: if (start) next_state = UNIT_START;
        UNIT_START: next_state = WAIT_UNIT;
        WAIT_UNIT: if (single_unit_done) next_state = NEXT_STEP;
        NEXT_STEP: begin if (step >= 32) next_state = DONE;  else next_state = UNIT_START; end
        DONE: next_state = IDLE;
        default: next_state = IDLE;
    endcase

    // 状态输出
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            single_unit_start <= 0; control_freq <= 0; wait_response_delay_us <= 0;
            step <= 0;
            done <= 0;
            for (i=0; i<=32; i=i+1) begin
                amp_list[i] <= 0;
                phase_list[i] <= 0;
            end
        end else 
        case (state)
            IDLE: begin
                single_unit_start <= 0; control_freq <= 0; wait_response_delay_us <= 0;
                step <= 0;
                done <= 0;
            end
            WAIT_START: ;

            UNIT_START: begin
                single_unit_start <= 1;
                control_freq <= freq_table[step] / 100;
                if (step <= 10) wait_response_delay_us<=10000;
                else wait_response_delay_us<=5000;
            end
            WAIT_UNIT: begin
                single_unit_start <= 0;
            end
            NEXT_STEP: begin
                amp_list[step] = amp_out;
                phase_list[step] = phase_out;
                step <= step+1;
            end
            DONE: begin
                done <= 1;
            end


            default: begin
                single_unit_start <= 0; control_freq <= 0; wait_response_delay_us <= 0;
                step <= 0;
                done <= 0;
            end

        endcase
    end

endmodule
