`timescale 1ns / 1ps

module single_freq_swap(
    input clk, rst, start,
    input signed [9:0] d_in,
    input [7:0] raddr,

    output reg done,
    output wire signed [13:0] d_out,
    output reg [13:0] freq_read_out, //u14
    output reg [11:0] amp_read_out, //Q12.0
    output reg [11:0] phase_read_out//Q3.9 [-pi, pi]
    );
    integer i;
    // 记录各个频点的幅度相位特性
    reg [11:0] amp_list [0:123];
    reg [11:0] phase_list [0:123];
    
    // 扫描频率表 (提高精度，50点/div, 去除重复频点)
reg [13:0] freq_table [0:123];
initial begin
    freq_table[0] = 5;
    freq_table[1] = 6;
    freq_table[2] = 7;
    freq_table[3] = 8;
    freq_table[4] = 9;
    freq_table[5] = 10;
    freq_table[6] = 11;
    freq_table[7] = 12;
    freq_table[8] = 13;
    freq_table[9] = 14;
    freq_table[10] = 15;
    freq_table[11] = 16;
    freq_table[12] = 17;
    freq_table[13] = 18;
    freq_table[14] = 19;
    freq_table[15] = 20;
    freq_table[16] = 21;
    freq_table[17] = 22;
    freq_table[18] = 23;
    freq_table[19] = 24;
    freq_table[20] = 25;
    freq_table[21] = 26;
    freq_table[22] = 27;
    freq_table[23] = 28;
    freq_table[24] = 29;
    freq_table[25] = 30;
    freq_table[26] = 32;
    freq_table[27] = 34;
    freq_table[28] = 36;
    freq_table[29] = 38;
    freq_table[30] = 40;
    freq_table[31] = 42;
    freq_table[32] = 44;
    freq_table[33] = 46;
    freq_table[34] = 48;
    freq_table[35] = 50;
    freq_table[36] = 52;
    freq_table[37] = 54;
    freq_table[38] = 56;
    freq_table[39] = 58;
    freq_table[40] = 60;
    freq_table[41] = 65;
    freq_table[42] = 70;
    freq_table[43] = 75;
    freq_table[44] = 80;
    freq_table[45] = 85;
    freq_table[46] = 90;
    freq_table[47] = 95;
    freq_table[48] = 100;
    freq_table[49] = 110;
    freq_table[50] = 120;
    freq_table[51] = 130;
    freq_table[52] = 140;
    freq_table[53] = 150;
    freq_table[54] = 160;
    freq_table[55] = 170;
    freq_table[56] = 180;
    freq_table[57] = 190;
    freq_table[58] = 200;
    freq_table[59] = 210;
    freq_table[60] = 220;
    freq_table[61] = 230;
    freq_table[62] = 240;
    freq_table[63] = 250;
    freq_table[64] = 260;
    freq_table[65] = 270;
    freq_table[66] = 280;
    freq_table[67] = 290;
    freq_table[68] = 300;
    freq_table[69] = 320;
    freq_table[70] = 340;
    freq_table[71] = 360;
    freq_table[72] = 380;
    freq_table[73] = 400;
    freq_table[74] = 420;
    freq_table[75] = 440;
    freq_table[76] = 460;
    freq_table[77] = 480;
    freq_table[78] = 500;
    freq_table[79] = 550;
    freq_table[80] = 600;
    freq_table[81] = 650;
    freq_table[82] = 700;
    freq_table[83] = 750;
    freq_table[84] = 800;
    freq_table[85] = 850;
    freq_table[86] = 900;
    freq_table[87] = 950;
    freq_table[88] = 1000;
    freq_table[89] = 1100;
    freq_table[90] = 1200;
    freq_table[91] = 1300;
    freq_table[92] = 1400;
    freq_table[93] = 1500;
    freq_table[94] = 1600;
    freq_table[95] = 1700;
    freq_table[96] = 1800;
    freq_table[97] = 1900;
    freq_table[98] = 2000;
    freq_table[99] = 2200;
    freq_table[100] = 2400;
    freq_table[101] = 2600;
    freq_table[102] = 2800;
    freq_table[103] = 3000;
    freq_table[104] = 3200;
    freq_table[105] = 3400;
    freq_table[106] = 3600;
    freq_table[107] = 3800;
    freq_table[108] = 4000;
    freq_table[109] = 4200;
    freq_table[110] = 4400;
    freq_table[111] = 4600;
    freq_table[112] = 4800;
    freq_table[113] = 5000;
    freq_table[114] = 5500;
    freq_table[115] = 6000;
    freq_table[116] = 6500;
    freq_table[117] = 7000;
    freq_table[118] = 7500;
    freq_table[119] = 8000;
    freq_table[120] = 8500;
    freq_table[121] = 9000;
    freq_table[122] = 9500;
    freq_table[123] = 10000;
end

    // 地址读取
    always @(posedge clk) begin
        freq_read_out <= freq_table[raddr];
        amp_read_out <= amp_list[raddr];
        phase_read_out <= phase_list[raddr];
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
    always @(*) begin
        next_state = state;
        case (state)
        IDLE: next_state = WAIT_START;
        WAIT_START: if (start) next_state = UNIT_START;
        UNIT_START: next_state = WAIT_UNIT;
        WAIT_UNIT: if (single_unit_done) next_state = NEXT_STEP;
        NEXT_STEP: begin if (step >= 123) next_state = DONE;  else next_state = UNIT_START; end
        DONE: next_state = IDLE;
        default: next_state = IDLE;
    endcase
end
    // 状态输出
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            single_unit_start <= 0; control_freq <= 0; wait_response_delay_us <= 0;
            step <= 0;
            done <= 0;
            for (i=0; i<=123; i=i+1) begin
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
                control_freq <= freq_table[step];
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
