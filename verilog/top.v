module top (
input clk,
input wire signed [9:0]ad,
output reg [13:0]da,
output wire ad_clk,
output wire da_clk,
output wire PD,
output wire [2:0]CTRL,
input wire key,
input wire SCK ,
input wire CS  ,
input wire MOSI,
output wire MISO,
output wire LED
);

//clk

wire sysclk;
wire rst = key;

clk_wiz_0 pll0(
.clk_in1(clk),
.clk_out1(sysclk),
.clk_out2(ad_clk)
);

//spi

reg [15:0]send_data=0;
wire [15:0]recv_data;
reg done;
reg [1:0]done_reg=0;
wire done_r;

always@(posedge clk)begin
    done_reg<={done_reg[0],done_r};
    done<=done_reg[1];
end

spi spi0(
.clk       (sysclk   ),
.SCK       (SCK      ),
.CS        (CS       ),
.MOSI      (MOSI     ),
.MISO      (MISO     ),
.done      (done_r   ),
.send_data (send_data),
.recv_data (recv_data)
);

//INPUT BUF

reg signed [9:0] in_r;

always@(posedge sysclk)begin
    in_r<=ad;
end

//DDS single freq

wire signed [13:0]sin_out;
reg sfo_state=0;
reg [15:0]freq=0;
reg [14:0]vpp=0;

single_freq_output sfo0( 
.clk      (sysclk ),
.rst      (rst    ),
.en       (sfo_state ),
.freq     (freq   ),
.vpp      (vpp    ),
.sin_out  (sin_out)
);

// swap

reg [7:0]raddr=0;
reg swap_state=0;
reg swap_start=0;
wire signed [13:0]swap_o;
wire [13:0] freq_read_out ;
wire [11:0] amp_read_out  ;
wire [11:0] phase_read_out;
wire swap_done;

single_freq_swap swap0(
.clk             (sysclk        ),
.rst             (rst           ),
.start           (swap_start    ),
.d_in            (in_r          ),
.raddr           (raddr         ),
.done            (swap_done     ),
.d_out           (swap_o        ),
.freq_read_out   (freq_read_out ),
.amp_read_out    (amp_read_out  ),
.phase_read_out  (phase_read_out)
);

//freq_get

parameter DEBUG = 0;

wire [13:0]hrb_freq;
freq_get get0(
.clk(sysclk),
.in_data(in_r),
.th(100),
.freq(hrb_freq)
);

//hrb 

wire signed [13:0]hrb_out;
reg prog=0;
reg [3:0]har_num=0;
reg hrb_state=0;
reg signed[9:0]I,Q;

harmonic_rebuild hrb0(
.clk(sysclk),
.prog(prog),
.rst(rst),
.en(hrb_state),
.num(har_num),
.Base_freq(DEBUG?10:hrb_freq),
.I_in(I),
.Q_in(Q),
.in_data(in_r),
.out_data(hrb_out)
);

//Compute

reg signed [13:0] out_r;

always@(posedge sysclk)begin
    out_r <= sfo_state ? sin_out : swap_state ? swap_o : hrb_state ? hrb_out : 0;
end

//OUTPUT BUF

always@(posedge sysclk)begin
    da <= {~out_r[13],out_r[12:0]};
end

//State Machine

parameter spi_idle                  = 0;
parameter spi_wait                  = 1;
parameter spi_sfo_wait_vpp          = 2;
parameter spi_sfo_wait_freq         = 3;
parameter spi_swap_wait_start       = 4;
parameter spi_swap_wait_done        = 5;
parameter spi_swap_wait_transmit    = 6;
parameter spi_swap_wait_send_end    = 7;
parameter spi_hrb_wait_send_freq    = 8;
parameter spi_hrb_wait_get_state    = 9;
parameter spi_hrb_wait_get_I        = 10;
parameter spi_hrb_wait_get_Q        = 11;


reg [3:0]spi_state=0;
reg without_spi_trig=0;
reg [1:0]send_cnt=0;
wire get_sfo_state  = recv_data == 16'h1;
wire get_swap_state = recv_data == 16'h2;
reg done_flag=0;
reg [3:0]hrb_recv_cnt=0;

always@(posedge sysclk or negedge rst)begin
    if(!rst)begin
        send_data<=0;
        sfo_state<=0;
        swap_state<=0;
        freq<=0;
        spi_state<=0;
        hrb_state <= 0;
        without_spi_trig<=1;
    end else if(done || without_spi_trig) begin
        case(spi_state)
        
            spi_idle:begin
                spi_state <= spi_wait;
                without_spi_trig<=0;
            end
            
            spi_wait:begin 
                if(recv_data == 16'h1)begin
                    spi_state <= spi_sfo_wait_vpp;
                end else if(recv_data == 16'h2)begin
                    swap_state<= 1;
                    spi_state <= spi_swap_wait_start;
                    without_spi_trig<=1;
                    swap_start<= 1;
                    send_data <= 16'h2;
                end else if(recv_data == 16'h4)begin
                    hrb_state <= 1;
                    spi_state <= spi_hrb_wait_send_freq;
                    send_data <= {2'b0,hrb_freq};
                end
            end
            
            spi_sfo_wait_vpp:begin
                sfo_state<=recv_data[15];
                vpp<=recv_data[14:0];
                spi_state <= spi_sfo_wait_freq;
            end
            
            spi_sfo_wait_freq:begin
                freq <= recv_data;
                spi_state <= spi_wait;
            end
            
            spi_swap_wait_start:begin
                swap_start<=0;
                spi_state <= spi_swap_wait_done ;
            end
            
            spi_swap_wait_done:begin
                if(swap_done)begin
                    done_flag<=1;
                    raddr<=0;
                    send_cnt<=0;
                    without_spi_trig<=0;
                end 
                if(done_flag)begin
                    done_flag<=0;
                    spi_state <= spi_swap_wait_transmit;
                    send_data<=16'hf002;
                end
            end
            
            spi_swap_wait_transmit:begin
            
                case(send_cnt)
                    0:send_data<={2'b0,freq_read_out};
                    1:send_data<={4'b0,amp_read_out};
                    2:send_data<={phase_read_out,4'b0};
                    default:;
                endcase
                
                if(send_cnt<2)
                    send_cnt<=send_cnt+1;
                else begin
                    send_cnt<=0;
                    if(raddr<123)begin
                        raddr<=raddr+1;
                    end else begin
                        raddr<=0;
                        spi_state <= spi_swap_wait_send_end;
                    end 
                end
            end
            
            spi_swap_wait_send_end:begin
                send_data <= 16'hfffd;
                swap_state<= 0;
                spi_state <= spi_wait ;
            end
            
            spi_hrb_wait_send_freq:begin
                spi_state <= spi_hrb_wait_get_state;
            end
            
            spi_hrb_wait_get_state:begin
                if(recv_data == 16'hF004)begin
                    spi_state <= spi_hrb_wait_get_I;
                    hrb_recv_cnt<=0;
                end else if(recv_data == 16'hF008)begin
                    spi_state <= spi_wait;
                    hrb_state <= 0;
                end
            end
            
            spi_hrb_wait_get_I:begin
                har_num <= recv_data[15:12];
                I <= recv_data[9:0];
                spi_state <= spi_hrb_wait_get_Q;
            end
            
            spi_hrb_wait_get_Q:begin
                if(!prog)begin
                    prog<=1;
                    without_spi_trig<=1;
                    har_num <= recv_data[15:12];
                    Q <= recv_data[9:0];
                end else begin
                    prog<=0;
                    without_spi_trig<=0;
                    if(hrb_recv_cnt < 10)begin
                        hrb_recv_cnt<=hrb_recv_cnt+1;
                        spi_state <= spi_hrb_wait_get_I;
                    end else begin
                        spi_state <= spi_hrb_wait_send_freq;
                        send_data <= {2'b0,hrb_freq};
                    end
                end
            end
            
            default:spi_state<=spi_idle;
        endcase
    end
end


//MISC Connect

assign PD = 0;
assign CTRL = 3'b101;
assign da_clk = ~sysclk;
assign LED = hrb_freq == 10;

endmodule