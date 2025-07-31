module single_freq_synth_core(
    input clk, rst, ctrl,
    input [13:0]freq,
    input signed [9:0]in_data,

    input wire signed [9:0]I_o,
    input wire signed [9:0]Q_o,

    output wire signed [9:0]I,
    output wire signed [9:0]Q,
    output wire signed [9:0]out_data
    );

    // DDS本振
    wire signed [13:0] dds_I, dds_Q;
    dds dds_m_I(
    .clk(clk),
    .freq(freq),
    .dds_o(dds_I)
    );
    dds #(
    .phase(90)
    )dds_m_Q(
    .clk(clk),
    .freq(freq),
    .dds_o(dds_Q)
    );

    reg signed [9:0]rebulid;

    reg signed [22:0]mul_I,mul_Q;

    reg signed [22:0]mul_i_I,mul_i_Q; 
    reg signed [22:0]mul_o_I,mul_o_Q;

    wire signed [9:0]cut_I    = mul_I[22:13];  
    wire signed [9:0]cut_Q    = mul_Q[22:13];  

    wire signed [13:0]cut_i_I  = mul_i_I[22:9]; 
    wire signed [13:0]cut_i_Q  = mul_i_Q[22:9]; 

    wire signed [9:0]cut_o_I  = mul_o_I[22:13];
    wire signed [9:0]cut_o_Q  = mul_o_Q[22:13];

    // IIR
    iir_4_ord iir_m_I(
    .clk(clk),
    .rst(rst),
    .in_data(cut_I),
    .out_data(I)
    );
    iir_4_ord iir_m_Q(
    .clk(clk),
    .rst(rst),
    .in_data(cut_Q),
    .out_data(Q)
    );


    always @(posedge clk or posedge rst) begin
        if (!rst) begin
            mul_I <= 0; mul_Q <= 0;
            mul_i_I <= 0; mul_i_Q <= 0;
            mul_o_I <= 0; mul_o_Q <= 0;
            rebulid <= 0;
        end else begin
            mul_I<=in_data*dds_I;
            mul_Q<=in_data*dds_Q;

            mul_i_I<=dds_I*I_o+dds_Q*Q_o;
            mul_i_Q<=dds_Q*I_o-dds_I*Q_o;

            mul_o_I<=cut_i_I*I;
            mul_o_Q<=cut_i_Q*Q;
            
            rebulid<=(cut_o_I+cut_o_Q)*4;
        end

        
    end 

    assign out_data = ctrl?rebulid:dds_I[13:4];

endmodule