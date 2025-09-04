module sqrt #(
parameter N = 16
)(
input clk,
input [2*N-1:0]num,
output reg [N-1:0]sqrt_result
);



function [N-1:0] sqrt;
      input [2*N-1:0] num;  //declare input
      //intermediate signals.
      reg [2*N-1:0] a;
      reg [N-1:0] q;
      reg [N+1:0] left,right,r;    
      integer i;
  begin
      //initialize all the variables.
      a = num;
      q = 0;
      i = 0;
      left = 0;   //input to adder/sub
      right = 0;  //input to adder/sub
      r = 0;  //remainder
      //run the calculations for 16 iterations.
      for(i=0;i<16;i=i+1) begin 
          right = {q,r[N+1],1'b1};
          left = {r[N-1:0],a[2*N-1:2*N-1]};
          a = {a[2*N-3:0],2'b00};    //left shift by 2 bits.
          if (r[17] == 1) //add if r is negative
              r = left + right;
          else    //subtract if r is positive
              r = left - right;
          q = {q[N-2:0],!r[N+1]};       
      end
      sqrt = q;   //final assignment of output.
  end
  endfunction
  
  always@(posedge clk)
    sqrt_result<=sqrt(num);
  
  
  endmodule