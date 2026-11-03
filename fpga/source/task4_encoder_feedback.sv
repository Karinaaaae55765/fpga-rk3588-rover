`default_nettype none
// Parallel task-four feedback. Original raw decoder is retained in the motor module.
// Joint A/B qualification rejects short states, never infers direction from a command.
module task4_encoder_feedback #(
 parameter integer CLK_FREQ=50000000,STABLE_US=5
)(
 input wire clk,rst_n,enc_a,enc_b,
 output reg signed [31:0] count,
 output reg [31:0] rejected,invalid,
 output wire [1:0] accepted_state
);
 localparam integer N0=(CLK_FREQ/1000*STABLE_US+999)/1000;
 localparam integer N=N0>0?N0:1;
 localparam integer W=N>1?$clog2(N+1):1;
 reg [1:0] sync1,sync2,candidate,accepted;
 reg [W-1:0] age;
 assign accepted_state=accepted;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sync1<=0;sync2<=0;end
  else begin sync1<={enc_a,enc_b};sync2<=sync1;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin candidate<=0;accepted<=0;age<=0;count<=0;rejected<=0;invalid<=0;end
  else if(sync2!=candidate)begin
   if(candidate!=accepted && rejected!=32'hffffffff)rejected<=rejected+1'b1;
   candidate<=sync2;age<=1;
  end else if(age<N)age<=age+1'b1;
  else if(accepted!=candidate)begin
   case({accepted,candidate})
    4'b0001,4'b0111,4'b1110,4'b1000:count<=count+1'b1;
    4'b0010,4'b1011,4'b1101,4'b0100:count<=count-1'b1;
    default:if(invalid!=32'hffffffff)invalid<=invalid+1'b1;
   endcase
   // Resynchronize even after a two-bit jump; do not invent the missing edges.
   accepted<=candidate;
  end
 end
endmodule
`default_nettype wire
