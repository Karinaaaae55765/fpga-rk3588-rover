`default_nettype none
module task6_control #(parameter integer LEASE_TICKS=15,MAX_RPM=30)(
 input wire clk,rst_n,tick,ready,hard_fault,obstacle,slow,stop_request,manual_takeover,idle,released,
 input wire valid,invalid,input wire [7:0] command,
 input wire [31:0] session_in,challenge_in,input wire [15:0] sequence_in,
 input wire signed [15:0] left_in,right_in,
 output reg armed,output wire enable,
 output reg [31:0] session,challenge,output reg [15:0] accepted_sequence,
 output reg [7:0] reason,output wire signed [31:0] left_request,right_request
);
 reg [15:0] lease;reg [31:0] ticks,closed_session;reg ready_old;
 reg signed [15:0] left_saved,right_saved;
 wire [15:0] delta=sequence_in-accepted_sequence;
 wire fresh=delta!=0 && delta<16'h8000;
 wire in_range=left_in>=-MAX_RPM && left_in<=MAX_RPM && right_in>=-MAX_RPM && right_in<=MAX_RPM;
 wire cancelled=hard_fault || !ready || obstacle || stop_request || manual_takeover || invalid;
 assign enable=armed && !cancelled && lease<LEASE_TICKS;
 wire signed [31:0] l={{16{left_saved[15]}},left_saved},r={{16{right_saved[15]}},right_saved};
 assign left_request=!enable?32'sd0:slow?(l>10?32'sd10:l< -10?-32'sd10:l):l;
 assign right_request=!enable?32'sd0:slow?(r>10?32'sd10:r< -10?-32'sd10:r):r;
 task revoke(input [7:0] code);
  begin armed<=0;left_saved<=0;right_saved<=0;reason<=code;closed_session<=session;
   if(armed)challenge<=challenge+32'h9e3779b9;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin armed<=0;session<=0;challenge<=32'h60360001;accepted_sequence<=0;reason<=0;
   lease<=LEASE_TICKS;ticks<=0;closed_session<=0;ready_old<=0;left_saved<=0;right_saved<=0;end
  else begin
   if(tick)ticks<=ticks+1'b1;
   ready_old<=ready;
   if(ready && !ready_old && !armed)challenge<=ticks^32'h60360001;
   if(tick && armed && lease<LEASE_TICKS)lease<=lease+1'b1;
   if(cancelled)revoke(stop_request?8'd1:manual_takeover?8'd2:invalid?8'd3:obstacle?8'd4:8'd5);
   else if(armed && (lease>=LEASE_TICKS || (tick && lease==LEASE_TICKS-1)))revoke(8'd6);
   else if(valid)begin
    if(command==3 || command==4)revoke(8'd1);
    else if(command==1)begin
     if(!armed && idle && released && ready && session_in!=0 && session_in!=closed_session &&
        challenge_in==challenge && left_in==0 && right_in==0)begin
      armed<=1;session<=session_in;accepted_sequence<=sequence_in;lease<=0;reason<=0;left_saved<=0;right_saved<=0;
     end
    end else if(armed && session_in==session && fresh)begin
     if(command==2)begin
      if(!in_range || (left_in<0 && right_in<0))revoke(8'd7);
      else begin left_saved<=left_in;right_saved<=right_in;accepted_sequence<=sequence_in;lease<=0;reason<=0;end
     end
     // HEARTBEAT deliberately does not advance accepted_sequence or renew velocity.
    end
   end
  end
 end
endmodule
`default_nettype wire
