`default_nettype none
// Trigger once after each legacy 28-byte V3 frame. Completes within its 47 ms
// idle gap at 115200 baud. Never interrupts a legacy frame.
// Layout stays 41 payload bytes: flags[6]=motor fault, [7]=remote blocked.
// fault_code is the unified 2026-10-25 reason; waypoint is the scan/leg phase.
module task123_telemetry #(parameter integer CLK_FREQ=50000000,BAUD_RATE=115200)(
 input wire clk,rst_n,trigger,
 input wire [3:0] nav_state,
 input wire [7:0] fault_code,waypoint,flags,last_rx,
 input wire [15:0] rx_errors,distance_mm,
 input wire signed [31:0] x_mm,y_mm,left_count,right_count,
 input wire signed [15:0] heading,
 input wire signed [31:0] left_actual,right_actual,left_target,right_target,
 input wire [9:0] left_duty,right_duty,
 output wire tx,
 output wire busy
);
 reg [1:0] state;
 reg [5:0] index;
 reg [15:0] sequence_number,crc;
 reg [327:0] payload;
 reg [7:0] data;
 wire ready,done;
 assign busy=state!=0;
 always @*begin
  case(index)
   0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h28;4:data=41;
   46:data=crc[7:0];47:data=crc[15:8];
   default:data=payload[7:0];
  endcase
 end
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;
  begin v=c^{b,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 uart_tx_v3 #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) uart(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(state==1),.ready(ready),.tx(tx),.done(done));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;index<=0;sequence_number<=0;crc<=65535;payload<=0;end
  else case(state)
   0:if(trigger)begin
    payload<={6'd0,right_duty,6'd0,left_duty,rx_errors,last_rx,distance_mm,
      right_target[15:0],left_target[15:0],right_actual[15:0],left_actual[15:0],
      right_count,left_count,heading,y_mm,x_mm,flags,waypoint,fault_code,4'd0,nav_state,sequence_number};
    sequence_number<=sequence_number+1'b1;index<=0;crc<=65535;state<=1;
   end
   1:if(ready)begin
    if(index>=2 && index<=45)crc<=crc_byte(crc,data);
    if(index>=5 && index<=45)payload<=payload>>8;
    state<=2;
   end
   2:if(done)begin
    if(index==47)state<=0;
    else begin index<=index+1'b1;state<=1;end
   end
   default:state<=0;
  endcase
 end
endmodule
`default_nettype wire
