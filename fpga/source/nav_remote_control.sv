`default_nettype none
// Only a measured, explicitly confirmed key can start a mission.
// Require 300 ms without IR edges before a new press; repeats never start.
module nav_remote_control #(
 parameter integer CLK_FREQ=50000000,RELEASE_MS=300,KEY_CONFIRMED=0,RUNTIME=0,
 parameter [15:0] START_ADDRESS=16'h0000,
 parameter [7:0] START_COMMAND=8'h00
)(
 input wire clk,rst_n,frame_valid,repeat_valid,invalid_frame,activity,
 input wire [15:0] address,input wire [7:0] command,
 input wire mission_active,stop_request,
 input wire configured_key,
 input wire [15:0] configured_address,
 input wire [7:0] configured_command,
 output reg start_request,output reg released
);
 localparam integer RELEASE_CYCLES=CLK_FREQ/1000*RELEASE_MS;
 reg [31:0] quiet;
 wire key_ready=RUNTIME?configured_key:(KEY_CONFIRMED!=0);
 wire [15:0] key_address=RUNTIME?configured_address:START_ADDRESS;
 wire [7:0] key_command=RUNTIME?configured_command:START_COMMAND;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin quiet<=0;released<=0;start_request<=0;end
  else begin
   start_request<=0;
   if(activity)quiet<=0;else if(quiet<RELEASE_CYCLES)quiet<=quiet+1'b1;
   if(mission_active || stop_request)released<=0;
   else if(quiet>=RELEASE_CYCLES)released<=1;
   if(frame_valid || repeat_valid || invalid_frame)begin
    released<=0;
    if(frame_valid && !repeat_valid && !invalid_frame && key_ready &&
       released && !mission_active && !stop_request &&
       address==key_address && command==key_command)start_request<=1;
   end
  end
 end
endmodule

// Read-only key discovery: append after the navigation frame, never learn
// or authorize a start automatically from the first received remote.
module nav_ir_telemetry_tx #(parameter integer CLK_FREQ=50000000,BAUD_RATE=115200)(
 input wire clk,rst_n,trigger,
 input wire [15:0] address,valid_count,error_count,
 input wire [7:0] command,flags,
 output wire tx,busy
);
 reg [1:0] state;
 reg [4:0] index;
 reg [15:0] sequence_number,crc;
 reg [79:0] payload;
 reg [7:0] data;
 wire ready,done;
 assign busy=state!=0;
 always @*case(index)
  0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h24;4:data=10;
  15:data=crc[7:0];16:data=crc[15:8];default:data=payload[7:0];
 endcase
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
    payload<={error_count,valid_count,flags,command,address,sequence_number};
    sequence_number<=sequence_number+1'b1;index<=0;crc<=65535;state<=1;
   end
   1:if(ready)begin
    if(index>=2 && index<=14)crc<=crc_byte(crc,data);
    if(index>=5 && index<=14)payload<=payload>>8;
    state<=2;
   end
   2:if(done)begin if(index==16)state<=0;else begin index<=index+1'b1;state<=1;end end
   default:state<=0;
  endcase
 end
endmodule
`default_nettype wire
