`default_nettype none
// NEC has no key-up packet. Release is inferred after HOLD_MS without a
// valid repeat belonging to the most recently accepted full frame.
module remote_drive_control #(
 parameter integer CLK_FREQ=50000000,HOLD_MS=250,RELEASE_MS=300,
 parameter [15:0] ADDRESS=16'h0000,
 parameter [7:0] KEY_2=0,KEY_8=0,KEY_4=0,KEY_6=0,KEY_5=0,KEY_0=0,KEY_AUTO=0
)(
 input wire clk,rst_n,configured,frame_valid,repeat_valid,invalid_frame,activity,
 input wire [15:0] address,input wire [7:0] command,
 input wire autonomous,blocked,
 output wire [2:0] manual_action,
 output reg auto_start,stop_request,output wire released
);
 localparam integer HOLD=CLK_FREQ/1000*HOLD_MS,RELEASE=CLK_FREQ/1000*RELEASE_MS;
 reg [31:0] age,quiet;reg [2:0] held;reg inhibit,auto_latched;
 reg [2:0] decoded;
 always @*begin
  decoded=0;
  if(command==KEY_2)decoded=1;else if(command==KEY_8)decoded=2;
  else if(command==KEY_4)decoded=3;else if(command==KEY_6)decoded=4;
  else if(command==KEY_5)decoded=5;
 end
 assign released=quiet>=RELEASE;
 assign manual_action=configured && !blocked && !inhibit && age<HOLD?held:3'd0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin age<=HOLD;quiet<=0;held<=0;inhibit<=1;auto_latched<=0;auto_start<=0;stop_request<=0;end
  else begin
   auto_start<=0;stop_request<=0;
   // Raw edges and malformed frames must not keep the re-arm gate locked.
   // A decoded repeat still counts as held, including during a fault.
   if((frame_valid && address==ADDRESS) || repeat_valid)quiet<=0;
   else if(quiet<RELEASE)quiet<=quiet+1'b1;
   if(age<HOLD)age<=age+1'b1;
   if(released)begin inhibit<=0;auto_latched<=0;end
   if(configured && frame_valid && address==ADDRESS && command==KEY_0)begin
    stop_request<=1;held<=0;age<=HOLD;inhibit<=1;
   end else if(!configured || blocked)begin held<=0;age<=HOLD;inhibit<=1;end
   else if(invalid_frame)begin held<=0;age<=HOLD;end
   else if(frame_valid)begin
    held<=0;age<=HOLD;
    if(address==ADDRESS)begin
     if(command==KEY_0)begin stop_request<=1;inhibit<=1;end
     else if(decoded!=0)begin
      if(autonomous)begin stop_request<=1;inhibit<=1;end
      else if(!inhibit)begin held<=decoded;age<=0;end
     end else if(command==KEY_AUTO)begin
      if(!inhibit && !auto_latched && !autonomous)auto_start<=1;
      auto_latched<=1;
     end
    end
   end else if(repeat_valid && held!=0 && age<HOLD && !inhibit && !autonomous)age<=0;
  end
 end
endmodule
`default_nettype wire
