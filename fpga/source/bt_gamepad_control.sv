`default_nettype none
// Fixed speed actions reuse the existing vehicle controller. A timeout is a
// conservative motion lease, NOT a claim about the vendor's report cadence.
module bt_gamepad_control #(
 parameter integer CLK_FREQ=50000000,TIMEOUT_MS=500,RELEASE_MS=300,DEADZONE=24,
 parameter integer FORWARD_NEGATIVE_Y=1
)(
 input wire clk,rst_n,data_valid,connect_event,disconnect_event,invalid_frame,
 input wire signed [7:0] lx,ly,
 input wire [7:0] dpad,buttons2,buttons3,
 input wire autonomous,blocked,cancel,
 output reg online,auto_start,stop_request,
 output wire [2:0] manual_action,
 output wire released
);
 localparam integer TIMEOUT=CLK_FREQ/1000*TIMEOUT_MS;
 localparam integer RELEASE=CLK_FREQ/1000*RELEASE_MS;
 reg [31:0] age,neutral_age;
 reg neutral_seen,armed;
 reg [2:0] held,decoded;
 wire axis_center=($signed(lx)>=-DEADZONE && $signed(lx)<=DEADZONE &&
                   $signed(ly)>=-DEADZONE && $signed(ly)<=DEADZONE);
 wire neutral=axis_center && dpad==0 && !buttons2[1] && !buttons3[3];
 // A neutral idle session and an explicit autonomous task are not joystick
 // leases. Event-driven controllers may send nothing until the next change.
 wire idle_neutral=neutral_seen && held==0;
 wire fresh=online && (age<TIMEOUT || idle_neutral || autonomous);
 wire forward_axis=FORWARD_NEGATIVE_Y ? $signed(ly)<-DEADZONE : $signed(ly)>DEADZONE;
 wire backward_axis=FORWARD_NEGATIVE_Y ? $signed(ly)>DEADZONE : $signed(ly)<-DEADZONE;
 assign released=!online || (fresh && neutral_seen && neutral_age>=RELEASE);
 assign manual_action=fresh && armed && !blocked && !autonomous ? held:3'd0;
 always @*begin
  decoded=0;
  // D-pad takes precedence. Rear diagonals request straight reverse; front
  // diagonals use existing forward arcs. No new spin or speed modes.
  case(dpad)
   1:decoded=1;
   2,3:decoded=4;
   4,5,6:decoded=2;
   7,8:decoded=3;
   0:begin
    if(backward_axis)decoded=2;
    else if($signed(lx)<-DEADZONE)decoded=3;
    else if($signed(lx)>DEADZONE)decoded=4;
    else if(forward_axis)decoded=1;
   end
   default:decoded=0;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   online<=0;age<=TIMEOUT;neutral_age<=0;neutral_seen<=0;armed<=0;held<=0;
   auto_start<=0;stop_request<=0;
  end else begin
   auto_start<=0;stop_request<=0;
   if(age<TIMEOUT)age<=age+1'b1;
   if(fresh && neutral_seen && neutral_age<RELEASE)neutral_age<=neutral_age+1'b1;
   if(!blocked && fresh && neutral_seen && neutral_age>=RELEASE)armed<=1;
   if(blocked)begin armed<=0;held<=0;end
   if(disconnect_event || (online && age>=TIMEOUT && !autonomous && !idle_neutral && !data_valid))begin
    stop_request<=online;online<=0;age<=TIMEOUT;
    held<=0;armed<=0;neutral_seen<=0;neutral_age<=0;
   end else if(connect_event)begin
    // Rebinding or reconnecting never restores a previous command.
    online<=1;age<=0;held<=0;armed<=0;neutral_seen<=0;neutral_age<=0;
    stop_request<=1;
   end else if(invalid_frame)begin
    stop_request<=online;held<=0;armed<=0;neutral_seen<=0;neutral_age<=0;
   end else if(cancel)begin
    // External STOP/source switch requires a neutral report AFTER the stop.
    held<=0;armed<=0;neutral_seen<=0;neutral_age<=0;
   end else if(data_valid)begin
    online<=1;age<=0;
    if(neutral)begin
     held<=0;
     if(!neutral_seen)neutral_age<=0;
     neutral_seen<=1;
    end else begin neutral_seen<=0;neutral_age<=0;end
    if(buttons2[1])begin // B has priority over START and all axes.
     stop_request<=1;held<=0;armed<=0;
    end else if(blocked || !fresh)begin held<=0;armed<=0;end
    else if(decoded!=0 && autonomous)begin
     stop_request<=1;held<=0;armed<=0;
    end else if(buttons3[3])begin
     held<=0;
     if(armed && !autonomous && axis_center && dpad==0)auto_start<=1;
     armed<=0; // Holding START cannot generate repeated launches.
    end else if(armed)held<=decoded;
   end
  end
 end
endmodule
`default_nettype wire
