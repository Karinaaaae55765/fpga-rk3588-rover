`timescale 1ns/1ps
module tb_task4_gamepad_spin;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,blocked=0,autonomous=0;
 wire dv,ce,de,err,online,auto_start,stop_request,released;
 wire signed [7:0] lx,ly,ax,ay;
 wire [7:0] dp,b2,b3,rt,lt;
 wire [15:0] good,bad;
 wire [2:0] action;
 reg [2:0] ir_action=0;reg ir_auto=0,ir_stop=0;
 wire [2:0] chosen;wire chosen_auto,chosen_stop,chosen_release,ir_block,changed;
 wire [7:0] launch_b3=b3 | (b2[3]?8'h08:8'h00);
 reg route_selected=0;
 always @(posedge clk or negedge rst)begin
  if(!rst)route_selected<=0;
  else if(dv)route_selected<=b2[3] && !b3[3];
 end
 wire [3:0] vehicle_state;
 wire vehicle_auto,vehicle_enable,route_mode;
 wire signed [31:0] vl,vr;
 reg obstacle=0,hard_fault=0;
 task4_controller vehicle(
  .clk(clk),.rst_n(rst),.tick(1'b0),.configured(1'b1),.sensors_ready(1'b1),
  .hard_fault(hard_fault),.stop_request(chosen_stop),.auto_start(chosen_auto),
  .route_start(online && route_selected),.manual_action(chosen),.released(chosen_release),
  .safety_reason(8'd0),.avoid_reason(8'd0),.obstacle(obstacle),.slow(1'b0),.pose_valid(1'b1),
  .avoid_done(1'b0),.avoid_failed(1'b0),.avoid_waiting(1'b0),.heading(16'sd0),
  .distance_valid(1'b1),.distance_mm(16'd3000),.avoid_left(32'sd0),.avoid_right(32'sd0),
  .nav_left(32'sd0),.nav_right(32'sd0),.x_mm(32'sd0),.y_mm(32'sd0),
  .left_actual(32'sd0),.right_actual(32'sd0),.nav_dwelling(1'b0),.nav_done(1'b0),
  .nav_failed(1'b0),.nav_goal_reached(1'b0),.nav_reason(8'd0),.waypoint(4'd0),
  .state(vehicle_state),.autonomous(vehicle_auto),.motor_enable(vehicle_enable),
  .route_mode(route_mode),.left_rpm(vl),.right_rpm(vr));
 task expect_motor(input integer l,r);
 begin
  cycles(8);
  if(vl!==l || vr!==r || vehicle_enable!==((l!=0)||(r!=0)))
   $fatal(1,"motor expected %0d,%0d actual %0d,%0d enable=%b state=%0d",l,r,vl,vr,vehicle_enable,vehicle_state);
 end endtask
 integer starts=0,stops=0,frames=0;
 reg [7:0] packet[0:14];integer i;
 bt_gamepad_rx #(.CLK_FREQ(1000000)) receiver(clk,rst,rx,dv,ce,de,err,lx,ly,ax,ay,dp,b2,b3,rt,lt,good,bad);
 task4_gamepad_control #(.CLK_FREQ(1000000),.TIMEOUT_MS(40),.RELEASE_MS(5)) control(
  .clk(clk),.rst_n(rst),.data_valid(dv),.connect_event(ce),.disconnect_event(de),.invalid_frame(err),
  .lx(lx),.ly(ly),.dpad(dp),.buttons2(b2),.buttons3(launch_b3),.autonomous(autonomous),
  .blocked(blocked),.cancel(ir_stop || changed),.online(online),.auto_start(auto_start),
  .stop_request(stop_request),.manual_action(action),.released(released));
 remote_input_arbiter arb(clk,rst,online,ir_action,action,ir_auto,auto_start,ir_stop,stop_request,
  1'b1,released,chosen,chosen_auto,chosen_stop,chosen_release,ir_block,changed);
 always @(posedge clk)begin
  if(auto_start)starts=starts+1;
  if(chosen_stop)stops=stops+1;
  if(dv)frames=frames+1;
 end
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task byte_tx(input [7:0] b);
  begin
   @(negedge clk);rx=0;cycles(9);
   for(integer j=0;j<8;j=j+1)begin @(negedge clk);rx=b[j];cycles(9);end
   @(negedge clk);rx=1;cycles(9);
  end
 endtask
 task send_packet(input integer n,input reg corrupt);
  reg [7:0] checksum;
  begin
   checksum=0;
   for(integer j=0;j<n-1;j=j+1)begin byte_tx(packet[j]);checksum=checksum+packet[j];end
   byte_tx(checksum^{7'd0,corrupt});cycles(12);
  end
 endtask
 task report(input [7:0] x,y,d,buttons,extra,input reg corrupt);
  begin
   for(integer j=0;j<15;j=j+1)packet[j]=0;
   packet[0]=8'haa;packet[1]=8'h55;packet[2]=15;packet[3]=8'ha0;packet[4]=1;
   packet[5]=x;packet[6]=y;packet[9]=d;packet[10]=buttons;packet[11]=extra;
   send_packet(15,corrupt);
  end
 endtask
 task status(input [7:0] state_value);
  begin
   packet[0]=8'haa;packet[1]=8'h55;packet[2]=7;packet[3]=8'ha0;packet[4]=0;packet[5]=state_value;
   send_packet(7,0);
  end
 endtask
 task neutral;begin report(0,0,0,0,0,0);cycles(5100);end endtask
 task expect_action(input [2:0] expected);
  if(action!==expected || chosen!==expected)$fatal(1,"action expected %d got %d chosen %d",expected,action,chosen);
 endtask
 integer before_count,before_stops;
 initial begin
  cycles(8);rst=1;cycles(10);
  if(online || action!=0)$fatal(1,"power up idle");
  status(8'h55);report(0,0,1,0,0,0);expect_action(0);
  neutral();if(!released)$fatal(1,"neutral qualification");
  for(i=1;i<=8;i=i+1)begin
   report(0,0,i,0,0,0);
   case(i)1:expect_action(1);2,3:expect_action(4);4,5,6:expect_action(2);7,8:expect_action(3);endcase
  end
  report(0,0,0,0,0,0);expect_action(0);
  report(24,8'he8,0,0,0,0);expect_action(0); // inclusive +/-24 deadzone
  report(0,8'he7,0,0,0,0);expect_action(1);
  report(0,25,0,0,0,0);expect_action(2);
  report(8'he7,0,0,0,0,0);expect_action(3);
  report(25,0,0,0,0,0);expect_action(4);
  // A D-pad request overrides the stick.
  report(100,100,1,0,0,0);expect_action(1);
  for(i=0;i<4;i=i+1)begin cycles(20000);report(0,0,1,0,0,0);expect_action(1);end
  ir_action=2;ir_auto=1;cycles(10);expect_action(1);
  if(chosen_auto || !ir_block)$fatal(1,"BT source priority");
  ir_stop=1;cycles(5);if(!chosen_stop || action!=0)$fatal(1,"IR stop during BT");
  ir_stop=0;ir_action=0;ir_auto=0;report(0,0,1,0,0,0);expect_action(0);
  neutral();ir_stop=1;cycles(5);ir_stop=0;cycles(5);
  report(0,0,1,0,0,0);expect_action(0); // pre-STOP neutral must not re-arm
  neutral();report(0,0,1,0,0,0);expect_action(1);
  before_count=frames;before_stops=stops;
  report(0,0,1,0,0,1);expect_action(0);
  if(frames!=before_count || stops<=before_stops)$fatal(1,"bad checksum must stop, not publish");
  report(0,0,1,0,0,0);expect_action(0);
  neutral();report(0,0,1,0,0,0);expect_action(1);
  cycles(41000);expect_action(0);if(online)$fatal(1,"silent loss timeout");
  report(0,0,1,0,0,0);expect_action(0); // can rediscover after reset, cannot move
  neutral();report(0,0,1,0,0,0);expect_action(1);
  status(8'hff);expect_action(0);if(online)$fatal(1,"disconnect notification");
  status(8'h55);report(0,0,1,0,0,0);expect_action(0);
  neutral();before_count=starts;
  report(0,0,0,2,8,0);if(starts!=before_count)$fatal(1,"B beats START");
  neutral();report(0,0,0,0,8,0);
  if(starts!=before_count+1)$fatal(1,"START launch");
  for(i=0;i<3;i=i+1)report(0,0,0,0,8,0);
  if(starts!=before_count+1)$fatal(1,"held START repeated");
  autonomous=1;neutral();before_stops=stops;cycles(120000);
  if(!online || stops!=before_stops || action!=0)$fatal(1,"autonomous must survive event-driven silence");
  report(0,0,1,0,0,0);
  if(stops<=before_stops || action!=0)$fatal(1,"direction cancels auto");
  autonomous=0;report(0,0,1,0,0,0);expect_action(0);
  neutral();report(0,0,1,0,0,0);expect_action(1);
  blocked=1;cycles(10);expect_action(0);
  report(0,0,1,0,0,0);blocked=0;report(0,0,1,0,0,0);expect_action(0);
  blocked=1;neutral();if(!released)$fatal(1,"fault recovery can observe neutral while blocked");
  blocked=0;cycles(5);report(0,0,1,0,0,0);expect_action(1);
  // A qualified idle neutral is not an outstanding motion command.
  neutral();cycles(120000);report(0,0,1,0,0,0);expect_action(1);
  // A truncated frame may not keep a previous drive lease alive.
  byte_tx(8'haa);byte_tx(8'h55);byte_tx(15);byte_tx(8'ha0);cycles(5100);expect_action(0);
  before_count=frames;
  byte_tx(8'haa);byte_tx(8'h55);byte_tx(0);
  byte_tx(8'haa);byte_tx(8'h55);byte_tx(255);
  report(0,0,9,0,0,0);report(8'h80,0,0,0,0,0);
  if(frames!=before_count)$fatal(1,"invalid lengths/fields published");
  // Correct checksum on unsupported IDs/types is still rejected.
  report(0,0,0,0,0,0);before_count=frames;
  packet[3]=8'h20;send_packet(15,0);packet[3]=8'ha0;packet[4]=2;send_packet(15,0);
  if(frames!=before_count)$fatal(1,"unsupported protocol published");
  // UART framing/break must clear command, then recover on a fresh packet.
  neutral();report(0,0,1,0,0,0);expect_action(1);
  @(negedge clk);rx=0;cycles(110);rx=1;cycles(20);expect_action(0);
  neutral();report(0,0,1,0,0,0);expect_action(1);
  // Overlapping AA header resynchronizes; junk bytes cannot renew a lease.
  byte_tx(8'haa);report(0,0,0,0,0,0);expect_action(0);
  neutral();report(0,0,1,0,0,0);
  for(i=0;i<450;i=i+1)byte_tx(8'h00);
  cycles(100);if(online || action!=0)$fatal(1,"noise renewed control lease");
  status(8'h55);neutral();before_count=starts;report(0,0,0,0,8,0);
  if(starts!=before_count+1)$fatal(1,"single START after idle");
  autonomous=1;report(0,0,0,0,0,0);cycles(120000);before_stops=stops;
  status(8'hff);if(online || stops<=before_stops)$fatal(1,"explicit disconnect stops autonomous");
  autonomous=0;status(8'h55);report(0,0,1,0,0,0);expect_action(0);
  // New shoulder actions exercise the UART -> map -> arbiter -> vehicle path.
  report(0,0,0,2,0,0);neutral();
  report(0,0,0,8'h40,0,0);expect_action(6);expect_motor(-30,30);
  for(i=0;i<4;i=i+1)begin cycles(20000);report(0,0,0,8'h40,0,0);expect_motor(-30,30);end
  report(0,0,0,0,0,0);expect_action(0);expect_motor(0,0);
  report(0,0,0,8'h80,0,0);expect_action(7);expect_motor(30,-30);
  report(0,0,0,0,0,0);expect_motor(0,0);
  report(0,0,0,8'hc0,0,0);expect_action(0);expect_motor(0,0);
  report(0,0,0,8'h40,0,0);expect_action(0);expect_motor(0,0);
  neutral();report(0,0,0,8'h40,0,0);expect_motor(-30,30);
  report(0,0,0,8'h42,0,0);expect_action(0);expect_motor(0,0);
  neutral();report(0,0,0,8'h80,0,0);expect_motor(30,-30);
  cycles(41000);expect_action(0);expect_motor(0,0);if(online)$fatal(1,"spin lease did not expire");
  status(8'h55);report(0,0,0,8'h80,0,0);expect_action(0);expect_motor(0,0);
  neutral();report(0,0,0,8'h80,0,0);expect_motor(30,-30);
  report(0,0,0,8'h80,0,1);expect_action(0);expect_motor(0,0);
  neutral();report(0,0,0,8'h40,0,0);expect_motor(-30,30);
  status(8'hff);expect_action(0);expect_motor(0,0);
  status(8'h55);neutral();before_count=starts;
  report(0,0,0,8'h40,8,0);expect_motor(0,0);if(starts!=before_count)$fatal(1,"LB+START launched");
  neutral();report(0,0,0,8'h88,0,0);expect_motor(0,0);if(starts!=before_count)$fatal(1,"RB+X launched");
  neutral();report(0,0,1,8'h40,0,0);expect_motor(0,0);
  neutral();report(25,0,0,8'h80,0,0);expect_motor(0,0);
  neutral();report(0,0,0,0,8,0);cycles(10);
  if(vehicle_state!=3 || route_mode || starts!=before_count+1)$fatal(1,"START no longer task3 cruise");
  expect_motor(60,60);
  report(0,0,0,2,0,0);expect_motor(0,0);neutral();
  report(0,0,0,8'h08,0,0);cycles(10);
  if(vehicle_state!=8 || !route_mode)$fatal(1,"X no longer selects navigation");
  autonomous=1;report(0,0,0,8'h40,0,0);expect_action(0);expect_motor(0,0);
  autonomous=0;report(0,0,0,8'h40,0,0);expect_motor(0,0);
  neutral();report(0,0,0,8'h40,0,0);expect_motor(-30,30);
  obstacle=1;cycles(8);expect_motor(0,0);
  obstacle=0;cycles(8);expect_motor(0,0); // obstacle latch requires release
  neutral();report(0,0,0,8'h80,0,0);expect_motor(30,-30);
  hard_fault=1;cycles(8);expect_motor(0,0);
  $display("PASS LB/RB motor signs at30RPM, fresh hold, release, conflict, B, timeout, reconnect, invalid UART, obstacle and hard-fault inhibition; START task3 and X navigation");
  $display("PASS Bluetooth UART protocol, all D-pad/axis actions, deadzone, checksum/framing/length/gap errors, source priority, STOP, reconnect, timeout, fault recovery and START edge");
  $finish;
 end
 initial begin #30000000;$fatal(1,"BT test timeout");end
endmodule
