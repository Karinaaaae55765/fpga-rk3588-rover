`timescale 1ns/1ps
`include "task123_config.vh"
module tb_task4_top;
 reg clk=0;always #5 clk=~clk;
 reg corrupt_report=0;
 reg rst=0,ir=1,bt=1,serial_rx=1;wire stby,p1,p2,a1,a2,b1,b2,telemetry_tx;
 task6_top #(.CLK_FREQ(1000000)) dut(
  .sensor_uart_tx(telemetry_tx),.clk50M(clk),.reset_n(rst),.tof_uart_rx(1'b1),.imu_uart_rx(1'b1),.control_rx(serial_rx),.ir_rx(ir),.bt_rx(bt),
  .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
  .camera_vsync(1'b0),.camera_href(1'b0),.camera_pclk(clk),.camera_data(8'd0),
  .motor_stby(stby),.motor_pwm1(p1),.motor_pwm2(p2),.motor_ain1(a1),.motor_ain2(a2),.motor_bin1(b1),.motor_bin2(b2));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task level(input reg v,input integer n);@(negedge clk);ir=v;cycles(n);endtask
 task key(input [7:0] command);
  reg [31:0] bits;integer i;
  begin
   bits={~command,command,16'(`T123_ADDRESS)};level(0,9000);level(1,4500);
   for(i=0;i<32;i=i+1)begin level(0,560);level(1,bits[i]?1690:560);end
   level(0,560);level(1,10);
  end
 endtask
 task rep;begin level(0,9000);level(1,2250);level(0,560);level(1,10);end endtask
 task uart_byte(input [7:0] b,input reg serial_port);
  begin
   @(negedge clk);if(serial_port)serial_rx=0;else bt=0;cycles(9);
   for(integer j=0;j<8;j=j+1)begin
    @(negedge clk);if(serial_port)serial_rx=b[j];else bt=b[j];cycles(9);
   end
   @(negedge clk);if(serial_port)serial_rx=1;else bt=1;cycles(9);
  end
 endtask
 task bt_report(input [7:0] d,input [7:0] buttons,input [7:0] extra);
  reg [7:0] packet[0:14];reg [7:0] sum;
  begin
   for(integer j=0;j<15;j=j+1)packet[j]=0;
   packet[0]=8'haa;packet[1]=8'h55;packet[2]=15;packet[3]=8'ha0;packet[4]=1;
   packet[9]=d;packet[10]=buttons;packet[11]=extra;sum=0;
   for(integer j=0;j<14;j=j+1)begin uart_byte(packet[j],0);sum=sum+packet[j];end
   uart_byte(corrupt_report?(sum^8'h01):sum,0);cycles(20);
  end
 endtask
 task bt_neutral;begin bt_report(0,0,0);cycles(310000);end endtask
 task rk_stop;
  reg [7:0] p[0:5];reg [15:0] crc;
  begin
   p[0]=8'h4e;p[1]=8'h56;p[2]=1;p[3]=1;p[4]=2;p[5]=0;crc=16'hffff;
   for(integer j=0;j<6;j=j+1)begin
    uart_byte(p[j],1);crc=crc^{p[j],8'd0};
    for(integer k=0;k<8;k=k+1)crc=crc[15]?(crc<<1)^16'h1021:crc<<1;
   end
   uart_byte(crc[7:0],1);uart_byte(crc[15:8],1);cycles(20);
  end
 endtask
 integer uart_index=0,uart_size=0,uart_i,diag_frames=0,imu_frames=0,imu_before=0,status_frames=0,legacy_frames=0,ir_frames=0;
 integer heading_frames=0,heading_before=0;
 reg [7:0] uart_byte_value,uart_packet[0:63];reg [15:0] uart_crc,last_status_seq;
 reg [31:0] last_raw_left,last_raw_right;
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer j;begin v=c^{b,8'd0};for(j=0;j<8;j=j+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 initial begin
  if($test$plusargs("DIAGNOSTIC_ONLY"))begin
   wait(rst);
   forever begin
    @(negedge telemetry_tx);cycles(4);if(telemetry_tx!==0)$fatal(1,"diagnostic UART start");
    for(uart_i=0;uart_i<8;uart_i=uart_i+1)begin cycles(9);uart_byte_value[uart_i]=telemetry_tx;end
    cycles(9);if(telemetry_tx!==1)$fatal(1,"UART collision");
    uart_packet[uart_index]=uart_byte_value;
    if(uart_index==0 && uart_byte_value!=8'ha5)$fatal(1,"UART header");
    if(uart_index==1 && uart_byte_value!=8'h5a)$fatal(1,"UART header2");
    if(uart_index==4)begin uart_size=uart_byte_value+7;if(uart_size>64)$fatal(1,"UART length");end
    if(uart_index>=4 && uart_index==uart_size-1)begin
     uart_crc=65535;for(integer j=2;j<uart_size-2;j=j+1)uart_crc=crc_byte(uart_crc,uart_packet[j]);
     if(uart_crc!={uart_packet[uart_size-1],uart_packet[uart_size-2]})$fatal(1,"UART CRC");
     case(uart_packet[3])
      8'h10:legacy_frames=legacy_frames+1;
      8'h24:ir_frames=ir_frames+1;
      8'h28:begin
       if(uart_size!=48)$fatal(1,"status layout changed");
       status_frames=status_frames+1;last_status_seq={uart_packet[6],uart_packet[5]};
       last_raw_left={uart_packet[24],uart_packet[23],uart_packet[22],uart_packet[21]};
       last_raw_right={uart_packet[28],uart_packet[27],uart_packet[26],uart_packet[25]};
      end
      8'h29:begin
       if(uart_size!=48 || {uart_packet[6],uart_packet[5]}!=last_status_seq || uart_packet[45]!=5)$fatal(1,"diagnostic sequence/layout");
       if({uart_packet[40],uart_packet[39],uart_packet[38],uart_packet[37]}!=last_raw_left ||
          {uart_packet[44],uart_packet[43],uart_packet[42],uart_packet[41]}!=last_raw_right)$fatal(1,"raw/clean not sampled atomically");
       if({uart_packet[12],uart_packet[11],uart_packet[10],uart_packet[9]}!=32'd1000 ||
          {uart_packet[16],uart_packet[15],uart_packet[14],uart_packet[13]}!=-32'sd1000)$fatal(1,"qualified diagnostic count fields");
       diag_frames=diag_frames+1;
      end
      8'h2a:begin
       if(uart_size!=49 || {uart_packet[6],uart_packet[5]}!=last_status_seq)$fatal(1,"IMU sequence/layout");
       if({uart_packet[18],uart_packet[17]}!=0)$fatal(1,"IMU filtered field");
       imu_frames=imu_frames+1;
      end
      8'h2b:begin
       if(uart_size!=55 || {uart_packet[6],uart_packet[5]}!=last_status_seq || uart_packet[52]!=1 ||
          {uart_packet[42],uart_packet[41]}!=110 || {uart_packet[46],uart_packet[45],uart_packet[44],uart_packet[43]}!=9042)
        $fatal(1,"heading sequence/layout/calibration");
       if(dut.route_mode && ({uart_packet[24],uart_packet[23],uart_packet[22],uart_packet[21]}!=1000 ||
          {uart_packet[28],uart_packet[27],uart_packet[26],uart_packet[25]}!=-32'sd1000))$fatal(1,"heading counts");
       heading_frames=heading_frames+1;
      end
      default:$fatal(1,"unknown UART frame");
     endcase
     uart_index=0;
    end else uart_index=uart_index+1;
   end
  end
 end
 integer i;reg seen_pwm;
 always @(negedge clk)if(p1 || p2)seen_pwm=1;
 always @(negedge clk)if(rst && !dut.motor_enable && (stby || p1 || p2 || a1 || a2 || b1 || b2))
  $fatal(1,"physical pads bypassed immediate controller disable");
 initial begin
  seen_pwm=0;cycles(5);rst=1;
  // 204928 standalone avoidance and independent navigation interlocks.
  // Sensor packet parsing/video regression are separate tests. Here inject
  // fresh decoded samples while exercising the real NEC receiver and pads.
  force dut.system_n=rst;force dut.imu_valid=1'b1;force dut.distance_valid=1'b1;
  force dut.tof_strength=16'd500;force dut.distance_mm=16'd2000;
  force dut.yaw_cdeg=16'sd0;cycles(310000);
  if($test$plusargs("HEADING_ONLY"))begin
   bt_report(0,0,0);bt_neutral();bt_report(0,8,0);cycles(600000);
   if(!dut.route_mode || dut.state!=8 || !stby)$fatal(1,"heading fixture launch");
   // Remove enable between clock edges: physical STOP must not wait for q.
   @(negedge clk);#1;force dut.motor_enable=1'b0;#1;
   if(!dut.motor_enable_q || stby || p1 || p2 || a1 || a2 || b1 || b2)$fatal(1,"pipelined internal enable delayed physical STOP");
   release dut.motor_enable;cycles(3);
   force dut.controller_left_request=32'sd120;force dut.controller_right_request=-32'sd120;cycles(3);
   if(dut.left_request!=103 || dut.right_request!=-103)$fatal(1,"navigation speed cap");
   force dut.yaw_cdeg=16'sd1500;cycles(30000);
   if(!dut.nav_degraded || dut.left_request!=44 || dut.right_request!=-44)$fatal(1,"disagreement speed cap");
   release dut.controller_left_request;release dut.controller_right_request;
   force dut.yaw_cdeg=16'sd4000;cycles(1050000);
   if(stby || p1 || p2 || dut.reason!=8'h3c || dut.waypoint!=0)$fatal(1,"heading disagreement physical gate state=%0d reason=%h",dut.state,dut.reason);
   cycles(100000);if(stby || dut.autonomous)$fatal(1,"heading fault replay");
   bt_report(0,2,0);bt_neutral();bt_report(0,8,0);cycles(100000);
   if(dut.state!=8 || !dut.route_mode || dut.nav_pose_fault || dut.path.failed || dut.heading!=0)$fatal(1,"fresh X did not reset heading origin/fault");
   bt_report(0,2,0);bt_neutral();bt_report(0,0,8);cycles(100000);
   force dut.nav_pose_fault=1'b1;force dut.nav_pose_reason=8'h3c;cycles(1000);
   if(dut.state!=3 || dut.route_mode || !stby || dut.left_request!=60 || dut.right_request!=60)$fatal(1,"task4 pose fault/speed leaked into task3");
   release dut.nav_pose_fault;release dut.nav_pose_reason;
   rk_stop();if(stby)$fatal(1,"RK STOP after heading test");
   $display("PASS task4 103/44 RPM caps, disagreement physical STOP, latch, X reset and task3 pose/speed isolation");$finish;
  end
  if($test$plusargs("DIAGNOSTIC_ONLY"))begin
   if(diag_frames!=0 || imu_frames<3 || heading_frames<3)$fatal(1,"idle diagnostic gating");
   imu_before=imu_frames;heading_before=heading_frames;
   force dut.left_count=32'sd2000;force dut.right_count=-32'sd1500;
   force dut.nav_left_count=32'sd1000;force dut.nav_right_count=-32'sd1000;
   bt_report(0,0,0);bt_neutral();bt_report(0,8,0);cycles(220000);
   if(diag_frames<3 || legacy_frames<3 || ir_frames<3 || status_frames<3 || imu_frames<imu_before+3 || heading_frames<heading_before+3)$fatal(1,"mixed diagnostic traffic missing");
   bt_report(0,2,0);cycles(20000);i=diag_frames;cycles(100000);
   if(diag_frames!=i || stby)$fatal(1,"diagnostic restarted after STOP");
   bt_neutral();bt_report(0,0,8);cycles(30000);imu_before=imu_frames;heading_before=heading_frames;cycles(180000);
   if(diag_frames!=i || dut.route_mode || imu_frames!=imu_before || heading_frames!=heading_before || dut.state!=3)$fatal(1,"task3 diagnostics leaked");
   rk_stop();if(stby)$fatal(1,"diagnostic RK STOP");
   $display("PASS mixed UART 0x10/24/28/29/2A/2B CRC, sequence, idle observers, task3 gating and STOP");$finish;
  end
  if($test$plusargs("STARTUP_ONLY"))begin
   bt_report(0,0,0);bt_neutral();bt_report(0,8,0);cycles(200000);
   if(dut.state!=8 || !stby || dut.left_duty>200 || dut.right_duty>200 ||
      dut.left_duty<=dut.motors.configurable_motors.left_motor.pi_duty ||
      dut.right_duty<=dut.motors.configurable_motors.right_motor.pi_duty)$fatal(1,"navigation launch did not apply bounded independent supplement");
   bt_report(0,2,0);if(stby || p1 || p2)$fatal(1,"B did not cancel boost immediately");
   cycles(350000);if(stby || dut.autonomous)$fatal(1,"boost replayed after B");
   bt_neutral();bt_report(0,8,0);cycles(200000);
   force dut.imu_valid=1'b0;cycles(510000);
   if(stby || p1 || p2)$fatal(1,"sensor fault did not close physical motor gate");
   // A lease STOP may clear route_mode before the original PI's next 20ms update.
   // Physical pads must already be off; permit that original sampled telemetry to settle.
   cycles(30000);
   if(stby || p1 || p2 || dut.left_duty!=0 || dut.right_duty!=0)$fatal(1,"sensor-stop PI duty did not settle to zero");
   force dut.imu_valid=1'b1;bt_report(0,2,0);bt_neutral();bt_report(0,0,8);cycles(200000);
   if(dut.route_mode || dut.left_duty!=dut.motors.configurable_motors.left_motor.pi_duty ||
      dut.right_duty!=dut.motors.configurable_motors.right_motor.pi_duty)$fatal(1,"task3 did not bypass assist");
   rk_stop();if(stby)$fatal(1,"RK stop after boost mode switch");
   $display("PASS real UART X boost, B cancellation/no replay, sensor gate, START bypass, RK STOP");$finish;
  end
  if($test$plusargs("POLARITY_ONLY"))begin
   // Actual launch packets select the pose mapping; counts emulate the captured raw polarity.
   force dut.left_count=32'sd1000;force dut.nav_left_count=32'sd1000;force dut.right_count=-32'sd1000;force dut.nav_right_count=-32'sd1000;
   bt_report(0,0,0);bt_neutral();bt_report(0,0,8);cycles(100000);
   if(dut.route_mode || dut.pose_signs!=0 || dut.state!=3)$fatal(1,"START changed task3 pose mapping");
   force dut.left_count=32'sd1100;force dut.nav_left_count=32'sd1100;force dut.right_count=-32'sd1100;force dut.nav_right_count=-32'sd1100;cycles(25000);
   if(dut.x_mm!=0 || dut.y_mm!=0)$fatal(1,"task3 baseline count interpretation changed");
   bt_report(0,2,0);if(stby)$fatal(1,"B failed before route launch");
   bt_neutral();bt_report(0,8,0);cycles(100000);
   if(!dut.route_mode || dut.pose_signs!=2 || dut.state!=8 || dut.path.RIGHT_SIGN!=-1)$fatal(1,"X did not select consistent navigation mapping");
   if(dut.x_mm!=0 || dut.y_mm!=0)$fatal(1,"X did not reset origin after polarity selection");
   force dut.left_count=32'sd1200;force dut.nav_left_count=32'sd1200;force dut.right_count=-32'sd1200;force dut.nav_right_count=-32'sd1200;cycles(25000);
   if(dut.x_mm<13 || dut.x_mm>14 || dut.y_mm!=0 || dut.pose_fault || dut.nav_failed)$fatal(1,"forward counts cancelled in navigation x=%0d y=%0d",dut.x_mm,dut.y_mm);
   // At zero yaw an equal physical in-place turn must not integrate translation.
   force dut.left_count=32'sd1300;force dut.nav_left_count=32'sd1300;force dut.right_count=-32'sd1100;force dut.nav_right_count=-32'sd1100;cycles(25000);
   if(dut.x_mm<13 || dut.x_mm>14 || dut.y_mm!=0)$fatal(1,"in-place turn integrated false distance");
   bt_report(0,2,0);if(stby)$fatal(1,"B failed after corrected navigation");
   bt_neutral();bt_report(0,0,8);cycles(100000);
   if(dut.route_mode || dut.pose_signs!=0 || dut.state!=3 || dut.x_mm!=0)$fatal(1,"switch back to START failed baseline/origin");
   rk_stop();if(stby || dut.autonomous)$fatal(1,"RK STOP after mode switch");
   $display("PASS actual START/X/B/RK packets: task3 mapping preserved, task4 negative-right correction, origin reset, forward/turn odometry");$finish;
  end
  if($test$plusargs("FEEDBACK_ONLY"))begin
   bt_report(0,0,0);bt_neutral();bt_report(0,8,0);cycles(300000);
   cycles(400000);
   if(dut.state!=8 || !dut.route_mode || !stby)$fatal(1,"feedback test did not enter navigation");
   // Unit test checks the real 2-second interval; here check fault-to-pads.
   force dut.path.missing_l=8'd100;cycles(25000);release dut.path.missing_l;
   if(stby || dut.reason!=8'h37 || dut.left_request!=0 || dut.right_request!=0)$fatal(1,"feedback fault gate state=%0d reason=%h stby=%b l=%0d r=%0d settle=%0d missing=%0d ff=%b fr=%h",dut.state,dut.reason,stby,dut.left_request,dut.right_request,dut.path.drive_settle,dut.path.missing_l,dut.path.feedback_fault,dut.path.feedback_reason);
   cycles(400000);if(stby || dut.autonomous)$fatal(1,"feedback fault auto-resumed");
   bt_report(0,2,0);bt_neutral();bt_report(0,8,0);cycles(100000);
   if(dut.state!=8 || !dut.route_mode || dut.path.failed)$fatal(1,"fresh X did not reset feedback fault");
   bt_report(0,2,0);if(stby || dut.autonomous)$fatal(1,"B after feedback recovery");
   $display("PASS feedback fault -> physical motor gate, no automatic restart, fresh X reset, B stop");$finish;
  end
`ifdef AVOIDANCE_FOCUS
  bt_report(0,0,0);bt_neutral();bt_report(0,0,8);cycles(100000);
  if(!dut.autonomous)$fatal(1,"focused START did not arm");
  force dut.distance_mm=16'd80;cycles(450000);
  if(dut.avoid_phase!=4 || !stby || dut.left_request!=-`T123_BACKUP_RPM || dut.right_request!=-`T123_BACKUP_RPM)
   $fatal(1,"focused backup state=%0d phase=%0d stby=%b l=%0d r=%0d fault=%b reason=%h",dut.state,dut.avoid_phase,stby,dut.left_request,dut.right_request,dut.hard_fault,dut.reason);
  bt_report(0,2,0);if(stby || dut.autonomous)$fatal(1,"STOP during real top backup");
  force dut.distance_mm=16'd2000;bt_neutral();bt_report(0,0,8);cycles(100000);
  force dut.distance_mm=16'd150;cycles(20);
  if(dut.state!=4)$fatal(1,"handoff fixture did not enter avoidance");
  // Geometric route is tested separately. Inject the end of a valid local
  // forward leg here to inspect the real motor gate cycle by cycle.
  force dut.distance_mm=16'd2000;
  force dut.avoidance.state=4'd7;
  cycles(120000);if(!stby)$fatal(1,"handoff fixture must be moving");
  force dut.avoidance.timer=25;
  release dut.avoidance.state;cycles(1);
  repeat(1000)begin cycles(1);if(!stby)$fatal(1,"motor enable dropped at cruise handoff");end
  if(dut.state!=3)$fatal(1,"handoff did not reach cruise");
  release dut.avoidance.timer;
  bt_report(0,2,0);if(stby)$fatal(1,"STOP after handoff");
  $display("PASS focused Bluetooth START, reverse/B STOP, uninterrupted motor gate at cruise handoff");$finish;
`endif
  key(`T123_KEY_2);cycles(120000);
  if(dut.state!=1 || dut.left_request!=`T123_CRUISE_RPM || dut.right_request!=`T123_CRUISE_RPM || !stby || !seen_pwm)$fatal(1,"IR to PWM forward");
  force dut.distance_mm=16'd400;cycles(5);
  if(dut.left_request!=(`T123_CRUISE_RPM/2) || dut.right_request!=(`T123_CRUISE_RPM/2))$fatal(1,"manual slowdown");
  force dut.distance_mm=16'd2000;
  rep();cycles(50000);key(`T123_KEY_0);cycles(5);
  if(stby || p1 || p2 || dut.state!=0)$fatal(1,"IR zero must stop");
  cycles(310000);key(`T123_KEY_8);cycles(120000);
  if(dut.left_request!=-`T123_CRUISE_RPM || dut.right_request!=-`T123_CRUISE_RPM || !stby)$fatal(1,"reverse end to end");
  key(`T123_KEY_0);cycles(310000);key(`T123_KEY_AUTO);cycles(120000);
  if(!dut.autonomous || dut.state!=3 || !stby)$fatal(1,"autonomous start");
  cycles(280000);if(!dut.autonomous || !stby)$fatal(1,"auto must survive remote release");
  force dut.distance_mm=16'd400;cycles(5);if(dut.left_request!=`T123_TURN_RPM)$fatal(1,"slow range");
  force dut.distance_mm=16'd150;cycles(20);
  if(dut.state!=4 || dut.left_request!=0 || dut.right_request!=0)$fatal(1,"brake before detour");
  key(`T123_KEY_0);cycles(5);if(stby || dut.state!=0)$fatal(1,"stop during detour");
  force dut.distance_mm=16'd2000;cycles(310000);key(`T123_KEY_2);cycles(90000);
  force dut.distance_valid=1'b0;force dut.imu_valid=1'b0;
  // Keep the manual key held so sensor loss, rather than key release, stops it.
  for(i=0;i<5;i=i+1)begin cycles(80000);rep();end
  cycles(60000);
  if(stby || dut.state!=5)$fatal(1,"stale sensors stop");
  force dut.distance_valid=1'b1;force dut.imu_valid=1'b1;
  cycles(650000);
  if(dut.state!=0 || stby)$fatal(1,"healthy sensor recovery must become idle without replay");
  key(`T123_KEY_2);cycles(100000);if(!stby || dut.state!=1)$fatal(1,"fresh key after recovery without zero/reset");
  force dut.distance_mm=16'd150;cycles(5);if(stby)$fatal(1,"front obstacle immediate brake");
  key(`T123_KEY_8);cycles(100000);
  if(!stby || dut.left_request>=0 || dut.right_request>=0)$fatal(1,"front obstacle must permit explicit backward");
  cycles(400000);if(stby)$fatal(1,"backward still requires valid repeats");
  force dut.distance_mm=16'd2000;cycles(2100000);
  key(`T123_KEY_2);cycles(100000);if(!stby || dut.motor_fault)$fatal(1,"long idle fresh key");
  $display("CHECKPOINT original NEC/safety scenarios complete t=%0t",$time);
  // New B15 UART path, retaining all prior NEC and safety regression above.
  bt_report(1,0,0);if(stby || dut.state!=0)$fatal(1,"BT takeover must first stop IR");
  bt_report(1,0,0);if(stby)$fatal(1,"held direction at connect");
  bt_neutral();bt_report(1,0,0);cycles(100000);
  $display("CHECKPOINT BT forward t=%0t",$time);
  if(!stby || dut.left_request!=`T123_CRUISE_RPM || dut.right_request!=`T123_CRUISE_RPM)$fatal(1,"BT UART to motor forward");
  key(`T123_KEY_8);if(dut.manual_action!=1)$fatal(1,"IR motion must not override BT");
  bt_report(0,0,0);if(stby)$fatal(1,"BT neutral immediate stop");
  bt_report(5,0,0);cycles(100000);if(!stby || dut.left_request>=0 || dut.right_request>=0)$fatal(1,"BT backward");
  force dut.distance_mm=16'd150;cycles(20);if(!stby)$fatal(1,"BT explicit reverse may exit front obstacle");
  bt_report(1,0,0);cycles(20);if(stby)$fatal(1,"BT forward obstacle blocking");
  force dut.distance_mm=16'd2000;bt_report(0,0,0);bt_report(3,0,0);cycles(100000);
  if(!stby || dut.left_request!=`T123_CRUISE_RPM || dut.right_request!=`T123_TURN_RPM)$fatal(1,"BT right arc");
  rk_stop();if(stby || dut.state!=0)$fatal(1,"RK UART STOP during BT");
  $display("CHECKPOINT RK STOP t=%0t",$time);
  bt_report(1,0,0);if(stby)$fatal(1,"RK STOP must require BT neutral");
  bt_neutral();bt_report(1,0,0);cycles(100000);if(!stby)$fatal(1,"BT neutral rearm");
  key(`T123_KEY_0);if(stby)$fatal(1,"IR STOP during BT");
  bt_neutral();bt_report(0,0,8);cycles(100000);
  $display("CHECKPOINT BT autonomous t=%0t",$time);
  if(!dut.autonomous || !stby)$fatal(1,"BT START autonomous");
  bt_report(0,0,0);cycles(1200000);
  if(!dut.autonomous || !stby || !dut.bt_online)$fatal(1,"single START survives silence beyond BT lease");
  force dut.distance_mm=16'd150;cycles(20);
  if(dut.state!=4 || stby)$fatal(1,"BT autonomous brakes into avoidance");
  // Allow the existing 40 ms motor reversal gate after the 300 ms brake.
  force dut.distance_mm=16'd80;cycles(450000);
  if(dut.avoid_phase!=4 || !stby || dut.left_request!=-`T123_BACKUP_RPM || dut.right_request!=-`T123_BACKUP_RPM)
   $fatal(1,"below 10 cm reverse state=%0d phase=%0d stby=%b l=%0d r=%0d fault=%b reason=%h timer=%0d",dut.state,dut.avoid_phase,stby,dut.left_request,dut.right_request,dut.hard_fault,dut.reason,dut.avoidance.timer);
  bt_report(0,2,0);if(stby || dut.autonomous)$fatal(1,"BT B stops auto");
  force dut.distance_mm=16'd2000;
  bt_neutral();bt_report(1,0,0);cycles(100000);if(!stby)$fatal(1,"BT forward before loss");
  cycles(410000);if(stby || dut.bt_online)$fatal(1,"BT silence stops within lease");
  rep();cycles(10);if(stby)$fatal(1,"old IR repeat cannot resume after BT loss");
  cycles(310000);key(`T123_KEY_2);cycles(100000);if(!stby)$fatal(1,"fresh IR fallback");
  // New mapping: START remains task 3, X is task 4. Mode changes cannot
  // occur while autonomous; STOP and a fresh neutral report must come first.
  key(`T123_KEY_0);bt_report(0,0,0);bt_neutral();bt_report(0,0,8);cycles(100000);
  if(dut.state!=3 || dut.route_mode || !stby)$fatal(1,"START lost task-three cruise");
  bt_report(0,8,0);cycles(10000);
  if(dut.state!=3 || dut.route_mode)$fatal(1,"X switched a running task-three session");
  bt_report(0,2,0);bt_report(0,8,0);cycles(10000);
  if(dut.autonomous || stby)$fatal(1,"X bypassed post-STOP neutral requirement");
  bt_neutral();corrupt_report=1;bt_report(0,8,0);corrupt_report=0;
  if(dut.autonomous || stby)$fatal(1,"bad-checksum X launched navigation");
  bt_neutral();bt_report(0,8,0);cycles(300000);
  if(dut.state!=8 || !dut.route_mode || !stby || dut.waypoint!=0)$fatal(1,"X did not launch waypoint route");
  bt_report(0,8,0);cycles(600000);
  if(dut.state!=8 || !dut.route_mode || !stby)$fatal(1,"held X or silence restarted/cancelled route");
  bt_report(0,0,8);cycles(10000);
  if(dut.state!=8 || !dut.route_mode)$fatal(1,"START switched running waypoint mode");
  bt_report(0,10,0);if(dut.autonomous || stby)$fatal(1,"B did not beat X");
  bt_neutral();bt_report(0,8,8);cycles(100000);
  if(dut.state!=3 || dut.route_mode)$fatal(1,"START plus X must select task-three cruise");
  bt_report(0,2,0);bt_neutral();bt_report(0,8,0);cycles(300000);
  force dut.distance_mm=16'd80;cycles(450000);
  if(dut.state!=4 || !dut.route_mode || dut.avoid_phase!=4 || dut.left_request!=-`T123_BACKUP_RPM)$fatal(1,"route did not call new task-three backup");
  bt_report(0,2,0);if(stby || dut.autonomous)$fatal(1,"B during waypoint backup");
  force dut.distance_mm=16'd2000;bt_neutral();bt_report(0,8,0);cycles(300000);
  rk_stop();if(stby || dut.autonomous)$fatal(1,"serial STOP during waypoint navigation");
  bt_neutral();bt_report(0,8,0);cycles(300000);
  key(`T123_KEY_0);if(stby || dut.autonomous)$fatal(1,"IR STOP during waypoint navigation");
  // Reach P1 through injected pose, keep the real UART/B/STOP and motor gate.
  bt_neutral();bt_report(0,8,0);cycles(300000);
  force dut.x_mm=32'sd690;force dut.y_mm=-32'sd220;
  force dut.left_actual=32'sd0;force dut.right_actual=32'sd0;
  cycles(300000);
  if(!dut.nav_dwelling || dut.waypoint!=0 || stby || dut.left_request!=0 || dut.right_request!=0)$fatal(1,"top dwell not stopped");
  bt_report(0,2,0);cycles(20);
  if(dut.autonomous || stby || dut.nav_dwelling)$fatal(1,"B failed during dwell");
  cycles(100000);if(stby || dut.autonomous)$fatal(1,"dwell replay after B");
  release dut.x_mm;release dut.y_mm;release dut.left_actual;release dut.right_actual;
  $display("PASS actual Bluetooth B stops intermediate dwell without replay");
  $display("PASS dual mode: START/task3, X/task4, no running switch, neutral, held/chord/invalid X, task4 backup and B/RK/IR STOP");
  $display("PASS actual NEC and Bluetooth UART to motor pads, RK/IR/B STOP, source switching, neutral/reconnect/timeout, autonomous, obstacle and stale sensors");$finish;
 end
 initial begin
  if($test$plusargs("TRACE_CLOCK"))forever begin
   #1000000;$display("TRACE t=%0t state=%0d route=%0d imu_busy=%0d",$time,dut.state,dut.route_mode,dut.imu_diagnostic_busy);
  end
 end
 initial begin #250000000;$fatal(1,"timeout");end
endmodule
