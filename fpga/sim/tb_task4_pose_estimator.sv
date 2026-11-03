`timescale 1ns/1ps
module tb_task4_pose_estimator;
 `include "route_config.vh"
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,origin=0,active=1,iv=1;
 reg signed [31:0] lc=0,rc=0;reg signed [15:0] yaw=0;
 wire signed [31:0] x,y;wire signed [15:0] h,eh,ih,d,c;
 wire valid,fault,trusted,degraded;wire [7:0] reason,quality;wire [15:0] bad,agree,age;
 task4_pose_estimator dut(clk,rst,tick,origin,active,iv,lc,rc,yaw,x,y,h,eh,ih,d,c,valid,fault,trusted,degraded,reason,quality,bad,agree,age);
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task sample;begin tick=1;cycles(1);tick=0;cycles(100);end endtask
 task reset_pose;begin origin=1;cycles(1);origin=0;sample();end endtask
 function integer norm(input integer a);norm=a>=18000?a-36000:a< -18000?a+36000:a;endfunction
 integer i,prev,oldx,oldy;real predicted;
 initial begin
  cycles(4);rst=1;cycles(4);reset_pose();repeat(30)sample();
  if(!valid || fault || !trusted || degraded || quality[7])$fatal(1,"initial trust/calibration flags");
  // 7250 physical counts of a 65 mm wheel / 1480 CPR ~= 1000.3 mm.
  for(i=0;i<725;i=i+1)begin lc=lc+10;rc=rc-10;sample();end
  if(x<999 || x>1002 || y!=0 || h!=0 || fault)$fatal(1,"straight geometry x=%0d y=%0d h=%0d",x,y,h);
  reset_pose();
  for(i=1;i<=63;i=i+1)begin
   lc=lc+10;rc=rc+10;yaw=$rtoi(-i*20.0*65*18000/(1480*110));sample();
  end
  if(eh> -9040 || eh< -9070 || x!=0 || y!=0 || fault)$fatal(1,"right turn sign/scale eh=%0d",eh);
  oldx=x;oldy=y;
  repeat(100)begin lc=lc+10;rc=rc-10;sample();end
  if(x< -3 || x>2 || y> -136 || y< -140 || fault)$fatal(1,"translation at rotated heading");
  // Cross the +/-180 degree representation without a disagreement spike.
  reset_pose();predicted=0;
  for(i=0;i<300;i=i+1)begin
   lc=lc-10;rc=rc-10;predicted=predicted+20.0*65*18000/(1480*110);
   yaw=norm($rtoi(predicted)+dut.origin_yaw);sample();
   if(fault || d>5 || d< -5)$fatal(1,"wrap disagreement d=%0d",d);
  end
  // IMU-only changes cannot rotate a stationary encoder-led pose.
  reset_pose();repeat(30)sample();prev=h;yaw=norm(yaw+600);
  repeat(100)sample();if(h!=prev || c!=0 || fault)$fatal(1,"stationary IMU rotated pose");
  // While driving, correction obeys configured slew and absolute bounds.
  repeat(160)begin
   prev=c;lc=lc+5;rc=rc-5;sample();
   if(c-prev>`T4_ODO_CORRECTION_STEP || c-prev< -`T4_ODO_CORRECTION_STEP || c>`T4_ODO_CORRECTION_LIMIT || c< -`T4_ODO_CORRECTION_LIMIT || fault)$fatal(1,"correction bound");
  end
  if(c!=`T4_ODO_CORRECTION_LIMIT)$fatal(1,"trusted correction did not reach bounded offset c=%0d",c);
  yaw=norm(yaw+1000);prev=c;sample();
  if(!degraded || trusted || c!=prev || fault)$fatal(1,"disagreement did not degrade/freeze correction");
  yaw=norm(yaw+2000);
  repeat(49)sample();if(fault)$fatal(1,"disagreement alarm early");
  sample();if(!fault || valid || reason!=8'h3c)$fatal(1,"sustained divergence must stop");
  yaw=dut.origin_yaw;repeat(10)sample();if(!fault)$fatal(1,"fault auto cleared while active");
  reset_pose();if(fault || !valid || x!=0 || y!=0)$fatal(1,"fresh reset failed");
  // Both positive and negative 32-bit count wrap produce ordinary deltas.
  lc=32'sh7ffffffc;rc=32'sh80000004;reset_pose();lc=32'sh80000006;rc=32'sh7ffffffa;sample();
  if(fault || x<1 || x>2 || h!=0)$fatal(1,"encoder count wrap");
  lc=lc+201;sample();if(!fault || reason!=8'h3d)$fatal(1,"count jump missed");
  reset_pose();lc=lc+32'sh80000000;sample();if(!fault || reason!=8'h3d)$fatal(1,"INT_MIN jump missed");
  reset_pose();iv=0;repeat(26)sample();if(!fault || reason!=8'h3e)$fatal(1,"IMU freshness timeout");
  iv=1;reset_pose();tick=1;cycles(3);tick=0;cycles(100);
  if(!fault || reason!=8'h3f)$fatal(1,"unfinished tick deadline");
  active=0;cycles(3);sample();if(fault || !valid)$fatal(1,"idle recovery");
  $display("PASS encoder-led geometry, sign, wraps, bounded IMU correction, degradation, latched disagreement, reset/freshness/jump/deadline");$finish;
 end
 initial begin #10000000;$fatal(1,"pose unit timeout");end
endmodule
