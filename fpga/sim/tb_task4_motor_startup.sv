`timescale 1ns/1ps
module tb_task4_motor_startup;
 reg clk=0;always #5 clk=~clk;
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 // Unit clock scale is 100kHz, so 2000 cycles represent each 20ms sample.
 reg urst=0,nav=1,request=0,dir=0,gate=0;
 reg signed [31:0] mag=15,cnt=0,actual=0;
 reg [9:0] base=136;
 wire [9:0] duty;
 task4_startup_assist #(.CLK_FREQ(100000)) u(
 .clk(clk),.rst_n(urst),.navigation_mode(nav),.request_active(request),.command_direction(dir),
 .drive_allowed(gate),.request_magnitude(mag),.count(cnt),.actual(actual),.base_duty(base),.duty(duty));
 task unit_reset;begin
  urst=0;request=0;gate=0;nav=1;dir=0;mag=15;cnt=0;actual=0;base=136;cycles(5);urst=1;cycles(5);
 end endtask
 integer i,peak,active_cycles;
 reg unit_done=0,plant_done=0,eq_done=0,trim_done=0;
 reg tn=1,tt=1,tr=1,tg=1,td=0;
 reg signed [31:0] tm=22,ta=0;
 reg [9:0] tb=150;wire [9:0] tout,tout_right;
 task4_turn_pwm_trim trim(.navigation_mode(tn),.turning(tt),.request_active(tr),.drive_allowed(tg),
  .command_direction(td),.request_magnitude(tm),.actual(ta),.base_duty(tb),.duty(tout));
 task4_turn_pwm_trim #(.ENCODER_SIGN(-1)) trim_right(.navigation_mode(tn),.turning(tt),.request_active(tr),.drive_allowed(tg),
  .command_direction(td),.request_magnitude(tm),.actual(ta),.base_duty(tb),.duty(tout_right));
 initial begin
  cycles(2);if(tout!=250)$fatal(1,"15 percent did not become 25 percent");
  tb=180;cycles(1);if(tout!=280)$fatal(1,"18 percent did not become 28 percent");
  tb=240;cycles(1);if(tout!=340)$fatal(1,"startup peak did not become 34 percent");
  tb=350;cycles(1);if(tout!=360)$fatal(1,"turn PWM exceeds total 36 percent cap");
  tb=150;ta=22;cycles(1);if(tout!=250 || tout_right!=150)$fatal(1,"direction qualification");
  ta=-22;cycles(1);if(tout!=150 || tout_right!=250)$fatal(1,"right encoder sign qualification");
  td=1;cycles(1);if(tout!=250 || tout_right!=150)$fatal(1,"reverse turn qualification");
  ta=-26;cycles(1);if(tout!=150)$fatal(1,"overspeed did not remove added PWM");
  ta=0;tt=0;cycles(1);if(tout!=150)$fatal(1,"straight drive was boosted");
  tt=1;tb=0;cycles(1);if(tout!=0)$fatal(1,"zero base was energized");
  tb=150;tg=0;cycles(1);if(tout!=0)$fatal(1,"reversal gate bypassed");
  tg=1;tr=0;cycles(1);if(tout!=0)$fatal(1,"STOP did not remove turn PWM");
  tn=0;cycles(1);if(tout!=150)$fatal(1,"non-navigation bypass changed");
  trim_done=1;$display("PASS +100 permille turn trim: 15->25, 18->28, 24->34 percent, cap36, speed/sign/gate/STOP, straight and task3 bypass");
 end
 initial begin
  unit_reset();request=1;gate=1;peak=0;active_cycles=0;
  repeat(160000)begin
   cycles(1);if(duty>peak)peak=duty;
   if(duty>base)active_cycles=active_cycles+1;
   if(duty>240)$fatal(1,"assist cap exceeded");
  end
  if(peak!=240 || active_cycles>100000 || duty!=base || u.state!=3 || u.attempts!=2)$fatal(1,"bounded retry/deadline failed peak=%0d cycles=%0d attempts=%0d",peak,active_cycles,u.attempts);
  // Loss/recovery of drive enable cannot replay a consumed attempt.
  gate=0;cycles(2000);gate=1;cycles(10000);if(duty!=base)$fatal(1,"repeated assist on drive gate recovery");
  // A sign change is a new episode, after the direction dead-time gate opens.
  dir=1;gate=0;cycles(5000);if(duty!=0)$fatal(1,"assist bypassed reversal gate");
  gate=1;cycles(12000);if(duty<=base)$fatal(1,"direction change did not rearm once");
  request=0;#1;if(duty!=0)$fatal(1,"STOP cancellation not immediate");cycles(30000);if(duty!=0)$fatal(1,"STOP replay");
  unit_reset();request=1;gate=1;cycles(13000);
  // Two 20ms twitches are not a successful launch. Five moving samples qualify.
  cnt=3;actual=8;cycles(2000);cnt=6;cycles(2000);
  if(u.state!=1)$fatal(1,"brief twitch incorrectly ended boost");
  for(i=0;i<3;i=i+1)begin cnt=cnt+3;cycles(2000);end
  if(u.state!=2)$fatal(1,"sustained movement did not enter fade");
  for(i=0;i<20;i=i+1)begin cnt=cnt+3;cycles(2000);end
  if(duty!=base || u.attempts!=1)$fatal(1,"moving wheel did not stay on PI");
  // Stall after handoff: no immediate kick, then one bounded recovery attempt.
  actual=0;cycles(10000);if(duty!=base)$fatal(1,"retry ignored stall qualification");
  peak=0;
  for(i=0;i<100000;i=i+1)begin cycles(1);if(duty>peak)peak=duty;if(duty>240)$fatal(1,"retry cap");end
  if(peak!=240 || u.attempts!=2 || duty!=base || u.state!=3)$fatal(1,"post-handoff recovery not bounded");
  // Coasting/stale RPM on gate-open does not permanently suppress launch assist.
  unit_reset();actual=6;request=1;gate=1;cycles(2000);actual=0;
  cycles(35000);if(duty<=base || u.attempts!=1)$fatal(1,"stale RPM consumed launch opportunity");
  // Gate interruption consumes the episode; re-opening is not a retry trigger.
  gate=0;#1;if(duty!=0)$fatal(1,"gate cancellation not immediate");cycles(1);
  gate=1;cycles(60000);if(duty!=base)$fatal(1,"gate interruption replayed boost");
  unit_reset();request=1;gate=1;cycles(10000);cnt=-8;cycles(2000);
  if(u.state!=3 || duty!=base)$fatal(1,"opposite feedback did not cancel supplement");
  unit_reset();request=1;gate=1;cycles(10000);actual=30;#1;
  if(duty!=base)$fatal(1,"overspeed did not bypass supplement immediately");cycles(2000);
  cycles(60000);if(duty!=base)$fatal(1,"overspeed caused retry");
  unit_reset();request=1;gate=1;cycles(10000);
  for(i=0;i<6;i=i+1)begin cnt=i&1;cycles(2000);if(u.state==2)$fatal(1,"single-edge chatter accepted as sustained movement");end
  unit_reset();nav=0;base=260;request=1;gate=1;cycles(20000);if(duty!=260)$fatal(1,"task3 PI bypass changed");
  unit_reset();base=260;request=1;gate=1;cycles(20000);if(duty!=260)$fatal(1,"supplement reduced existing PI output");
  // Recorded failure: a wheel keeps its forward sign when a 103 RPM straight
  // command becomes a 22 RPM pivot. The other wheel's reversal briefly closes
  // the shared drive gate. Do not classify this high-speed phase as DONE.
  unit_reset();mag=103;actual=50;request=1;gate=1;
  for(i=0;i<10;i=i+1)begin cnt=cnt+20;cycles(2000);if(duty!=base)$fatal(1,"boost during high-speed cruise");end
  mag=44;gate=0;cycles(4000);if(duty!=0)$fatal(1,"cruise gate bypass");
  mag=22;gate=1;
  for(i=0;i<5;i=i+1)begin cnt=cnt+12;cycles(2000);if(duty!=base)$fatal(1,"boost while coasting above turn target");end
  actual=0;cycles(10000);if(duty!=base)$fatal(1,"cruise-to-turn skipped stall qualification");
  peak=0;
  for(i=0;i<140000;i=i+1)begin
   cycles(1);if(duty>peak)peak=duty;
   if(duty>240)$fatal(1,"cruise-to-turn cap exceeded");
  end
  if(peak!=240 || u.attempts!=2 || duty!=base || u.state!=3)$fatal(1,"same-sign turn launch missing or unbounded");
  // Crossing the speed threshold with no real motion must not replenish retries.
  for(i=0;i<6;i=i+1)begin
   mag=103;actual=0;cycles(12000);mag=22;cycles(15000);
   if(duty!=base || u.attempts!=2)$fatal(1,"threshold chatter renewed blocked-wheel attempts");
  end
  // Sustained measured cruise is a new successful motion episode. A later
  // low-speed stall may receive a fresh bounded recovery, without a sign flip.
  mag=103;actual=40;
  for(i=0;i<6;i=i+1)begin cnt=cnt+12;cycles(2000);end
  mag=22;actual=0;cycles(35000);
  if(duty<=base || u.attempts!=1)$fatal(1,"measured cruise did not rearm later low-speed recovery");
  request=0;#1;if(duty!=0)$fatal(1,"STOP during cruise-to-turn recovery");
  $display("PASS same-sign 103-to-22 RPM pivot, shared gate/coasting, bounded stalls, threshold chatter and measured-motion rearm");
  $display("PASS unit floor <=240permille, <=2x500ms, qualified motion/retry, stale RPM, reversal, STOP, opposite/chatter/overspeed, task3 bypass");unit_done=1;
 end
 // Real PI plus hysteretic friction model. Thresholds are sensitivity cases,
 // not measured motor constants; high resistance must not cause unlimited boost.
 reg prst=0,pupdate=0,prequest=0;
 reg signed [31:0] pact=0,pcount=0;
 wire [9:0] pduty,pi_duty,old_duty;
 PI_controller_fast pi(.clk(clk),.rst_n(prst),.target_rpm(prequest?32'sd15:32'sd0),.actual_rpm(pact),.update_en(pupdate),.duty(pi_duty),.duty_pi(),.error(),.integral());
 PI_controller_fast old_pi(.clk(clk),.rst_n(prst),.target_rpm(prequest?32'sd15:32'sd0),.actual_rpm(32'sd0),.update_en(pupdate),.duty(old_duty),.duty_pi(),.error(),.integral());
 task4_startup_assist #(.CLK_FREQ(100000)) pu(.clk(clk),.rst_n(prst),.navigation_mode(1'b1),.request_active(prequest),.command_direction(1'b0),.drive_allowed(prequest),.request_magnitude(32'sd15),.count(pcount),.actual(pact),.base_duty(pi_duty),.duty(pduty));
 integer case_id,k,threshold,first_motion;reg moving;
 initial begin
  for(case_id=0;case_id<3;case_id=case_id+1)begin
   threshold=case_id==0?160:case_id==1?225:245;
   prst=0;prequest=0;pupdate=0;pact=0;pcount=0;moving=0;first_motion=-1;cycles(5);prst=1;cycles(5);prequest=1;
   for(k=0;k<90;k=k+1)begin
    cycles(1990);
    if(!moving && pduty>=threshold)begin moving=1;if(first_motion<0)first_motion=k;end
    else if(moving && pduty<125)moving=0;
    pact=moving?6:0;if(moving)pcount=pcount+3;
    pupdate=1;cycles(1);pupdate=0;cycles(9);
    if(pu.state==1 || pu.state==2)if(pduty>240)$fatal(1,"plant cap exceeded");
   end
   if(case_id<2 && (first_motion<0 || first_motion>15 || pcount<30))$fatal(1,"breakaway/handoff failed threshold=%0d counts=%0d",threshold,pcount);
   if(case_id==1 && old_duty>=threshold)$fatal(1,"old baseline unexpectedly reaches comparison threshold");
   if(case_id==2 && (pcount!=0 || pu.state!=3 || pduty!=pi_duty))$fatal(1,"blocked plant re-boosted or fabricated counts");
   prequest=0;#1;if(pduty!=0)$fatal(1,"plant STOP not immediate");
   $display("PASS PI/friction sensitivity threshold=%0d first_motion_ms=%0d count=%0d old_PI_final=%0d",threshold,first_motion<0?-1:first_motion*20,pcount,old_duty);
  end
  plant_done=1;
 end
 // Both complete controllers see the same real encoder pins. Compare every
 // cycle, including PWM pads, ramps, watchdog, telemetry, reversal and STOP.
 reg rst=0,en=0,navmode=0,run_ticks=1;
 reg signed [31:0] lreq=0,rreq=0;
 reg [1:0] lab=0,rab=0;
 reg tick=0;integer timer=0;
 wire [6:0] old_pads,new_pads;
 wire signed [31:0] olc,orc,ola,ora,nlc,nrc,nla,nra;
 wire [9:0] old_ld,old_rd,new_ld,new_rd;wire old_fault,new_fault;wire [7:0] old_reason,new_reason;
 always @(negedge clk)begin
  tick=0;
  if(timer==1999)begin timer=0;if(run_ticks)tick=1;end else timer=timer+1;
  if(rst && !navmode)begin
   if({old_pads,olc,orc,ola,ora,old_ld,old_rd,old_fault,old_reason} !==
      {new_pads,nlc,nrc,nla,nra,new_ld,new_rd,new_fault,new_reason})$fatal(1,"task3 cycle equivalence failed");
  end
 end
 motor_closed_loop #(.CLK_FREQ(100000),.PWM_FREQ(1000),.RUNTIME(1)) old_core(
 .clk(clk),.rst_n(rst),.enable(en),.command_tick(tick),
 .configured_rpm_scale(32'd132843),.configured_max_rpm(16'd60),.configured_motor_reverse(2'd3),
 .left_request(lreq),.right_request(rreq),.enc1_a(rab[1]),.enc1_b(rab[0]),.enc2_a(lab[1]),.enc2_b(lab[0]),
 .motor_pwm1(old_pads[0]),.motor_ain1(old_pads[1]),.motor_ain2(old_pads[2]),
 .motor_pwm2(old_pads[3]),.motor_bin1(old_pads[4]),.motor_bin2(old_pads[5]),.motor_stby(old_pads[6]),
 .left_count(olc),.right_count(orc),.left_actual_rpm(ola),.right_actual_rpm(ora),
 .left_duty(old_ld),.right_duty(old_rd),.fault(old_fault),.fault_reason(old_reason));
 task4_motor_closed_loop #(.CLK_FREQ(100000),.PWM_FREQ(1000),.RUNTIME(1)) new_core(
 .navigation_mode(navmode),
 .clk(clk),.rst_n(rst),.enable(en),.command_tick(tick),
 .configured_rpm_scale(32'd132843),.configured_max_rpm(16'd60),.configured_motor_reverse(2'd3),
 .left_request(lreq),.right_request(rreq),.enc1_a(rab[1]),.enc1_b(rab[0]),.enc2_a(lab[1]),.enc2_b(lab[0]),
 .motor_pwm1(new_pads[0]),.motor_ain1(new_pads[1]),.motor_ain2(new_pads[2]),
 .motor_pwm2(new_pads[3]),.motor_bin1(new_pads[4]),.motor_bin2(new_pads[5]),.motor_stby(new_pads[6]),
 .left_count(nlc),.right_count(nrc),.left_actual_rpm(nla),.right_actual_rpm(nra),
 .left_duty(new_ld),.right_duty(new_rd),.fault(new_fault),.fault_reason(new_reason));
 function [1:0] ab(input integer n);case(n&3)0:ab=0;1:ab=1;2:ab=3;3:ab=2;endcase endfunction
 integer j,k2;
 task drive_edges(input integer ticks,input integer ld,input integer rd);
 begin
  for(j=0;j<ticks;j=j+1)begin
   for(k2=0;k2<4;k2=k2+1)begin lab=ab(ld*k2);rab=ab(rd*k2);cycles(500);end
  end
 end endtask
 initial begin
  cycles(10);rst=1;cycles(100);en=1;lreq=60;rreq=60;drive_edges(30,1,-1);
  lreq=-20;rreq=-20;drive_edges(25,-1,1);
  lreq=15;rreq=-15;drive_edges(25,1,1);
  lreq=0;rreq=20;drive_edges(20,0,-1);
  en=0;cycles(2);if(new_pads!=0)$fatal(1,"STOP pads not closed");cycles(3000);
  // Exercise navigation, then return while disabled and compare the original branch again.
  navmode=1;en=1;lreq=15;rreq=-15;drive_edges(25,1,1);
  // P1-style four-second stop followed by an opposite-wheel restart with
  // stationary encoder pins. Both wheels must receive bounded breakaway duty.
  lreq=0;rreq=0;cycles(400000);
  if(new_ld!=0 || new_rd!=0)$fatal(1,"dwell has motor duty");
  lreq=-15;rreq=15;
  begin: restart_check
   integer n,peak_l,peak_r;
   peak_l=0;peak_r=0;
   for(n=0;n<100000;n=n+1)begin
    cycles(1);if(new_ld>peak_l)peak_l=new_ld;if(new_rd>peak_r)peak_r=new_rd;
   end
   if(peak_l<190 || peak_r<190)$fatal(1,"post-dwell restart missed boost L=%0d R=%0d",peak_l,peak_r);
  end
  en=0;cycles(2);if(new_pads!=0)$fatal(1,"navigation STOP pads not closed");
  cycles(3000);navmode=0;cycles(10);
  en=1;lreq=20;rreq=20;drive_edges(20,1,-1);
  en=0;cycles(3000);
  en=1;lreq=20;rreq=20;drive_edges(10,1,-1);run_ticks=0;cycles(12000);
  if(!new_fault || new_reason!=8'h10 || new_pads!=0)$fatal(1,"watchdog protection changed");
  rst=0;en=0;cycles(5);rst=1;run_ticks=1;lreq=61;rreq=0;en=1;cycles(5);
  if(!new_fault || new_reason!=8'h11 || new_pads!=0)$fatal(1,"range protection changed");
  en=0;rst=0;cycles(5);navmode=1;rst=1;run_ticks=1;lreq=103;rreq=103;en=1;cycles(30000);
  if(new_fault || !new_pads[6])$fatal(1,"navigation 103 RPM rejected");
  if(new_ld!=new_core.configurable_motors.left_motor.assisted_duty || new_rd!=new_core.configurable_motors.right_motor.assisted_duty)
   $fatal(1,"straight navigation received turn increment");
  lreq=-22;rreq=22;
  begin: rolling_pivot_check
   integer n,peak_l,peak_r;
   peak_l=0;peak_r=0;
   for(n=0;n<150000;n=n+1)begin
    cycles(1);if(new_ld>peak_l)peak_l=new_ld;if(new_rd>peak_r)peak_r=new_rd;
    if(new_ld>360 || new_rd>360)$fatal(1,"full-controller turn cap exceeded");
   end
   if(peak_l<340 || peak_r<340)$fatal(1,"full-controller turn did not reach increased startup peak L=%0d R=%0d",peak_l,peak_r);
  end
  lreq=121;cycles(5);if(!new_fault || new_reason!=8'h11 || new_pads!=0)$fatal(1,"navigation upper bound lost");
  en=0;rst=0;cycles(5);eq_done=1;$display("PASS task3 complete motor controller cycle-equivalent to immutable baseline; watchdog/range/STOP preserved");
 end
 reg range_done=0;
 initial begin: compact_range_equivalence
  integer val,lim,n;reg signed [31:0] sample_value;reg expected;
  // Exhaust all representable command values around both limits, then ensure
  // arbitrary high bits cannot turn an invalid 32-bit input into a legal byte.
  for(lim=1;lim<=120;lim=lim+1)begin
   for(val=-256;val<=256;val=val+1)begin
    expected=(val>=-lim && val<=lim);
    if(new_core.bounded_request(val,lim[7:0])!==expected)$fatal(1,"compact range mismatch %0d/%0d",val,lim);
   end
   for(n=0;n<100;n=n+1)begin
    sample_value=$random;expected=sample_value>=-lim && sample_value<=lim;
    if(new_core.bounded_request(sample_value,lim[7:0])!==expected)$fatal(1,"range truncation accepted invalid value");
   end
  end
  range_done=1;$display("PASS narrow range equivalent: all -256..256 at limits1..120 and 12000 full-width random inputs");
 end
 initial begin wait(unit_done && plant_done && eq_done && trim_done && range_done);$display("PASS all startup-assist regressions");$finish;end
 initial begin #30000000;$fatal(1,"startup suite timeout");end
endmodule
