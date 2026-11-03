`timescale 1ns/1ps
module tb_task4_navigation;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,stop=0,obs=0,slow=0,dvalid=0,hard_fault=0;
 reg [2:0] manual=0;
 reg signed [31:0] model_lc=0,model_rc=0;
 reg signed [31:0] x=0,y=0,actual_l=0,actual_r=0;reg signed [15:0] h=0;
 reg [15:0] distance=3000;
 wire signed [31:0] nl,nr,al,ar,l,r;wire nav_dwelling;wire nav_goal_reached;wire recovery_follow;wire nd,nf,ad,af,aw,ne,reset_origin,as,ac,me,autonomous,blocked;
 wire [3:0] wp,state,phase;wire [7:0] nreason,areason,reason;
 task4_path_follower path(.clk(clk),.rst_n(rst),.tick(tick),.reset_route(reset_origin),.drive_active(me),.left_count(model_lc),.right_count(model_rc),.enable(ne),.allow_advance(state==8),.direct_goal(recovery_follow),.slow(slow || state==9),.pose_valid(1'b1),
 .heading_fault(1'b0),.heading_reason(8'd0),.x_mm(x),.y_mm(y),.heading(h),.left_actual(actual_l),.right_actual(actual_r),.left_rpm(nl),.right_rpm(nr),.waypoint(wp),.done(nd),.dwelling(nav_dwelling),.goal_reached(nav_goal_reached),.failed(nf),.reason(nreason));
 task4_goal_avoidance avoid(.clk(clk),.rst_n(rst),.tick(tick),.start(as),.cancel(ac),.clearance_verified(1'b1),
 .x_mm(x),.y_mm(y),.heading(h),.distance_valid(dvalid),.distance_mm(distance),.waypoint(wp),.left_actual(actual_l),.right_actual(actual_r),.obstacle(obs),
 .left_rpm(al),.right_rpm(ar),.done(ad),.failed(af),.waiting(aw),.phase(phase),.reason(areason));
 task4_controller control(.clk(clk),.rst_n(rst),.tick(tick),.configured(1'b1),.sensors_ready(1'b1),.hard_fault(hard_fault),.stop_request(stop),.auto_start(start),.route_start(1'b1),
 .heading(h),.avoid_waiting(aw),.distance_valid(dvalid),.distance_mm(distance),.left_actual(actual_l),.right_actual(actual_r),.manual_action(manual),.released(1'b0),.safety_reason(8'h13),.avoid_reason(areason),.obstacle(obs),.slow(slow),.pose_valid(1'b1),.avoid_done(ad),.avoid_failed(af),
 .avoid_left(al),.avoid_right(ar),.nav_left(nl),.nav_right(nr),.x_mm(x),.y_mm(y),.nav_dwelling(nav_dwelling),.nav_goal_reached(nav_goal_reached),.nav_done(nd),.nav_failed(nf),.nav_reason(nreason),.waypoint(wp),
 .state(state),.reason(reason),.autonomous(autonomous),.nav_enable(ne),.recovery_follow(recovery_follow),.reset_origin(reset_origin),.avoid_start(as),.avoid_cancel(ac),.motor_enable(me),.remote_blocked(blocked),.left_rpm(l),.right_rpm(r));
 real px,py,angle,vl,vr,lin,angular,rad,range_v,noise,lag;
 real gx[0:6],gy[0:6],minimum[0:6];
 real bx0,bx1,by0,by1,closest_x,closest_y;
 real cdx,cdy,bhx,bhy,co,si;
 reg separated;
 integer scenario,step,prev_wp,prev_state,avoids,stops,fd,first_case;
 integer parked_ticks,completed_dwells;
 integer previous_l,previous_r,previous_probe;
 reg has_box,checked_stop;
 reg awaiting_resume,awaiting_arrival;reg [3:0] avoidance_goal;
 integer recovered_arrivals;
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 function real ray_distance(input real xx,yy,aa);
  real sx,sy,dx,dy,lo,hi,t1,t2,tmp;reg miss;
  begin
   dx=$cos(aa);dy=$sin(aa);sx=xx+50*dx;sy=yy+50*dy;ray_distance=3000;
   if(has_box)begin
    lo=0;hi=3000;miss=0;
    if(dx>-0.000001 && dx<0.000001)begin if(sx<bx0 || sx>bx1)miss=1;end
    else begin t1=(bx0-sx)/dx;t2=(bx1-sx)/dx;if(t1>t2)begin tmp=t1;t1=t2;t2=tmp;end if(t1>lo)lo=t1;if(t2<hi)hi=t2;end
    if(dy>-0.000001 && dy<0.000001)begin if(sy<by0 || sy>by1)miss=1;end
    else begin t1=(by0-sy)/dy;t2=(by1-sy)/dy;if(t1>t2)begin tmp=t1;t1=t2;t2=tmp;end if(t1>lo)lo=t1;if(t2<hi)hi=t2;end
    if(!miss && hi>=lo && hi>=0)ray_distance=lo;
   end
   if(ray_distance<1)ray_distance=1;
  end
 endfunction
 task initialize(input integer which);
  begin
   rst=0;start=0;stop=0;hard_fault=0;tick=0;dvalid=0;obs=0;slow=0;manual=0;
   cycles(5);px=0;py=0;angle=0;vl=0;vr=0;x=0;y=0;h=0;actual_l=0;actual_r=0;
   parked_ticks=0;completed_dwells=0;model_lc=0;model_rc=0;prev_wp=0;prev_state=0;avoids=0;stops=0;previous_l=0;previous_r=0;previous_probe=0;checked_stop=0;
   awaiting_resume=0;awaiting_arrival=0;avoidance_goal=0;recovered_arrivals=0;
   has_box=which==3; // Synthetic static obstacle fixture, not a surveyed map object.
   if(which==3)begin bx0=350;bx1=470;by0=-150;by1=-30;end
   if(which==4)begin bx0=3350;bx1=3750;by0=-2100;by1=-1200;end
   if(which==5)begin bx0=3400;bx1=3700;by0=-2250;by1=-1850;end
   lag=which==1?0.12:0.25;
   for(integer j=0;j<7;j=j+1)minimum[j]=100000;
   rst=1;cycles(10);start=1;cycles(2);start=0;cycles(10);
  end
 endtask
 initial begin
  gx[0]=690;gy[0]=-220;gx[1]=980;gy[1]=-360;gx[2]=750;gy[2]=-760;gx[3]=1150;gy[3]=-760;
  gx[4]=1690;gy[4]=-760;gx[5]=560;gy[5]=-1030;gx[6]=-70;gy[6]=-1100;
  fd=$fopen("trajectory.csv","w");$fdisplay(fd,"scenario,tick,x_mm,y_mm,heading_cdeg,waypoint,state,left_rpm,right_rpm,distance_mm");
  first_case=0;if($value$plusargs("FIRST_CASE=%d",first_case))begin end
  for(scenario=first_case;scenario<4;scenario=scenario+1)begin
   initialize(scenario);
   for(step=0;step<30000 && state!=6 && state!=5;step=step+1)begin
    rad=angle*3.141592653589793/18000;
    range_v=ray_distance(px,py,rad);distance=$rtoi(range_v);
    // A transient front obstruction tests real avoidance handoff, not static-map clearance.
    if(scenario==2 && step>=150 && step<155)distance=150;
    if(distance<200)obs=1;else if(distance>=300)obs=0;
    slow=distance<500;dvalid=1;cycles(1);dvalid=0;
    tick=1;cycles(1);tick=0;cycles(1400);
    if(me)begin vl=vl+lag*(l-vl);vr=vr+lag*(r-vr);end
    else begin vl=vl*0.55;vr=vr*0.55;end
    lin=(vl+vr)*0.5*3.403392;angular=(vr-vl)*3.403392/110;
    px=px+lin*0.02*$cos(rad+angular*0.01);py=py+lin*0.02*$sin(rad+angular*0.01);
    angle=angle+angular*0.02*18000/3.141592653589793;
    if(angle>18000)angle=angle-36000;if(angle< -18000)angle=angle+36000;
    rad=angle*3.141592653589793/18000;
    noise=scenario==1?25*$sin(step*0.31):0;
    x=$rtoi(px);y=$rtoi(py);h=$rtoi(angle+noise);if(h>18000)h=h-36000;if(h< -18000)h=h+36000;
    actual_l=$rtoi(vl);actual_r=$rtoi(vr);model_lc=model_lc+$rtoi(vl*1480/3000);model_rc=model_rc+$rtoi(vr*1480/3000);
    if(l>103 || l< -103 || r>103 || r< -103)$fatal(1,"speed bounds");
    for(integer j=0;j<7;j=j+1)begin
     range_v=$sqrt((px-gx[j])*(px-gx[j])+(py-gy[j])*(py-gy[j]));if(range_v<minimum[j])minimum[j]=range_v;
    end
    if((state==4 || state==7) && (prev_state==4 || prev_state==7) && wp!=prev_wp)
     $fatal(1,"waypoint changed while local avoidance owns motion");
    if(state==8 && l==0 && r==0 && actual_l<=3 && actual_l>=-3 && actual_r<=3 && actual_r>=-3 &&
       $sqrt((px-gx[prev_wp])*(px-gx[prev_wp])+(py-gy[prev_wp])*(py-gy[prev_wp]))<=52)
      parked_ticks=parked_ticks+1;
    else if(wp==prev_wp && state!=6)parked_ticks=0;
    if(wp!=prev_wp)begin
     if(parked_ticks<200)$fatal(1,"goal advanced without four stopped seconds case=%0d wp=%0d ticks=%0d",scenario,prev_wp,parked_ticks);
     completed_dwells=completed_dwells+1;parked_ticks=0;
     if(wp!=prev_wp+1 || minimum[prev_wp]>52)$fatal(1,"waypoint skipped or outside gate");
     if(awaiting_arrival)begin
      if(prev_wp!=avoidance_goal)$fatal(1,"avoidance resumed toward a different goal");
      recovered_arrivals=recovered_arrivals+1;awaiting_arrival=0;
     end
     $display("GATE case=%0d wp=%0d t=%f closest=%f",scenario,prev_wp,step*0.02,minimum[prev_wp]);prev_wp=wp;
    end
    if(state==4 && prev_state!=4)begin
     avoids=avoids+1;avoidance_goal=wp;awaiting_resume=1;
    end
    if(awaiting_resume && state==8)begin
     if(wp!=avoidance_goal || !recovery_follow)$fatal(1,"avoidance lost original waypoint before navigation resumed");
     awaiting_resume=0;awaiting_arrival=1;
    end
    if(state==8 && prev_state==8 && l<=0 && r<=0 && step>30 && !(wp==6 && minimum[6]<100))stops=stops+1;
    if(prev_state==9 && state==8 && previous_probe!=4 && !nav_goal_reached)
     $fatal(1,"navigation released without completed sector validation");
    previous_probe=control.rejoin_phase;prev_state=state;
    closest_x=px+50*$cos(rad);closest_y=py+50*$sin(rad);co=$cos(rad);si=$sin(rad);
    // User confirmed the sheet is not a physical wall: body may extend outside.
    if(has_box)begin
     // Exact oriented 200 x 145 mm footprint with axle 150 mm behind nose.
     // SAT on both rectangle axes; a circumscribed disc falsely rejects some
     // corner clearances during stationary rotation of this asymmetric body.
     closest_x=px+50*$cos(rad);closest_y=py+50*$sin(rad);
     cdx=closest_x-(bx0+bx1)/2;cdy=closest_y-(by0+by1)/2;bhx=(bx1-bx0)/2;bhy=(by1-by0)/2;
     co=$cos(rad);si=$sin(rad);
     separated=$abs(cdx)>bhx+100*$abs(co)+72.5*$abs(si) || $abs(cdy)>bhy+100*$abs(si)+72.5*$abs(co) ||
      $abs(cdx*co+cdy*si)>100+bhx*$abs(co)+bhy*$abs(si) || $abs(-cdx*si+cdy*co)>72.5+bhx*$abs(si)+bhy*$abs(co);
     if(!separated)$fatal(1,"body collision case=%0d x=%f y=%f state=%0d phase=%0d",scenario,px,py,state,phase);
    end
    if(step%10==0)$fdisplay(fd,"%0d,%0d,%f,%f,%f,%0d,%0d,%0d,%0d,%0d",scenario,step,px,py,angle,wp,state,l,r,distance);
   end
   if(state!=6)$fatal(1,"route failed case=%0d state=%0d reason=%h wp=%0d x=%f y=%f h=%f avoids=%0d",scenario,state,reason,wp,px,py,angle,avoids);
   if(minimum[6]>100 || me || l!=0 || r!=0)$fatal(1,"goal not stopped");
   if(completed_dwells!=6 || parked_ticks<200)$fatal(1,"missing waypoint/final dwell");
   if(scenario>=2 && avoids==0)$fatal(1,"obstacle scenario did not engage original avoidance");
   if(scenario>=2 && recovered_arrivals==0)$fatal(1,"obstacle scenario never resumed original goal and completed its dwell");
   if(scenario>=2)$display("PASS original-goal recovery and four-second dwell case=%0d recovered_arrivals=%0d",scenario,recovered_arrivals);
   $display("PASS ROUTE case=%0d seconds=%f goal_error=%f avoidances=%0d stop_ticks=%0d",scenario,step*0.02,minimum[6],avoids,stops);
   cycles(20);if(me)$fatal(1,"completion resumed without fresh start");
  end
  // Reproduce the measured P1 stop; the next goal is nearly straight ahead.
  initialize(0);x=653;y=-253;h=-2149;actual_l=0;actual_r=0;
  repeat(220)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=1 || l<=0 || r<=0 || nf)$fatal(1,"P1 departure unnecessarily spins instead of approaching P2");
  // At P2 the P3 bearing needs a large clockwise turn; preserve P3 until reached.
  x=980;y=-360;h=-1800;
  repeat(220)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=2 || l<=0 || r>=0 || nf)$fatal(1,"P2 to P3 failed to request the required clockwise turn");
  // Complete that turn using observed heading, never elapsed time alone.
  h=-12000;actual_l=0;actual_r=0;
  repeat(15)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=2 || l<=0 || r<=0 || nf)$fatal(1,"P2 to P3 did not resume forward after alignment");
  $display("PASS measured P1 departure, P2/P3 turn direction, target retention and forward resumption");
  // Stopped clock is based on measured wheels, and interruption never skips a goal.
  initialize(0);x=690;y=-220;actual_l=12;actual_r=12;
  repeat(240)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=0 || path.dwell_ticks!=0 || l!=0 || r!=0)$fatal(1,"moving car counted as parked");
  actual_l=0;actual_r=0;
  repeat(100)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=0 || !nav_dwelling || path.dwell_ticks==0)$fatal(1,"no dwell at reached goal");
  force control.session_ticks=30000;force control.progress_ticks=3000;cycles(3);
  if(state!=8 || me)$fatal(1,"planned dwell tripped deadline");
  release control.session_ticks;release control.progress_ticks;
  force control.session_ticks=100;force control.progress_ticks=100;cycles(2);
  release control.session_ticks;release control.progress_ticks;
  actual_l=5;repeat(2)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(path.dwell_ticks!=0 || wp!=0)$fatal(1,"motion did not restart dwell");
  actual_l=0;
  x=800;y=-220;repeat(2)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(nav_dwelling || path.dwell_ticks!=0 || wp!=0)$fatal(1,"drift outside gate did not reset dwell");
  x=690;y=-220;
  repeat(100)begin tick=1;cycles(1);tick=0;cycles(1400);end
  obs=1;cycles(5);if(state!=4 || wp!=0 || path.dwell_ticks!=0)$fatal(1,"obstacle did not cancel dwell retaining goal");
  // Model a completed avoidance handoff back at the same goal; no stale timer.
  obs=0;force control.state=4'd9;cycles(4);release control.state;
  repeat(100)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(state!=8 || wp!=0 || path.dwell_ticks>=100)$fatal(1,"rejoin skipped fresh dwell");
  stop=1;cycles(3);if(state!=0 || me || path.dwell_ticks!=0)$fatal(1,"STOP during dwell");
  stop=0;repeat(240)begin tick=1;cycles(1);tick=0;cycles(1400);end
  if(state!=0 || wp!=0 || me)$fatal(1,"dwell resumed after STOP");
  initialize(0);x=690;y=-220;
  repeat(40)begin tick=1;cycles(1);tick=0;cycles(1400);end
  hard_fault=1;cycles(3);if(state!=5 || me || path.dwell_ticks!=0)$fatal(1,"fault during dwell");
  $display("PASS stopped-wheel dwell, 4s restart, avoid/rejoin retention, STOP and fault during dwell");
  // An intermediate gate reached while aligning is NOT consumed before NAV.
  initialize(0);force control.state=4'd9;x=690;y=-220;
  repeat(20)begin dvalid=1;cycles(1);dvalid=0;tick=1;cycles(1);tick=0;cycles(1400);end
  if(wp!=0)$fatal(1,"rejoin advanced waypoint before navigation release");
  release control.state;
  // Repeated encounters in the same region increase separation, with a cap.
  initialize(0);obs=1;cycles(10);
  if(!control.encounter_valid || !recovery_follow || control.depart_distance!=350)$fatal(1,"first encounter memory");
  obs=0;force ad=1;cycles(2);release ad;x=360;cycles(10);
  if(state!=9)$fatal(1,"first separation did not rejoin");
  obs=1;cycles(10);
  if(wp!=0 || !recovery_follow || control.depart_distance!=450)$fatal(1,"repeated encounter did not increase separation");
  stop=1;cycles(3);if(me)$fatal(1,"stop after repeated encounter");
  // Asynchronous output gates and cancel during real local-avoidance state.
  initialize(0);obs=1;cycles(10);if(state!=4 || me)$fatal(1,"obstacle takeover");
  stop=1;cycles(2);if(me || state!=0 || !ac)$fatal(1,"STOP during avoidance");
  stop=0;cycles(20);if(me)$fatal(1,"STOP replay");
  initialize(0);hard_fault=1;cycles(2);if(me || state!=5)$fatal(1,"sensor fault priority");
  hard_fault=0;cycles(50);if(me)$fatal(1,"fault replay");
  // Exercise the new bounded-mission guards without simulating ten minutes.
  initialize(0);force control.session_ticks=30000;cycles(2);
  if(state!=5 || reason!=8'h30 || me)$fatal(1,"session deadline gate");
  release control.session_ticks;
  initialize(0);force control.progress_ticks=3000;cycles(2);
  if(state!=5 || reason!=8'h31 || me)$fatal(1,"no-progress deadline gate");
  release control.progress_ticks;
  // A blocked wait is not lost navigation progress and must survive both
  // deadlines. The same goal is retained and fresh clearance resumes search.
  initialize(0);obs=1;cycles(10);
  force avoid.state=4'd8;distance=150;dvalid=1;
  force control.session_ticks=30000;force control.progress_ticks=3000;
  cycles(10);if(state!=4 || me || wp!=0 || !aw)$fatal(1,"waiting lost target or tripped navigation timers");
  release control.session_ticks;release control.progress_ticks;
  force control.session_ticks=100;force control.progress_ticks=100;cycles(2);
  release control.session_ticks;release control.progress_ticks;
  release avoid.state;obs=0;distance=700;
  repeat(20)begin tick=1;cycles(1);tick=0;cycles(40);end
  if(state!=4 || aw || wp!=0)$fatal(1,"waiting did not resume original scan");
  stop=1;cycles(2);if(me || state!=0)$fatal(1,"STOP after blocked wait");
  // A single apparently clear reading cannot release the rejoin motor gate.
  initialize(0);force control.state=4'd9;force nl=20;force nr=20;dvalid=0;cycles(3);
  if(me)$fatal(1,"rejoin translated without fresh observations");
  distance=700;dvalid=1;cycles(1);dvalid=0;cycles(2);
  if(me || control.clear_samples!=1)$fatal(1,"single clear sample accepted");
  distance=450;dvalid=1;cycles(1);dvalid=0;cycles(2);
  if(control.clear_samples!=0 || me)$fatal(1,"near report did not reject rejoin");
  release control.state;release nl;release nr;
  // A central ray cannot authorize translation until BOTH sides and centre pass.
  initialize(0);force control.state=4'd9;force nl=20;force nr=20;distance=700;h=0;dvalid=0;cycles(3);
  repeat(3)begin dvalid=1;cycles(1);dvalid=0;cycles(2);end
  if(control.rejoin_phase!=1 || ne || (l>0 && r>0))$fatal(1,"central ray bypassed sector scan");
  h=-2000;cycles(2);repeat(3)begin dvalid=1;cycles(1);dvalid=0;cycles(2);end
  if(control.rejoin_phase!=2)$fatal(1,"negative sector endpoint not verified");
  h=2000;cycles(2);repeat(3)begin dvalid=1;cycles(1);dvalid=0;cycles(2);end
  if(control.rejoin_phase!=3)$fatal(1,"positive sector endpoint not verified");
  h=0;cycles(2);repeat(3)begin dvalid=1;cycles(1);dvalid=0;cycles(2);end
  if(control.rejoin_phase!=4 || me)$fatal(1,"sector return translated before final approval");
  release control.state;
  repeat(3)begin dvalid=1;cycles(1);dvalid=0;cycles(2);end
  if(state!=8)$fatal(1,"valid sector could not release navigation");
  release nl;release nr;
  initialize(0);force control.state=4'd9;force control.rejoin_phase=1;distance=450;dvalid=1;cycles(1);dvalid=0;
  release control.state;release control.rejoin_phase;dvalid=1;cycles(2);dvalid=0;
  if(state!=4 || wp!=0)$fatal(1,"blocked side sector did not restart avoidance retaining goal");
  initialize(0);force control.state=4'd9;force control.rejoin_ticks=750;cycles(1);
  release control.state;cycles(2);
  if(state!=5 || reason!=8'h35 || me)$fatal(1,"sector timeout did not stop");
  release control.rejoin_ticks;
  $fclose(fd);$display("PASS navigation integration, waypoint retention, STOP/fault/deadline/wait/rejoin gates");$finish;
 end
 initial begin #6000000000;$fatal(1,"wall simulation timeout");end
endmodule

// Feedback regression: actual quadrature pins -> encoder -> pose -> follower.
// Physics feeds decoded IMU angle; this does not emulate sensor UART framing.
module tb_task4_odometry #(parameter integer RAW_RIGHT_SIGN=1,RIGHT_SIGN=1);
 integer mode;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,origin=0,enable=0;
 reg [1:0] lab=0,rab=0;
 reg signed [15:0] yaw=0;
 reg sample_imu=0;wire signed [15:0] filtered_yaw;wire filtered_valid;
 angle_iir_filter #(.SHIFT(2)) real_filter(.clk(clk),.rst_n(rst),.angle_in_cdeg(yaw),
  .sample_valid(sample_imu),.angle_out_cdeg(filtered_yaw),.output_valid(filtered_valid));
 wire signed [31:0] lc,rc,x,y,l,r;wire signed [15:0] h;
 reg signed [31:0] la=0,ra=0;
 wire valid,fault,done,failed,arrived,dwelling;wire [3:0] wp;wire [7:0] reason;
 task4_encoder_feedback #(.CLK_FREQ(1000000)) le(.clk(clk),.rst_n(rst),.enc_a(lab[1]),.enc_b(lab[0]),.count(lc),.rejected(),.invalid(),.accepted_state());
 task4_encoder_feedback #(.CLK_FREQ(1000000)) re(.clk(clk),.rst_n(rst),.enc_a(rab[1]),.enc_b(rab[0]),.count(rc),.rejected(),.invalid(),.accepted_state());
 wire [7:0] pose_reason;wire degraded;
 task4_pose_estimator #(.RIGHT_SIGN(RIGHT_SIGN)) pose(.clk(clk),.rst_n(rst),.tick(tick),.reset_origin(origin),.active(enable),.imu_valid(mode==10?filtered_valid:1'b1),.reason(pose_reason),.degraded(degraded),
 .left_count(lc),.right_count(rc),.yaw_cdeg(mode==10?filtered_yaw:yaw),.x_mm(x),.y_mm(y),.heading(h),.valid(valid),.fault(fault));
 task4_path_follower #(.RIGHT_SIGN(RIGHT_SIGN)) path(.clk(clk),.rst_n(rst),.tick(tick),.reset_route(origin),.drive_active(enable),.left_count(lc),.right_count(rc),.enable(enable),.allow_advance(1'b1),.direct_goal(1'b0),.slow(degraded),.pose_valid(valid),.heading_fault(fault),.heading_reason(pose_reason),
 .x_mm(x),.y_mm(y),.heading(h),.left_actual(la),.right_actual(ra),.left_rpm(l),.right_rpm(r),.waypoint(wp),.done(done),.failed(failed),.goal_reached(arrived),.dwelling(dwelling),.reason(reason));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 function [1:0] enc(input integer n);case(n&3)0:enc=0;1:enc=1;2:enc=3;3:enc=2;endcase endfunction
 real px=0,py=0,a=0,vl=0,vr=0,cl=0,cr=0,rad,ang,lin,err;
 real gx[0:6],gy[0:6];integer il=0,ir=0,tl,tr,k,oldwp=0,parked=0,noise=0,burst,phase0,prev_lc=0,prev_rc=0;
 initial begin
 gx[0]=690;gy[0]=-220;gx[1]=980;gy[1]=-360;gx[2]=750;gy[2]=-760;gx[3]=1150;gy[3]=-760;
 gx[4]=1690;gy[4]=-760;gx[5]=560;gy[5]=-1030;gx[6]=-70;gy[6]=-1100;
 mode=0;if($value$plusargs("MODE=%d",mode))begin end
 cycles(5);rst=1;cycles(5);sample_imu=1;cycles(1);sample_imu=0;cycles(3);origin=1;cycles(2);origin=0;tick=1;cycles(1);tick=0;cycles(800);
 if(!valid)$fatal(1,"pose init did not complete");enable=1;
 if(mode==4)begin force x=0;force y=0;end
 for(k=0;k<15000 && !done && !failed && !fault;k=k+1)begin
 tick=1;cycles(1);tick=0;cycles(800);
 // Match top-level validation/degraded caps, including during alignment.
 vl=vl+0.20*((l>(degraded?44:103)?(degraded?44:103):l< -(degraded?44:103)?-(degraded?44:103):l)-vl);
 vr=vr+0.20*((r>(degraded?44:103)?(degraded?44:103):r< -(degraded?44:103)?-(degraded?44:103):r)-vr);
 // Delayed physical start models a bounded 240ms breakaway delay; PWM friction is tested in tb_task4_motor_startup.
 if(mode==6 && k<12)begin vl=0;vr=0;end
 rad=a*3.141592653589793/18000;
 ang=(vr-vl)*3.403392/110;lin=(vl+vr)*0.5*3.403392;
 px=px+lin*0.02*$cos(rad+ang*0.01);py=py+lin*0.02*$sin(rad+ang*0.01);
 a=a+ang*0.02*18000/3.141592653589793;if(a>18000)a=a-36000;if(a< -18000)a=a+36000;
 cl=cl+vl*1480/3000;cr=cr+vr*1480/3000;tl=$rtoi(cl);tr=$rtoi(cr);
 // Reverse-going legal edge stream: raw data is preserved, never fabricated as forward.
 if(mode==7 && k>=50 && k<130)noise=noise+24;
 if(mode==7)tr=tr+RAW_RIGHT_SIGN*noise;
 while(il!=tl || ir!=tr)begin
 if(il<tl)il=il+1;else if(il>tl)il=il-1;
 if(ir<tr)ir=ir+1;else if(ir>tr)ir=ir-1;
 if(mode==0 || mode==3 || mode==4 || mode==5 || mode==6 || mode==7 || mode==8 || mode==9 || mode==10)begin lab=enc(il);rab=enc(RAW_RIGHT_SIGN*ir);end
 else if(mode==2)begin lab=enc(-il);rab=enc(RAW_RIGHT_SIGN*ir);end
 cycles(20);
 end
 if(mode==8 && (k==50 || k==65 || k==80 || k==95))begin
  phase0=RAW_RIGHT_SIGN*ir;
  for(burst=1;burst<=160;burst=burst+1)begin rab=enc(phase0+burst);cycles(1);end
 end
 cycles(20);
 yaw=mode==3?0:mode==9?$rtoi(a*0.6666667):$rtoi(a);la=$rtoi((lc-prev_lc)*3000.0/1480);ra=$rtoi((rc-prev_rc)*3000.0/1480);prev_lc=lc;prev_rc=rc;
 // Production SHIFT=2 angle filter at the 10 Hz rate measured on the car.
 if(mode==10 && k%5==0)begin sample_imu=1;cycles(1);sample_imu=0;cycles(3);end
 if(l==0 && r==0 && la<=3 && la>=-3 && ra<=3 && ra>=-3)parked=parked+1;
 else if(wp==oldwp)parked=0;
 if(wp!=oldwp)begin
 err=$sqrt((px-gx[oldwp])*(px-gx[oldwp])+(py-gy[oldwp])*(py-gy[oldwp]));
 if(err>65 || parked<200)$fatal(1,"feedback arrival failed wp=%0d physical_error=%f parked=%0d",oldwp,err,parked);
 $display("ENCODER GATE wp=%0d t=%f physical_error=%f estimated=(%0d,%0d)",oldwp,k*0.02,err,x,y);
 oldwp=wp;parked=0;
 end
 end
 if(mode!=0 && mode!=6 && mode!=8 && mode!=10)begin
  cycles(4);
  if(!failed || l!=0 || r!=0 || ((mode==3 || mode==9)?(wp>2 || k>3000):(wp!=0 || k>200)))$fatal(1,"bad feedback did not stop in bounded time mode=%0d k=%0d wp=%0d reason=%h",mode,k,wp,reason);
  // Mode 1 has physical IMU rotation but no encoder feedback: the new
  // disagreement timer expires before the original two-second count lease.
  if((mode==1 && reason!=8'h3c) || ((mode==2 || mode==5 || mode==7) && reason!=8'h38) || ((mode==3 || mode==9) && reason!=8'h3c) || (mode==4 && reason!=8'h39))$fatal(1,"wrong diagnostic mode=%0d reason=%h",mode,reason);
  $display("PASS feedback guard mode=%0d stop_at=%f physical=(%f,%f) estimated=(%0d,%0d) reason=%h",mode,k*0.02,px,py,x,y,reason);
  enable=0;cycles(4);enable=1;repeat(220)begin tick=1;cycles(1);tick=0;cycles(100);end
  if(!failed || l!=0 || r!=0 || wp!=oldwp)$fatal(1,"feedback fault resumed without reset");
  origin=1;cycles(2);origin=0;cycles(2);if(failed || reason!=0)$fatal(1,"new route did not clear diagnostic latch");
  $finish;
 end
 if(!done || failed || fault || wp!=6 || parked<200)$fatal(1,"feedback route failure k=%0d done=%b failed=%b pose_fault=%b wp=%0d x=%0d y=%0d",k,done,failed,fault,wp,x,y);
 $display("PASS quadrature pins and IMU -> pose -> all waypoint stops, raw_right=%0d corrected_right=%0d t=%f",RAW_RIGHT_SIGN,RIGHT_SIGN,k*0.02);$finish;
 end
 initial begin #4000000000;$fatal(1,"feedback test timeout");end
endmodule

module tb_task4_feedback_window;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,origin=0,enable=0,drive=1;
 reg signed [31:0] lc=0,rc=0,x=0,y=0,la=0,ra=0;
 wire signed [31:0] l,r;wire [3:0] wp;wire done,failed,arrived,dwelling;wire [7:0] reason;
 integer mode,k,motion_frames=0;reg backward_encoder=0;
 // A physically forward left wheel can legitimately count negative; use
 // the SAME calibrated sign in both odometry and feedback checking.
 task4_path_follower #(.LEFT_SIGN(-1),.RIGHT_SIGN(1)) path(
 .clk(clk),.rst_n(rst),.tick(tick),.reset_route(origin),.enable(enable),.drive_active(drive),.left_count(lc),.right_count(rc),
 .allow_advance(1'b1),.direct_goal(1'b0),.slow(1'b0),.pose_valid(1'b1),.heading_fault(1'b0),.heading_reason(8'd0),.x_mm(x),.y_mm(y),.heading(-16'sd1800),
 .left_actual(la),.right_actual(ra),.left_rpm(l),.right_rpm(r),.waypoint(wp),.done(done),.failed(failed),.goal_reached(arrived),.dwelling(dwelling),.reason(reason));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 initial begin
 mode=0;if($value$plusargs("WINDOW_MODE=%d",mode))begin end
 cycles(5);rst=1;cycles(5);origin=1;cycles(1);origin=0;enable=1;
 for(k=0;k<300 && !failed;k=k+1)begin
 drive=!(mode==3 || (mode==2 && k<75));
 if(drive && k>20)begin
  if(mode==4)begin lc=(k&1)?1:0;rc=(k&1)?1:0;end
  else begin lc=lc-4;rc=rc+4;end
  if(mode!=4)begin x=x+1;if(k%3==0)y=y-1;end
 end
 la=mode==1?(l>0?-10:l<0?10:0):0;ra=mode==1?(r>0?10:r<0?-10:0):0;
 tick=1;cycles(1);tick=0;cycles(800);
 if(l>=8 && r>=8)motion_frames=motion_frames+1;
 end
 if(mode<=2)begin
  if(failed || wp!=0 || done || motion_frames<100)$fatal(1,"false feedback alarm mode=%0d reason=%h",mode,reason);
  $display("PASS no false trip window_mode=%0d zero-RPM/count-progress/calibrated-sign/startup-wait",mode);
 end else begin
  if(!failed || l!=0 || r!=0 || (mode==3 && reason!=8'h3b) || (mode==4 && reason!=8'h37))$fatal(1,"missing drive/count chatter not distinguished mode=%0d reason=%h",mode,reason);
  $display("PASS bounded feedback stop window_mode=%0d reason=%h time=%f",mode,reason,k*0.02);
 end
 $finish;
 end
 initial begin #20000000;$fatal(1,"feedback window timeout");end
endmodule
