`timescale 1ns/1ps
module tb_task4_goal_avoidance;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,cancel=0,dv=0,obs=0;
 reg signed [31:0] x=1650,y=-300,actual_l=0,actual_r=0;
 reg signed [15:0] h=0;reg [15:0] distance=1000;
 wire signed [31:0] l,r;wire done,failed,waiting;wire [3:0] phase;wire [7:0] reason;
 task4_goal_avoidance dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.cancel(cancel),.clearance_verified(1'b1),
  .x_mm(x),.y_mm(y),.heading(h),.waypoint(4'd0),.left_actual(actual_l),.right_actual(actual_r),.distance_valid(dv),.distance_mm(distance),.obstacle(obs),
  .left_rpm(l),.right_rpm(r),.done(done),.failed(failed),.waiting(waiting),.phase(phase),.reason(reason));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task frame(input reg fresh);begin dv=fresh;cycles(1);dv=0;tick=1;cycles(1);tick=0;cycles(60);end endtask
 task init(input integer angle);begin
  rst=0;tick=0;dv=0;start=0;cancel=0;obs=0;distance=1000;x=1650;y=-300;h=angle;actual_l=0;actual_r=0;
  cycles(5);rst=1;cycles(5);start=1;cycles(1);start=0;cycles(40);
 end endtask
 integer mode,k,hh,turn_step,retry;reg injected,slow_sector_seen;reg [2:0] checked;
 initial begin
  // A previous probe's large distance must not complete a newly entered probe.
  init(0);force dut.state=4'd7;dut.probe_x=x-1000;dut.probe_y=y;cycles(5);
  if(!dut.forward_math_valid || dut.forward_distance!=1000)$fatal(1,"forward pipeline measurement");
  force dut.state=4'd0;cycles(1);dut.probe_x=x;dut.probe_y=y;
  force dut.state=4'd7;force dut.timer=32'd25;cycles(1);release dut.state;
  tick=1;cycles(1);tick=0;
  if(done || dut.forward_math_valid)$fatal(1,"stale probe completed new forward segment");
  cycles(5);release dut.timer;
  if(done || dut.forward_distance!=0)$fatal(1,"new probe pipeline did not settle to zero");
  $display("PASS forward distance pipeline flush prevents stale-probe completion");
  for(mode=0;mode<4;mode=mode+1)begin
   init(mode==1?-17000:0);injected=0;checked=0;slow_sector_seen=0;
   for(k=0;k<3000 && !done && !failed;k=k+1)begin
    actual_l=l;actual_r=r;hh=h;
    turn_step=mode==3 && (phase==9 || phase==10 || phase==11)?10:200;
    if(mode==3 && (phase==9 || phase==10 || phase==11) && dut.timer>250)slow_sector_seen=1;
    if(l<0 && r>0)hh=hh+turn_step;
    if(l>0 && r<0)hh=hh-turn_step;
    if(l>0 && r>0)x=x+4;
    if(hh>18000)hh=hh-36000;if(hh< -18000)hh=hh+36000;h=hh;
    distance=1000;
    if(mode==2 && phase==9 && !injected)begin distance=350;injected=1;checked=0;end
    else if(phase==9)checked[0]=1;else if(phase==10)checked[1]=1;else if(phase==11)checked[2]=1;
    frame(1);
    if((l<0 && r>0 || l>0 && r<0) && (l>22 || l< -22 || r>22 || r< -22))$fatal(1,"scan speed cap");
    if(l>0 && r>0 && (l!=59 || r!=59))$fatal(1,"avoidance forward target must be 200mm/s");
    if(l>0 && r>0 && checked!=7)$fatal(1,"forward before full sector validation mode=%0d",mode);
   end
   if(!done || failed || (mode==2 && !injected))$fatal(1,"navigation avoidance failed mode=%0d h=%0d phase=%0d reason=%h",mode,h,phase,reason);
   if(mode==3 && !slow_sector_seen)$fatal(1,"slow sector did not exercise former five-second deadline");
   $display("PASS navigation segmented search/goal/sector mode=%0d ticks=%0d",mode,k);
   cancel=1;cycles(1);if(l!=0 || r!=0 || done)$fatal(1,"cancel gate");
  end
  // A clear sector alone cannot claim successful translation when wheels stall.
  init(0);force dut.state=4'd7;cycles(1);release dut.state;
  dut.probe_x=x;dut.probe_y=y;
  repeat(105)frame(1);
  if(!failed || reason!=8'h37 || done || l!=0 || r!=0)$fatal(1,"stationary probe falsely completed");
  // Slow real displacement extends the lease, not the absolute deadline.
  init(0);force dut.state=4'd7;cycles(1);release dut.state;dut.probe_x=x;dut.probe_y=y;
  for(k=0;k<300 && !done && !failed;k=k+1)begin if(k%3==0)x=x+1;frame(1);end
  if(!done || failed || k<=100)$fatal(1,"slow forward probe rejected despite measured progress ticks=%0d",k);
  // Position chatter cannot masquerade as ever-increasing displacement.
  init(0);force dut.state=4'd7;cycles(1);release dut.state;dut.probe_x=x;dut.probe_y=y;
  for(k=0;k<120 && !failed;k=k+1)begin x=1650+(k%2)*10;frame(1);end
  if(!failed || reason!=8'h37)$fatal(1,"position oscillation renewed probe lease");
  init(0);force dut.state=4'd7;cycles(1);release dut.state;dut.probe_x=x;dut.probe_y=y;
  for(k=0;k<510 && !failed;k=k+1)begin if(k%18==0)x=x+1;frame(1);end
  if(!failed || reason!=8'h37 || k<500)$fatal(1,"forward absolute deadline not preserved ticks=%0d",k);
  init(0);force dut.state=4'd7;cycles(1);release dut.state;distance=150;obs=1;cycles(1);
  if(l!=0 || r!=0 || done)$fatal(1,"new obstacle did not immediately gate probe");
  $display("PASS forward progress lease: slow movement completes, no-motion/oscillation stops, new obstacle gates immediately");
  // Turning must reach the measured angle; stationary or oscillating heading
  // must still fault, even with fresh clear distance reports.
  for(mode=0;mode<2;mode=mode+1)begin
   init(0);force dut.state=4'd9;cycles(1);release dut.state;dut.sector_heading=0;
   for(k=0;k<270 && !failed;k=k+1)begin h=mode==0?0:(k%2)*100;frame(1);end
   if(!failed || reason!=8'h36 || l!=0 || r!=0)$fatal(1,"stalled/oscillating sector was not bounded mode=%0d",mode);
  end
  init(-2500);force dut.state=4'd10;cycles(1);release dut.state;dut.sector_heading=0;
  for(k=0;k<760 && !failed;k=k+1)begin h=h+2;frame(1);end
  if(!failed || reason!=8'h36 || k<750)$fatal(1,"sector absolute deadline not preserved ticks=%0d",k);
  $display("PASS angular progress lease, oscillation rejection and absolute sector deadline");
  // Inconsistent stopped-heading verification gets two new candidates, then
  // a bounded pause. It never authorizes blind translation or loses autonomy.
  init(0);
  for(retry=0;retry<3;retry=retry+1)begin
   force dut.state=4'd2;cycles(1);release dut.state;dut.timer=0;dut.stable=0;
   for(k=0;k<105 && phase==2;k=k+1)begin h=h+100;frame(1);end
   if(failed || done || (l>0 && r>0))$fatal(1,"verification timeout abandoned autonomy or moved forward");
   if(retry<2 && (phase!=1 || !dut.force_step))$fatal(1,"verification did not reselect candidate");
  end
  if(!waiting || reason!=8'h22 || l!=0 || r!=0)$fatal(1,"verification retry bound missing");
  repeat(90)frame(1);if(!waiting)$fatal(1,"verification wait skipped cooldown");
  repeat(20)frame(1);if(failed || waiting || dut.verify_reselects!=0)$fatal(1,"fresh clear reports did not resume verification");
  init(0);force dut.state=4'd2;cycles(1);release dut.state;actual_l=10;
  repeat(105)frame(1);if(!failed || reason!=8'h22)$fatal(1,"non-stopping wheels allowed rescan");
  $display("PASS bounded candidate reselection, cooldown/recheck, non-stopping-wheel fault");
  init(0);frame(1);repeat(26)frame(0);
  if(!failed || reason!=8'h14 || l!=0 || r!=0)$fatal(1,"stale range gate");
  init(0);distance=80;obs=1;repeat(20)frame(1);
  if(phase!=4 || l!=-15 || r!=-15)$fatal(1,"backup missing");
  x=x-101;frame(1);if(!waiting || l!=0 || r!=0)$fatal(1,"backup bound");
  cancel=1;cycles(1);if(l!=0 || r!=0)$fatal(1,"cancel waiting");
  $display("PASS pre-forward sector gate, rejection/retry, goal preference, freshness, backup and STOP");$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
