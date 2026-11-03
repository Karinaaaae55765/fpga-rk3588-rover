`timescale 1ns/1ps
module tb_task4_encoder_feedback;
 parameter integer TEST_US=5;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [1:0] ab=0;
 wire signed [31:0] raw,clean;wire [31:0] rejected,invalid;
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 function [1:0] quad(input integer n);case(n&3)0:quad=0;1:quad=1;2:quad=3;3:quad=2;endcase endfunction
 task reset;begin rst=0;ab=0;cycles(5);rst=1;cycles(30);end endtask
 quadrature_encoder original(.clk(clk),.rst_n(rst),.enc_a(ab[1]),.enc_b(ab[0]),.count(raw),.direction());
 task4_encoder_feedback #(.CLK_FREQ(1000000),.STABLE_US(TEST_US)) filter(
  .clk(clk),.rst_n(rst),.enc_a(ab[1]),.enc_b(ab[0]),.count(clean),.rejected(rejected),.invalid(invalid),.accepted_state());
 integer i;reg filter_done=0,pi_done=0;
 initial begin
  reset();
  // Legal 20us-separated edges exceed the configured 60rpm command limit;
  // qualification must not silently clamp real motion to that command limit.
  for(i=1;i<=400;i=i+1)begin ab=quad(i);cycles(20);end
  cycles(30);if(raw!=400 || clean!=400 || rejected!=0 || invalid!=0)$fatal(1,"legal forward lost edges us=%0d raw=%0d clean=%0d",TEST_US,raw,clean);
  for(i=399;i>=0;i=i-1)begin ab=quad(i);cycles(20);end
  cycles(30);if(raw!=0 || clean!=0)$fatal(1,"real reverse was suppressed");
  reset();
  // Legal quadrature bursts with each state shorter than the qualification interval.
  for(i=1;i<=160;i=i+1)begin ab=quad(i);cycles(1);end
  cycles(30);if(raw!=160 || clean!=0 || rejected==0)$fatal(1,"short quadrature burst not separated raw=%0d clean=%0d rejected=%0d",raw,clean,rejected);
  reset();ab=2'b11;cycles(30);
  if(clean!=0 || invalid!=1)$fatal(1,"two-bit jump invented movement");
  reset();
  for(i=0;i<40;i=i+1)begin ab=1;cycles(20);ab=0;cycles(20);end
  if(clean!=0)$fatal(1,"one disconnected phase produced net forward movement");
  $display("PASS qualification %0dus: real fast forward/reverse, short 160-edge burst, illegal transition, one-phase oscillation",TEST_US);filter_done=1;
 end
 reg mrst=0;reg [1:0] mab=0;
 wire signed [31:0] mc,ma,nc,na;wire [9:0] duty;
 task4_runtime_motor #(.CLK_FREQ(1000000),.PWM_FREQ(20000)) motor(
  .clk(clk),.rst_n(mrst),.target(32'sd20),.direction(1'b0),.navigation_mode(1'b1),
  .request_active(1'b1),.command_direction(1'b0),.drive_allowed(1'b1),.request_magnitude(32'sd20),
  .scale_q16(32'd132843),.enc_a(mab[1]),.enc_b(mab[0]),
  .count(mc),.actual(ma),.nav_count(nc),.nav_actual(na),.rejected(),.invalid(),.duty(duty),.pwm(),.in1(),.in2());
 integer j;
 initial begin
  cycles(5);mrst=1;cycles(30000);
  for(j=1;j<=160;j=j+1)begin mab=quad(j);cycles(1);end
  cycles(10000);
  if(mc!=160 || nc!=0 || ma<300 || na!=0 || motor.raw_pi_duty!=0 || duty<120)
   $fatal(1,"spike/PI replay failed raw_count=%0d clean=%0d raw_rpm=%0d clean_rpm=%0d raw_duty=%0d selected=%0d",mc,nc,ma,na,motor.raw_pi_duty,duty);
  $display("PASS raw burst produces >300rpm and original PI=0; task4 qualified feedback=0 and PI remains driven; raw evidence retained");pi_done=1;
 end
 initial begin wait(filter_done && pi_done);$finish;end
 initial begin #5000000;$fatal(1,"encoder test timeout");end
endmodule
