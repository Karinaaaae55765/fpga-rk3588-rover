`default_nettype none
// Upstream motor limits: OUT_MAX=260, DUTY_MAX=360, DEAD_ZONE=120.
// No UART velocity bypass. Encoder inactivity no longer latches a fault;
// tick watchdog, range checks, output limits and reversal gates are retained.
module motor_closed_loop #(
 parameter integer CLK_FREQ=50000000,OUTPUT_UNLOCK=1,COUNTS_PER_REV=1480,
 parameter integer PWM_FREQ=20000,MAX_RPM=60,RUNTIME=0
)(
 input wire clk,rst_n,enable,command_tick,
 input wire [31:0] configured_rpm_scale,
 input wire [15:0] configured_max_rpm,
 input wire [1:0] configured_motor_reverse,
 input wire signed [31:0] left_request,right_request,
 input wire enc1_a,enc1_b,enc2_a,enc2_b,
 output wire motor_pwm1,motor_ain1,motor_ain2,motor_pwm2,motor_bin1,motor_bin2,motor_stby,
 output wire signed [31:0] left_count,right_count,left_actual_rpm,right_actual_rpm,
 output wire [9:0] left_duty,right_duty,
 output reg fault,output reg [7:0] fault_reason
);
 wire ld=left_request<0,rd=right_request<0;
 // Pipeline magnitudes before the 32-bit ramp arithmetic. Raw sign/range and
 // enable still close the physical gate immediately on reversal/STOP/fault.
 reg signed [31:0] lm,rm;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin lm<=0;rm<=0;end
  else begin lm<=ld?-left_request:left_request;rm<=rd?-right_request:right_request;end
 wire signed [31:0] rpm_limit=RUNTIME?$signed({16'd0,configured_max_rpm}):MAX_RPM;
 wire range_ok=rpm_limit>0 && rpm_limit<=60 &&
               left_request>=-rpm_limit && left_request<=rpm_limit &&
               right_request>=-rpm_limit && right_request<=rpm_limit;
 localparam integer LEASE=CLK_FREQ/1000*100,SETTLE=CLK_FREQ/1000*40;
 reg [31:0] age,settling;
 reg lease_seen;
 reg [1:0] direction;
 wire changed=direction!={ld,rd};
 wire allowed=OUTPUT_UNLOCK!=0 && enable && range_ok && age<LEASE && !fault;
 wire drive=allowed && !changed && settling==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin age<=LEASE;settling<=SETTLE;direction<=0;fault<=0;fault_reason<=0;lease_seen<=0;end
  else begin
   if(!enable)begin age<=LEASE;lease_seen<=0;end
   else if(command_tick)begin age<=0;lease_seen<=1;end
   else if(age<LEASE)age<=age+1'b1;
   if(!allowed || changed)begin direction<={ld,rd};settling<=SETTLE;end
   else if(settling!=0)settling<=settling-1'b1;
   // Once an active controller has lost its tick lease, a later tick must
   // not silently resume the motor. Explicit FPGA reset is required.
   if(enable && !fault)begin
    if(!range_ok)begin fault<=1;fault_reason<=8'h11;end
    else if(lease_seen && age>=LEASE)begin fault<=1;fault_reason<=8'h10;end
   end
  end
 end
 wire signed [31:0] ramp_l,ramp_r;wire ramp_ld,ramp_rd;
 dual_rpm_command_ramp #(.CLK_FREQ(CLK_FREQ)) ramp(
  .clk(clk),.rst_n(rst_n),.enable(drive),
  .target_left_rpm(lm),.target_right_rpm(rm),.target_left_dir(ld),.target_right_dir(rd),
  .left_rpm(ramp_l),.right_rpm(ramp_r),.left_dir(ramp_ld),.right_dir(ramp_rd));
 wire lp,la,lb,rp,ra,rb;
 reg signed [31:0] pi_left_target,pi_right_target;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pi_left_target<=0;pi_right_target<=0;end
  else begin pi_left_target<=drive?ramp_l:32'sd0;pi_right_target<=drive?ramp_r:32'sd0;end
 end
 generate if(RUNTIME)begin: configurable_motors
 tasks123_runtime_motor #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ)) left_motor(
  .clk(clk),.rst_n(rst_n),.target(pi_left_target),.direction(ramp_ld),
  .scale_q16(configured_rpm_scale),.enc_a(enc2_a),.enc_b(enc2_b),
  .count(left_count),.actual(left_actual_rpm),.duty(left_duty),.pwm(lp),.in1(la),.in2(lb));
 tasks123_runtime_motor #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ)) right_motor(
  .clk(clk),.rst_n(rst_n),.target(pi_right_target),.direction(ramp_rd),
  .scale_q16(configured_rpm_scale),.enc_a(enc1_a),.enc_b(enc1_b),
  .count(right_count),.actual(right_actual_rpm),.duty(right_duty),.pwm(rp),.in1(ra),.in2(rb));
 end else begin: fixed_motors
 dual_motor_control #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ),.COUNTS_PER_REV(COUNTS_PER_REV),
  .SAMPLE_TIME_MS(20),.OUT_MAX(260),.DUTY_MAX(360),.DEAD_ZONE(120),.I_MAX(200000),.FAST_PI(1)) motors(
  .clk(clk),.rst_n(rst_n),.left_target_rpm(pi_left_target),.right_target_rpm(pi_right_target),
  .left_dir(ramp_ld),.right_dir(ramp_rd),.left_enc_a(enc2_a),.left_enc_b(enc2_b),
  .right_enc_a(enc1_a),.right_enc_b(enc1_b),
  .left_pwm(lp),.left_in1(la),.left_in2(lb),.right_pwm(rp),.right_in1(ra),.right_in2(rb),
  .left_encoder_count(left_count),.right_encoder_count(right_count),
  .left_actual_rpm(left_actual_rpm),.right_actual_rpm(right_actual_rpm),
  .left_duty(left_duty),.right_duty(right_duty),.left_error(),.right_error());
 end endgenerate
 wire reverse_l=RUNTIME?configured_motor_reverse[0]:1'b0;
 wire reverse_r=RUNTIME?configured_motor_reverse[1]:1'b0;
 tb6612_dual_safe_adapter #(.CLK_FREQ(CLK_FREQ),.CH1_REVERSED(RUNTIME?0:1),.CH2_REVERSED(RUNTIME?0:1)) adapter(
  .clk(clk),.rst_n(rst_n),.enable(drive),
  .ch1_pwm_in(rp),.ch1_in1_in(reverse_r?rb:ra),.ch1_in2_in(reverse_r?ra:rb),
  .ch2_pwm_in(lp),.ch2_in1_in(reverse_l?lb:la),.ch2_in2_in(reverse_l?la:lb),
  .driver_stby(motor_stby),.ch1_pwm(motor_pwm1),.ch1_in1(motor_ain1),.ch1_in2(motor_ain2),
  .ch2_pwm(motor_pwm2),.ch2_in1(motor_bin1),.ch2_in2(motor_bin2));
endmodule

// Runtime scale replaces compile-time CPR without a combinational divider.
// Host derives Q16 RPM/count for the fixed 20 ms sample interval.
module tasks123_runtime_motor #(parameter integer CLK_FREQ=50000000,PWM_FREQ=20000)(
 input wire clk,rst_n,direction,input wire signed [31:0] target,
 input wire [31:0] scale_q16,input wire enc_a,enc_b,
 output wire signed [31:0] count,output reg signed [31:0] actual,
 output wire [9:0] duty,output wire pwm,in1,in2
);
 localparam integer SAMPLE=CLK_FREQ/1000*20;
 wire feedback;
 quadrature_encoder #(.COUNT_WIDTH(32)) encoder(
  .clk(clk),.rst_n(rst_n),.enc_a(enc_a),.enc_b(enc_b),.direction(feedback),.count(count));
 reg [31:0] timer;
 reg signed [31:0] previous,delta,magnitude;
 reg signed [63:0] product;
 reg [3:0] update_pipe;
 reg pi_update;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin timer<=0;previous<=0;delta<=0;product<=0;actual<=0;magnitude<=0;update_pipe<=0;pi_update<=0;end
  else begin
   update_pipe<={update_pipe[2:0],timer==SAMPLE-1};
   if(timer==SAMPLE-1)begin timer<=0;delta<=count-previous;previous<=count;end
   else timer<=timer+1'b1;
   if(update_pipe[0])product<=delta*$signed(scale_q16);
   if(update_pipe[1])actual<=product>>>16;
   if(update_pipe[2])magnitude<=actual[31]?-actual:actual;
   pi_update<=update_pipe[2];
  end
 end
 PI_controller_fast #(.OUT_MAX(260),.I_MAX(200000),.DEAD_ZONE(120),.DUTY_MAX(360),.I_SHIFT(6)) pi(
  .clk(clk),.rst_n(rst_n),.target_rpm(target),.actual_rpm(magnitude),.update_en(pi_update),
  .duty_pi(),.duty(duty),.error(),.integral());
 motor_control #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ)) output_driver(
  .clk(clk),.rst_n(rst_n),.duty(duty),.dir(direction),.pwm_out(pwm),.in1(in1),.in2(in2));
endmodule
`default_nettype wire
