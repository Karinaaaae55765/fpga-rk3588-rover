// Task-four-local copy of the frozen motor controller; task-three bypass is cycle-equivalence tested.
`default_nettype none
// Upstream motor limits: OUT_MAX=260, DUTY_MAX=360, DEAD_ZONE=120.
// No UART velocity bypass. Encoder inactivity no longer latches a fault;
// tick watchdog, range checks, output limits and reversal gates are retained.
module task4_motor_closed_loop #(
 parameter integer CLK_FREQ=50000000,OUTPUT_UNLOCK=1,COUNTS_PER_REV=1480,
 parameter integer PWM_FREQ=20000,MAX_RPM=60,RUNTIME=0,NAV_MAX_RPM=120
)(
 input wire clk,rst_n,enable,command_tick,navigation_mode,
 input wire [31:0] configured_rpm_scale,
 input wire [15:0] configured_max_rpm,
 input wire [1:0] configured_motor_reverse,
 input wire signed [31:0] left_request,right_request,
 input wire enc1_a,enc1_b,enc2_a,enc2_b,
 output wire motor_pwm1,motor_ain1,motor_ain2,motor_pwm2,motor_bin1,motor_bin2,motor_stby,
 output wire signed [31:0] left_count,right_count,left_actual_rpm,right_actual_rpm,
 output wire [9:0] left_duty,right_duty,
 output wire signed [31:0] nav_left_count,nav_right_count,nav_left_actual,nav_right_actual,
 output wire [31:0] left_rejected,right_rejected,left_invalid,right_invalid,
 output reg fault,output reg [7:0] fault_reason
);
 wire ld=left_request<0,rd=right_request<0;
 wire nav_turn=navigation_mode && left_request!=0 && right_request!=0 && ld!=rd;
 // Pipeline magnitudes before the 32-bit ramp arithmetic. Raw sign/range and
 // enable still close the physical gate immediately on reversal/STOP/fault.
 reg signed [31:0] lm,rm;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin lm<=0;rm<=0;end
  else begin lm<=ld?-left_request:left_request;rm<=rd?-right_request:right_request;end
 wire signed [31:0] rpm_limit=navigation_mode?NAV_MAX_RPM:(RUNTIME?$signed({16'd0,configured_max_rpm}):MAX_RPM);
 // The accepted domain is at most +/-120 RPM. Check sign extension first,
 // then compare nine-bit signed values; never truncate an invalid 32-bit request.
 function automatic bounded_request(input signed [31:0] value,input [7:0] limit);
  reg signed [8:0] small_value,small_limit;
  begin
   small_value={value[7],value[7:0]};small_limit={1'b0,limit};
   bounded_request=(value=={{24{value[7]}},value[7:0]}) &&
                   small_value>=-small_limit && small_value<=small_limit;
  end
 endfunction
 wire range_ok=rpm_limit>0 && rpm_limit<=(navigation_mode?120:60) &&
               bounded_request(left_request,rpm_limit[7:0]) && bounded_request(right_request,rpm_limit[7:0]);
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
 task4_runtime_motor #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ),.ENCODER_SIGN(1)) left_motor(
  .clk(clk),.rst_n(rst_n),.target(pi_left_target),.direction(ramp_ld),
  .navigation_mode(navigation_mode),.turning(nav_turn),.request_active(enable && lm!=0),.request_magnitude(lm),.command_direction(ld),.drive_allowed(drive && ramp_ld==ld),
  .scale_q16(configured_rpm_scale),.enc_a(enc2_a),.enc_b(enc2_b),
  .count(left_count),.actual(left_actual_rpm),.nav_count(nav_left_count),.nav_actual(nav_left_actual),.rejected(left_rejected),.invalid(left_invalid),.duty(left_duty),.pwm(lp),.in1(la),.in2(lb));
 task4_runtime_motor #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ),.ENCODER_SIGN(-1)) right_motor(
  .clk(clk),.rst_n(rst_n),.target(pi_right_target),.direction(ramp_rd),
  .navigation_mode(navigation_mode),.turning(nav_turn),.request_active(enable && rm!=0),.request_magnitude(rm),.command_direction(rd),.drive_allowed(drive && ramp_rd==rd),
  .scale_q16(configured_rpm_scale),.enc_a(enc1_a),.enc_b(enc1_b),
  .count(right_count),.actual(right_actual_rpm),.nav_count(nav_right_count),.nav_actual(nav_right_actual),.rejected(right_rejected),.invalid(right_invalid),.duty(right_duty),.pwm(rp),.in1(ra),.in2(rb));
 end else begin: fixed_motors
 assign nav_left_count=left_count;assign nav_right_count=right_count;
 assign nav_left_actual=left_actual_rpm;assign nav_right_actual=right_actual_rpm;
 assign left_rejected=0;assign right_rejected=0;assign left_invalid=0;assign right_invalid=0;
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
module task4_runtime_motor #(parameter integer CLK_FREQ=50000000,PWM_FREQ=20000,ENCODER_SIGN=1)(
 input wire clk,rst_n,direction,input wire signed [31:0] target,
 input wire navigation_mode,turning,request_active,command_direction,drive_allowed,
 input wire signed [31:0] request_magnitude,
 input wire [31:0] scale_q16,input wire enc_a,enc_b,
 output wire signed [31:0] count,output reg signed [31:0] actual,
 output wire signed [31:0] nav_count,output reg signed [31:0] nav_actual,
 output wire [31:0] rejected,invalid,
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
 // Independent qualified feedback is sampled on the same 20ms cadence.
 task4_encoder_feedback #(.CLK_FREQ(CLK_FREQ)) qualified_encoder(
  .clk(clk),.rst_n(rst_n),.enc_a(enc_a),.enc_b(enc_b),.count(nav_count),
  .rejected(rejected),.invalid(invalid),.accepted_state());
 reg signed [31:0] nav_previous,nav_delta,nav_magnitude;
 reg signed [63:0] nav_product;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin nav_previous<=0;nav_delta<=0;nav_product<=0;nav_actual<=0;nav_magnitude<=0;end
  else begin
   if(timer==SAMPLE-1)begin nav_delta<=nav_count-nav_previous;nav_previous<=nav_count;end
   if(update_pipe[0])nav_product<=nav_delta*$signed(scale_q16);
   if(update_pipe[1])nav_actual<=nav_product>>>16;
   if(update_pipe[2])nav_magnitude<=nav_actual[31]?-nav_actual:nav_actual;
  end
 end
 wire [9:0] raw_pi_duty,nav_pi_duty;
 wire [9:0] pi_duty=navigation_mode?nav_pi_duty:raw_pi_duty;
 PI_controller_fast #(.OUT_MAX(260),.I_MAX(200000),.DEAD_ZONE(120),.DUTY_MAX(360),.I_SHIFT(6)) pi(
  .clk(clk),.rst_n(rst_n),.target_rpm(target),.actual_rpm(magnitude),.update_en(pi_update),
  .duty_pi(),.duty(raw_pi_duty),.error(),.integral());
 PI_controller_fast #(.OUT_MAX(260),.I_MAX(200000),.DEAD_ZONE(120),.DUTY_MAX(360),.I_SHIFT(6)) nav_pi(
  .clk(clk),.rst_n(rst_n),.target_rpm(navigation_mode?target:32'sd0),.actual_rpm(nav_magnitude),.update_en(pi_update),
  .duty_pi(),.duty(nav_pi_duty),.error(),.integral());
 // Keep startup budgeting separate from the requested +100 permille turn trim.
 wire [9:0] assisted_duty,trimmed_duty;
 reg [9:0] navigation_duty_q;
 task4_startup_assist #(.CLK_FREQ(CLK_FREQ),.ENCODER_SIGN(ENCODER_SIGN)) assist(
  .clk(clk),.rst_n(rst_n),.navigation_mode(navigation_mode),.request_active(request_active),
  .command_direction(command_direction),.request_magnitude(request_magnitude),
  .drive_allowed(drive_allowed && target>0),.count(nav_count),.actual(nav_actual),.base_duty(pi_duty),.duty(assisted_duty));
 task4_turn_pwm_trim #(.ENCODER_SIGN(ENCODER_SIGN)) turn_trim(
  .navigation_mode(navigation_mode),.turning(turning),.request_active(request_active),
  .drive_allowed(drive_allowed && target>0),.command_direction(command_direction),
  .request_magnitude(request_magnitude),.actual(nav_actual),.base_duty(assisted_duty),.duty(trimmed_duty));
 // End navigation arithmetic at a register before PWM/DIR and telemetry.
 // The outer drive/adapter gate still closes immediately on STOP/reversal/fault.
 // Raw task-three/manual PI output keeps its original cycle-by-cycle path.
 always @(posedge clk or negedge rst_n)
  if(!rst_n)navigation_duty_q<=0;else navigation_duty_q<=trimmed_duty;
 assign duty=navigation_mode?navigation_duty_q:assisted_duty;
 motor_control #(.CLK_FREQ(CLK_FREQ),.PWM_FREQ(PWM_FREQ)) output_driver(
  .clk(clk),.rst_n(rst_n),.duty(duty),.dir(direction),.pwm_out(pwm),.in1(in1),.in2(in2));
endmodule
`default_nettype wire

`default_nettype none
// Add ten percentage points only during task-four opposite-wheel turns.
// Never energize a zero/gated output, exceed the existing 36% total ceiling,
// or add torque while the wheel is overspeeding or clearly coasting backwards.
module task4_turn_pwm_trim #(parameter integer ENCODER_SIGN=1)(
 input wire navigation_mode,turning,request_active,drive_allowed,command_direction,
 input wire signed [31:0] request_magnitude,actual,
 input wire [9:0] base_duty,output wire [9:0] duty
);
 wire signed [31:0] speed=actual<0?-actual:actual;
 wire signed [31:0] normalized_actual=actual*ENCODER_SIGN;
 wire direction_ok=command_direction?normalized_actual<=4:normalized_actual>=-4;
 wire add_turn=navigation_mode && turning && request_active && drive_allowed &&
               request_magnitude>0 && base_duty!=0 && direction_ok && speed<request_magnitude+4;
 wire [10:0] increased={1'b0,base_duty}+11'd100;
 assign duty=!navigation_mode?base_duty:
             (!request_active || !drive_allowed)?10'd0:
             add_turn?(increased>360?10'd360:increased[9:0]):base_duty;
endmodule
`default_nettype wire

`default_nettype none
// Bounded breakaway attempts per nonzero command episode/direction.
// A brief twitch is not a successful launch: qualify motion before handing back
// to PI, and allow one retry after a sustained stall. Gate loss cancels retries.
// Raw counts are never filtered or replaced by commanded motion.
module task4_startup_assist #(
 parameter integer CLK_FREQ=50000000,ENCODER_SIGN=1,
 parameter integer DUTY_CAP=240,MAX_MS=500,LOW_RPM=35,STEP=10,
 parameter integer MAX_ATTEMPTS=2,RETRY_WAIT_MS=200,MOTION_SAMPLES=5
)(
 input wire clk,rst_n,navigation_mode,request_active,command_direction,drive_allowed,
 input wire signed [31:0] request_magnitude,count,actual,
 input wire [9:0] base_duty,output wire [9:0] duty
);
 localparam integer SAMPLE_CYCLES=(CLK_FREQ/1000*20)>0?(CLK_FREQ/1000*20):1;
 localparam integer MAX_CYCLES=(CLK_FREQ/1000*MAX_MS)>0?(CLK_FREQ/1000*MAX_MS):1;
 localparam integer STALL_SAMPLES=(RETRY_WAIT_MS+19)/20;
 localparam IDLE=0,BOOST=1,FADE=2,DONE=3,MONITOR=4,CRUISE=5;
 reg [2:0] state;
 reg direction_seen;
 reg [31:0] elapsed,sample_timer;
 reg [9:0] floor_duty;
 reg [15:0] good_samples,stall_samples,attempts;
 reg signed [31:0] sample_count;
 wire signed [31:0] normalized_delta=(count-sample_count)*ENCODER_SIGN;
 wire signed [31:0] directed_delta=command_direction?-normalized_delta:normalized_delta;
 wire signed [31:0] speed=actual<0?-actual:actual;
 wire signed [31:0] motion_rpm=request_magnitude/2<4?32'sd4:request_magnitude/2;
 wire moving_sample=directed_delta>=2 && speed>=motion_rpm;
 wire eligible=request_active && request_magnitude>0 && request_magnitude<=LOW_RPM;
 wire boost_window=state==BOOST || state==FADE;
 wire supplement=navigation_mode && eligible && drive_allowed && direction_seen==command_direction && boost_window && elapsed<MAX_CYCLES && speed<request_magnitude+4;
 wire [9:0] capped_floor=floor_duty>DUTY_CAP?DUTY_CAP:floor_duty;
 // Cap the added floor, not the independently limited PI output.
 wire [9:0] raised=base_duty>capped_floor?base_duty:capped_floor;
 // Task-three uses exactly the original PI duty, including its original update timing.
 assign duty=!navigation_mode?base_duty:
             (!request_active || !drive_allowed || direction_seen!=command_direction)?10'd0:
             supplement?raised:base_duty;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;direction_seen<=0;elapsed<=0;sample_timer<=0;floor_duty<=0;good_samples<=0;stall_samples<=0;attempts<=0;sample_count<=0;end
  else if(!navigation_mode || !request_active || direction_seen!=command_direction)begin
   state<=IDLE;direction_seen<=command_direction;elapsed<=0;sample_timer<=0;floor_duty<=0;good_samples<=0;stall_samples<=0;attempts<=0;sample_count<=count;
  end else if(state==IDLE)begin
   // High-speed operation suspends breakaway assistance; it is not a failed
   // launch. The same wheel can later slow for a turn without changing sign.
   if(!eligible)begin state<=CRUISE;sample_count<=count;sample_timer<=0;good_samples<=0;end
   else if(drive_allowed)begin
    // A stale/coasting RPM sample must not consume the entire launch episode.
    if(speed>=4)state<=MONITOR;
    else begin state<=BOOST;attempts<=1;floor_duty<=base_duty>DUTY_CAP?DUTY_CAP:base_duty<120?120:base_duty;end
    sample_count<=count;sample_timer<=0;elapsed<=0;good_samples<=0;stall_samples<=0;
   end
  end else if(boost_window)begin
   // Loss of the drive gate consumes this attempt. A later gate opening cannot replay it.
   if(!eligible)begin state<=CRUISE;floor_duty<=0;sample_count<=count;sample_timer<=0;good_samples<=0;end
   else if(!drive_allowed)begin state<=DONE;floor_duty<=0;good_samples<=0;end
   else if(elapsed>=MAX_CYCLES-1)begin
    state<=attempts>=MAX_ATTEMPTS?DONE:MONITOR;floor_duty<=0;
    sample_count<=count;sample_timer<=0;good_samples<=0;stall_samples<=0;
   end
   else begin
    elapsed<=elapsed+1'b1;
    if(sample_timer>=SAMPLE_CYCLES-1)begin
     sample_timer<=0;sample_count<=count;
     // Overspeed or clearly opposite feedback cancels supplemental torque immediately.
     if(directed_delta<=-4)begin state<=DONE;floor_duty<=0;end
     else if(speed>=request_magnitude+4)begin
      state<=attempts>=MAX_ATTEMPTS?DONE:MONITOR;floor_duty<=0;good_samples<=0;stall_samples<=0;
     end
     else if(state==BOOST)begin
      if(moving_sample)begin
       if(good_samples>=MOTION_SAMPLES-1)begin state<=FADE;good_samples<=0;end
       else good_samples<=good_samples+1'b1;
      end else good_samples<=0;
      if(floor_duty+STEP>=DUTY_CAP)floor_duty<=DUTY_CAP;
      else floor_duty<=floor_duty+STEP;
     end else begin
      if(floor_duty<=base_duty || floor_duty<=STEP)begin
       state<=attempts>=MAX_ATTEMPTS?DONE:MONITOR;floor_duty<=0;stall_samples<=0;
      end
      else floor_duty<=floor_duty-STEP;
     end
    end else sample_timer<=sample_timer+1'b1;
   end
  end else if(state==MONITOR)begin
   if(!eligible)begin state<=CRUISE;floor_duty<=0;sample_count<=count;sample_timer<=0;good_samples<=0;end
   else if(!drive_allowed)begin state<=DONE;floor_duty<=0;good_samples<=0;end
   else if(sample_timer>=SAMPLE_CYCLES-1)begin
    sample_timer<=0;sample_count<=count;
    if(directed_delta<=-4)begin state<=DONE;floor_duty<=0;end
    else if(moving_sample || speed>=request_magnitude+4)stall_samples<=0;
    else if(stall_samples>=STALL_SAMPLES-1)begin
     stall_samples<=0;
     if(attempts>=MAX_ATTEMPTS)state<=DONE;
     else begin
      state<=BOOST;attempts<=attempts+1'b1;elapsed<=0;good_samples<=0;
      floor_duty<=base_duty>DUTY_CAP?DUTY_CAP:base_duty<120?120:base_duty;
     end
    end else stall_samples<=stall_samples+1'b1;
   end else sample_timer<=sample_timer+1'b1;
  end else if(state==CRUISE || state==DONE)begin
   // A request crossing LOW_RPM is not evidence that a blocked wheel moved.
   // Only five consecutive forward-progress samples at a high-speed request
   // renew the attempt budget. Threshold chatter or gate chatter cannot do so.
   floor_duty<=0;
   if(!drive_allowed)begin sample_count<=count;sample_timer<=0;good_samples<=0;end
   else if(!eligible)begin
    if(sample_timer>=SAMPLE_CYCLES-1)begin
     sample_timer<=0;sample_count<=count;
     if(directed_delta>=2 && speed>=4)begin
      if(good_samples>=MOTION_SAMPLES-1)begin state<=CRUISE;attempts<=0;good_samples<=0;end
      else good_samples<=good_samples+1'b1;
     end else good_samples<=0;
    end else sample_timer<=sample_timer+1'b1;
   end else begin
    sample_count<=count;sample_timer<=0;good_samples<=0;
    // Wait for coasting to subside and qualify a stall before supplementing.
    if(state==CRUISE)begin
     state<=attempts<MAX_ATTEMPTS?MONITOR:DONE;
     elapsed<=0;stall_samples<=0;
    end
   end
  end
 end
endmodule
`default_nettype wire
