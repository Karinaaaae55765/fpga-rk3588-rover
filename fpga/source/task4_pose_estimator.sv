`default_nettype none
// Independent task-four odometry. Encoder motion is evidence, not motor demand.
// IMU can only adjust a bounded offset after agreement; disagreement is latched
// as a fault during navigation, not silently treated as trusted wheel motion.
module task4_pose_estimator #(parameter integer LEFT_SIGN=1,RIGHT_SIGN=-1,MAX_DELTA=200)(
 input wire clk,rst_n,tick,reset_origin,active,imu_valid,
 input wire signed [31:0] left_count,right_count,
 input wire signed [15:0] yaw_cdeg,
 output reg signed [31:0] x_mm,y_mm,
 output reg signed [15:0] heading,encoder_heading,imu_heading,disagreement,correction,
 output reg valid,fault,trusted,degraded,
 output reg [7:0] reason,
 output wire [7:0] quality,
 output reg [15:0] bad_ticks,agree_ticks,imu_age
);
 // Generated from route_6x4.json. Units mm, rpm, 50 Hz ticks.
`define T4_DWELL_TICKS 200
`define T4_APPROACH_RPM 35
`define T4_APPROACH_MM 350
`define T4_COUNT 214
`define T4_GOALS 7
`define T4_CRUISE_RPM 103
`define T4_MIN_RPM 15
`define T4_ALIGN_RPM 22
`define T4_NEAR_RPM 18
`define T4_NEAR_MM 110
`define T4_AVOID_FORWARD_RPM 59
`define T4_SECTOR_RPM 17
`define T4_SECTOR_TOL_CDEG 500
`define T4_PROBE_MM 80
`define T4_PASS_MM 50
`define T4_GOAL_MM 50
`define T4_LOOK_STEPS 6
`define T4_TRACK_MM 110
`define T4_SESSION_TICKS 30000
`define T4_PROGRESS_TICKS 3000
`define T4_PREFERRED_CDEG 7500
`define T4_SEARCH_STEP_CDEG 2000
`define T4_SECTOR_CDEG 2500
`define T4_PREFERENCE_EXTRA_CDEG 9000
`define T4_DEPART_MM 350
`define T4_REPEAT_STEP_MM 100
`define T4_MAX_DEPART_MM 650
`define T4_REPEAT_REGION_MM 600
`define T4_ODO_LEFT_MM_Q16 9042
`define T4_ODO_LEFT_TURN_Q16 470990
`define T4_ODO_RIGHT_MM_Q16 9042
`define T4_ODO_RIGHT_TURN_Q16 470990
`define T4_ODO_AGREE_CDEG 800
`define T4_ODO_STOP_CDEG 2500
`define T4_ODO_BAD_TICKS 50
`define T4_ODO_TRUST_TICKS 25
`define T4_ODO_CORRECTION_STEP 4
`define T4_ODO_CORRECTION_LIMIT 100
`define T4_ODO_DEGRADED_RPM 44
`define T4_ODO_VALIDATION_RPM 103
`define T4_ODO_CALIBRATED 0
function automatic signed [31:0] goal_x(input [3:0] i);
 case(i)
 4'd0:goal_x=690;
 4'd1:goal_x=980;
 4'd2:goal_x=750;
 4'd3:goal_x=1150;
 4'd4:goal_x=1690;
 4'd5:goal_x=560;
 4'd6:goal_x=-70;
 default:goal_x=-70;
 endcase
endfunction
function automatic signed [31:0] goal_y(input [3:0] i);
 case(i)
 4'd0:goal_y=-220;
 4'd1:goal_y=-360;
 4'd2:goal_y=-760;
 4'd3:goal_y=-760;
 4'd4:goal_y=-760;
 4'd5:goal_y=-1030;
 4'd6:goal_y=-1100;
 default:goal_y=-1100;
 endcase
endfunction
function automatic signed [31:0] gate_index(input [3:0] i);
 case(i)
 4'd0:gate_index=36;
 4'd1:gate_index=52;
 4'd2:gate_index=75;
 4'd3:gate_index=95;
 4'd4:gate_index=122;
 4'd5:gate_index=180;
 4'd6:gate_index=213;
 default:gate_index=213;
 endcase
endfunction

 reg initialized,imu_seen;
 reg signed [15:0] origin_yaw;
 reg signed [31:0] last_l,last_r,encoder_q16,x_q16,y_q16,ds_q16;
 // Bounds are checked on full 32-bit deltas before these small operands are
 // used. Avoid synthesizing full-width multipliers for +/-200-count samples.
 localparam integer DELTA_BITS=$clog2(MAX_DELTA+1)+1;
 reg signed [DELTA_BITS-1:0] dl,dr;
 reg signed [63:0] pl,pr,tl,tr,next_encoder,prod_x,prod_y,sum_x,sum_y;
 reg [3:0] phase;
 reg math_start;
 reg signed [15:0] midpoint,pending_heading;
 reg signed [31:0] heading_step;
 wire math_done;wire signed [31:0] cosv,sinv;
 wire fresh=imu_seen && imu_age<25;
 assign quality={`T4_ODO_CALIBRATED!=0,active,fault,degraded,trusted,fresh,imu_seen,valid};
 function automatic signed [31:0] norm(input signed [31:0] a);
  norm=a>=18000?a-36000:a< -18000?a+36000:a;
 endfunction
 function automatic signed [63:0] normq(input signed [63:0] a);
  normq=a>=64'sd1179648000 ? a-64'sd2359296000 : a< -64'sd1179648000 ? a+64'sd2359296000 : a;
 endfunction
 function automatic signed [31:0] abs32(input signed [31:0] a);abs32=a<0?-a:a;endfunction
 function automatic signed [15:0] offset_step(input signed [15:0] old_value,target);
  reg signed [31:0] bounded;
  begin
   bounded=target>`T4_ODO_CORRECTION_LIMIT?`T4_ODO_CORRECTION_LIMIT:
           target< -`T4_ODO_CORRECTION_LIMIT?-`T4_ODO_CORRECTION_LIMIT:target;
   offset_step=bounded>old_value+`T4_ODO_CORRECTION_STEP?old_value+`T4_ODO_CORRECTION_STEP:
               bounded<old_value-`T4_ODO_CORRECTION_STEP?old_value-`T4_ODO_CORRECTION_STEP:bounded;
  end
 endfunction
 wire signed [31:0] raw_dl=left_count-last_l,raw_dr=right_count-last_r;
 wire signed [15:0] next_correction=(trusted && fresh && abs32(disagreement)<=`T4_ODO_AGREE_CDEG && (dl!=0 || dr!=0))?
                                  offset_step(correction,disagreement):correction;
 wire signed [15:0] next_heading=norm($signed(encoder_heading)+$signed(next_correction));
 nav_math trig(.clk(clk),.rst_n(rst_n),.clear(reset_origin),.start(math_start),.mode(1'b0),
  .x_in(32'sd0),.y_in(32'sd0),.angle_in(midpoint),.busy(),.done(math_done),.bearing(),.cos_q15(cosv),.sin_q15(sinv));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin imu_seen<=0;imu_age<=65535;end
  else if(imu_valid)begin imu_seen<=1;imu_age<=0;end
  else if(tick && imu_age!=65535)imu_age<=imu_age+1'b1;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   initialized<=0;origin_yaw<=0;last_l<=0;last_r<=0;dl<=0;dr<=0;encoder_q16<=0;
   x_q16<=0;y_q16<=0;x_mm<=0;y_mm<=0;ds_q16<=0;heading<=0;encoder_heading<=0;imu_heading<=0;
   disagreement<=0;correction<=0;valid<=0;fault<=0;reason<=0;trusted<=0;degraded<=1;
   bad_ticks<=0;agree_ticks<=0;phase<=0;math_start<=0;midpoint<=0;pending_heading<=0;heading_step<=0;
   pl<=0;pr<=0;tl<=0;tr<=0;next_encoder<=0;prod_x<=0;prod_y<=0;sum_x<=0;sum_y<=0;
  end else if(reset_origin || (!initialized && imu_valid))begin
   initialized<=1;origin_yaw<=yaw_cdeg;last_l<=left_count;last_r<=right_count;
   encoder_q16<=0;x_q16<=0;y_q16<=0;x_mm<=0;y_mm<=0;heading<=0;encoder_heading<=0;imu_heading<=0;
   disagreement<=0;correction<=0;valid<=0;fault<=0;reason<=0;trusted<=0;degraded<=1;
   bad_ticks<=0;agree_ticks<=0;phase<=0;math_start<=0;pending_heading<=0;heading_step<=0;
  end else begin
   math_start<=0;
   if(!active && fault)begin fault<=0;reason<=0;initialized<=0;valid<=0;phase<=0;end
   else if(initialized && !fault)begin
    if(tick && phase!=0)begin fault<=1;reason<=8'h3f;valid<=0;end
    else case(phase)
     0:if(tick)begin
      last_l<=left_count;last_r<=right_count;dl<=raw_dl*LEFT_SIGN;dr<=raw_dr*RIGHT_SIGN;
      imu_heading<=norm($signed(yaw_cdeg)-$signed(origin_yaw));
      if(raw_dl>MAX_DELTA || raw_dl< -MAX_DELTA || raw_dr>MAX_DELTA || raw_dr< -MAX_DELTA)begin fault<=1;reason<=8'h3d;valid<=0;end
      else if(active && !fresh)begin fault<=1;reason<=8'h3e;valid<=0;end
      else phase<=1;
     end
     1:begin
      pl<=$signed(dl)*32'sd`T4_ODO_LEFT_MM_Q16;pr<=$signed(dr)*32'sd`T4_ODO_RIGHT_MM_Q16;
      tl<=$signed(dl)*32'sd`T4_ODO_LEFT_TURN_Q16;tr<=$signed(dr)*32'sd`T4_ODO_RIGHT_TURN_Q16;phase<=2;
     end
     2:begin ds_q16<=(pl+pr)>>>1;next_encoder<=$signed(encoder_q16)+(tr-tl);phase<=3;end
     3:begin
      encoder_q16<=normq(next_encoder);encoder_heading<=normq(next_encoder)>>>16;
      disagreement<=norm($signed(imu_heading)-$signed(normq(next_encoder)>>>16));phase<=4;
     end
     4:begin
      if(fresh && abs32(disagreement)<=`T4_ODO_AGREE_CDEG)begin
       if(agree_ticks<`T4_ODO_TRUST_TICKS)agree_ticks<=agree_ticks+1'b1;
       if(agree_ticks>=`T4_ODO_TRUST_TICKS-1)begin trusted<=1;degraded<=0;end
      end else begin agree_ticks<=0;trusted<=0;degraded<=1;end
      if(active && abs32(disagreement)>`T4_ODO_STOP_CDEG)begin
       if(bad_ticks>=`T4_ODO_BAD_TICKS-1)begin fault<=1;reason<=8'h3c;valid<=0;end
       else bad_ticks<=bad_ticks+1'b1;
      end else bad_ticks<=0;
      correction<=next_correction;pending_heading<=next_heading;phase<=9;
     end
     // Separate three normalization/add stages; complete well before 20 ms.
     9:begin heading_step<=norm($signed(pending_heading)-$signed(heading));phase<=10;end
     10:begin midpoint<=norm($signed(heading)+(heading_step>>>1));heading<=pending_heading;phase<=5;end
     5:begin math_start<=1;phase<=6;end
     6:if(math_done)begin prod_x<=$signed(ds_q16)*$signed(cosv[16:0]);prod_y<=$signed(ds_q16)*$signed(sinv[16:0]);phase<=7;end
     7:begin sum_x<=$signed(x_q16)+(prod_x>>>15);sum_y<=$signed(y_q16)+(prod_y>>>15);phase<=8;end
     8:begin
      if(sum_x>64'sd1966080000 || sum_x< -64'sd1966080000 || sum_y>64'sd1966080000 || sum_y< -64'sd1966080000)begin fault<=1;reason<=8'h3d;valid<=0;end
      else begin x_q16<=sum_x;y_q16<=sum_y;x_mm<=sum_x>>>16;y_mm<=sum_y>>>16;valid<=1;end
      phase<=0;
     end
     default:begin fault<=1;reason<=8'h3f;valid<=0;end
    endcase
   end
  end
 end
endmodule
`default_nettype wire
