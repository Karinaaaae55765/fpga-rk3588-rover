`default_nettype none
// 204928 baseline; navigation forward probe is sector-validated before handoff.
// Shared motor arbitration: START keeps task-three cruise; X selects the route.
module task4_controller #(
 parameter integer CRUISE_RPM=60,TURN_RPM=45,RECOVERY_TICKS=15,SPIN_RPM=30
)(
 input wire clk,rst_n,tick,configured,sensors_ready,hard_fault,stop_request,auto_start,route_start,
 input wire [2:0] manual_action,input wire released,
 input wire [7:0] safety_reason,avoid_reason,
 input wire obstacle,slow,pose_valid,avoid_done,avoid_failed,avoid_waiting,
 input wire signed [15:0] heading,
 input wire distance_valid,input wire [15:0] distance_mm,
 input wire signed [31:0] avoid_left,avoid_right,nav_left,nav_right,x_mm,y_mm,left_actual,right_actual,
 input wire nav_dwelling,nav_done,nav_failed,nav_goal_reached,input wire [7:0] nav_reason,input wire [3:0] waypoint,
 output reg [3:0] state,output reg [7:0] reason,
 output wire autonomous,nav_enable,output reg reset_origin,avoid_start,route_mode,recovery_follow,
 output wire avoid_cancel,motor_enable,remote_blocked,
 output reg signed [31:0] left_rpm,right_rpm
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

 localparam IDLE=0,MANUAL=1,INIT=2,CRUISE=3,AVOID=4,FAULT=5,COMPLETE=6,DEPART=7,NAV=8,REJOIN=9;
 reg signed [31:0] encounter_x,encounter_y;
 reg encounter_valid;reg [31:0] depart_distance;
 wire repeated_encounter=encounter_valid && abs32(x_mm-encounter_x)<=`T4_REPEAT_REGION_MM && abs32(y_mm-encounter_y)<=`T4_REPEAT_REGION_MM;
 reg [2:0] rejoin_phase;
 reg signed [15:0] rejoin_heading;
 reg [31:0] rejoin_ticks;
 function automatic signed [31:0] norm(input signed [31:0] a);norm=a>18000?a-36000:a< -18000?a+36000:a;endfunction
 wire probing=state==REJOIN && rejoin_phase>=1 && rejoin_phase<=3;
 wire signed [31:0] probe_target=norm($signed(rejoin_heading)+(rejoin_phase==1?-`T4_SECTOR_CDEG:rejoin_phase==2?`T4_SECTOR_CDEG:0));
 wire signed [31:0] probe_error=norm(probe_target-$signed(heading));
 reg [31:0] session_ticks,progress_ticks,best_goal_distance;
 reg [7:0] healthy_ticks;reg [3:0] previous_waypoint;
 reg manual_obstacle;reg signed [31:0] depart_x,depart_y;
 reg signed [31:0] request_l,request_r,metric_dx,metric_dy;
 reg [31:0] goal_distance;reg [2:0] clear_samples;
 function automatic signed [31:0] abs32(input signed [31:0] v);abs32=v<0?-v:v;endfunction
 wire waiting=(state==AVOID && avoid_waiting) || (state==NAV && nav_dwelling);
 wire route_running=autonomous && route_mode;
 wire stopped=abs32(left_actual)<=3 && abs32(right_actual)<=3;
 wire forward_nav=nav_left>0 && nav_right>0;
 assign autonomous=state==INIT || state==CRUISE || state==NAV || state==AVOID || state==DEPART || state==REJOIN;
 assign nav_enable=(state==NAV || (state==REJOIN && !probing)) && configured && sensors_ready && !hard_fault && !stop_request && !obstacle;
 assign avoid_cancel=state!=AVOID || stop_request || hard_fault || !configured;
 assign remote_blocked=!configured || !sensors_ready || hard_fault || state==FAULT;
 assign motor_enable=configured && sensors_ready && !hard_fault && !stop_request &&
  (state==MANUAL || state==CRUISE || state==NAV || state==AVOID || state==DEPART || state==REJOIN) &&
  (state!=MANUAL || (manual_action!=0 && ((!obstacle && !manual_obstacle) || (manual_action==2 && left_rpm<0 && right_rpm<0)))) &&
  (state!=CRUISE || !obstacle) &&
  ((state!=NAV && state!=REJOIN) || (!obstacle && !nav_done && !nav_failed)) &&
  (state!=DEPART || !obstacle) && (left_rpm!=0 || right_rpm!=0);
 always @*begin
  request_l=0;request_r=0;
  if(state==MANUAL && ((!manual_obstacle && !obstacle) || manual_action==2))begin
   case(manual_action)
    1:begin request_l=CRUISE_RPM;request_r=CRUISE_RPM;end
    2:begin request_l=-CRUISE_RPM;request_r=-CRUISE_RPM;end
    3:begin request_l=TURN_RPM;request_r=CRUISE_RPM;end
    4:begin request_l=CRUISE_RPM;request_r=TURN_RPM;end
    5:begin request_l=TURN_RPM;request_r=-TURN_RPM;end
    6:begin request_l=-SPIN_RPM;request_r=SPIN_RPM;end // LB: counterclockwise
    7:begin request_l=SPIN_RPM;request_r=-SPIN_RPM;end // RB: clockwise
    default:begin end
   endcase
   if(slow && request_l>0 && request_r>0)begin request_l=request_l/2;request_r=request_r/2;end
  end else if(state==CRUISE && !obstacle)begin request_l=slow?TURN_RPM:CRUISE_RPM;request_r=request_l;end
  else if((state==NAV || state==REJOIN) && !obstacle && !nav_done && !nav_failed)begin
   // REJOIN may align in place but cannot translate before fresh range approval.
   if(probing)begin
    if(probe_error>`T4_SECTOR_TOL_CDEG)begin request_l=-`T4_SECTOR_RPM;request_r=`T4_SECTOR_RPM;end
    else if(probe_error< -`T4_SECTOR_TOL_CDEG)begin request_l=`T4_SECTOR_RPM;request_r=-`T4_SECTOR_RPM;end
   end else if(state==NAV || (nav_left<=0 && nav_right>=0) || (nav_right<=0 && nav_left>=0))begin request_l=nav_left;request_r=nav_right;end
  end else if(state==AVOID && !avoid_failed)begin request_l=avoid_left;request_r=avoid_right;end
  else if(state==DEPART && !obstacle)begin request_l=slow?`T4_APPROACH_RPM:`T4_AVOID_FORWARD_RPM;request_r=request_l;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin left_rpm<=0;right_rpm<=0;end
  else if(stop_request || hard_fault || !configured)begin left_rpm<=0;right_rpm<=0;end
  else begin left_rpm<=request_l;right_rpm<=request_r;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin metric_dx<=0;metric_dy<=0;goal_distance<=0;end
  else begin metric_dx<=goal_x(waypoint)-x_mm;metric_dy<=goal_y(waypoint)-y_mm;goal_distance<=abs32(metric_dx)+abs32(metric_dy);end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;reason<=0;reset_origin<=0;avoid_start<=0;route_mode<=0;recovery_follow<=0;rejoin_phase<=0;rejoin_heading<=0;rejoin_ticks<=0;encounter_valid<=0;encounter_x<=0;encounter_y<=0;depart_distance<=`T4_DEPART_MM;
   session_ticks<=0;progress_ticks<=0;best_goal_distance<=32'h7fffffff;
   healthy_ticks<=0;previous_waypoint<=0;manual_obstacle<=0;depart_x<=0;depart_y<=0;clear_samples<=0;
  end else begin
   reset_origin<=0;avoid_start<=0;
   if(!route_running)begin session_ticks<=0;progress_ticks<=0;best_goal_distance<=32'h7fffffff;end
   else if(waypoint!=previous_waypoint)begin best_goal_distance<=32'h7fffffff;progress_ticks<=0;end
   else if(tick && !waiting)begin
    session_ticks<=session_ticks+1'b1;
    // Actual improvement toward the goal, not just distance travelled in circles.
    if(goal_distance+100<best_goal_distance)begin best_goal_distance<=goal_distance;progress_ticks<=0;end
    else progress_ticks<=progress_ticks+1'b1;
   end
   previous_waypoint<=waypoint;
   if(!route_running || waypoint!=previous_waypoint)begin
    encounter_valid<=0;depart_distance<=`T4_DEPART_MM;recovery_follow<=0;
   end else if(avoid_start)begin
    recovery_follow<=1;
    if(repeated_encounter)depart_distance<=depart_distance+`T4_REPEAT_STEP_MM>`T4_MAX_DEPART_MM?`T4_MAX_DEPART_MM:depart_distance+`T4_REPEAT_STEP_MM;
    else begin encounter_x<=x_mm;encounter_y<=y_mm;encounter_valid<=1;depart_distance<=`T4_DEPART_MM;end
   end
   if(state!=REJOIN)begin clear_samples<=0;rejoin_phase<=0;rejoin_ticks<=0;end
   else if(tick)rejoin_ticks<=rejoin_ticks+1'b1;
   if(state!=FAULT || hard_fault || !sensors_ready || !configured)healthy_ticks<=0;
   else if(tick && healthy_ticks<RECOVERY_TICKS)healthy_ticks<=healthy_ticks+1'b1;
   if(stop_request)begin
    state<=IDLE;manual_obstacle<=0;route_mode<=0;
    // A BT lease STOP can follow a navigation fault as autonomous drops.
    // Stop the motors immediately but retain the diagnostic until a fresh launch.
    if(route_mode && nav_failed)reason<=nav_reason;
    else if(reason<8'h30 || reason>8'h3f)reason<=1;
   end
   else if(state!=IDLE && state!=FAULT && state!=COMPLETE && (!configured || !sensors_ready || hard_fault))begin state<=FAULT;reason<=safety_reason!=0?safety_reason:8'h12;end
   else if(route_running && !waiting && session_ticks>=`T4_SESSION_TICKS)begin state<=FAULT;reason<=8'h30;end
   else if(route_running && !waiting && progress_ticks>=`T4_PROGRESS_TICKS)begin state<=FAULT;reason<=8'h31;end
   else case(state)
    IDLE,COMPLETE:begin
     manual_obstacle<=0;
     if(configured && sensors_ready && !hard_fault)begin
      if(auto_start)begin state<=INIT;route_mode<=route_start;reset_origin<=1;reason<=0;end
      else if(manual_action!=0)begin state<=MANUAL;route_mode<=0;reason<=0;end
     end
    end
    MANUAL:begin
     if(obstacle)manual_obstacle<=1;
     if(auto_start)begin state<=INIT;route_mode<=route_start;reset_origin<=1;reason<=0;end
     else if(manual_action==0)state<=IDLE;
    end
    INIT:if(pose_valid && !reset_origin)state<=route_mode?NAV:CRUISE;
    CRUISE:if(obstacle)begin state<=AVOID;avoid_start<=1;end
    NAV:begin
     if(nav_failed)begin state<=FAULT;reason<=nav_reason;end
     else if(nav_done)begin state<=COMPLETE;reason<=0;end
     else if(obstacle)begin state<=AVOID;avoid_start<=1;end
    end
    AVOID:if(avoid_failed)begin state<=FAULT;reason<=avoid_reason!=0?avoid_reason:8'h20;end
          else if(avoid_done)begin
           reason<=0;
           if(route_mode)begin state<=DEPART;depart_x<=x_mm;depart_y<=y_mm;end
           else state<=CRUISE; // Preserve the baseline's uninterrupted handoff.
          end else reason<=avoid_reason;
    // Low-speed separation along the locally selected opening, always gated
    // by the current front range. It does not declare the whole obstacle clear.
    DEPART:begin
     if(obstacle)begin state<=AVOID;avoid_start<=1;end
     else if(abs32(x_mm-depart_x)>=depart_distance || abs32(y_mm-depart_y)>=depart_distance)state<=REJOIN;
    end
    REJOIN:begin
     if(nav_failed)begin state<=FAULT;reason<=nav_reason;end
     else if(nav_done)begin state<=COMPLETE;reason<=0;end
     else if(obstacle)begin state<=AVOID;avoid_start<=1;end
     else if(!probing && nav_goal_reached && stopped)state<=NAV; // Stationary arrival confirmation needs no forward release.
     else if(rejoin_ticks>=750)begin state<=FAULT;reason<=8'h35;end
     else if(probing)begin
      // Validate the swept front sector, not just a single grazing ray.
      if(distance_valid && distance_mm<500)begin state<=AVOID;avoid_start<=1;clear_samples<=0;end
      else if(abs32(probe_error)<=`T4_SECTOR_TOL_CDEG && stopped && distance_valid)begin
       if(clear_samples>=2)begin rejoin_phase<=rejoin_phase+1'b1;clear_samples<=0;end
       else clear_samples<=clear_samples+1'b1;
      end else if(abs32(probe_error)>`T4_SECTOR_TOL_CDEG || !stopped)clear_samples<=0;
     end else if(!forward_nav || !stopped)clear_samples<=0;
     else if(distance_valid)begin
      if(distance_mm<500)begin clear_samples<=0;state<=AVOID;avoid_start<=1;end
      else if(clear_samples>=2)begin
       clear_samples<=0;
       if(rejoin_phase==0)begin rejoin_heading<=heading;rejoin_phase<=1;end
       else state<=NAV;
      end
      else clear_samples<=clear_samples+1'b1;
     end
    end
    FAULT:if(configured && sensors_ready && !hard_fault && released && healthy_ticks>=RECOVERY_TICKS)begin state<=IDLE;manual_obstacle<=0;end
    default:begin state<=FAULT;reason<=255;end
   endcase
  end
 end
endmodule
`default_nettype wire
