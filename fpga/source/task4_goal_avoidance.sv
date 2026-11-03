`default_nettype none
// Reactive front-range avoidance: brake, turn one way, stop/recheck, forward.
// Up to 20 degrees per segment; fresh clear range stops rotation early.
// Navigation derivative: stopped goal selection and a fresh sector check BEFORE forward.
module task4_goal_avoidance #(
 parameter integer CRUISE_RPM=60,NEAR_TURN_RPM=20,CLEAR_MM=500,MARGIN_MM=50,
 parameter integer SETTLE_TICKS=10,SAFE_SAMPLES=3,
 parameter integer EMERGENCY_MM=100,BACKUP_MM=100,BACKUP_RPM=15,BACKUP_TICKS=200
)(
 input wire clk,rst_n,tick,start,cancel,clearance_verified,
 input wire signed [31:0] x_mm,y_mm,input wire signed [15:0] heading,
 input wire [3:0] waypoint,
 input wire signed [31:0] left_actual,right_actual,
 input wire distance_valid,input wire [15:0] distance_mm,input wire obstacle,
 output wire signed [31:0] left_rpm,right_rpm,
 output wire done,failed,waiting,output wire [3:0] phase,output reg [7:0] reason
);
 reg [31:0] scan_travel;
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

 reg math_start,goal_ready;
 reg signed [31:0] math_x,math_y;
 wire math_done,math_busy;wire signed [15:0] math_bearing;
 reg signed [15:0] goal_bearing;
 reg preference_seen;reg [31:0] preference_origin;
 nav_math goal_math(.clk(clk),.rst_n(rst_n),.clear(cancel),.start(math_start),.mode(1'b1),
  .x_in(math_x),.y_in(math_y),.angle_in(16'sd0),.busy(math_busy),.done(math_done),.bearing(math_bearing),.cos_q15(),.sin_q15());
 wire signed [31:0] goal_error=norm($signed(goal_bearing)-$signed(heading));
 wire goal_preferred=goal_ready && goal_error>=-`T4_PREFERRED_CDEG && goal_error<=`T4_PREFERRED_CDEG;
 wire preference_expired=preference_seen && scan_travel>=preference_origin+`T4_PREFERENCE_EXTRA_CDEG;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin math_start<=0;math_x<=0;math_y<=0;goal_ready<=0;goal_bearing<=0;end
  else begin
   math_start<=0;
   if(cancel || start)goal_ready<=0;
   if(!cancel && (start || tick) && !math_busy)begin
    math_x<=goal_x(waypoint)-x_mm;math_y<=goal_y(waypoint)-y_mm;math_start<=1;
   end
   if(math_done && !cancel && !start)begin goal_bearing<=math_bearing;goal_ready<=1;end
  end
 end
 localparam IDLE=0,SEARCH=1,VERIFY=2,BACKUP=4,FORWARD=7,WAITING=8,DONE=13,FAULT=14,BRAKE=15;
 localparam integer BRAKE_TICKS=15,RETRY_TICKS=100,PROBE_TICKS=25;
 // At 50 Hz: slow but improving rotation may take up to 15 s per sector;
 // no angular improvement for 5 s still stops it. Translation has a 2 s
 // progress lease and a separate 10 s absolute bound.
 localparam integer SECTOR_LIMIT_TICKS=750,SECTOR_STALL_TICKS=250;
 localparam integer FORWARD_LIMIT_TICKS=500,FORWARD_STALL_TICKS=100;
 localparam integer VERIFY_RESELECTS=2;
 localparam integer SEARCH_STEP_CDEG=`T4_SEARCH_STEP_CDEG; // Commanded segment; physical coasting may exceed it.
 localparam integer ROTATE_CLEAR_MM=EMERGENCY_MM+MARGIN_MM;
 localparam integer TURN_SPEED=`T4_ALIGN_RPM;
 localparam integer PROBE_SPEED=`T4_AVOID_FORWARD_RPM;
 localparam SECTOR_LEFT=9,SECTOR_RIGHT=10,SECTOR_RETURN=11;
 reg [3:0] state;
 reg force_step,sector_done;
 reg signed [15:0] sector_heading;
 wire sector=state==SECTOR_LEFT || state==SECTOR_RIGHT || state==SECTOR_RETURN;
 wire stopped=left_actual>=-3 && left_actual<=3 && right_actual>=-3 && right_actual<=3;
 reg signed [31:0] sector_target,sector_error;
 reg [3:0] sector_state_d1,sector_state_d2;
 wire sector_math_valid=sector_state_d1==state && sector_state_d2==state;
 wire sector_aligned=sector_math_valid && sector_error>=-`T4_SECTOR_TOL_CDEG && sector_error<=`T4_SECTOR_TOL_CDEG;
 reg goal_accept;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sector_target<=0;sector_error<=0;sector_state_d1<=0;sector_state_d2<=0;goal_accept<=0;end
  else begin
   sector_state_d1<=state;sector_state_d2<=sector_state_d1;
   sector_target<=norm($signed(sector_heading)+(state==SECTOR_LEFT?-`T4_SECTOR_CDEG:state==SECTOR_RIGHT?`T4_SECTOR_CDEG:0));
   sector_error<=norm(sector_target-$signed(heading));
   goal_accept<=goal_preferred || preference_expired;
  end
 end

 reg [31:0] timer,range_age,wrong_travel,progress_age;
 reg [1:0] verify_reselects;
 reg [3:0] motion_watch_state;
 reg [31:0] sector_best_error,sector_stall,forward_best_distance,forward_stall;
 reg [7:0] samples,stable;
 reg [15:0] latest_range;
 reg signed [15:0] previous_heading,progress_heading;
 reg signed [31:0] backup_x,backup_y,probe_x,probe_y;
 reg backup_used;
 reg signed [15:0] segment_heading;
 wire signed [31:0] segment_turn=norm($signed(heading)-$signed(segment_heading));
 function automatic signed [31:0] norm(input signed [31:0] a);
  norm=a>18000?a-36000:a< -18000?a+36000:a;
 endfunction
 wire signed [31:0] yaw_step=norm($signed(heading)-$signed(previous_heading));
 wire [31:0] yaw_motion=yaw_step<0?-yaw_step:yaw_step;
 wire signed [31:0] progress=norm($signed(heading)-$signed(progress_heading));
 wire signed [31:0] dx=x_mm-backup_x,dy=y_mm-backup_y;
 function automatic [31:0] abs32(input signed [31:0] a);abs32=a<0?-a:a;endfunction
 wire [31:0] sector_error_size=abs32(sector_error);
 // Split subtraction, absolute value and max into separate 50 MHz stages.
 // Validity is flushed outside FORWARD, so an earlier probe cannot complete
 // a new probe or renew its progress timeout with stale position data.
 reg signed [31:0] forward_delta_x,forward_delta_y;
 reg [31:0] forward_dx,forward_dy,forward_distance;
 reg [2:0] forward_pipeline;
 wire forward_math_valid=&forward_pipeline;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   forward_delta_x<=0;forward_delta_y<=0;forward_dx<=0;forward_dy<=0;
   forward_distance<=0;forward_pipeline<=0;
  end else begin
   forward_delta_x<=x_mm-probe_x;forward_delta_y<=y_mm-probe_y;
   forward_dx<=abs32(forward_delta_x);forward_dy<=abs32(forward_delta_y);
   forward_distance<=forward_dx>forward_dy?forward_dx:forward_dy;
   if(cancel || start || state!=FORWARD)forward_pipeline<=0;
   else forward_pipeline<={forward_pipeline[1:0],1'b1};
  end
 end
 // Improvement must beat the best observation, so oscillation near one angle
 // or position cannot indefinitely renew the progress lease.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   motion_watch_state<=IDLE;sector_best_error<=32'hffffffff;sector_stall<=0;
   forward_best_distance<=0;forward_stall<=0;
  end else begin
   motion_watch_state<=state;
   if(cancel || start || !sector || motion_watch_state!=state || !sector_math_valid)begin
    sector_best_error<=32'hffffffff;sector_stall<=0;
   end else if(tick)begin
    if(sector_error_size+200<=sector_best_error)begin sector_best_error<=sector_error_size;sector_stall<=0;end
    else if(sector_stall<SECTOR_STALL_TICKS)sector_stall<=sector_stall+1'b1;
   end
   if(cancel || start || state!=FORWARD || motion_watch_state!=state)begin forward_best_distance<=0;forward_stall<=0;end
   else if(tick)begin
    if(forward_math_valid && forward_distance>=forward_best_distance+5)begin forward_best_distance<=forward_distance;forward_stall<=0;end
    else if(forward_stall<FORWARD_STALL_TICKS)forward_stall<=forward_stall+1'b1;
   end
  end
 end
 reg signed [31:0] backup_dx,backup_dy;
 reg [63:0] backup_dx2,backup_dy2,reverse_distance2;
 reg [2:0] backup_pipeline;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin backup_dx<=0;backup_dy<=0;backup_dx2<=0;backup_dy2<=0;reverse_distance2<=0;backup_pipeline<=0;end
  else begin
   backup_dx<=dx;backup_dy<=dy;
   backup_dx2<=$signed(backup_dx)*$signed(backup_dx);backup_dy2<=$signed(backup_dy)*$signed(backup_dy);
   reverse_distance2<=backup_dx2+backup_dy2;
   backup_pipeline<={backup_pipeline[1:0],state==BACKUP};
  end
 end
 wire [15:0] front=distance_valid?distance_mm:latest_range;
 wire critical=front!=0 && front<EMERGENCY_MM;
 wire fresh_clear=distance_valid && distance_mm>=CLEAR_MM && !obstacle;
 wire active=state!=IDLE && state!=FAULT;
 wire guard=cancel || !active || range_age>=25;
 wire advancing=(state==FORWARD || state==DONE) && !obstacle && !critical;
 assign left_rpm=guard?0:state==BACKUP?-BACKUP_RPM:state==SEARCH?-TURN_SPEED:sector && sector_math_valid?(sector_error>`T4_SECTOR_TOL_CDEG?-`T4_SECTOR_RPM:sector_error< -`T4_SECTOR_TOL_CDEG?`T4_SECTOR_RPM:0):advancing?PROBE_SPEED:0;
 assign right_rpm=guard?0:state==BACKUP?-BACKUP_RPM:state==SEARCH?TURN_SPEED:sector && sector_math_valid?(sector_error>`T4_SECTOR_TOL_CDEG?`T4_SECTOR_RPM:sector_error< -`T4_SECTOR_TOL_CDEG?-`T4_SECTOR_RPM:0):advancing?PROBE_SPEED:0;
 assign done=state==DONE && !guard && !obstacle && !critical;
 assign failed=state==FAULT;
 assign waiting=state==WAITING;
 assign phase=state==BRAKE?4'd0:state;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;reason<=0;timer<=0;range_age<=25;latest_range<=0;
   samples<=0;stable<=0;scan_travel<=0;wrong_travel<=0;progress_age<=0;verify_reselects<=0;
   previous_heading<=0;progress_heading<=0;probe_x<=0;probe_y<=0;backup_x<=0;backup_y<=0;backup_used<=0;segment_heading<=0;force_step<=0;sector_done<=0;sector_heading<=0;preference_seen<=0;preference_origin<=0;
  end else begin
   if(distance_valid && distance_mm!=0)begin latest_range<=distance_mm;range_age<=0;end
   else if(tick && range_age<25)range_age<=range_age+1'b1;
   if(tick)previous_heading<=heading;
   if(cancel)begin state<=IDLE;reason<=0;timer<=0;samples<=0;verify_reselects<=0;end
   else if(start)begin
    state<=clearance_verified?BRAKE:FAULT;reason<=clearance_verified?0:8'h24;
    timer<=0;range_age<=0;samples<=0;stable<=0;scan_travel<=0;verify_reselects<=0;
    wrong_travel<=0;progress_age<=0;progress_heading<=heading;
    previous_heading<=heading;backup_used<=0;segment_heading<=0;force_step<=0;sector_done<=0;sector_heading<=0;preference_seen<=0;preference_origin<=0;
   end else if(active)begin
    if(tick && timer<32'hffffffff)timer<=timer+1'b1;
    if(range_age>=25)begin state<=FAULT;reason<=8'h14;end
    else begin
     if(tick)begin
      // IMU measures coverage and stalled/reversed rotation, not opening width.
      if(state==SEARCH || state==VERIFY)begin
       if(yaw_step>=0 && yaw_step<3000)scan_travel<=scan_travel+yaw_step;
       else if(yaw_step<0 && yaw_step> -3000)
        scan_travel<=scan_travel>=-yaw_step?scan_travel+yaw_step:0;
      end
      if(state==SEARCH)begin
       if(yaw_step<0 && yaw_step> -3000)wrong_travel<=wrong_travel-yaw_step;
       else if(yaw_step>0)wrong_travel<=0;
       if(progress>=500)begin progress_heading<=heading;progress_age<=0;end
       else if(progress_age<400)progress_age<=progress_age+1'b1;
      end else begin progress_heading<=heading;progress_age<=0;wrong_travel<=0;end
      if(state==VERIFY && stopped && yaw_motion<=80)begin
       if(stable<SETTLE_TICKS)stable<=stable+1'b1;
      end else stable<=0;
     end
     case(state)
      BRAKE:if(timer>=BRAKE_TICKS)begin
       timer<=0;samples<=0;stable<=0;
       if(critical)begin
        if(!backup_used)begin state<=BACKUP;backup_used<=1;backup_x<=x_mm;backup_y<=y_mm;end
        else begin state<=WAITING;reason<=8'h2a;end
       end else if(fresh_clear)state<=VERIFY;
       else begin state<=SEARCH;segment_heading<=heading;end
      end
      SEARCH:begin
       if(critical)begin state<=BRAKE;timer<=0;end
       else if(wrong_travel>=1000)begin state<=FAULT;reason<=8'h27;end
       else if(progress_age>=400)begin state<=FAULT;reason<=8'h21;end
       // A fresh clear ray stops immediately; no added clearance angle.
       else if((fresh_clear && !force_step) || segment_turn>=SEARCH_STEP_CDEG)begin
        state<=VERIFY;timer<=0;samples<=0;stable<=0;force_step<=0;sector_done<=0;
       end else if(scan_travel>=36000)begin
        state<=WAITING;reason<=8'h20;timer<=0;samples<=0;
       end
      end
      VERIFY:begin
       if(critical)begin state<=BRAKE;timer<=0;samples<=0;end
       else if(timer>=100)begin
        timer<=0;samples<=0;stable<=0;sector_done<=0;
        // A stopped car with inconsistent clearance rejects this candidate,
        // like task three's bad-range branch; it does not abandon the goal.
        // If the wheels will not stop, do not command another scan.
        if(!stopped)begin state<=FAULT;reason<=8'h22;end
        else if(verify_reselects<VERIFY_RESELECTS && scan_travel<36000)begin
         verify_reselects<=verify_reselects+1'b1;state<=SEARCH;segment_heading<=heading;force_step<=1;
        end else begin state<=WAITING;reason<=8'h22;end
       end
       // Ignore coasting reports. Three NEW reports at the stopped heading
       // authorize forward. A bad report cancels the candidate, not autonomy.
       else if(timer<BRAKE_TICKS || stable<SETTLE_TICKS || !stopped)samples<=0;
       else if(distance_valid)begin
        if(fresh_clear)begin
         if(samples>=SAFE_SAMPLES-1)begin
          timer<=0;samples<=0;
          if(!preference_seen)begin preference_seen<=1;preference_origin<=scan_travel;end
          if(!goal_accept)begin state<=SEARCH;segment_heading<=heading;force_step<=1;sector_done<=0;end
          else if(!sector_done)begin state<=SECTOR_LEFT;sector_heading<=heading;end
          else begin state<=FORWARD;sector_done<=0;probe_x<=x_mm;probe_y<=y_mm;verify_reselects<=0;end
         end
         else samples<=samples+1'b1;
        end else begin
         timer<=0;samples<=0;sector_done<=0;
         if(scan_travel>=36000)begin state<=WAITING;reason<=8'h20;end
         else begin state<=SEARCH;segment_heading<=heading;end
        end
       end
      end
      SECTOR_LEFT,SECTOR_RIGHT,SECTOR_RETURN:begin
       if(critical)begin state<=BRAKE;timer<=0;samples<=0;sector_done<=0;end
       else if(timer>=SECTOR_LIMIT_TICKS || sector_stall>=SECTOR_STALL_TICKS)begin state<=FAULT;reason<=8'h36;end
       else if(distance_valid && !fresh_clear)begin
        state<=SEARCH;segment_heading<=heading;force_step<=1;sector_done<=0;timer<=0;samples<=0;
       end else if(sector_aligned && stopped && distance_valid)begin
        if(samples>=SAFE_SAMPLES-1)begin
         samples<=0;timer<=0;
         if(state==SECTOR_LEFT)state<=SECTOR_RIGHT;
         else if(state==SECTOR_RIGHT)state<=SECTOR_RETURN;
         else begin state<=VERIFY;sector_done<=1;stable<=0;end
        end else samples<=samples+1'b1;
       end else if(!sector_aligned || !stopped)samples<=0;
      end
      FORWARD:begin
       if(obstacle || critical)begin state<=BRAKE;timer<=0;scan_travel<=0;sector_done<=0;preference_seen<=0;preference_origin<=0;end
       else if(forward_math_valid && forward_distance>=`T4_PROBE_MM && timer>=PROBE_TICKS)state<=DONE;
       else if(forward_stall>=FORWARD_STALL_TICKS || timer>=FORWARD_LIMIT_TICKS)begin state<=FAULT;reason<=8'h37;end
      end
      DONE:if(obstacle || critical)begin state<=BRAKE;timer<=0;scan_travel<=0;sector_done<=0;preference_seen<=0;preference_origin<=0;end
      BACKUP:if((&backup_pipeline && reverse_distance2>=BACKUP_MM*BACKUP_MM) || timer>=BACKUP_TICKS)begin
       state<=WAITING;reason<=timer>=BACKUP_TICKS?8'h29:8'h2b;timer<=0;samples<=0;
      end
      WAITING:begin
       if(timer<((reason==8'h20 || reason==8'h22) ? RETRY_TICKS : BRAKE_TICKS))samples<=0;
       else if(distance_valid)begin
        if(distance_mm>=ROTATE_CLEAR_MM)begin
         if(samples>=SAFE_SAMPLES-1)begin
          state<=fresh_clear?VERIFY:SEARCH;timer<=0;samples<=0;stable<=0;
          scan_travel<=0;reason<=0;segment_heading<=heading;force_step<=0;sector_done<=0;preference_seen<=0;preference_origin<=0;verify_reselects<=0;
         end else samples<=samples+1'b1;
        end else samples<=0;
       end
      end
      default:begin state<=FAULT;reason<=8'hff;end
     endcase
    end
   end
  end
 end
endmodule
`default_nettype wire
