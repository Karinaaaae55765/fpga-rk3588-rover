`default_nettype none
// Arc-length lookahead, approximate-distance pure pursuit; 50 Hz output.
// Mandatory waypoint gates are based on observed pose, never ROM progress alone.
module task4_path_follower #(parameter ROUTE_FILE="route.mem",parameter integer LEFT_SIGN=1,RIGHT_SIGN=1)(
 input wire clk,rst_n,tick,reset_route,enable,slow,pose_valid,allow_advance,direct_goal,drive_active,
 input wire signed [31:0] left_count,right_count,
 input wire signed [31:0] x_mm,y_mm,left_actual,right_actual,
 input wire signed [15:0] heading,
 input wire heading_fault,input wire [7:0] heading_reason,
 output reg signed [31:0] left_rpm,right_rpm,
 output reg [3:0] waypoint,output reg done,failed,output reg goal_reached,
 output reg dwelling,output reg [7:0] reason
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

 reg [31:0] rom[0:511];initial $readmemh(ROUTE_FILE,rom);
 reg [4:0] state;
 reg [8:0] progress,scan,best_index,last_gate,look_index;
 reg [31:0] best_distance,goal_distance;
 wire [8:0] look_steps=`T4_LOOK_STEPS;
 reg signed [31:0] dx,dy,gdx,gdy,saved_x,saved_y;
 reg signed [15:0] saved_h;
 reg [63:0] d2;
 reg [31:0] magnitude,denominator,quotient,remainder,numerator;
 reg [5:0] div_bit;
 reg signed [31:0] error,base,delta,wanted_l,wanted_r;
 reg [15:0] dwell_ticks;
 reg turning,was_enabled;reg departure_align;reg [7:0] stable;
 reg math_start,math_mode;
 reg signed [31:0] math_x,math_y;reg signed [15:0] math_angle;
 wire math_done;wire signed [15:0] bearing;wire signed [31:0] cosine,sine;
 nav_math math(.clk(clk),.rst_n(rst_n),.clear(!enable || reset_route),.start(math_start),.mode(math_mode),
 .x_in(math_x),.y_in(math_y),.angle_in(math_angle),.busy(),.done(math_done),.bearing(bearing),.cos_q15(cosine),.sin_q15(sine));
 function automatic signed [31:0] abs32(input signed [31:0] v);abs32=v<0?-v:v;endfunction
 function automatic signed [31:0] norm(input signed [31:0] v);norm=v>18000?v-36000:v< -18000?v+36000:v;endfunction
 function automatic signed [31:0] slew(input signed [31:0] old_value,target);
   slew=target>old_value+2?old_value+2:target<old_value-2?old_value-2:target;
 endfunction
 // Measure quadrature count progress only after the physical drive is ready.
 // Signs must match pose_estimator, not motor electrical reversal flags.
 reg feedback_fault;reg [7:0] feedback_reason;
 reg [7:0] missing_l,missing_r,wrong_sign,translation_ticks,rotation_ticks,drive_settle,drive_wait;
 reg signed [31:0] count_anchor_l,count_anchor_r,last_count_l,last_count_r,sign_anchor_l,sign_anchor_r;
 reg signed [31:0] watch_x,watch_y;reg signed [15:0] watch_heading;
 reg [1:0] watch_direction;
 // Displacement protection runs at 50 Hz; split subtraction and Manhattan
 // distance over FPGA clocks instead of placing both in one 20 ns path.
 reg signed [31:0] watch_dx,watch_dy;
 reg [31:0] watch_distance;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin watch_dx<=0;watch_dy<=0;watch_distance<=0;end
  else begin
   watch_dx<=x_mm-watch_x;watch_dy<=y_mm-watch_y;
   watch_distance<=abs32(watch_dx)+abs32(watch_dy);
  end
 end
 wire commanded_l=abs32(left_rpm)>=8,commanded_r=abs32(right_rpm)>=8;
 wire signed [31:0] delta_l=(left_count-sign_anchor_l)*LEFT_SIGN,delta_r=(right_count-sign_anchor_r)*RIGHT_SIGN;
 wire contradictory=(commanded_l && abs32(delta_l)>=8 && left_rpm[31]!=delta_l[31]) ||
                    (commanded_r && abs32(delta_r)>=8 && right_rpm[31]!=delta_r[31]);
 wire translating=left_rpm>=8 && right_rpm>=8;
 wire rotating=(left_rpm<=-8 && right_rpm>=8) || (left_rpm>=8 && right_rpm<=-8);
 wire requested_motion=commanded_l || commanded_r;
 wire same_direction=watch_direction=={left_rpm[31],right_rpm[31]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   feedback_fault<=0;feedback_reason<=0;missing_l<=0;missing_r<=0;wrong_sign<=0;
   translation_ticks<=0;rotation_ticks<=0;drive_settle<=0;drive_wait<=0;watch_direction<=0;
   sign_anchor_l<=0;sign_anchor_r<=0;count_anchor_l<=0;count_anchor_r<=0;last_count_l<=0;last_count_r<=0;
   watch_x<=0;watch_y<=0;watch_heading<=0;
  end else if(reset_route)begin
   feedback_fault<=0;feedback_reason<=0;missing_l<=0;missing_r<=0;wrong_sign<=0;
   translation_ticks<=0;rotation_ticks<=0;drive_settle<=0;drive_wait<=0;watch_direction<={left_rpm[31],right_rpm[31]};
   sign_anchor_l<=left_count;sign_anchor_r<=right_count;count_anchor_l<=left_count;count_anchor_r<=right_count;last_count_l<=left_count;last_count_r<=right_count;
   watch_x<=x_mm;watch_y<=y_mm;watch_heading<=heading;
  end else if(!enable || done || failed || !pose_valid)begin
   missing_l<=0;missing_r<=0;wrong_sign<=0;translation_ticks<=0;rotation_ticks<=0;drive_settle<=0;drive_wait<=0;
   last_count_l<=left_count;last_count_r<=right_count;
  end else if(tick && !feedback_fault)begin
   last_count_l<=left_count;last_count_r<=right_count;watch_direction<={left_rpm[31],right_rpm[31]};
   // Missing drive enable is distinct from absent encoder feedback.
   if(requested_motion && !drive_active)begin
    if(drive_wait>=99)begin feedback_fault<=1;feedback_reason<=8'h3b;end
    else drive_wait<=drive_wait+1'b1;
   end else drive_wait<=0;
   if(!drive_active || !requested_motion || !same_direction)begin
    drive_settle<=0;missing_l<=0;missing_r<=0;wrong_sign<=0;translation_ticks<=0;rotation_ticks<=0;
    sign_anchor_l<=left_count;sign_anchor_r<=right_count;count_anchor_l<=left_count;count_anchor_r<=right_count;
   end else if(drive_settle<15)begin
    // 300 ms of stable, enabled direction after ramp/dead-time changes.
    drive_settle<=drive_settle+1'b1;sign_anchor_l<=left_count;sign_anchor_r<=right_count;count_anchor_l<=left_count;count_anchor_r<=right_count;
    missing_l<=0;missing_r<=0;wrong_sign<=0;translation_ticks<=0;rotation_ticks<=0;
   end else begin
    // Four net edges renew a two-second count lease; rounding RPM to zero
    // cannot masquerade as a disconnected encoder. Pure +/-1 chatter does not renew it.
    if(!commanded_l || abs32(left_count-count_anchor_l)>=4)begin missing_l<=0;count_anchor_l<=left_count;end
    else if(missing_l>=99)begin feedback_fault<=1;feedback_reason<=8'h37;end
    else missing_l<=missing_l+1'b1;
    if(!commanded_r || abs32(right_count-count_anchor_r)>=4)begin missing_r<=0;count_anchor_r<=right_count;end
    else if(missing_r>=99)begin feedback_fault<=1;feedback_reason<=8'h37;end
    else missing_r<=missing_r+1'b1;
    // Judge direction from one second of NET displacement. Alternating
    // single-edge chatter must not become a sustained reverse-direction alarm.
    if(wrong_sign==0)begin sign_anchor_l<=left_count;sign_anchor_r<=right_count;end
    if(wrong_sign>=49)begin
     wrong_sign<=0;
     if(contradictory)begin feedback_fault<=1;feedback_reason<=8'h38;end
    end else wrong_sign<=wrong_sign+1'b1;
    if(translating)begin
     if(translation_ticks==0)begin watch_x<=x_mm;watch_y<=y_mm;end
     if(translation_ticks>=100)begin
      translation_ticks<=0;
      if(watch_distance<10)begin feedback_fault<=1;feedback_reason<=8'h39;end
     end else translation_ticks<=translation_ticks+1'b1;
    end else translation_ticks<=0;
    if(rotating)begin
     if(rotation_ticks==0)watch_heading<=heading;
     if(rotation_ticks>=100)begin
      rotation_ticks<=0;
      if(abs32(norm($signed(heading)-$signed(watch_heading)))<300)begin feedback_fault<=1;feedback_reason<=8'h3a;end
     end else rotation_ticks<=rotation_ticks+1'b1;
    end else rotation_ticks<=0;
   end
  end
 end
 wire [32:0] shifted_remainder={remainder,numerator[31]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   dwelling<=0;dwell_ticks<=0;state<=0;progress<=0;scan<=0;best_index<=0;last_gate<=0;look_index<=0;best_distance<=0;goal_distance<=0;
   dx<=0;dy<=0;gdx<=0;gdy<=0;saved_x<=0;saved_y<=0;saved_h<=0;d2<=0;magnitude<=0;
   denominator<=1;quotient<=0;remainder<=0;numerator<=0;div_bit<=0;error<=0;base<=0;delta<=0;
   wanted_l<=0;wanted_r<=0;turning<=0;departure_align<=0;stable<=0;was_enabled<=0;math_start<=0;math_mode<=0;
   math_x<=0;math_y<=0;math_angle<=0;left_rpm<=0;right_rpm<=0;waypoint<=0;done<=0;failed<=0;goal_reached<=0;reason<=0;
  end else begin
   math_start<=0;was_enabled<=enable;
   if(!enable)goal_reached<=0;
   if(reset_route)begin
    dwelling<=0;dwell_ticks<=0;state<=0;progress<=0;last_gate<=0;waypoint<=0;done<=0;failed<=0;goal_reached<=0;reason<=0;turning<=0;departure_align<=0;stable<=0;left_rpm<=0;right_rpm<=0;
   end else if(!enable || !pose_valid || done || failed)begin
    dwelling<=0;dwell_ticks<=0;state<=0;left_rpm<=0;right_rpm<=0;stable<=0;turning<=0;
   end else case(state)
    0:if(tick)begin
     if(x_mm>30000 || x_mm< -30000 || y_mm>30000 || y_mm< -30000)begin failed<=1;reason<=8'h34;end
     else begin saved_x<=x_mm;saved_y<=y_mm;saved_h<=heading;
      gdx<=goal_x(waypoint)-x_mm;gdy<=goal_y(waypoint)-y_mm;state<=1;end
    end
    1:begin d2<=$signed(gdx)*$signed(gdx)+$signed(gdy)*$signed(gdy);state<=2;end
    2:begin
     // Arrival applies to every goal, including while rejoining; only NAV
     // can count the dwell or consume a goal. Drift or motion resets the clock.
     goal_reached<=d2<=(waypoint==`T4_GOALS-1 ? `T4_GOAL_MM*`T4_GOAL_MM : `T4_PASS_MM*`T4_PASS_MM);
     goal_distance<=d2>32'hffffffff ? 32'hffffffff : d2[31:0];
     if(d2<=(waypoint==`T4_GOALS-1 ? `T4_GOAL_MM*`T4_GOAL_MM : `T4_PASS_MM*`T4_PASS_MM))begin
      dwelling<=1;left_rpm<=0;right_rpm<=0;wanted_l<=0;wanted_r<=0;
      if(abs32(left_actual)<=3 && abs32(right_actual)<=3 && allow_advance)begin
       if(stable<10)begin stable<=stable+1'b1;dwell_ticks<=0;end
       else if(dwell_ticks>=`T4_DWELL_TICKS-1)begin
        dwell_ticks<=0;stable<=0;dwelling<=0;turning<=0;departure_align<=1;
        if(waypoint==`T4_GOALS-1)done<=1;
        else begin last_gate<=gate_index(waypoint);waypoint<=waypoint+1'b1;end
       end else dwell_ticks<=dwell_ticks+1'b1;
      end else begin stable<=0;dwell_ticks<=0;end
      state<=0;
     end else begin
      dwelling<=0;dwell_ticks<=0;stable<=turning?stable:0;state<=3;
     end
    end
    3:begin scan<=last_gate;best_index<=last_gate;best_distance<=32'hffffffff;state<=4;end
    4:begin dx<=$signed(rom[scan][31:16])-saved_x;dy<=$signed(rom[scan][15:0])-saved_y;state<=5;end
    5:begin d2<=$signed(dx)*$signed(dx)+$signed(dy)*$signed(dy);state<=6;end
    6:begin
     if(d2<best_distance)begin best_distance<=d2[31:0];best_index<=scan;end
     if(scan>=gate_index(waypoint))state<=7;
     else begin scan<=scan+1'b1;state<=4;end
    end
    7:begin
     progress<=best_index;
     // Clamp lookahead to the current stop. Gate completion requires
     // observed proximity, stopped wheels and the complete dwell.
     if(best_index>=gate_index(waypoint) && goal_distance>90000)look_index<=gate_index(waypoint);
     else look_index<=best_index+look_steps>=gate_index(waypoint)?gate_index(waypoint):best_index+look_steps;
     state<=8;
    end
    8:begin
     // After avoidance, approach the retained goal instead of the old blocked line.
     dx<=direct_goal || departure_align || turning || goal_distance<360000?goal_x(waypoint)-saved_x:$signed(rom[look_index][31:16])-saved_x;
     dy<=direct_goal || departure_align || turning || goal_distance<360000?goal_y(waypoint)-saved_y:$signed(rom[look_index][15:0])-saved_y;state<=9;
    end
    9:begin
     math_x<=dx;math_y<=dy;math_mode<=1;math_start<=1;
     magnitude<=abs32(dx)>abs32(dy)?abs32(dx)+(abs32(dy)>>1):abs32(dy)+(abs32(dx)>>1);
     state<=10;
    end
    10:if(math_done)begin error<=norm($signed(bearing)-$signed(saved_h));state<=11;end
    11:begin
     departure_align<=0;
     if(turning || abs32(error)>(departure_align?4500:6500))begin
      turning<=1;
      if(abs32(error)<=1200)begin
       wanted_l<=0;wanted_r<=0;
       if(abs32(left_actual)<=3 && abs32(right_actual)<=3)begin
        if(stable>=4)begin turning<=0;stable<=0;end else stable<=stable+1'b1;
       end else stable<=0;
      end else begin
       stable<=0;wanted_l<=error>0?-`T4_ALIGN_RPM:`T4_ALIGN_RPM;wanted_r<=error>0?`T4_ALIGN_RPM:-`T4_ALIGN_RPM;
      end
      state<=18;
     end else begin
      base<=goal_distance<`T4_NEAR_MM*`T4_NEAR_MM?`T4_NEAR_RPM:
       slow || abs32(error)>2500 || goal_distance<`T4_APPROACH_MM*`T4_APPROACH_MM?`T4_APPROACH_RPM:`T4_CRUISE_RPM;
      math_mode<=0;math_angle<=error;math_start<=1;state<=12;
     end
    end
    12:if(math_done)begin
     numerator<=(base*`T4_TRACK_MM*abs32(sine))>>15;
     denominator<=magnitude<80?80:magnitude;quotient<=0;remainder<=0;div_bit<=0;state<=13;
    end
    13:begin
     numerator<=numerator<<1;
     if(shifted_remainder>={1'b0,denominator})begin remainder<=shifted_remainder-denominator;quotient<={quotient[30:0],1'b1};end
     else begin remainder<=shifted_remainder[31:0];quotient<=quotient<<1;end
     if(div_bit==31)state<=14;else div_bit<=div_bit+1'b1;
    end
    14:begin delta<=abs32(error)>9000 || quotient>base-`T4_MIN_RPM?base-`T4_MIN_RPM:quotient;state<=15;end
    15:begin
     wanted_l<=error>=0?base-delta:(base+delta>`T4_CRUISE_RPM?`T4_CRUISE_RPM:base+delta);
     wanted_r<=error>=0?(base+delta>`T4_CRUISE_RPM?`T4_CRUISE_RPM:base+delta):base-delta;state<=18;
    end
    18:begin left_rpm<=slew(left_rpm,wanted_l);right_rpm<=slew(right_rpm,wanted_r);state<=0;end
    default:begin failed<=1;reason<=8'h3f;state<=0;end
   endcase
   // Latched diagnostic fault wins over normal route updates. Never consume
   // a waypoint merely because a timeout elapsed or wheels were commanded.
   if(feedback_fault && !reset_route)begin
    failed<=1;reason<=feedback_reason;left_rpm<=0;right_rpm<=0;dwelling<=0;dwell_ticks<=0;
   end
   if(heading_fault && !reset_route)begin
    failed<=1;reason<=heading_reason;left_rpm<=0;right_rpm<=0;dwelling<=0;dwell_ticks<=0;
   end
   // Rejoining after a local avoidance step is not normal curve following:
   // aim while stationary first, so the forward sensor sees the new direction.
   if(enable && !was_enabled && !reset_route && !done && !failed)turning<=1;
  end
 end
endmodule
`default_nettype wire
