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
