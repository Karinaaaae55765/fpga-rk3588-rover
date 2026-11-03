// Task5 camera transport + task6 UART external control; baseline 20261102_095046.
// Dual task-three/task-four integration based on 20261028_204928.
// USER-AUTHORIZED BASELINE: motion/avoidance enabled; directions and clearance NOT physically verified
`define T123_KEYS_CONFIRMED 1
`define T123_VEHICLE_CONFIRMED 1
`define T123_CLEARANCE_CONFIRMED 1
`define T123_ADDRESS 65280
`define T123_RPM_Q16 132843
`define T123_MM_Q16 9042
`define T123_LEFT_SIGN 1
`define T123_RIGHT_SIGN 1
`define T123_YAW_SIGN 1
`define T123_MOTOR_REVERSE 3
`define T123_MAX_RPM 60
`define T123_CRUISE_RPM 60
`define T123_TURN_RPM 45
`define T123_SIDE_MM 300
`define T123_FORWARD_MM 1000
`define T123_STOP_MM 200
`define T123_RELEASE_MM 300
`define T123_SLOW_MM 500
`define T123_WIDTH_MM 145
`define T123_CLEAR_MM 500
`define T123_MARGIN_MM 50
`define T123_NEAR_TURN_RPM 20
`define T123_ALIGN_CDEG 500
`define T123_STABLE_TICKS 10
`define T123_KEY_2 24
`define T123_KEY_8 82
`define T123_KEY_4 8
`define T123_KEY_6 90
`define T123_KEY_5 28
`define T123_KEY_0 22
`define T123_KEY_AUTO 67
`define T123_EMERGENCY_MM 100
`define T123_BACKUP_MM 100
`define T123_BACKUP_RPM 15
`define T123_BACKUP_TICKS 200
`define T123_SCAN_STEP_CDEG 1500
`define T123_GAP_CDEG 3000
`define T123_SAFE_SAMPLES 3
module task6_top #(parameter SENSOR_BARS=0,CLK_FREQ=50000000,OUTPUT_UNLOCK=1,
 parameter BT_TIMEOUT_MS=500,BT_RELEASE_MS=300,parameter ROUTE_FILE="route.mem")(
 input wire clk50M,reset_n,tof_uart_rx,imu_uart_rx,
 input wire control_rx,ir_rx,bt_rx,enc1_a,enc1_b,enc2_a,enc2_b,
 output wire sensor_uart_tx,
 output wire camera_sclk,inout wire camera_sdat,
 input wire camera_vsync,camera_href,camera_pclk,input wire [7:0] camera_data,
 output wire camera_xclk,camera_rst_n,camera_pwdn,
 output wire eth_rst_n,eth_mdc,inout wire eth_mdio,
 output wire rgmii_gtxc,rgmii_txen,output wire [3:0] rgmii_txd,
 output wire [8:0] led,
 output wire motor_pwm1,motor_ain1,motor_ain2,motor_pwm2,motor_bin1,motor_bin2,motor_stby
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


 // 2026-10-31 on-car capture: forward target (+20,+20), raw RPM (+18,-19), fault 0x38.
 // Only task four changes encoder interpretation; raw telemetry and motor wiring stay unchanged.
 localparam integer NAV_LEFT_SIGN=1,NAV_RIGHT_SIGN=-1;
 localparam [2:0] BASE_POSE_SIGNS={`T123_YAW_SIGN<0,`T123_RIGHT_SIGN<0,`T123_LEFT_SIGN<0};
 localparam [2:0] NAV_POSE_SIGNS={`T123_YAW_SIGN<0,NAV_RIGHT_SIGN<0,NAV_LEFT_SIGN<0};
 wire route_mode;
 wire [2:0] pose_signs=route_mode?NAV_POSE_SIGNS:BASE_POSE_SIGNS;
 wire clk125M,system_n,net_n,cam_clock_good,phy_n;
 udp_clock_reset clocks(clk50M,reset_n,clk125M,camera_xclk,system_n,net_n,cam_clock_good);
 udp_reset_release phy_reset(clk50M,net_n,phy_n);
 wire initialized;wire [7:0] init_error;wire [15:0] chip_id,failed_register;
 ov5640_setup #(.SENSOR_BARS(SENSOR_BARS)) camera_config(.clk(clk50M),.reset_n(cam_clock_good),
  .camera_reset_n(camera_rst_n),.camera_pwdn(camera_pwdn),.scl(camera_sclk),.sda(camera_sdat),
  .ready(initialized),.error_code(init_error),.chip_id(chip_id),.failed_register(failed_register));
 wire phy_link;wire [7:0] phy_fault;wire [31:0] phy_id;
 wire [7:0] txd;wire txen;
 ethernet_phy phy(.clk(clk50M),.reset_n(phy_n),.phy_reset_n(eth_rst_n),.mdc(eth_mdc),.mdio(eth_mdio),
  .link_up(phy_link),.fault(phy_fault),.phy_id(phy_id),.tx_clk(clk125M),.tx_data(txd),.tx_en(txen),
  .rgmii_txc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
 wire fifo_n=net_n&&cam_clock_good;
 wire capture_n;udp_reset_release cap_reset(camera_pclk,fifo_n,capture_n);
 (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [1:0] cfg_sync,link_cam,link_net;
 always @(posedge camera_pclk or negedge capture_n)
  if(!capture_n)begin cfg_sync<=0;link_cam<=0;end
  else begin cfg_sync<={cfg_sync[0],initialized};link_cam<={link_cam[0],phy_link};end
 always @(posedge clk125M or negedge net_n)
  if(!net_n)link_net<=0;else link_net<={link_net[0],phy_link};
 wire we,wcommit,wready,wend;wire [10:0] wa;wire [7:0] wd;
 wire [31:0] wf,good,bad;wire [15:0] wi,wl;
 camera_frontend capture(.clk(camera_pclk),.reset_n(capture_n),.configured(cfg_sync[1]),
  .enabled(link_cam[1]),.vsync(camera_vsync),.href(camera_href),.data(camera_data),
  .slot_ready(wready),.wr_en(we),.wr_commit(wcommit),.wr_addr(wa),.wr_data(wd),
  .wr_frame(wf),.wr_index(wi),.wr_length(wl),.wr_end(wend),.good_frames(good),.bad_frames(bad));
 wire pv,release_slot,pend;wire [10:0] pa;wire [7:0] pd;
 wire [31:0] pf;wire [15:0] pi,pl;
 cdc_packet_buffer packets(camera_pclk,clk125M,fifo_n,we,wcommit,wa,wd,wf,wi,wl,wend,wready,
  pv,release_slot,pa,pd,pf,pi,pl,pend);
 wire ml,ms,mready,mdone;wire [10:0] ma;wire [7:0] mb;wire [15:0] mlen;
 wire [31:0] sent;
 udp_video_tx sender(clk125M,net_n,link_net[1],pv,pf,pi,pl,pend,
  release_slot,pa,pd,ml,mready,mdone,ma,mb,ms,mlen,sent);
 ethernet_mac_tx mac(clk125M,net_n,ml,ma,mb,ms,mlen,mready,txd,txen,mdone);
 wire [15:0] distance_mm,tof_strength,tof_error_count,imu_error_count;
 wire signed [15:0] roll_cdeg,pitch_cdeg,yaw_cdeg;
 wire distance_valid,imu_valid;
 perception_hub #(.CLK_FREQ(CLK_FREQ)) sensor_hub(.clk(clk50M),.rst_n(system_n),.tof_uart_rx(tof_uart_rx),.imu_uart_rx(imu_uart_rx),
  .front_distance_mm(distance_mm),.tof_strength(tof_strength),.distance_valid(distance_valid),
  .roll_cdeg(roll_cdeg),.pitch_cdeg(pitch_cdeg),.yaw_cdeg(yaw_cdeg),.imu_valid(imu_valid),
  .tof_error_count(tof_error_count),.imu_error_count(imu_error_count));
 wire legacy_tx,legacy_done;
 sensor_telemetry_tx_v3 #(.CLK_FREQ(CLK_FREQ)) telemetry(.clk(clk50M),.rst_n(system_n),.distance_mm(distance_mm),
  .tof_strength(tof_strength),.distance_valid(distance_valid),.roll_cdeg(roll_cdeg),
  .pitch_cdeg(pitch_cdeg),.yaw_cdeg(yaw_cdeg),.imu_valid(imu_valid),
  .tof_error_count(tof_error_count),.imu_error_count(imu_error_count),
  .uart_tx_pin(legacy_tx),.frame_done(legacy_done),.frame_toggle());
 // 50 Hz autonomous control, independent of RK3588 command timing.
 reg [19:0] tick_counter;
 wire tick=tick_counter==CLK_FREQ/50-1;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)tick_counter<=0;else if(tick)tick_counter<=0;else tick_counter<=tick_counter+1'b1;
 wire ir_frame,ir_repeat,ir_invalid,ir_activity;
 wire [15:0] ir_address,ir_count,ir_errors;wire [7:0] ir_command;
 ir_remote_rx #(.CLK_FREQ(CLK_FREQ)) ir(.clk(clk50M),.rst_n(system_n),.ir_rx(ir_rx),
  .frame_valid(ir_frame),.repeat_valid(ir_repeat),.invalid_frame(ir_invalid),.activity(ir_activity),
  .address(ir_address),.command(ir_command),.valid_count(ir_count),.error_count(ir_errors));
 wire [2:0] manual_action;wire auto_start,remote_stop,serial_stop,remote_blocked,released;
 wire [2:0] ir_action,bt_action;
 wire ir_auto,ir_stop,ir_released,ir_block,source_changed;
 wire bt_online,bt_auto,bt_stop,bt_released,bt_valid,bt_connect,bt_disconnect,bt_invalid;
 wire signed [7:0] bt_lx,bt_ly,bt_rx_axis,bt_ry_axis;
 wire [7:0] bt_dpad,bt_buttons2,bt_buttons3,bt_rt,bt_lt;
 wire [15:0] bt_valid_count,bt_error_count;
 wire autonomous,reset_origin,avoid_start,avoid_cancel,motor_enable;
 wire base_motor_enable;
 wire ext_armed,ext_enable;wire signed [31:0] ext_left,ext_right;
 wire [31:0] ext_session,ext_challenge;wire [15:0] ext_accepted;wire [7:0] ext_reason;
 wire [3:0] state,avoid_phase;wire [7:0] reason,avoid_reason,motor_reason;
 wire signed [31:0] left_count,right_count,left_actual,right_actual,left_request,right_request;
 wire signed [31:0] controller_left_request,controller_right_request;
 wire nav_pose_valid,nav_pose_fault,nav_trusted,nav_degraded;
 wire [7:0] nav_pose_reason,nav_quality;
 wire signed [31:0] nav_x,nav_y,legacy_x,legacy_y;
 wire signed [15:0] nav_heading,encoder_heading,relative_imu,heading_difference,heading_correction,legacy_heading;
 wire legacy_pose_valid,legacy_pose_fault;
 wire [15:0] heading_bad_ticks,heading_agree_ticks,heading_imu_age;
 wire signed [31:0] route_rpm_limit=nav_degraded?`T4_ODO_DEGRADED_RPM:`T4_ODO_VALIDATION_RPM;
 function automatic signed [31:0] limited(input signed [31:0] value,limit);
  limited=value>limit?limit:value< -limit?-limit:value;
 endfunction
 // Register navigation-only clipping before the downstream range/reversal gates.
 // Task-three retains its direct path; motor_enable still gates the physical pads immediately.
 reg signed [31:0] nav_left_limited,nav_right_limited;
 always @(posedge clk50M or negedge system_n)begin
  if(!system_n)begin nav_left_limited<=0;nav_right_limited<=0;end
  else begin
   nav_left_limited<=limited(controller_left_request,route_rpm_limit);
   nav_right_limited<=limited(controller_right_request,route_rpm_limit);
  end
 end
 // Pipeline external speed arithmetic, but never delay the physical enable interlock.
 reg signed [31:0] ext_left_q,ext_right_q;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)begin ext_left_q<=0;ext_right_q<=0;end
  else begin ext_left_q<=ext_left;ext_right_q<=ext_right;end
 assign left_request=ext_armed?ext_left_q:route_mode?nav_left_limited:controller_left_request;
 assign right_request=ext_armed?ext_right_q:route_mode?nav_right_limited:controller_right_request;
 wire signed [31:0] nav_left_count,nav_right_count,nav_left_actual,nav_right_actual;
 wire [31:0] left_rejected,right_rejected,left_invalid,right_invalid;
 wire signed [31:0] control_left_count=route_mode?nav_left_count:left_count;
 wire signed [31:0] control_right_count=route_mode?nav_right_count:right_count;
 wire signed [31:0] control_left_actual=route_mode?nav_left_actual:left_actual;
 wire signed [31:0] control_right_actual=route_mode?nav_right_actual:right_actual;
 wire [9:0] left_duty,right_duty;wire motor_fault,pose_fault,pose_valid;
 wire signed [31:0] x_mm,y_mm;wire signed [15:0] heading;
 assign x_mm=route_mode?nav_x:legacy_x;assign y_mm=route_mode?nav_y:legacy_y;
 assign heading=route_mode?nav_heading:legacy_heading;
 assign pose_valid=route_mode?nav_pose_valid:legacy_pose_valid;
 assign pose_fault=route_mode?nav_pose_fault:legacy_pose_fault;
 wire obstacle,slow;wire raw_sensors_ready,raw_hard_fault;wire [7:0] raw_safety_reason;
 reg sensors_ready,hard_fault;reg [7:0] safety_reason;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)begin sensors_ready<=0;hard_fault<=1;safety_reason<=8'h13;end
  else begin
   sensors_ready<=raw_sensors_ready;hard_fault<=raw_hard_fault || (route_mode && nav_pose_fault);
   safety_reason<=route_mode && nav_pose_fault?nav_pose_reason:raw_safety_reason;
  end
 wire [7:0] rx_sequence;wire [15:0] rx_errors;
 nav_command_rx #(.CLK_FREQ(CLK_FREQ)) serial_commands(.clk(clk50M),.rst_n(system_n),.rx(control_rx),
  .stop_request(serial_stop),.last_sequence(rx_sequence),.errors(rx_errors));
 wire ext_valid,ext_invalid;wire [7:0] ext_command;
 wire stop_request=remote_stop || serial_stop || (ext_valid && (ext_command==3 || ext_command==4));
 wire [31:0] rx_session,rx_challenge;wire [15:0] rx_seq;
 wire signed [15:0] rx_left,rx_right;
 task6_command_rx #(.CLK_FREQ(CLK_FREQ)) external_rx(.clk(clk50M),.rst_n(system_n),.rx(control_rx),
  .valid(ext_valid),.invalid(ext_invalid),.command(ext_command),.session(rx_session),.challenge(rx_challenge),
  .sequence_number(rx_seq),.left_rpm(rx_left),.right_rpm(rx_right));
 task6_control external_control(.clk(clk50M),.rst_n(system_n),.tick(tick),
  .ready(raw_sensors_ready),.hard_fault(raw_hard_fault),.obstacle(obstacle),.slow(slow),
  .stop_request(stop_request),.manual_takeover(manual_action!=0 || auto_start || source_changed || bt_disconnect),
  .idle(state==0 || state==6),.released(released),.valid(ext_valid),.invalid(ext_invalid),
  .command(ext_command),.session_in(rx_session),.challenge_in(rx_challenge),.sequence_in(rx_seq),
  .left_in(rx_left),.right_in(rx_right),.armed(ext_armed),.enable(ext_enable),
  .session(ext_session),.challenge(ext_challenge),.accepted_sequence(ext_accepted),.reason(ext_reason),
  .left_request(ext_left),.right_request(ext_right));
 assign motor_enable=ext_armed?ext_enable:base_motor_enable;
 remote_drive_control #(.CLK_FREQ(CLK_FREQ),.ADDRESS(`T123_ADDRESS),
  .KEY_2(`T123_KEY_2),.KEY_8(`T123_KEY_8),.KEY_4(`T123_KEY_4),.KEY_6(`T123_KEY_6),
  .KEY_5(`T123_KEY_5),.KEY_0(`T123_KEY_0),.KEY_AUTO(`T123_KEY_AUTO)) remote(
  .clk(clk50M),.rst_n(system_n),.configured(`T123_KEYS_CONFIRMED!=0),
  .frame_valid(ir_frame),.repeat_valid(ir_repeat),.invalid_frame(ir_invalid),.activity(ir_activity),
  .address(ir_address),.command(ir_command),.autonomous(autonomous),.blocked(remote_blocked || serial_stop || ir_block),
  .manual_action(ir_action),.auto_start(ir_auto),.stop_request(ir_stop),.released(ir_released));
 bt_gamepad_rx #(.CLK_FREQ(CLK_FREQ)) bluetooth_rx(
  .clk(clk50M),.rst_n(system_n),.rx(bt_rx),.data_valid(bt_valid),
  .connect_event(bt_connect),.disconnect_event(bt_disconnect),.invalid_frame(bt_invalid),
  .lx(bt_lx),.ly(bt_ly),.rx_axis(bt_rx_axis),.ry_axis(bt_ry_axis),.dpad(bt_dpad),
  .buttons2(bt_buttons2),.buttons3(bt_buttons3),.rt(bt_rt),.lt(bt_lt),
  .valid_count(bt_valid_count),.error_count(bt_error_count));
 // Vendor button2 bit3 is X; button3 bit3 is START. Reuse the original
 // neutral/arming/B/lease logic for both launch buttons. START wins a chord.
 wire [7:0] launch_buttons3=bt_buttons3 | (bt_buttons2[3]?8'h08:8'h00);
 reg bt_route_selected;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)bt_route_selected<=0;
  else if(bt_valid)bt_route_selected<=bt_buttons2[3] && !bt_buttons3[3];
 // Task-four-local shoulder mapping: LB left spin, RB right spin; legacy
 // START/X/B handling and source arbitration remain on the same path.
 task4_gamepad_control #(.CLK_FREQ(CLK_FREQ),.TIMEOUT_MS(BT_TIMEOUT_MS),.RELEASE_MS(BT_RELEASE_MS)) bluetooth_control(
  .clk(clk50M),.rst_n(system_n),.data_valid(bt_valid),.connect_event(bt_connect),
  .disconnect_event(bt_disconnect),.invalid_frame(bt_invalid),.lx(bt_lx),.ly(bt_ly),
  .dpad(bt_dpad),.buttons2(bt_buttons2),.buttons3(launch_buttons3),.autonomous(autonomous),
  .blocked(remote_blocked),.cancel(serial_stop || ir_stop || source_changed),
  .online(bt_online),.auto_start(bt_auto),.stop_request(bt_stop),.manual_action(bt_action),.released(bt_released));
 remote_input_arbiter input_select(.clk(clk50M),.rst_n(system_n),.bt_online(bt_online),
  .ir_action(ir_action),.bt_action(bt_action),.ir_auto(ir_auto),.bt_auto(bt_auto),
  .ir_stop(ir_stop),.bt_stop(bt_stop),.ir_released(ir_released),.bt_released(bt_released),
  .manual_action(manual_action),.auto_start(auto_start),.stop_request(remote_stop),
  .released(released),.ir_block(ir_block),.source_changed(source_changed));
 task123_safety #(.CLK_FREQ(CLK_FREQ),.STOP_MM(`T123_STOP_MM),.RELEASE_MM(`T123_RELEASE_MM),.SLOW_MM(`T123_SLOW_MM)) safety(.clk(clk50M),.rst_n(system_n),.imu_valid(imu_valid),
  .distance_valid(distance_valid && tof_strength>=100 && distance_mm!=0),.distance_mm(distance_mm),
  .motor_fault(motor_fault),.motor_reason(motor_reason),.pose_fault(autonomous && state!=2 && pose_fault),.stop_request(stop_request),
  .sensors_ready(raw_sensors_ready),.hard_fault(raw_hard_fault),.obstacle(obstacle),.slow(slow),.reason(raw_safety_reason));
 pose_estimator #(.MM_PER_COUNT_Q16(`T123_MM_Q16),.LEFT_SIGN(`T123_LEFT_SIGN),
  .RIGHT_SIGN(`T123_RIGHT_SIGN),.YAW_SIGN(`T123_YAW_SIGN),.RUNTIME(1)) pose(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.reset_origin(reset_origin),
  .configured_mm_q16(32'd`T123_MM_Q16),.configured_signs(pose_signs),
  .left_count(control_left_count),.right_count(control_right_count),.yaw_cdeg(yaw_cdeg),
  .x_mm(legacy_x),.y_mm(legacy_y),.heading(legacy_heading),.valid(legacy_pose_valid),.fault(legacy_pose_fault));
 task4_pose_estimator #(.LEFT_SIGN(NAV_LEFT_SIGN),.RIGHT_SIGN(NAV_RIGHT_SIGN)) nav_pose(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.reset_origin(reset_origin),.active(route_mode),.imu_valid(imu_valid),
  .left_count(nav_left_count),.right_count(nav_right_count),.yaw_cdeg(yaw_cdeg),
  .x_mm(nav_x),.y_mm(nav_y),.heading(nav_heading),.encoder_heading(encoder_heading),.imu_heading(relative_imu),
  .disagreement(heading_difference),.correction(heading_correction),.valid(nav_pose_valid),.fault(nav_pose_fault),
  .trusted(nav_trusted),.degraded(nav_degraded),.reason(nav_pose_reason),.quality(nav_quality),
  .bad_ticks(heading_bad_ticks),.agree_ticks(heading_agree_ticks),.imu_age(heading_imu_age));
 wire nav_enable,nav_done,nav_failed,recovery_follow;wire [7:0] nav_reason;wire [3:0] waypoint;
 wire signed [31:0] avoid_left,avoid_right;wire avoid_done,avoid_failed,avoid_waiting;
 wire signed [31:0] free_left,free_right,goal_left,goal_right;
 wire free_done,free_failed,free_waiting,goal_done,goal_failed,goal_waiting;
 wire [3:0] free_phase,goal_phase;wire [7:0] free_reason,goal_reason;
 assign avoid_left=route_mode?goal_left:free_left;assign avoid_right=route_mode?goal_right:free_right;
 assign avoid_done=route_mode?goal_done:free_done;assign avoid_failed=route_mode?goal_failed:free_failed;
 assign avoid_waiting=route_mode?goal_waiting:free_waiting;assign avoid_phase=route_mode?goal_phase:free_phase;
 assign avoid_reason=route_mode?goal_reason:free_reason;
 avoidance_controller #(.CRUISE_RPM(`T123_CRUISE_RPM),.NEAR_TURN_RPM(`T123_NEAR_TURN_RPM),
  .CLEAR_MM(`T123_CLEAR_MM),.MARGIN_MM(`T123_MARGIN_MM),.SETTLE_TICKS(`T123_STABLE_TICKS),
  .EMERGENCY_MM(`T123_EMERGENCY_MM),.BACKUP_MM(`T123_BACKUP_MM),.BACKUP_RPM(`T123_BACKUP_RPM),
  .BACKUP_TICKS(`T123_BACKUP_TICKS),.SAFE_SAMPLES(`T123_SAFE_SAMPLES)) avoidance(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.start(avoid_start && !route_mode),.cancel(avoid_cancel || route_mode),
  .clearance_verified(`T123_CLEARANCE_CONFIRMED!=0),.x_mm(legacy_x),.y_mm(legacy_y),.heading(legacy_heading),
  .distance_valid(distance_valid && tof_strength>=100 && distance_mm!=0),.distance_mm(distance_mm),
  .obstacle(obstacle),.left_rpm(free_left),.right_rpm(free_right),
  .done(free_done),.failed(free_failed),.waiting(free_waiting),.phase(free_phase),.reason(free_reason));
 task4_goal_avoidance #(.CRUISE_RPM(`T4_AVOID_FORWARD_RPM),.NEAR_TURN_RPM(`T4_ALIGN_RPM),
  .CLEAR_MM(`T123_CLEAR_MM),.MARGIN_MM(`T123_MARGIN_MM),.SETTLE_TICKS(`T123_STABLE_TICKS),
  .EMERGENCY_MM(`T123_EMERGENCY_MM),.BACKUP_MM(`T123_BACKUP_MM),.BACKUP_RPM(`T123_BACKUP_RPM),
  .BACKUP_TICKS(`T123_BACKUP_TICKS),.SAFE_SAMPLES(`T123_SAFE_SAMPLES)) goal_avoidance(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.start(avoid_start && route_mode),.cancel(avoid_cancel || !route_mode),
  .clearance_verified(`T123_CLEARANCE_CONFIRMED!=0),.x_mm(nav_x),.y_mm(nav_y),.heading(nav_heading),
  .distance_valid(distance_valid && tof_strength>=100 && distance_mm!=0),.distance_mm(distance_mm),
  .waypoint(waypoint),.left_actual(control_left_actual),.right_actual(control_right_actual),.obstacle(obstacle),.left_rpm(goal_left),.right_rpm(goal_right),
  .done(goal_done),.failed(goal_failed),.waiting(goal_waiting),.phase(goal_phase),.reason(goal_reason));
 wire nav_dwelling;wire nav_goal_reached;wire signed [31:0] nav_left,nav_right;
 // Pipeline only follower bookkeeping. Controller STOP/obstacle/fault and
 // physical motor interlocks retain their original immediate gating.
 reg path_enable_q,path_drive_q;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)begin path_enable_q<=0;path_drive_q<=0;end
  else begin path_enable_q<=nav_enable;path_drive_q<=motor_stby;end

 task4_path_follower #(.ROUTE_FILE(ROUTE_FILE),.LEFT_SIGN(NAV_LEFT_SIGN),.RIGHT_SIGN(NAV_RIGHT_SIGN)) path(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.reset_route(reset_origin),.drive_active(path_drive_q),.left_count(control_left_count),.right_count(control_right_count),.enable(path_enable_q),.allow_advance(state==8),.direct_goal(recovery_follow),.slow(slow || state==9 || nav_degraded),.pose_valid(pose_valid),
  .heading_fault(route_mode && nav_pose_fault),.heading_reason(nav_pose_reason),
  .x_mm(x_mm),.y_mm(y_mm),.heading(heading),.left_actual(control_left_actual),.right_actual(control_right_actual),
  .left_rpm(nav_left),.right_rpm(nav_right),.waypoint(waypoint),.done(nav_done),.dwelling(nav_dwelling),.goal_reached(nav_goal_reached),.failed(nav_failed),.reason(nav_reason));
 task4_controller #(.CRUISE_RPM(`T123_CRUISE_RPM),.TURN_RPM(`T123_TURN_RPM)) control(
  .clk(clk50M),.rst_n(system_n),.tick(tick),.configured(`T123_VEHICLE_CONFIRMED!=0),
  .sensors_ready(sensors_ready),.hard_fault(hard_fault),.stop_request(stop_request || ext_armed),
  .auto_start(auto_start && `T123_CLEARANCE_CONFIRMED!=0),.route_start(bt_online && bt_route_selected),.manual_action(manual_action),
  .obstacle(obstacle),.slow(slow),.pose_valid(pose_valid),.avoid_done(avoid_done),.avoid_failed(avoid_failed),
  .released(released),.safety_reason(safety_reason),.avoid_reason(avoid_reason),
  .avoid_left(avoid_left),.avoid_right(avoid_right),.state(state),.reason(reason),
  .heading(heading),.avoid_waiting(avoid_waiting),.distance_valid(distance_valid && tof_strength>=100 && distance_mm!=0),.distance_mm(distance_mm),
  .left_actual(control_left_actual),.right_actual(control_right_actual),.nav_left(nav_left),.nav_right(nav_right),
  .nav_dwelling(nav_dwelling),.nav_goal_reached(nav_goal_reached),.nav_done(nav_done),.nav_failed(nav_failed),.nav_reason(nav_reason),.waypoint(waypoint),
  .nav_enable(nav_enable),.route_mode(route_mode),.recovery_follow(recovery_follow),.x_mm(x_mm),.y_mm(y_mm),
  .autonomous(autonomous),.reset_origin(reset_origin),.avoid_start(avoid_start),.avoid_cancel(avoid_cancel),
  .motor_enable(base_motor_enable),.remote_blocked(remote_blocked),.left_rpm(controller_left_request),.right_rpm(controller_right_request));
 // Break the range -> controller -> assist -> adapter timing chain. Only
 // internal enable waits one 50 MHz cycle; original STOP/obstacle/fault
 // conditions independently mask every physical output immediately.
 reg motor_enable_q;
 wire internal_pwm1,internal_pwm2,internal_ain1,internal_ain2,internal_bin1,internal_bin2,internal_stby;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)motor_enable_q<=0;else motor_enable_q<=motor_enable;
 assign motor_pwm1=internal_pwm1 && motor_enable;
 assign motor_pwm2=internal_pwm2 && motor_enable;
 assign motor_ain1=internal_ain1 && motor_enable;
 assign motor_ain2=internal_ain2 && motor_enable;
 assign motor_bin1=internal_bin1 && motor_enable;
 assign motor_bin2=internal_bin2 && motor_enable;
 assign motor_stby=internal_stby && motor_enable;
 task4_motor_closed_loop #(.CLK_FREQ(CLK_FREQ),.RUNTIME(1),.NAV_MAX_RPM(`T4_ODO_VALIDATION_RPM),.OUTPUT_UNLOCK(OUTPUT_UNLOCK)) motors(
  .clk(clk50M),.rst_n(system_n),.enable(motor_enable_q),.command_tick(tick),.navigation_mode(route_mode),
  .configured_rpm_scale(`T123_RPM_Q16),.configured_max_rpm(16'(`T123_MAX_RPM)),
  .configured_motor_reverse(2'(`T123_MOTOR_REVERSE)),.left_request(left_request),.right_request(right_request),
  .enc1_a(enc1_a),.enc1_b(enc1_b),.enc2_a(enc2_a),.enc2_b(enc2_b),
  .motor_pwm1(internal_pwm1),.motor_ain1(internal_ain1),.motor_ain2(internal_ain2),
  .motor_pwm2(internal_pwm2),.motor_bin1(internal_bin1),.motor_bin2(internal_bin2),.motor_stby(internal_stby),
  .left_count(left_count),.right_count(right_count),.left_actual_rpm(left_actual),.right_actual_rpm(right_actual),
  .nav_left_count(nav_left_count),.nav_right_count(nav_right_count),.nav_left_actual(nav_left_actual),.nav_right_actual(nav_right_actual),
  .left_rejected(left_rejected),.right_rejected(right_rejected),.left_invalid(left_invalid),.right_invalid(right_invalid),
  .left_duty(left_duty),.right_duty(right_duty),.fault(motor_fault),.fault_reason(motor_reason));
 wire status_tx,status_busy,ir_tx,ir_busy;
 task123_telemetry #(.CLK_FREQ(CLK_FREQ)) status_uart(.clk(clk50M),.rst_n(system_n),.trigger(legacy_done),
  .nav_state(state),.fault_code(hard_fault?safety_reason:reason),.waypoint({(route_mode?waypoint:4'd0),avoid_phase}),
  .flags({remote_blocked,motor_fault,autonomous,released,motor_stby,obstacle,pose_valid,sensors_ready}),
  .last_rx(rx_sequence),.rx_errors(rx_errors),.distance_mm(distance_mm),.x_mm(x_mm),.y_mm(y_mm),.heading(heading),
  .left_count(left_count),.right_count(right_count),.left_actual(left_actual),.right_actual(right_actual),
  .left_target(left_request),.right_target(right_request),.left_duty(left_duty),.right_duty(right_duty),
  .tx(status_tx),.busy(status_busy));
 reg status_was_busy;
 always @(posedge clk50M or negedge system_n)if(!system_n)status_was_busy<=0;else status_was_busy<=status_busy;
 nav_ir_telemetry_tx #(.CLK_FREQ(CLK_FREQ)) key_uart(.clk(clk50M),.rst_n(system_n),
  .trigger(status_was_busy && !status_busy),.address(ir_address),.valid_count(ir_count),.error_count(ir_errors),
  .command(ir_command),.flags(8'(`T123_KEYS_CONFIRMED)),.tx(ir_tx),.busy(ir_busy));
 wire diagnostic_tx,diagnostic_busy;
 reg ir_was_busy;
 always @(posedge clk50M or negedge system_n)if(!system_n)ir_was_busy<=0;else ir_was_busy<=ir_busy;
 task4_encoder_diagnostics #(.CLK_FREQ(CLK_FREQ)) encoder_diagnostics(
  .clk(clk50M),.rst_n(system_n),.snapshot(legacy_done),.trigger(ir_was_busy && !ir_busy),.active(route_mode),
  .state(state),.flags({remote_blocked,motor_fault,autonomous,released,motor_stby,obstacle,pose_valid,sensors_ready}),
  .left_count(nav_left_count),.right_count(nav_right_count),.left_actual(nav_left_actual),.right_actual(nav_right_actual),
  .raw_left_count(left_count),.raw_right_count(right_count),
  .left_rejected(left_rejected),.right_rejected(right_rejected),.left_invalid(left_invalid),.right_invalid(right_invalid),
  .tx(diagnostic_tx),.busy(diagnostic_busy));
 // Idle/manual diagnosis remains available without pressing X. Standalone
 // task-three suppresses new IMU packets; an in-flight packet finishes.
 wire imu_diagnostic_tx,imu_diagnostic_busy;
 reg diagnostic_was_busy;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)diagnostic_was_busy<=0;else diagnostic_was_busy<=diagnostic_busy;
 wire imu_diagnostic_trigger=(diagnostic_was_busy && !diagnostic_busy) ||
                             (ir_was_busy && !ir_busy && !route_mode);
 task4_imu_diagnostics #(.CLK_FREQ(CLK_FREQ)) imu_diagnostics(
  .clk(clk50M),.rst_n(system_n),.imu_rx(imu_uart_rx),.snapshot(legacy_done),
  .trigger(imu_diagnostic_trigger),.active(!autonomous || route_mode),
  .route_mode(route_mode),.pose_valid(pose_valid),.state(state),
  .filtered_yaw(yaw_cdeg),.heading(heading),.filtered_valid(imu_valid),
  .tx(imu_diagnostic_tx),.busy(imu_diagnostic_busy));
 wire heading_diagnostic_tx,heading_diagnostic_busy;
 reg imu_diagnostic_was_busy;
 always @(posedge clk50M or negedge system_n)
  if(!system_n)imu_diagnostic_was_busy<=0;else imu_diagnostic_was_busy<=imu_diagnostic_busy;
 task4_heading_diagnostics #(.CLK_FREQ(CLK_FREQ)) heading_diagnostics(
  .clk(clk50M),.rst_n(system_n),.snapshot(legacy_done),.trigger(imu_diagnostic_was_busy && !imu_diagnostic_busy),
  .active(!autonomous || route_mode),.quality(nav_quality),.reason(nav_pose_reason),
  .encoder_heading(encoder_heading),.imu_heading(relative_imu),.heading(nav_heading),
  .disagreement(heading_difference),.correction(heading_correction),.imu_age(heading_imu_age),
  .left_count(nav_left_count),.right_count(nav_right_count),.x_mm(nav_x),.y_mm(nav_y),
  .bad_ticks(heading_bad_ticks),.agree_ticks(heading_agree_ticks),.rpm_limit(route_rpm_limit[7:0]),
  .tx(heading_diagnostic_tx),.busy(heading_diagnostic_busy));
 wire ext_tx,ext_busy;
 reg [15:0] tof_seq,imu_seq;reg [31:0] uptime_ms;reg [31:0] millisecond_count;
 always @(posedge clk50M or negedge system_n)begin
  if(!system_n)begin tof_seq<=0;imu_seq<=0;uptime_ms<=0;millisecond_count<=0;end
  else begin
   if(distance_valid)tof_seq<=tof_seq+1'b1;
   if(imu_valid)imu_seq<=imu_seq+1'b1;
   if(millisecond_count==CLK_FREQ/1000-1)begin millisecond_count<=0;uptime_ms<=uptime_ms+1'b1;end
   else millisecond_count<=millisecond_count+1'b1;
  end
 end
 reg ext_pending;reg [3:0] diagnostic_quiet;
 wire diagnostics_busy=status_busy || ir_busy || diagnostic_busy || imu_diagnostic_busy || heading_diagnostic_busy;
 wire ext_trigger=ext_pending && diagnostic_quiet==8 && !diagnostics_busy && !ext_busy;
 always @(posedge clk50M or negedge system_n)begin
  if(!system_n)begin ext_pending<=0;diagnostic_quiet<=0;end
  else begin
   if(legacy_done)begin ext_pending<=1;diagnostic_quiet<=0;end
   else if(diagnostics_busy)diagnostic_quiet<=0;
   else if(ext_pending && diagnostic_quiet<8)diagnostic_quiet<=diagnostic_quiet+1'b1;
   if(ext_trigger)ext_pending<=0;
  end
 end
 // External feedback is continuous qualified wheel counts, forward-positive on both sides.
 // Raw encoder evidence remains in the inherited diagnostic packets.
 wire signed [31:0] ext_left_count=NAV_LEFT_SIGN<0?-nav_left_count:nav_left_count;
 wire signed [31:0] ext_right_count=NAV_RIGHT_SIGN<0?-nav_right_count:nav_right_count;
 wire signed [31:0] ext_left_actual=NAV_LEFT_SIGN<0?-nav_left_actual:nav_left_actual;
 wire signed [31:0] ext_right_actual=NAV_RIGHT_SIGN<0?-nav_right_actual:nav_right_actual;
 task6_telemetry #(.CLK_FREQ(CLK_FREQ)) external_status(.clk(clk50M),.rst_n(system_n),
  .trigger(ext_trigger),.challenge(ext_challenge),.session(ext_session),.uptime_ms(uptime_ms),
  .accepted_sequence(ext_accepted),.distance_mm(distance_mm),.tof_sequence(tof_seq),.imu_sequence(imu_seq),
  .state(ext_armed?8'd1:8'd0),.reason(ext_reason),
  .flags({1'b0,ext_enable,released,motor_fault,obstacle,raw_sensors_ready,raw_sensors_ready,raw_sensors_ready}),
  .yaw(yaw_cdeg),.left_rpm(ext_left_actual[15:0]),.right_rpm(ext_right_actual[15:0]),
  .left_count(ext_left_count),.right_count(ext_right_count),.tx(ext_tx),.busy(ext_busy));
 assign sensor_uart_tx=legacy_tx & status_tx & ir_tx & diagnostic_tx & imu_diagnostic_tx & heading_diagnostic_tx & ext_tx;
 assign led={motor_fault,autonomous,motor_stby,sensors_ready,obstacle,phy_link,initialized,system_n,1'b0};
endmodule
