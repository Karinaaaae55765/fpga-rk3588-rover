`default_nettype none
// Reactive front-range avoidance: brake, turn one way, stop/recheck, forward.
// Up to 20 degrees per segment; fresh clear range stops rotation early.
// No target-heading alignment, recenter, or waypoint leg.
module avoidance_controller #(
 parameter integer CRUISE_RPM=60,NEAR_TURN_RPM=20,CLEAR_MM=500,MARGIN_MM=50,
 parameter integer SETTLE_TICKS=10,SAFE_SAMPLES=3,
 parameter integer EMERGENCY_MM=100,BACKUP_MM=100,BACKUP_RPM=15,BACKUP_TICKS=200
)(
 input wire clk,rst_n,tick,start,cancel,clearance_verified,
 input wire signed [31:0] x_mm,y_mm,input wire signed [15:0] heading,
 input wire distance_valid,input wire [15:0] distance_mm,input wire obstacle,
 output wire signed [31:0] left_rpm,right_rpm,
 output wire done,failed,waiting,output wire [3:0] phase,output reg [7:0] reason
);
 localparam IDLE=0,SEARCH=1,VERIFY=2,BACKUP=4,FORWARD=7,WAITING=8,DONE=13,FAULT=14,BRAKE=15;
 localparam integer BRAKE_TICKS=15,RETRY_TICKS=100,PROBE_TICKS=25;
 localparam integer SEARCH_STEP_CDEG=2000; // Commanded segment; physical coasting may exceed it.
 localparam integer ROTATE_CLEAR_MM=EMERGENCY_MM+MARGIN_MM;
 localparam integer TURN_SPEED=NEAR_TURN_RPM<20?NEAR_TURN_RPM:20;
 localparam integer PROBE_SPEED=CRUISE_RPM<TURN_SPEED?CRUISE_RPM:TURN_SPEED;
 reg [3:0] state;
 reg [31:0] timer,range_age,scan_travel,wrong_travel,progress_age;
 reg [7:0] samples,stable;
 reg [15:0] latest_range;
 reg signed [15:0] previous_heading,progress_heading;
 reg signed [31:0] backup_x,backup_y;
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
 wire [63:0] reverse_distance2=$signed(dx)*$signed(dx)+$signed(dy)*$signed(dy);
 wire [15:0] front=distance_valid?distance_mm:latest_range;
 wire critical=front!=0 && front<EMERGENCY_MM;
 wire fresh_clear=distance_valid && distance_mm>=CLEAR_MM && !obstacle;
 wire active=state!=IDLE && state!=FAULT;
 wire guard=cancel || !active || range_age>=25;
 wire advancing=(state==FORWARD || state==DONE) && !obstacle && !critical;
 assign left_rpm=guard?0:state==BACKUP?-BACKUP_RPM:state==SEARCH?-TURN_SPEED:advancing?PROBE_SPEED:0;
 assign right_rpm=guard?0:state==BACKUP?-BACKUP_RPM:state==SEARCH?TURN_SPEED:advancing?PROBE_SPEED:0;
 assign done=state==DONE && !guard && !obstacle && !critical;
 assign failed=state==FAULT;
 assign waiting=state==WAITING;
 assign phase=state==BRAKE?4'd0:state;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;reason<=0;timer<=0;range_age<=25;latest_range<=0;
   samples<=0;stable<=0;scan_travel<=0;wrong_travel<=0;progress_age<=0;
   previous_heading<=0;progress_heading<=0;backup_x<=0;backup_y<=0;backup_used<=0;segment_heading<=0;
  end else begin
   if(distance_valid && distance_mm!=0)begin latest_range<=distance_mm;range_age<=0;end
   else if(tick && range_age<25)range_age<=range_age+1'b1;
   if(tick)previous_heading<=heading;
   if(cancel)begin state<=IDLE;reason<=0;timer<=0;samples<=0;end
   else if(start)begin
    state<=clearance_verified?BRAKE:FAULT;reason<=clearance_verified?0:8'h24;
    timer<=0;range_age<=0;samples<=0;stable<=0;scan_travel<=0;
    wrong_travel<=0;progress_age<=0;progress_heading<=heading;
    previous_heading<=heading;backup_used<=0;segment_heading<=0;
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
      if(state==VERIFY && yaw_motion<=80)begin
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
       else if(fresh_clear || segment_turn>=SEARCH_STEP_CDEG)begin
        state<=VERIFY;timer<=0;samples<=0;stable<=0;
       end else if(scan_travel>=36000)begin
        state<=WAITING;reason<=8'h20;timer<=0;samples<=0;
       end
      end
      VERIFY:begin
       if(critical)begin state<=BRAKE;timer<=0;samples<=0;end
       else if(timer>=100)begin state<=FAULT;reason<=8'h22;end
       // Ignore coasting reports. Three NEW reports at the stopped heading
       // authorize forward. A bad report cancels the candidate, not autonomy.
       else if(timer<BRAKE_TICKS || stable<SETTLE_TICKS)samples<=0;
       else if(distance_valid)begin
        if(fresh_clear)begin
         if(samples>=SAFE_SAMPLES-1)begin state<=FORWARD;timer<=0;samples<=0;end
         else samples<=samples+1'b1;
        end else begin
         timer<=0;samples<=0;
         if(scan_travel>=36000)begin state<=WAITING;reason<=8'h20;end
         else begin state<=SEARCH;segment_heading<=heading;end
        end
       end
      end
      FORWARD:begin
       if(obstacle || critical)begin state<=BRAKE;timer<=0;scan_travel<=0;end
       else if(timer>=PROBE_TICKS)state<=DONE;
      end
      DONE:if(obstacle || critical)begin state<=BRAKE;timer<=0;scan_travel<=0;end
      BACKUP:if(reverse_distance2>=BACKUP_MM*BACKUP_MM || timer>=BACKUP_TICKS)begin
       state<=WAITING;reason<=timer>=BACKUP_TICKS?8'h29:8'h2b;timer<=0;samples<=0;
      end
      WAITING:begin
       if(timer<(reason==8'h20 ? RETRY_TICKS : BRAKE_TICKS))samples<=0;
       else if(distance_valid)begin
        if(distance_mm>=ROTATE_CLEAR_MM)begin
         if(samples>=SAFE_SAMPLES-1)begin
          state<=fresh_clear?VERIFY:SEARCH;timer<=0;samples<=0;stable<=0;
          scan_travel<=0;reason<=0;segment_heading<=heading;
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
