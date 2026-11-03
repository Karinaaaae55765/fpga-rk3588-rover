`default_nettype none
// Read-only, atomic 0x2B snapshot. Serialized after 0x2A, no control outputs.
module task4_heading_diagnostics #(parameter integer CLK_FREQ=50000000)(
 input wire clk,rst_n,snapshot,trigger,active,
 input wire [7:0] quality,reason,rpm_limit,
 input wire signed [15:0] encoder_heading,imu_heading,heading,disagreement,correction,
 input wire [15:0] imu_age,bad_ticks,agree_ticks,
 input wire signed [31:0] left_count,right_count,x_mm,y_mm,
 output wire tx,busy
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

 reg [1:0] phase;reg [5:0] index;reg [15:0] sequence_number,crc;
 reg [383:0] saved,payload;reg pending;reg [7:0] data;wire ready,done;
 assign busy=phase!=0;
 always @*begin
  case(index)
   0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h2b;4:data=48;
   53:data=crc[7:0];54:data=crc[15:8];default:data=payload[7:0];
  endcase
 end
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;
  begin v=c^{b,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 uart_tx_v3 #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(115200)) uart(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(phase==1),.ready(ready),.tx(tx),.done(done));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;index<=0;sequence_number<=0;crc<=16'hffff;saved<=0;payload<=0;pending<=0;end
  else begin
   if(snapshot)begin
    sequence_number<=sequence_number+1'b1;pending<=active;
    if(active)saved<={8'd1,rpm_limit,32'd`T4_ODO_RIGHT_MM_Q16,32'd`T4_ODO_LEFT_MM_Q16,16'd`T4_TRACK_MM,
      agree_ticks,bad_ticks,y_mm,x_mm,right_count,left_count,imu_age,correction,disagreement,
      heading,imu_heading,encoder_heading,reason,quality,sequence_number};
   end
   case(phase)
    0:if(trigger && pending && active)begin payload<=saved;pending<=0;phase<=1;index<=0;crc<=16'hffff;end
    1:if(ready)begin
     if(index>=2 && index<=52)crc<=crc_byte(crc,data);
     if(index>=5 && index<=52)payload<=payload>>8;
     phase<=2;
    end
    2:if(done)begin if(index==54)phase<=0;else begin index<=index+1'b1;phase<=1;end end
    default:phase<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
