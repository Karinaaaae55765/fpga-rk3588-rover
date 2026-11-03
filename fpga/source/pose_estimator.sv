`default_nettype none
module pose_estimator #(
 parameter integer MM_PER_COUNT_Q16=9042,LEFT_SIGN=1,RIGHT_SIGN=1,YAW_SIGN=1,RUNTIME=0,
 parameter integer MAX_DELTA=200,POSITION_LIMIT_MM=30000
)(
 input wire clk,rst_n,tick,reset_origin,
 input wire [31:0] configured_mm_q16,
 input wire [2:0] configured_signs,
 input wire signed [31:0] left_count,right_count,
 input wire signed [15:0] yaw_cdeg,
 output reg signed [31:0] x_mm,y_mm,
 output reg signed [15:0] heading,
 output reg valid,fault
);
 reg signed [31:0] last_l,last_r,x_q16,y_q16,ds_q16;
 reg signed [15:0] origin_yaw;
 reg origin_seen;
 reg [2:0] phase;
 reg math_start;
 wire math_done,math_busy;
 wire signed [31:0] cosv,sinv;
 wire signed [31:0] raw_dl=left_count-last_l,raw_dr=right_count-last_r;
 wire signed [31:0] raw_yaw=$signed(yaw_cdeg)-$signed(origin_yaw);
 wire signed [31:0] dl=RUNTIME?(configured_signs[0]?-raw_dl:raw_dl):raw_dl*LEFT_SIGN;
 wire signed [31:0] dr=RUNTIME?(configured_signs[1]?-raw_dr:raw_dr):raw_dr*RIGHT_SIGN;
 wire signed [31:0] relative_yaw=RUNTIME?(configured_signs[2]?-raw_yaw:raw_yaw):raw_yaw*YAW_SIGN;
 wire signed [31:0] mm_scale=RUNTIME?$signed(configured_mm_q16):MM_PER_COUNT_Q16;
 wire signed [15:0] norm_yaw=(relative_yaw>18000)?relative_yaw-36000:
                            (relative_yaw< -18000)?relative_yaw+36000:relative_yaw;
 reg signed [63:0] prod_x,prod_y,sum_x,sum_y;
 nav_math trig(.clk(clk),.rst_n(rst_n),.clear(reset_origin),.start(math_start),.mode(1'b0),
  .x_in(32'sd0),.y_in(32'sd0),.angle_in(heading),.busy(math_busy),.done(math_done),
  .bearing(),.cos_q15(cosv),.sin_q15(sinv));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   last_l<=0;last_r<=0;x_q16<=0;y_q16<=0;x_mm<=0;y_mm<=0;heading<=0;
   ds_q16<=0;origin_yaw<=0;origin_seen<=0;phase<=0;math_start<=0;
   valid<=0;fault<=0;prod_x<=0;prod_y<=0;sum_x<=0;sum_y<=0;
  end else if(reset_origin)begin
   last_l<=left_count;last_r<=right_count;origin_yaw<=yaw_cdeg;
   x_q16<=0;y_q16<=0;x_mm<=0;y_mm<=0;heading<=0;
   origin_seen<=1;valid<=0;fault<=0;phase<=0;math_start<=0;
  end else begin
   math_start<=0;
   if(origin_seen && !fault)case(phase)
    0: if(tick)begin
     last_l<=left_count;last_r<=right_count;heading<=norm_yaw;
     if(dl>MAX_DELTA || dl< -MAX_DELTA || dr>MAX_DELTA || dr< -MAX_DELTA)
      begin fault<=1;valid<=0;end
     else begin ds_q16<=((dl+dr)*mm_scale)>>>1;phase<=1;end
    end
    1:begin math_start<=1;phase<=2;end
    2:if(math_done)begin prod_x<=ds_q16*cosv;prod_y<=ds_q16*sinv;phase<=3;end
    3:begin sum_x<=$signed(x_q16)+(prod_x>>>15);sum_y<=$signed(y_q16)+(prod_y>>>15);phase<=4;end
    4:begin
     if(sum_x>POSITION_LIMIT_MM*65536 || sum_x< -POSITION_LIMIT_MM*65536 ||
        sum_y>POSITION_LIMIT_MM*65536 || sum_y< -POSITION_LIMIT_MM*65536)
      begin fault<=1;valid<=0;end
     else begin x_q16<=sum_x;y_q16<=sum_y;x_mm<=sum_x>>>16;y_mm<=sum_y>>>16;valid<=1;end
     phase<=0;
    end
    default:begin fault<=1;valid<=0;phase<=0;end
   endcase
  end
 end
endmodule
`default_nettype wire
