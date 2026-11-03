`default_nettype none
module task123_safety #(parameter integer CLK_FREQ=50000000,TIMEOUT_MS=500,STOP_MM=200,RELEASE_MM=300,SLOW_MM=500)(
 input wire clk,rst_n,imu_valid,distance_valid,
 input wire [15:0] distance_mm,
 input wire motor_fault,pose_fault,stop_request,
 input wire [7:0] motor_reason,
 output wire sensors_ready,hard_fault,obstacle,slow,
 output wire [7:0] reason
);
 localparam integer LIMIT=CLK_FREQ/1000*TIMEOUT_MS;
 reg [31:0] imu_age,range_age;
 reg imu_seen,range_seen,blocked;
 reg [15:0] distance;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin imu_age<=LIMIT;range_age<=LIMIT;imu_seen<=0;range_seen<=0;blocked<=1;distance<=0;end
  else begin
   if(imu_valid)begin imu_age<=0;imu_seen<=1;end else if(imu_age<LIMIT)imu_age<=imu_age+1'b1;
   if(distance_valid)begin
    range_age<=0;range_seen<=1;distance<=distance_mm;
    if(distance_mm<STOP_MM)blocked<=1;
    else if(distance_mm>=RELEASE_MM)blocked<=0;
   end else if(range_age<LIMIT)range_age<=range_age+1'b1;
  end
 end
 assign sensors_ready=imu_seen && range_seen && imu_age<LIMIT && range_age<LIMIT;
 assign obstacle=blocked || (distance_valid && distance_mm<STOP_MM);
 assign slow=distance<SLOW_MM || (distance_valid && distance_mm<SLOW_MM);
 assign hard_fault=!sensors_ready || motor_fault || pose_fault;
 assign reason=stop_request?8'd1:motor_fault?motor_reason:pose_fault?8'h12:
               (!imu_seen || imu_age>=LIMIT)?8'h13:(!range_seen || range_age>=LIMIT)?8'h14:8'd0;
endmodule
`default_nettype wire
