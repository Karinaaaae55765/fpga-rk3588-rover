`default_nettype none
// A5 5A 01 36 27 payload39 CRC16le; CRC covers version through payload.
module task6_telemetry #(parameter integer CLK_FREQ=50000000,BAUD_RATE=115200)(
 input wire clk,rst_n,trigger,input wire [31:0] challenge,session,uptime_ms,
 input wire [15:0] accepted_sequence,distance_mm,tof_sequence,imu_sequence,
 input wire [7:0] state,reason,flags,input wire signed [15:0] yaw,left_rpm,right_rpm,
 input wire signed [31:0] left_count,right_count,output wire tx,busy
);
 reg [1:0] phase;reg [5:0] index;reg [15:0] sequence_number,crc;reg [311:0] payload;
 reg [7:0] data;wire ready,done;
 assign busy=phase!=0;
 always @*case(index)
  0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h36;4:data=39;
  44:data=crc[7:0];45:data=crc[15:8];default:data=payload[7:0];
 endcase
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;begin v=c^{b,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 uart_tx_v3 #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) uart(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(phase==1),.ready(ready),.tx(tx),.done(done));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;index<=0;sequence_number<=0;crc<=16'hffff;payload<=0;end
  else case(phase)
   0:if(trigger)begin
    payload<={uptime_ms,imu_sequence,tof_sequence,right_rpm,left_rpm,right_count,left_count,yaw,distance_mm,
               flags,reason,state,accepted_sequence,session,challenge,sequence_number};
    sequence_number<=sequence_number+1'b1;index<=0;crc<=16'hffff;phase<=1;
   end
   1:if(ready)begin
    if(index>=2 && index<=43)crc<=crc_byte(crc,data);
    if(index>=5 && index<=43)payload<=payload>>8;
    phase<=2;
   end
   2:if(done)begin if(index==45)phase<=0;else begin index<=index+1'b1;phase<=1;end end
   default:phase<=0;
  endcase
 end
endmodule
`default_nettype wire
