`default_nettype none
// Read-only observer. No output from this module enters motion/sensor control.
// The two decoders mirror perception_hub; their observations are not a new
// selected input to that hub. Counters are cumulative and saturating.
module task4_imu_diagnostics #(parameter integer CLK_FREQ=50000000)(
 input wire clk,rst_n,imu_rx,snapshot,trigger,active,route_mode,pose_valid,
 input wire [3:0] state,
 input wire signed [15:0] filtered_yaw,heading,
 input wire filtered_valid,
 output wire tx,busy
);
 wire [7:0] byte_a,byte_b;
 wire done_a,done_b,uart_error_a,uart_error_b,valid_a,valid_b,checksum_a,checksum_b;
 wire signed [15:0] raw_a,raw_b,angle_a,angle_b;
 uart_rx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(9600)) rx_a(
  .clk(clk),.rst_n(rst_n),.rx(imu_rx),.data(byte_a),.done(done_a),.frame_error(uart_error_a));
 uart_rx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(115200)) rx_b(
  .clk(clk),.rst_n(rst_n),.rx(imu_rx),.data(byte_b),.done(done_b),.frame_error(uart_error_b));
 wt901_parser parser_a(.clk(clk),.rst_n(rst_n),.rx_data(byte_a),.rx_valid(done_a),
  .roll_raw(),.pitch_raw(),.yaw_raw(raw_a),.roll_cdeg(),.pitch_cdeg(),.yaw_cdeg(angle_a),
  .angle_valid(valid_a),.checksum_error(checksum_a));
 wt901_parser parser_b(.clk(clk),.rst_n(rst_n),.rx_data(byte_b),.rx_valid(done_b),
  .roll_raw(),.pitch_raw(),.yaw_raw(raw_b),.roll_cdeg(),.pitch_cdeg(),.yaw_cdeg(angle_b),
  .angle_valid(valid_b),.checksum_error(checksum_b));
 reg [31:0] count_a,count_b,filter_count;
 reg [15:0] errors_a,errors_b,age_a,age_b,switches,simultaneous;
 reg seen_a,seen_b,last_alt;
 reg signed [15:0] last_selected;
 localparam integer MS_CLKS=CLK_FREQ/1000;
 reg [$clog2(MS_CLKS+1)-1:0] ms_counter;
 wire ms_tick=ms_counter==MS_CLKS-1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   count_a<=0;count_b<=0;filter_count<=0;errors_a<=0;errors_b<=0;
   age_a<=65535;age_b<=65535;switches<=0;simultaneous<=0;
   seen_a<=0;seen_b<=0;last_alt<=0;last_selected<=0;ms_counter<=0;
  end else begin
   if(ms_tick)ms_counter<=0;else ms_counter<=ms_counter+1'b1;
   if(valid_a)begin seen_a<=1;age_a<=0;if(count_a!=32'hffffffff)count_a<=count_a+1'b1;end
   else if(ms_tick && age_a!=65535)age_a<=age_a+1'b1;
   if(valid_b)begin seen_b<=1;age_b<=0;if(count_b!=32'hffffffff)count_b<=count_b+1'b1;end
   else if(ms_tick && age_b!=65535)age_b<=age_b+1'b1;
   if((uart_error_a || checksum_a) && errors_a!=65535)errors_a<=errors_a+1'b1;
   if((uart_error_b || checksum_b) && errors_b!=65535)errors_b<=errors_b+1'b1;
   if(filtered_valid && filter_count!=32'hffffffff)filter_count<=filter_count+1'b1;
   if(valid_a || valid_b)begin
    last_selected<=valid_a?angle_a:angle_b;
    last_alt<=!valid_a;
    if((seen_a || seen_b) && last_alt!=(!valid_a) && switches!=65535)switches<=switches+1'b1;
   end
   if(valid_a && valid_b && simultaneous!=65535)simultaneous<=simultaneous+1'b1;
  end
 end
 // Same snapshot and sequence as 0x28; original 0x10 is sampled earlier.
 // Payload 42 bytes, total 49, CRC covers bytes 2..46.
 wire [7:0] flags={(age_b<500),(age_a<500),route_mode,pose_valid,
                   (simultaneous!=0),last_alt,seen_b,seen_a};
 reg [335:0] saved,payload;
 reg [15:0] sequence_number,crc;
 reg pending;
 reg [1:0] phase;
 reg [5:0] index;
 reg [7:0] data;
 wire ready,done;
 assign busy=phase!=0;
 always @*begin
  case(index)
   0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h2a;4:data=42;
   47:data=crc[7:0];48:data=crc[15:8];default:data=payload[7:0];
  endcase
 end
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;
  begin v=c^{b,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 uart_tx_v3 #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(115200)) uart(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(phase==1),.ready(ready),.tx(tx),.done(done));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   saved<=0;payload<=0;sequence_number<=0;crc<=65535;pending<=0;phase<=0;index<=0;
  end else begin
   if(snapshot)begin
    sequence_number<=sequence_number+1'b1;pending<=active;
    if(active)saved<={last_selected,filter_count,simultaneous,switches,age_b,age_a,
      errors_b,errors_a,count_b,count_a,heading,filtered_yaw,angle_b,raw_b,angle_a,raw_a,
      4'd0,state,flags,sequence_number};
   end
   case(phase)
    0:if(trigger && pending && active)begin payload<=saved;pending<=0;phase<=1;index<=0;crc<=65535;end
    1:if(ready)begin
     if(index>=2 && index<=46)crc<=crc_byte(crc,data);
     if(index>=5 && index<=46)payload<=payload>>8;
     phase<=2;
    end
    2:if(done)begin if(index==48)phase<=0;else begin index<=index+1'b1;phase<=1;end end
    default:phase<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
