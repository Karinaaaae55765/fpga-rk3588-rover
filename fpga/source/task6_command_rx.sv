`default_nettype none
// 20 bytes: 54 36 01 CMD session32 seq16 left16 right16 challenge32 CRC16le.
module task6_command_rx #(parameter integer CLK_FREQ=50000000,BAUD_RATE=115200)(
 input wire clk,rst_n,rx,
 output reg valid,invalid,
 output reg [7:0] command,
 output reg [31:0] session,challenge,
 output reg [15:0] sequence_number,
 output reg signed [15:0] left_rpm,right_rpm
);
 wire [7:0] data;wire done,frame_error;
 uart_rx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) uart(
  .clk(clk),.rst_n(rst_n),.rx(rx),.data(data),.done(done),.frame_error(frame_error));
 reg [4:0] index;reg [31:0] gap;reg [15:0] crc;reg [7:0] low_crc;
 reg [143:0] bytes;
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;begin v=c^{b,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin index<=0;gap<=0;crc<=16'hffff;low_crc<=0;bytes<=0;valid<=0;invalid<=0;
   command<=0;session<=0;challenge<=0;sequence_number<=0;left_rpm<=0;right_rpm<=0;end
  else begin
   valid<=0;invalid<=0;
   if(index!=0)gap<=gap+1'b1;else gap<=0;
   if(frame_error || (index!=0 && gap>=CLK_FREQ/1000*20))begin index<=0;gap<=0;invalid<=1;end
   else if(done)begin
    gap<=0;
    if(index==0)begin if(data==8'h54)begin index<=1;bytes[7:0]<=data;crc<=crc_byte(16'hffff,data);end end
    else if(index==1)begin
     if(data==8'h36)begin index<=2;bytes[15:8]<=data;crc<=crc_byte(crc,data);end
     else if(data==8'h54)begin bytes[7:0]<=data;crc<=crc_byte(16'hffff,data);end
     else index<=0;
    end else if(index<18)begin
     bytes[index*8 +: 8]<=data;crc<=crc_byte(crc,data);index<=index+1'b1;
    end else if(index==18)begin low_crc<=data;index<=19;end
    else begin
     index<=0;
     if({data,low_crc}!=crc || bytes[23:16]!=1 || bytes[31:24]<1 || bytes[31:24]>5)invalid<=1;
     else begin
      valid<=1;command<=bytes[31:24];session<=bytes[63:32];sequence_number<=bytes[79:64];
      left_rpm<=bytes[95:80];right_rpm<=bytes[111:96];challenge<=bytes[143:112];
     end
    end
   end
  end
 end
endmodule
`default_nettype wire
