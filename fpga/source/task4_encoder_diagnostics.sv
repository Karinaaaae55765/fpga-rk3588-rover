`default_nettype none
// 0x29 / 41-byte payload, CRC16-CCITT over version/type/length/payload.
// Snapshot coincides with 0x28; transmit only after the existing 0x24 finishes.
module task4_encoder_diagnostics #(parameter integer CLK_FREQ=50000000)(
 input wire clk,rst_n,snapshot,trigger,active,
 input wire [3:0] state,input wire [7:0] flags,
 input wire signed [31:0] left_count,right_count,left_actual,right_actual,raw_left_count,raw_right_count,
 input wire [31:0] left_rejected,right_rejected,left_invalid,right_invalid,
 output wire tx,output wire busy
);
 reg [1:0] phase;
 reg [5:0] index;
 reg [15:0] sequence_number,crc;
 reg [327:0] saved,payload;
 reg pending;
 reg [7:0] data;
 wire ready,done;
 assign busy=phase!=0;
 always @*begin
  case(index)
   0:data=8'ha5;1:data=8'h5a;2:data=1;3:data=8'h29;4:data=41;
   46:data=crc[7:0];47:data=crc[15:8];default:data=payload[7:0];
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
    if(active)saved<={8'd5,raw_right_count,raw_left_count,right_invalid,left_invalid,right_rejected,left_rejected,
      right_actual[15:0],left_actual[15:0],right_count,left_count,4'd0,state,flags,sequence_number};
   end
   case(phase)
    0:if(trigger && pending && active)begin payload<=saved;pending<=0;phase<=1;index<=0;crc<=16'hffff;end
    1:if(ready)begin
     if(index>=2 && index<=45)crc<=crc_byte(crc,data);
     if(index>=5 && index<=45)payload<=payload>>8;
     phase<=2;
    end
    2:if(done)begin if(index==47)phase<=0;else begin index<=index+1'b1;phase<=1;end end
    default:phase<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
