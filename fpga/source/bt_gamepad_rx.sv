`default_nettype none
// SBTCOM1 X-Protocol, manual V1.0 pp.10-11. Only complete checked
// gamepad/status frames publish data; unrelated traffic cannot renew control.
module bt_gamepad_rx #(
 parameter integer CLK_FREQ=50000000, BAUD=115200, FRAME_GAP_MS=5
)(
 input wire clk,rst_n,rx,
 output reg data_valid,connect_event,disconnect_event,invalid_frame,
 output reg signed [7:0] lx,ly,rx_axis,ry_axis,
 output reg [7:0] dpad,buttons2,buttons3,rt,lt,
 output reg [15:0] valid_count,error_count
);
 localparam integer DIV=(CLK_FREQ+BAUD/2)/BAUD;
 localparam integer GAP=CLK_FREQ/1000*FRAME_GAP_MS;
 (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [1:0] sync_rx;
 reg [2:0] uart_state,bit_index;
 reg [31:0] baud_count,gap_count;
 reg [7:0] shift,byte_data;
 reg byte_valid,byte_error;
 reg [3:0] pos;
 reg [7:0] length,sum,packet[0:13];
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)sync_rx<=2'b11;
  else sync_rx<={sync_rx[0],rx};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   uart_state<=0;bit_index<=0;baud_count<=0;shift<=0;
   byte_data<=0;byte_valid<=0;byte_error<=0;
  end else begin
   byte_valid<=0;byte_error<=0;
   case(uart_state)
    0:if(!sync_rx[1])begin baud_count<=DIV/2-1;uart_state<=1;end
    1:if(baud_count!=0)baud_count<=baud_count-1'b1;
      else if(!sync_rx[1])begin baud_count<=DIV-1;bit_index<=0;uart_state<=2;end
      else uart_state<=0;
    2:if(baud_count!=0)baud_count<=baud_count-1'b1;
      else begin
       shift[bit_index]<=sync_rx[1];baud_count<=DIV-1;
       if(bit_index==7)uart_state<=3;else bit_index<=bit_index+1'b1;
      end
    3:if(baud_count!=0)baud_count<=baud_count-1'b1;
      else if(sync_rx[1])begin byte_data<=shift;byte_valid<=1;uart_state<=0;end
      else begin byte_error<=1;uart_state<=4;end
    4:if(sync_rx[1])uart_state<=0;
    default:uart_state<=0;
   endcase
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   pos<=0;length<=0;sum<=0;gap_count<=0;
   data_valid<=0;connect_event<=0;disconnect_event<=0;invalid_frame<=0;
   lx<=0;ly<=0;rx_axis<=0;ry_axis<=0;dpad<=0;buttons2<=0;buttons3<=0;rt<=0;lt<=0;
   valid_count<=0;error_count<=0;
   for(i=0;i<14;i=i+1)packet[i]<=0;
  end else begin
   data_valid<=0;connect_event<=0;disconnect_event<=0;invalid_frame<=0;
   if(pos==0 || byte_valid)gap_count<=0;
   else if(gap_count<GAP)gap_count<=gap_count+1'b1;
   if(byte_error || (pos!=0 && gap_count>=GAP && !byte_valid))begin
    pos<=0;gap_count<=0;invalid_frame<=1;error_count<=error_count+1'b1;
   end else if(byte_valid)begin
    case(pos)
     0:if(byte_data==8'haa)begin pos<=1;sum<=8'haa;end
     1:if(byte_data==8'h55)begin pos<=2;sum<=8'hff;end
       else if(byte_data==8'haa)begin pos<=1;sum<=8'haa;end
       else pos<=0;
     2:if(byte_data==7 || byte_data==15)begin
        length<=byte_data;sum<=sum+byte_data;pos<=3;
       end else begin pos<=0;invalid_frame<=1;error_count<=error_count+1'b1;end
     default:if({4'd0,pos}==length-1'b1)begin
      pos<=0;
      if(byte_data==sum && packet[3]==8'ha0 && length==15 && packet[4]==1 &&
         packet[9]<=8 && packet[5]!=8'h80 && packet[6]!=8'h80 &&
         packet[7]!=8'h80 && packet[8]!=8'h80)begin
       lx<=packet[5];ly<=packet[6];rx_axis<=packet[7];ry_axis<=packet[8];
       dpad<=packet[9];buttons2<=packet[10];buttons3<=packet[11];
       rt<=packet[12];lt<=packet[13];data_valid<=1;valid_count<=valid_count+1'b1;
      end else if(byte_data==sum && packet[3]==8'ha0 && length==7 && packet[4]==0 &&
                  (packet[5]==8'h55 || packet[5]==8'hff))begin
       connect_event<=packet[5]==8'h55;disconnect_event<=packet[5]==8'hff;
       valid_count<=valid_count+1'b1;
      end else begin invalid_frame<=1;error_count<=error_count+1'b1;end
     end else begin packet[pos]<=byte_data;sum<=sum+byte_data;pos<=pos+1'b1;end
    endcase
   end
  end
 end
endmodule
`default_nettype wire
