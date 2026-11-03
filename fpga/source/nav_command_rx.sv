`default_nettype none
// v1: 4E 56 01 seq cmd 00 crc16le (8B).
// v2: 4E 56 02 seq cmd index value0le32 value1le32 crc16le (16B).
module nav_command_rx #(parameter integer CLK_FREQ=50000000,BAUD_RATE=115200)(
 input wire clk,rst_n,rx,
 output reg start_request,stop_request,query_request,bench_enter,bench_exit,
 output reg [1:0] bench_jog,output reg [7:0] last_sequence,output reg [15:0] errors,
 output reg manual_enter,manual_exit,extended_valid,
 output reg [7:0] extended_command,extended_index,output reg [31:0] value0,value1
);
 wire [7:0] data;wire done,frame_error;
 uart_rx #(.CLK_FREQ(CLK_FREQ),.BAUD_RATE(BAUD_RATE)) uart(
  .clk(clk),.rst_n(rst_n),.rx(rx),.data(data),.done(done),.frame_error(frame_error));
 reg [4:0] index;
 reg [7:0] seq,cmd,lo,version,arg_index;
 reg [31:0] a,b,gap;
 reg [15:0] crc;
 reg seen;
 wire [7:0] delta=seq-last_sequence;
 wire v1=version==1;
 wire [4:0] crc_index=v1?5'd6:5'd14;
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] d);
  reg [15:0] v;integer k;
  begin v=c^{d,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 task reject;begin if(errors!=65535)errors<=errors+1'b1;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   index<=0;seq<=0;cmd<=0;lo<=0;version<=0;arg_index<=0;crc<=65535;
   a<=0;b<=0;seen<=0;gap<=0;last_sequence<=0;errors<=0;
   start_request<=0;stop_request<=0;query_request<=0;bench_enter<=0;bench_exit<=0;bench_jog<=0;
   manual_enter<=0;manual_exit<=0;extended_valid<=0;extended_command<=0;extended_index<=0;value0<=0;value1<=0;
  end else begin
   start_request<=0;stop_request<=0;query_request<=0;bench_enter<=0;bench_exit<=0;bench_jog<=0;
   manual_enter<=0;manual_exit<=0;extended_valid<=0;
   if(index!=0)gap<=gap+1'b1;else gap<=0;
   if(frame_error || (index!=0 && gap>=CLK_FREQ/1000*20))begin index<=0;gap<=0;reject();end
   else if(done)begin
    gap<=0;
    if(index==0)begin if(data==8'h4e)begin index<=1;crc<=crc_byte(16'hffff,data);end end
    else if(index==1)begin
     if(data==8'h56)begin index<=2;crc<=crc_byte(crc,data);end
     else if(data==8'h4e)crc<=crc_byte(16'hffff,data);
     else index<=0;
    end else if(index==2)begin
     if(data==1 || data==2)begin version<=data;index<=3;crc<=crc_byte(crc,data);end
     else begin index<=0;reject();end
    end else if(index==crc_index)begin lo<=data;index<=index+1'b1;end
    else if(index==crc_index+1)begin
     index<=0;
     if({data,lo}!=crc || (v1 && (arg_index!=0 || cmd<1 || cmd>10)) ||
        (!v1 && !((cmd>=8'h20 && cmd<=8'h23) || cmd==8'h30)))reject();
     else if(v1 && cmd==2)begin stop_request<=1;seen<=0;end
     else if(v1 && cmd==5)bench_exit<=1;
     else if(v1 && cmd==10)manual_exit<=1;
     else if(seen && (delta==0 || delta>=128))reject();
     else begin
      last_sequence<=seq;seen<=1;
      if(!v1)begin extended_valid<=1;extended_command<=cmd;extended_index<=arg_index;value0<=a;value1<=b;end
      else case(cmd)
       1:start_request<=1;3:query_request<=1;4:bench_enter<=1;
       6:bench_jog<=1;7:bench_jog<=2;8:bench_jog<=3;9:manual_enter<=1;
       default:begin end
      endcase
     end
    end else begin
     crc<=crc_byte(crc,data);index<=index+1'b1;
     case(index)
      3:seq<=data;4:cmd<=data;5:arg_index<=data;
      6:a[7:0]<=data;7:a[15:8]<=data;8:a[23:16]<=data;9:a[31:24]<=data;
      10:b[7:0]<=data;11:b[15:8]<=data;12:b[23:16]<=data;13:b[31:24]<=data;
      default:begin end
     endcase
    end
   end
  end
 end
endmodule
`default_nettype wire