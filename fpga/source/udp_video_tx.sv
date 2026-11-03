`default_nettype none
module udp_video_tx #(parameter WIDTH=1280,HEIGHT=720,CHUNK=1280)(
 input wire clk,reset_n,link_up,
 input wire packet_valid,input wire [31:0] packet_frame,
 input wire [15:0] packet_index,packet_length,input wire packet_end,
 output wire packet_release,output wire [10:0] packet_addr,input wire [7:0] packet_data,
 output wire mac_load,input wire mac_ready,mac_done,
 output wire [10:0] mac_addr,output wire [7:0] mac_data,
 output wire mac_start,output wire [15:0] mac_length,
 output reg [31:0] sent_packets
);
 localparam COUNT=WIDTH*HEIGHT*2/CHUNK,FRAME_BYTES=WIDTH*HEIGHT*2;
 localparam IDLE=0,HEADER=1,COPY=2,SEND=3,WAIT_DONE=4,RELEASE=5;
 reg [2:0] state;reg [10:0] pos;
 reg [31:0] fid;reg [15:0] idx,len;reg ending;
 wire [255:0] header={32'h46554450,8'd1,8'd1,15'b0,ending,
       fid,idx,16'(COUNT),16'(WIDTH),16'(HEIGHT),len,16'd32,32'(FRAME_BYTES),32'b0};
 assign mac_load=(state==HEADER||state==COPY)&&mac_ready;
 assign mac_addr=(state==HEADER)?pos:pos+11'd32;
 assign mac_data=(state==HEADER)?header[255-pos*8-:8]:packet_data;
 assign packet_addr=(state==COPY)?pos+11'd1:11'b0;
 assign mac_start=(state==SEND)&&link_up;
 assign mac_length=len+16'd32;
 assign packet_release=(state==RELEASE);
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin state<=IDLE;pos<=0;fid<=0;idx<=0;len<=0;ending<=0;sent_packets<=0;end
  else case(state)
   IDLE:if(packet_valid)begin
    if(!link_up||packet_length>CHUNK)state<=RELEASE;
    else if(mac_ready)begin fid<=packet_frame;idx<=packet_index;len<=packet_length;
     ending<=packet_end;pos<=0;state<=HEADER;end
   end
   HEADER:if(mac_ready)begin
    if(pos==31)begin pos<=0;state<=(len==0)?SEND:COPY;end else pos<=pos+1'b1;
   end
   COPY:if(mac_ready)begin if(pos==len-1)state<=SEND;else pos<=pos+1'b1;end
   SEND:if(!link_up)state<=RELEASE;else if(mac_ready)state<=WAIT_DONE;
   WAIT_DONE:if(mac_done)begin sent_packets<=sent_packets+1'b1;state<=RELEASE;end
   RELEASE:state<=IDLE;
   default:state<=IDLE;
  endcase
endmodule
`default_nettype wire
