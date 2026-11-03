`default_nettype none
// IPv4/UDP transmitter, Ethernet broadcast destination on a dedicated cable.
// IPv4 UDP checksum is intentionally zero. Ethernet FCS is always generated.
module ethernet_mac_tx #(
 parameter [47:0] SRC_MAC=48'h020000000002,
 parameter [31:0] SRC_IP=32'hc0a83202,DST_IP=32'hc0a83201,
 parameter [15:0] SRC_PORT=5000,DST_PORT=6102
)(input wire clk,reset_n,
 input wire load_valid,input wire [10:0] load_addr,input wire [7:0] load_data,
 input wire start,input wire [15:0] payload_length,output wire ready,
 output reg [7:0] txd,output reg txen,output reg done);
 reg [7:0] payload[0:2047];
 reg [7:0] payload_q;
 localparam IDLE=0,PREAMBLE=1,HEADER=2,DATA=3,FCS=4,GAP=5;
 reg [2:0] state;
 reg [15:0] length;
 reg [5:0] header_index;
 reg [10:0] data_index;
 reg [3:0] phase_count;
 reg [7:0] header_q;
 reg [31:0] crc;
 wire [15:0] ip_length=length+16'd28,udp_length=length+16'd8;
 wire [31:0] sum=32'h4500+{16'b0,ip_length}+32'h4000+32'h4011+
             {16'b0,SRC_IP[31:16]}+{16'b0,SRC_IP[15:0]}+
             {16'b0,DST_IP[31:16]}+{16'b0,DST_IP[15:0]};
 wire [16:0] folded={1'b0,sum[15:0]}+{1'b0,sum[31:16]};
 wire [15:0] checksum=~(folded[15:0]+{15'b0,folded[16]});
 wire [10:0] prefetch=(state==DATA)?data_index+11'd1:11'd0;
 assign ready=(state==IDLE);
 function [31:0] next_crc(input [31:0] c,input [7:0] d);
  reg [31:0] r;integer j;
  begin r=c;for(j=0;j<8;j=j+1)r=(r>>1)^((r[0]^d[j])?32'hedb88320:32'b0);next_crc=r;end
 endfunction
 // Prepare the next header byte a cycle ahead: header selection and CRC
 // are separated by header_q. No variable-width barrel shift on the wire.
 function [7:0] header_byte(input [5:0] i);
  begin
   case(i)
    0,1,2,3,4,5:header_byte=8'hff;
    6:header_byte=SRC_MAC[47:40];7:header_byte=SRC_MAC[39:32];
    8:header_byte=SRC_MAC[31:24];9:header_byte=SRC_MAC[23:16];
    10:header_byte=SRC_MAC[15:8];11:header_byte=SRC_MAC[7:0];
    12:header_byte=8'h08;14:header_byte=8'h45;
    16:header_byte=ip_length[15:8];17:header_byte=ip_length[7:0];
    20,22:header_byte=8'h40;23:header_byte=8'h11;
    24:header_byte=checksum[15:8];25:header_byte=checksum[7:0];
    26:header_byte=SRC_IP[31:24];27:header_byte=SRC_IP[23:16];
    28:header_byte=SRC_IP[15:8];29:header_byte=SRC_IP[7:0];
    30:header_byte=DST_IP[31:24];31:header_byte=DST_IP[23:16];
    32:header_byte=DST_IP[15:8];33:header_byte=DST_IP[7:0];
    34:header_byte=SRC_PORT[15:8];35:header_byte=SRC_PORT[7:0];
    36:header_byte=DST_PORT[15:8];37:header_byte=DST_PORT[7:0];
    38:header_byte=udp_length[15:8];39:header_byte=udp_length[7:0];
    default:header_byte=0;
   endcase
  end
 endfunction
 always @(posedge clk)begin
  if(load_valid&&ready)payload[load_addr]<=load_data;
  payload_q<=payload[prefetch];
 end
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin
   state<=IDLE;length<=0;header_index<=0;data_index<=0;phase_count<=0;
   crc<=32'hffffffff;header_q<=8'hff;txd<=0;txen<=0;done<=0;
  end
  else begin
   done<=0;
   case(state)
    IDLE:begin
     txen<=0;txd<=0;
     if(start&&payload_length>=32&&payload_length<=1472)begin
      state<=PREAMBLE;phase_count<=0;length<=payload_length;
      crc<=32'hffffffff;header_index<=0;header_q<=8'hff;data_index<=0;
     end
    end
    PREAMBLE:begin
     txen<=1;txd<=(phase_count==7)?8'hd5:8'h55;
     if(phase_count==7)state<=HEADER;else phase_count<=phase_count+1'b1;
    end
    HEADER:begin
     txen<=1;txd<=header_q;crc<=next_crc(crc,header_q);
     header_q<=header_byte(header_index+6'd1);
     if(header_index==41)state<=DATA;else header_index<=header_index+1'b1;
    end
    DATA:begin
     txen<=1;txd<=payload_q;crc<=next_crc(crc,payload_q);
     if({5'b0,data_index}+16'd1==length)begin state<=FCS;phase_count<=0;end
     else data_index<=data_index+1'b1;
    end
    FCS:begin
     txen<=1;txd<=~crc[7:0];crc<={8'b0,crc[31:8]};
     if(phase_count==3)begin state<=GAP;phase_count<=12;end
     else phase_count<=phase_count+1'b1;
    end
    GAP:begin
     txen<=0;txd<=0;
     if(phase_count==12)done<=1;
     if(phase_count==1)state<=IDLE;else phase_count<=phase_count-1'b1;
    end
    default:begin state<=IDLE;txen<=0;txd<=0;end
   endcase
  end
endmodule
`default_nettype wire
