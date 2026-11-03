`default_nettype none
// Clause 22 master. 1 MHz MDC at 50 MHz, complete preamble, read TA check,
// eight idle clocks after every operation. MDIO is open drain.
module udp_mdio #(parameter HALF=25)(
 input wire clk,reset_n,go,reading,input wire [4:0] phy,addr,
 input wire [15:0] write_data,output reg [15:0] read_data,
 output reg busy,done,ack_error,output reg mdc,inout wire mdio
);
 reg [63:0] shift;reg [6:0] bitno;reg [15:0] div;
 reg rd;reg [3:0] idle_clocks;
 wire drive=busy&&(idle_clocks==0)&&!(rd&&bitno<=17);
 assign mdio=(drive&&!shift[bitno])?1'b0:1'bz;
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin shift<=0;bitno<=63;div<=0;rd<=0;idle_clocks<=0;
   read_data<=0;busy<=0;done<=0;ack_error<=0;mdc<=0;end
  else begin
   done<=0;
   if(!busy)begin
    mdc<=0;div<=0;
    if(go)begin
     shift<={32'hffffffff,2'b01,(reading?2'b10:2'b01),phy,addr,2'b10,write_data};
     rd<=reading;bitno<=63;busy<=1;idle_clocks<=0;read_data<=0;ack_error<=0;
    end
   end else if(div==HALF-1)begin
    div<=0;mdc<=!mdc;
    if(!mdc&&idle_clocks==0&&rd)begin
     if(bitno==16&&mdio!==1'b0)ack_error<=1;
     if(bitno<16)read_data[bitno]<=mdio;
    end
    if(mdc)begin
     if(idle_clocks!=0)begin
      if(idle_clocks==8)begin busy<=0;done<=1;end
      else idle_clocks<=idle_clocks+1'b1;
     end else if(bitno==0)idle_clocks<=1;
     else bitno<=bitno-1'b1;
    end
   end else div<=div+1'b1;
  end
endmodule

// RTL8211F-CG, address 5 per official ACG720 reference. Refuse to configure an
// unidentified PHY. MAC TX clock is edge aligned: enable PHY internal TX delay.
// Register definitions checked against Linux drivers/net/phy/realtek.
module ethernet_phy #(parameter CLOCK_HZ=50000000,parameter [4:0] PHY_ADDR=5)(
 input wire clk,reset_n,output wire phy_reset_n,output wire mdc,inout wire mdio,
 output reg link_up,output reg [7:0] fault,output reg [31:0] phy_id,
 input wire tx_clk,input wire [7:0] tx_data,input wire tx_en,
 output wire rgmii_txc,output wire [3:0] rgmii_txd,output wire rgmii_txen
);
 localparam RESET_TIME=CLOCK_HZ/20,BOOT_TIME=CLOCK_HZ/20,POLL_TIME=CLOCK_HZ/10;
 reg [31:0] timer;
 reg [2:0] state;reg [4:0] step;
 reg go;wire busy,done,ack_error;wire [15:0] result;
 reg [15:0] saved_delay,bmsr;reg reading;reg [4:0] addr;reg [15:0] data;
 assign phy_reset_n=reset_n&&(state!=0);
 udp_mdio #(.HALF(CLOCK_HZ/2000000)) mdio_master(clk,phy_reset_n,go,reading,
  PHY_ADDR,addr,data,result,busy,done,ack_error,mdc,mdio);
 always @* begin
  reading=0;addr=0;data=0;
  case(step)
   0:begin reading=1;addr=2;end
   1:begin reading=1;addr=3;end
   2:begin addr=31;data=16'h0d08;end
   3:begin reading=1;addr=17;end
   4:begin addr=17;data=saved_delay|16'h0100;end
   5:begin reading=1;addr=17;end
   6:begin addr=31;data=0;end
   7:begin addr=4;data=16'h0001;end // no 10/100 advertisement
   8:begin addr=9;data=16'h0200;end // 1000BASE-T full duplex only
   9:begin addr=0;data=16'h1200;end
   10,11:begin reading=1;addr=1;end
   12:begin reading=1;addr=10;end
   default:begin reading=1;addr=2;end
  endcase
 end
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin timer<=0;state<=0;step<=0;go<=0;link_up<=0;
   fault<=0;phy_id<=0;saved_delay<=0;bmsr<=0;end
  else begin
   go<=0;
   case(state)
    0:if(timer==RESET_TIME-1)begin timer<=0;state<=1;end else timer<=timer+1'b1;
    1:if(timer==BOOT_TIME-1)begin timer<=0;step<=0;state<=2;end else timer<=timer+1'b1;
    2:if(!busy)begin go<=1;state<=3;end
    3:if(done)begin
     if(ack_error)begin fault<=1;link_up<=0;state<=5;timer<=0;end
     else if(step==0&&result!=16'h001c)begin fault<=2;link_up<=0;state<=5;timer<=0;phy_id[31:16]<=result;end
     else if(step==1&&(result&16'hfff0)!=16'hc910)begin fault<=2;link_up<=0;state<=5;timer<=0;phy_id[15:0]<=result;end
     else if(step==5&&!result[8])begin fault<=3;link_up<=0;state<=5;timer<=0;end
     else begin
      if(step==0)phy_id[31:16]<=result;
      if(step==1)phy_id[15:0]<=result;
      if(step==3)saved_delay<=result;
      if(step==11)bmsr<=result;
      if(step==12)begin
       link_up<=bmsr[2]&&bmsr[5]&&result[11]&&!result[15];
       fault<=result[15]?8'd4:8'd0;
       timer<=0;state<=4;
      end else begin step<=step+1'b1;state<=2;end
     end
    end
    4:if(timer==POLL_TIME-1)begin step<=10;state<=2;timer<=0;end else timer<=timer+1'b1;
    5:if(timer==CLOCK_HZ-1)begin state<=0;timer<=0;end else timer<=timer+1'b1;
    default:state<=0;
   endcase
  end
`ifdef SIMULATION
 assign rgmii_txc=tx_clk;
 assign rgmii_txd=tx_clk?tx_data[3:0]:tx_data[7:4];
 assign rgmii_txen=tx_en;
`else
 genvar k;
 generate for(k=0;k<4;k=k+1)begin: tx_bits
  ODDR dq(.Q0(rgmii_txd[k]),.Q1(),.D0(tx_data[k]),.D1(tx_data[k+4]),.TX(1'b1),.CLK(tx_clk));
 end endgenerate
 ODDR ctl(.Q0(rgmii_txen),.Q1(),.D0(tx_en),.D1(tx_en),.TX(1'b1),.CLK(tx_clk));
 ODDR ck(.Q0(rgmii_txc),.Q1(),.D0(1'b1),.D1(1'b0),.TX(1'b1),.CLK(tx_clk));
`endif
endmodule
`default_nettype wire
