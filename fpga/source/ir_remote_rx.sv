`default_nettype none
// Active-low demodulated IR input. NEC/HT6221 pulse-distance frames,
// LSB first; retain both address bytes (standard or extended NEC).
// A frame is published only after the final mark and command complement.
module ir_remote_rx #(parameter integer CLK_FREQ=50000000)(
 input wire clk,rst_n,ir_rx,
 output reg frame_valid,repeat_valid,invalid_frame,
 output wire activity,
 output reg [15:0] address,valid_count,error_count,
 output reg [7:0] command
);
 localparam integer US=CLK_FREQ/1000000,TIMEOUT=US*12000;
 localparam IDLE=0,LEADER_LOW=1,LEADER_HIGH=2,BIT_LOW=3,BIT_HIGH=4,TRAIL=5,REPEAT_LOW=6;
 (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [1:0] sync;
 reg previous;
 wire rise=sync[1] && !previous,fall=!sync[1] && previous;
 assign activity=rise || fall;
 reg [2:0] state;
 reg [31:0] duration,bits;
 reg [5:0] bit_index;
 wire mark_ok=duration>=US*400 && duration<=US*750;
 wire zero_ok=mark_ok;
 wire one_ok=duration>=US*1400 && duration<=US*1900;
 task reject;
 begin state<=IDLE;invalid_frame<=1;if(error_count!=16'hffff)error_count<=error_count+1'b1;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sync<=3;previous<=1;end
  else begin sync<={sync[0],ir_rx};previous<=sync[1];end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;duration<=0;bits<=0;bit_index<=0;
   frame_valid<=0;repeat_valid<=0;invalid_frame<=0;
   address<=0;command<=0;valid_count<=0;error_count<=0;
  end else begin
   frame_valid<=0;repeat_valid<=0;invalid_frame<=0;
   if(activity)duration<=0;else if(duration<TIMEOUT)duration<=duration+1'b1;
   if(state!=IDLE && duration>=TIMEOUT)reject();
   else case(state)
    IDLE:if(fall)state<=LEADER_LOW;
    LEADER_LOW:if(rise)begin
     if(duration>=US*8000 && duration<=US*10000)state<=LEADER_HIGH;
     else reject();
    end
    LEADER_HIGH:if(fall)begin
     if(duration>=US*4000 && duration<=US*5000)begin state<=BIT_LOW;bit_index<=0;bits<=0;end
     else if(duration>=US*2000 && duration<=US*2800)state<=REPEAT_LOW;
     else reject();
    end
    BIT_LOW:if(rise)begin if(mark_ok)state<=BIT_HIGH;else reject();end
    BIT_HIGH:if(fall)begin
     if(zero_ok || one_ok)begin
      bits[bit_index]<=one_ok;
      if(bit_index==31)state<=TRAIL;
      else begin bit_index<=bit_index+1'b1;state<=BIT_LOW;end
     end else reject();
    end
    TRAIL:if(rise)begin
     if(mark_ok && (bits[31:24]^bits[23:16])==8'hff)begin
      address<=bits[15:0];command<=bits[23:16];frame_valid<=1;state<=IDLE;
      if(valid_count!=16'hffff)valid_count<=valid_count+1'b1;
     end else reject();
    end
    REPEAT_LOW:if(rise)begin if(mark_ok)begin repeat_valid<=1;state<=IDLE;end else reject();end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
`default_nettype wire
