`default_nettype none
// Four independent 2 KiB packet slots. Payload and descriptor remain immutable
// from commit until the reader releases the slot. Only Gray pointers cross CDC.
module cdc_packet_buffer(
 input wire wr_clk,rd_clk,reset_n,
 input wire wr_en,wr_commit,input wire [10:0] wr_addr,input wire [7:0] wr_data,
 input wire [31:0] wr_frame,input wire [15:0] wr_index,wr_length,
 input wire wr_end,output wire wr_ready,
 output wire rd_valid,input wire rd_release,input wire [10:0] rd_addr,
 output reg [7:0] rd_data,output wire [31:0] rd_frame,
 output wire [15:0] rd_index,rd_length,output wire rd_end
);
 reg [7:0] memory[0:8191];
 reg [64:0] descriptor[0:3];
 reg [2:0] wb,rb,wg,rg;
 (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [2:0] wr_release,rd_release_sync;
 always @(posedge wr_clk or negedge reset_n)
  if(!reset_n)wr_release<=0;else wr_release<={wr_release[1:0],1'b1};
 always @(posedge rd_clk or negedge reset_n)
  if(!reset_n)rd_release_sync<=0;else rd_release_sync<={rd_release_sync[1:0],1'b1};
 wire wr_n=wr_release[2],rd_n=rd_release_sync[2];
 (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [2:0] rg_meta,rg_sync,wg_meta,wg_sync;
 wire full=(wg=={~rg_sync[2:1],rg_sync[0]});
 assign wr_ready=wr_n&&!full;
 assign rd_valid=rd_n&&(rg!=wg_sync);
 assign {rd_frame,rd_index,rd_length,rd_end}=descriptor[rb[1:0]];
 always @(posedge wr_clk or negedge wr_n)
  if(!wr_n)begin wb<=0;wg<=0;rg_meta<=0;rg_sync<=0;end
  else begin
   rg_meta<=rg;rg_sync<=rg_meta;
   if(wr_commit&&!full)begin
    descriptor[wb[1:0]]<={wr_frame,wr_index,wr_length,wr_end};
    wb<=wb+3'd1;wg<=((wb+3'd1)>>1)^(wb+3'd1);
   end
  end
 always @(posedge wr_clk)
  if(wr_en&&wr_n&&!full)memory[{wb[1:0],wr_addr}]<=wr_data;
 always @(posedge rd_clk)rd_data<=memory[{rb[1:0],rd_addr}];
 always @(posedge rd_clk or negedge rd_n)
  if(!rd_n)begin rb<=0;rg<=0;wg_meta<=0;wg_sync<=0;end
  else begin
   wg_meta<=wg;wg_sync<=wg_meta;
   if(rd_release&&rd_valid)begin rb<=rb+3'd1;rg<=((rb+3'd1)>>1)^(rb+3'd1);end
  end
endmodule
`default_nettype wire
