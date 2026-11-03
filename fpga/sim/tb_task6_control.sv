`timescale 1ns/1ps
module tb_task6_control;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,ready=1,fault=0,obstacle=0,slow=0,stop=0,manual=0,idle=1,released=1;
 reg valid=0,invalid=0;reg [7:0] cmd=0;reg [31:0] sid=10,nonce=0;reg [15:0] seq=1;
 reg signed [15:0] l=0,r=0;
 wire armed,en;wire [31:0] session,challenge;wire [15:0] accepted;wire [7:0] reason;
 wire signed [31:0] left_req,right_req;
 task6_control dut(clk,rst,tick,ready,fault,obstacle,slow,stop,manual,idle,released,
  valid,invalid,cmd,sid,nonce,seq,l,r,armed,en,session,challenge,accepted,reason,left_req,right_req);
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task pulse_tick;begin @(negedge clk);tick=1;cycles(1);@(negedge clk);tick=0;cycles(1);end endtask
 task send(input [7:0] command,input [15:0] number,input signed [15:0] a,b);
  begin @(negedge clk);cmd=command;seq=number;l=a;r=b;valid=1;cycles(1);@(negedge clk);valid=0;cycles(1);end
 endtask
 task arm;begin sid=sid+1;nonce=challenge;send(1,1,0,0);if(!armed)$fatal(1,"ARM rejected");end endtask
 initial begin
  cycles(3);rst=1;cycles(3);
  send(2,2,20,20);if(en)$fatal(1,"velocity before ARM");
  arm();send(2,2,20,20);if(left_req!=20 || !en)$fatal(1,"velocity not applied");
  send(2,2,5,5);if(left_req!=20)$fatal(1,"duplicate applied");
  send(2,1,5,5);if(left_req!=20)$fatal(1,"old sequence applied");
  sid=sid+1;send(2,3,5,5);if(left_req!=20)$fatal(1,"wrong session applied");sid=sid-1;
  slow=1;cycles(1);if(left_req!=10)$fatal(1,"slow limit");slow=0;
  for(integer i=0;i<15;i=i+1)begin send(5,3+i,0,0);pulse_tick();end
  if(armed || en || left_req!=0 || reason!=6)$fatal(1,"heartbeat extended lease");
  send(1,20,0,0);if(armed)$fatal(1,"old nonce/session rearmed");
  arm();send(2,2,31,0);if(armed || reason!=7)$fatal(1,"range not rejected");
  arm();send(2,2,-10,-10);if(armed)$fatal(1,"automatic reverse allowed");
  arm();send(2,2,-10,10);if(!en || left_req!=-10)$fatal(1,"turn rejected");
  @(negedge clk);obstacle=1;#1;if(en)$fatal(1,"obstacle not immediate");cycles(1);obstacle=0;cycles(1);
  if(armed)$fatal(1,"obstacle did not revoke");
  arm();send(2,2,10,10);manual=1;cycles(1);manual=0;cycles(1);if(armed)$fatal(1,"manual takeover");
  arm();invalid=1;cycles(1);invalid=0;cycles(1);if(armed)$fatal(1,"bad frame revoke");
  arm();ready=0;cycles(1);ready=1;cycles(2);if(armed)$fatal(1,"sensor loss revoke");
  arm();sid=999;send(3,0,0,0);if(armed)$fatal(1,"STOP requires session");
  $display("PASS task6 session, sequence, lease, heartbeat, limits, obstacle, takeover, invalid frame and STOP");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
