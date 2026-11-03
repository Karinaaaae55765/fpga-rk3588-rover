`timescale 1ns/1ps
module tb_task6_top;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1;wire stby,p1,p2,tx;
 task6_top #(.CLK_FREQ(1000000)) dut(.clk50M(clk),.reset_n(rst),.tof_uart_rx(1'b1),.imu_uart_rx(1'b1),
 .control_rx(rx),.ir_rx(1'b1),.bt_rx(1'b1),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
 .camera_vsync(1'b0),.camera_href(1'b0),.camera_pclk(clk),.camera_data(8'd0),
 .motor_stby(stby),.motor_pwm1(p1),.motor_pwm2(p2),.sensor_uart_tx(tx));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task uart_byte(input [7:0] b);
  begin @(negedge clk);rx=0;cycles(9);
   for(integer i=0;i<8;i=i+1)begin @(negedge clk);rx=b[i];cycles(9);end
   @(negedge clk);rx=1;cycles(9);
  end
 endtask
 task send(input [7:0] cmd,input [31:0] sid,input [15:0] seq,input signed [15:0] l,r,input [31:0] challenge,input bad);
  reg [143:0] body;reg [15:0] crc;reg [7:0] b;
  begin
   body={challenge,r,l,seq,sid,cmd,8'd1,8'h36,8'h54};crc=16'hffff;
   for(integer i=0;i<18;i=i+1)begin
    b=body[i*8 +:8];uart_byte(b);crc=crc^{b,8'd0};
    for(integer j=0;j<8;j=j+1)crc=crc[15]?(crc<<1)^16'h1021:crc<<1;
   end
   uart_byte(crc[7:0]);uart_byte(crc[15:8]^(bad?8'h01:8'h00));cycles(30);
  end
 endtask
 wire [7:0] rxbyte;wire rxvalid,rxerror;
 uart_rx #(.CLK_FREQ(1000000)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.data(rxbyte),.done(rxvalid),.frame_error(rxerror));
 reg [7:0] history[0:511];integer count=0;reg saw_pwm=0;integer frames=0;reg [15:0] c;
 integer packet_index=0,packet_size=0,total_frames=0;reg [7:0] packet[0:127];reg [15:0] crc_check;
 always @(posedge clk)begin
  if(rst && rxerror)$fatal(1,"telemetry UART framing error");
  if(rxvalid)begin
   packet[packet_index]=rxbyte;
   if(packet_index==0 && rxbyte!=8'ha5)$fatal(1,"telemetry header lost");
   if(packet_index==1 && rxbyte!=8'h5a)$fatal(1,"telemetry second header lost");
   if(packet_index==4)begin packet_size=rxbyte+7;if(packet_size>128)$fatal(1,"oversized frame");end
   if(packet_index>=4 && packet_index==packet_size-1)begin
    crc_check=16'hffff;
    for(integer n=2;n<packet_size-2;n=n+1)begin
     crc_check=crc_check^{packet[n],8'd0};
     for(integer bitn=0;bitn<8;bitn=bitn+1)crc_check=crc_check[15]?(crc_check<<1)^16'h1021:crc_check<<1;
    end
    if({packet[packet_size-1],packet[packet_size-2]}!=crc_check)$fatal(1,"telemetry CRC/type=%h",packet[3]);
    if(packet[3]==8'h36)frames=frames+1;
    total_frames=total_frames+1;packet_index=0;
   end else packet_index=packet_index+1;
  end
  if(rxvalid)begin history[count%512]=rxbyte;count=count+1;end
  if(p1 || p2)saw_pwm=1;
 end
 initial begin
  cycles(5);rst=1;force dut.system_n=rst;
  force dut.imu_valid=1;force dut.distance_valid=1;force dut.tof_strength=500;
  force dut.distance_mm=2000;force dut.yaw_cdeg=0;
  force dut.nav_left_count=123;force dut.nav_right_count=-456;
  force dut.nav_left_actual=12;force dut.nav_right_actual=-14;
  cycles(3);
  if(dut.ext_left_count!=123 || dut.ext_right_count!=456 || dut.ext_left_actual!=12 || dut.ext_right_actual!=14)$fatal(1,"external feedback direction");
  release dut.nav_left_count;release dut.nav_right_count;release dut.nav_left_actual;release dut.nav_right_actual;
  cycles(310000);
  send(1,32'h12345678,1,0,0,dut.ext_challenge,0);
  if(!dut.ext_armed)$fatal(1,"real UART ARM failed ready=%b idle=%d release=%b",dut.raw_sensors_ready,dut.state,dut.released);
  send(2,32'h12345678,2,20,20,0,0);cycles(150000);
  if(!stby || !saw_pwm || dut.left_request!=20)$fatal(1,"UART to motor path failed armed=%b reason=%d en=%b stby=%b pwm=%b l=%d fault=%b state=%d",dut.ext_armed,dut.ext_reason,dut.ext_enable,stby,saw_pwm,dut.left_request,dut.raw_hard_fault,dut.state);
  send(2,32'h12345678,3,20,20,0,1);cycles(3);
  if(stby || dut.ext_armed)$fatal(1,"corrupt UART frame did not stop motors");
  send(1,32'h12345679,1,0,0,dut.ext_challenge,0);
  send(2,32'h12345679,2,20,20,0,0);cycles(310000);
  if(stby || dut.ext_armed || dut.ext_reason!=6)$fatal(1,"real top lease timeout");
  if(frames==0)$fatal(1,"no CRC-correct task6 telemetry frame");
  $display("Validated %0d multiplexed frames (%0d external)",total_frames,frames);
  $display("PASS real UART receiver to PWM, corrupt frame stop, lease timeout, telemetry CRC");$finish;
 end
 initial begin #50000000;$fatal(1,"timeout");end
endmodule
