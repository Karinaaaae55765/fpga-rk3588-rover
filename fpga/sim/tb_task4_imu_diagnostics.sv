`timescale 1ns/1ps
module tb_task4_imu_diagnostics;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,snapshot=0,trigger=0,active=1;
 wire signed [15:0] filtered;wire filtered_valid,tx,busy;
 perception_hub #(.CLK_FREQ(1000000)) hub(.clk(clk),.rst_n(rst),.tof_uart_rx(1'b1),.imu_uart_rx(rx),
  .front_distance_mm(),.tof_strength(),.distance_valid(),.roll_cdeg(),.pitch_cdeg(),
  .yaw_cdeg(filtered),.imu_valid(filtered_valid),.tof_error_count(),.imu_error_count());
 task4_imu_diagnostics #(.CLK_FREQ(1000000)) dut(.clk(clk),.rst_n(rst),.imu_rx(rx),
  .snapshot(snapshot),.trigger(trigger),.active(active),.route_mode(1'b0),.pose_valid(1'b1),.state(4'd0),
  .filtered_yaw(filtered),.heading(-16'sd3377),.filtered_valid(filtered_valid),.tx(tx),.busy(busy));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send_byte(input [7:0] b,input integer bitclks);
  begin
   @(negedge clk);rx=0;cycles(bitclks);
   for(integer j=0;j<8;j=j+1)begin @(negedge clk);rx=b[j];cycles(bitclks);end
   @(negedge clk);rx=1;cycles(bitclks);
  end
 endtask
 task angle_frame(input signed [15:0] raw,input integer bitclks,input reg corrupt);
  reg [7:0] p[0:10],sum;
  begin
   p[0]=8'h55;p[1]=8'h53;p[2]=0;p[3]=0;p[4]=0;p[5]=0;
   p[6]=raw[7:0];p[7]=raw[15:8];p[8]=0;p[9]=0;sum=0;
   for(integer j=0;j<10;j=j+1)sum=sum+p[j];p[10]=corrupt?(sum^8'h01):sum;
   for(integer j=0;j<11;j=j+1)send_byte(p[j],bitclks);
   cycles(1500);
  end
 endtask
 // The passive duplicate must match the live baseline pipeline on real UART bits.
 always @(posedge clk)if(rst)begin
  if(dut.valid_a!==hub.imu_angle_valid || dut.valid_b!==hub.imu_angle_alt_valid)$fatal(1,"observer pulse mismatch");
  if(dut.valid_a && (dut.raw_a!==hub.yaw_raw || dut.angle_a!==hub.yaw_cdeg_raw))$fatal(1,"9600 observer mismatch");
  if(dut.valid_b && (dut.raw_b!==hub.yaw_raw_alt || dut.angle_b!==hub.yaw_cdeg_alt))$fatal(1,"115200 observer mismatch");
 end
 reg [7:0] p[0:63],b;integer index=0,size=0,frames=0;reg [15:0] crc;
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] x);
  reg [15:0] v;integer k;begin v=c^{x,8'd0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 initial begin
  wait(rst);
  forever begin
   @(negedge tx);cycles(4);if(tx!==0)$fatal(1,"start bit");
   for(integer j=0;j<8;j=j+1)begin cycles(9);b[j]=tx;end
   cycles(9);if(tx!==1)$fatal(1,"stop bit");p[index]=b;
   if(index==4)begin size=b+7;if(size!=49)$fatal(1,"length");end
   if(index>=4 && index==size-1)begin
    crc=65535;for(integer j=2;j<size-2;j=j+1)crc=crc_byte(crc,p[j]);
    if(p[0]!=8'ha5 || p[1]!=8'h5a || p[2]!=1 || p[3]!=8'h2a || crc!={p[48],p[47]})$fatal(1,"frame CRC/header");
    frames=frames+1;index=0;
   end else index=index+1;
  end
 end
 task snap;begin @(negedge clk);snapshot=1;cycles(1);@(negedge clk);snapshot=0;cycles(1);end endtask
 task send;begin @(negedge clk);trigger=1;cycles(1);@(negedge clk);trigger=0;end endtask
 integer ca,cb,ea;reg signed [15:0] fy;reg [15:0] aa,ab;reg [31:0] fc;
 initial begin
  cycles(5);rst=1;cycles(10);
  if(dut.age_a!=65535 || dut.age_b!=65535 || dut.seen_a || dut.seen_b)$fatal(1,"initial validity");
  angle_frame(-16'sd16384,104,0);
  if(dut.count_a!=1 || dut.raw_a!=-16384 || dut.angle_a!=-9000 || filtered!=-9000 || dut.last_alt)$fatal(1,"9600 -90 conversion");
  ca=dut.count_a;ea=dut.errors_a;
  angle_frame(16'sd8192,104,1);
  if(dut.count_a!=ca || dut.errors_a<=ea || dut.raw_a!=-16384)$fatal(1,"bad checksum changed angle");
  cycles(510000);if(dut.age_a<500 || dut.flags[6])$fatal(1,"stale age");
  angle_frame(16'sd16384,9,0);
  if(dut.count_b!=1 || dut.raw_b!=16384 || dut.angle_b!=9000 || !dut.last_alt || dut.switches!=1)$fatal(1,"baud switch");
  for(integer j=0;j<40;j=j+1)angle_frame(16'sd32700,9,0);
  if(filtered<17955 || filtered>17962)$fatal(1,"positive endpoint convergence %0d",filtered);
  for(integer j=0;j<40;j=j+1)begin
   angle_frame(-16'sd32700,9,0);
   if(filtered>-17000 && filtered<17000)$fatal(1,"wrap took long path %0d",filtered);
  end
  if(filtered< -17966 || filtered> -17963)$fatal(1,"negative endpoint convergence");
  angle_frame(16'sd8192,104,0);
  if(dut.raw_a!=8192 || dut.angle_a!=4500 || dut.last_alt || dut.switches!=2)$fatal(1,"return to 9600");
  ca=dut.count_a;cb=dut.count_b;fy=filtered;aa=dut.age_a;ab=dut.age_b;fc=dut.filter_count;
  snap();
  // Change live input after snapshot: packet must retain prior measurements.
  angle_frame(16'sd0,104,0);send();cycles(30);active=0;
  wait(frames==1);cycles(20);
  if({p[6],p[5]}!=0 || $signed({p[10],p[9]})!=8192 || $signed({p[12],p[11]})!=4500 ||
     $signed({p[14],p[13]})!=-32700 || $signed({p[16],p[15]})!=-17963 ||
     $signed({p[18],p[17]})!=fy || $signed({p[20],p[19]})!=-3377)$fatal(1,"snapshot signed fields");
  if({p[24],p[23],p[22],p[21]}!=ca || {p[28],p[27],p[26],p[25]}!=cb ||
     {p[34],p[33]}!=aa || {p[36],p[35]}!=ab || {p[38],p[37]}!=2 ||
     {p[44],p[43],p[42],p[41]}!=fc || $signed({p[46],p[45]})!=4500)$fatal(1,"snapshot counters/ages/source");
  snap();send();cycles(6000);if(frames!=1 || busy)$fatal(1,"inactive packet sent");
  active=1;snap();send();wait(frames==2);cycles(20);if({p[6],p[5]}!=2)$fatal(1,"suppressed sequence not advanced");
  $display("PASS IMU real UART both bauds, checksum rejection, matching baseline observer, signed conversion, +/-180 short path, age/source, atomic 0x2A CRC and inactive/in-flight gating");$finish;
 end
 initial begin #30000000;$fatal(1,"IMU test timeout");end
endmodule
