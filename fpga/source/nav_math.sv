`default_nettype none
// Iterative CORDIC, one shift/add per clock. angle in 0.01 degree;
// mode=0 rotation -> Q15 sin/cos; mode=1 vector -> atan2(y,x).
// Contract: rotation angle [-18000,18000], vector coordinates +/-60000.
module nav_math(
 input wire clk,rst_n,clear,start,mode,
 input wire signed [31:0] x_in,y_in,
 input wire signed [15:0] angle_in,
 output reg busy,done,
 output reg signed [15:0] bearing,
 output reg signed [31:0] cos_q15,sin_q15
);
 reg signed [39:0] x,y;
 reg signed [31:0] z;
 reg [4:0] step;
 reg vector_mode;
 function automatic signed [31:0] atan_step(input [4:0] i);
  case(i)
   0:atan_step=4500;1:atan_step=2657;2:atan_step=1404;
   3:atan_step=713;4:atan_step=358;5:atan_step=179;
   6:atan_step=90;7:atan_step=45;8:atan_step=22;
   9:atan_step=11;10:atan_step=6;11:atan_step=3;
   12:atan_step=1;default:atan_step=1;
  endcase
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin x<=0;y<=0;z<=0;step<=0;busy<=0;done<=0;
   vector_mode<=0;bearing<=0;cos_q15<=0;sin_q15<=0;end
  else if(clear)begin busy<=0;done<=0;end
  else begin
   done<=0;
   if(start && !busy)begin
    busy<=1;step<=0;vector_mode<=mode;
    if(mode)begin
     if(x_in<0)begin
      x<=-($signed({{8{x_in[31]}},x_in})<<<8);
      y<=-($signed({{8{y_in[31]}},y_in})<<<8);
      z<=(y_in<0)?-18000:18000;
     end else begin
      x<=$signed({{8{x_in[31]}},x_in})<<<8;
      y<=$signed({{8{y_in[31]}},y_in})<<<8;z<=0;
     end
    end else begin
     // Extra fractional bits avoid precision loss in successive shifts.
     y<=0;
     if(angle_in>9000)begin x<=-5094007;z<=angle_in-18000;end
     else if(angle_in< -9000)begin x<=-5094007;z<=angle_in+18000;end
     else begin x<=5094007;z<=angle_in;end
    end
   end else if(busy)begin
    if(step==14)begin
     busy<=0;done<=1;
     if(z>18000)bearing<=z-36000;
     else if(z< -18000)bearing<=z+36000;
     else bearing<=z;
     cos_q15<=x>>>8;sin_q15<=y>>>8;
    end else begin
     step<=step+1'b1;
     if(vector_mode)begin
      if(y>0)begin x<=x+(y>>>step);y<=y-(x>>>step);z<=z+atan_step(step);end
      else if(y<0)begin x<=x-(y>>>step);y<=y+(x>>>step);z<=z-atan_step(step);end
     end else if(z>=0)begin
      x<=x-(y>>>step);y<=y+(x>>>step);z<=z-atan_step(step);
     end else begin
      x<=x+(y>>>step);y<=y-(x>>>step);z<=z+atan_step(step);
     end
    end
   end
  end
 end
endmodule
`default_nettype wire
