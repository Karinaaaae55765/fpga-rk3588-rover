// Reused SCCB/configuration from frozen clean_live_20261016_214106.
// Full sensor attribution and license: ../../README.md.

// BEGIN sccb_master.sv
`default_nettype none
// Single-master OV5640 SCCB, 16-bit register addresses and 8-bit data.
// SDA is open drain; SCL is push-pull (this interface has no clock stretching).
// error 1=NACK, 2=SDA not released. Request is accepted only while !busy.
module sccb_master #(
    parameter CLOCK_HZ=50000000, parameter BUS_HZ=100000,
    parameter IDLE_TIMEOUT_TICKS=400
)(input wire clk,reset_n,request,read_not_write,
  input wire [15:0] register_address,input wire [7:0] write_data,
  output reg busy,done,output reg [7:0] read_data,error,
  output reg scl,inout wire sda);
    localparam Q=(CLOCK_HZ+4*BUS_HZ-1)/(4*BUS_HZ);
    localparam CW=(Q<2)?1:$clog2(Q+1);
    localparam IDLE=0,CHECK=1,START=2,SEND=3,ACK=4,NEXT=5,
               RESTART=6,RECEIVE=7,MASTER_NACK=8,STOP=9;
    reg [3:0] state;
    reg [1:0] phase,byte_number;
    reg [2:0] bit_number;
    reg [7:0] tx;
    reg [15:0] saved_address;
    reg [7:0] saved_data;
    reg saved_read,sda_low;
    reg [CW-1:0] divider;
    reg [15:0] idle_ticks;
    (* ASYNC_REG="TRUE" *) reg sda_meta,sda_sample;
    assign sda=sda_low?1'b0:1'bz;
    always @(posedge clk or negedge reset_n)
        if(!reset_n) begin sda_meta<=1;sda_sample<=1;end
        else begin sda_meta<=sda;sda_sample<=sda_meta;end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;phase<=0;byte_number<=0;bit_number<=7;tx<=0;
            saved_address<=0;saved_data<=0;saved_read<=0;sda_low<=0;
            divider<=0;idle_ticks<=0;busy<=0;done<=0;read_data<=0;error<=0;scl<=1;
        end else begin
            done<=0;
            if(state==IDLE) begin
                scl<=1;sda_low<=0;divider<=0;
                if(request) begin
                    saved_address<=register_address;saved_data<=write_data;
                    saved_read<=read_not_write;read_data<=0;error<=0;
                    busy<=1;idle_ticks<=0;phase<=0;state<=CHECK;
                end
            end else if(divider==Q-1) begin
                divider<=0;
                case(state)
                    CHECK: begin
                        if(sda_sample) begin state<=START;phase<=0;end
                        else if(idle_ticks==IDLE_TIMEOUT_TICKS-1) begin
                            error<=2;busy<=0;done<=1;state<=IDLE;
                        end else idle_ticks<=idle_ticks+1'b1;
                    end
                    START: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=1;sda_low<=0;end
                            1:sda_low<=1;
                            2:scl<=0;
                            3:begin tx<=8'h78;byte_number<=0;bit_number<=7;state<=SEND;end
                        endcase
                    end
                    SEND: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=!tx[bit_number];end
                            1:scl<=1;
                            2:;
                            3:begin
                                scl<=0;
                                if(bit_number==0) state<=ACK;
                                else bit_number<=bit_number-1'b1;
                            end
                        endcase
                    end
                    ACK: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=0;end
                            1:scl<=1;
                            2:if(sda_sample) error<=1;
                            3:begin scl<=0;if(error!=0)state<=STOP;else state<=NEXT;end
                        endcase
                    end
                    NEXT: begin
                        phase<=0;bit_number<=7;
                        byte_number<=byte_number+1'b1;
                        case(byte_number)
                            0:begin tx<=saved_address[15:8];state<=SEND;end
                            1:begin tx<=saved_address[7:0];state<=SEND;end
                            2:if(saved_read)state<=RESTART;
                              else begin tx<=saved_data;state<=SEND;end
                            3:if(saved_read)state<=RECEIVE;else state<=STOP;
                        endcase
                    end
                    RESTART: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=0;end
                            1:scl<=1;
                            2:sda_low<=1;
                            3:begin scl<=0;tx<=8'h79;state<=SEND;end
                        endcase
                    end
                    RECEIVE: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=0;end
                            1:scl<=1;
                            2:read_data[bit_number]<=sda_sample;
                            3:begin
                                scl<=0;
                                if(bit_number==0)state<=MASTER_NACK;
                                else bit_number<=bit_number-1'b1;
                            end
                        endcase
                    end
                    MASTER_NACK: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=0;end
                            1:scl<=1;
                            2:;
                            3:begin scl<=0;state<=STOP;end
                        endcase
                    end
                    STOP: begin
                        phase<=phase+1'b1;
                        case(phase)
                            0:begin scl<=0;sda_low<=1;end
                            1:scl<=1;
                            2:sda_low<=0;
                            3:begin
                                if(!sda_sample && error==0)error<=2;
                                busy<=0;done<=1;state<=IDLE;
                            end
                        endcase
                    end
                    default:begin error<=2;state<=STOP;phase<=0;end
                endcase
            end else divider<=divider+1'b1;
        end
    end
endmodule
`default_nettype wire

// BEGIN ov5640_profile.sv
`default_nettype none
// Register values include an adapted STMicroelectronics baseline.
// Copyright (c) 2019-2020 STMicroelectronics. BSD-3-Clause.
// Attribution, full license and clock derivation: ../../README.md.
// Newly organized sequencing, geometry, clock and readback policy.
module ov5640_profile #(parameter SENSOR_BARS=0)(
    input wire [8:0] index,output reg [15:0] address,
    output reg [7:0] value,verify_mask,output reg reset_delay,finished);
    always @* begin
        address=0;value=0;verify_mask=0;reset_delay=0;finished=0;
        case(index)
            // Reset and stand-by
            9'd0:begin address=16'h3103;value=8'h11;verify_mask=8'h00;end
            9'd1:begin address=16'h3008;value=8'h82;verify_mask=8'h00; reset_delay=1;end
            9'd2:begin address=16'h3008;value=8'h42;verify_mask=8'h40;end
            9'd3:begin address=16'h3103;value=8'h03;verify_mask=8'h00;end
            // 24 MHz XCLK: VCO=672 MHz, byte PCLK=42 MHz, ISP=21 MHz
            9'd4:begin address=16'h3034;value=8'h18;verify_mask=8'h0f;end
            9'd5:begin address=16'h3035;value=8'h40;verify_mask=8'hff;end
            9'd6:begin address=16'h3036;value=8'h54;verify_mask=8'hff;end
            9'd7:begin address=16'h3037;value=8'h13;verify_mask=8'h1f;end
            9'd8:begin address=16'h3108;value=8'h01;verify_mask=8'h3f;end
            // Pads, DVP, disabled JPEG block
            9'd9:begin address=16'h3017;value=8'h7f;verify_mask=8'h7f;end
            9'd10:begin address=16'h3018;value=8'hfc;verify_mask=8'hfc;end
            9'd11:begin address=16'h300e;value=8'h18;verify_mask=8'h1c;end
            9'd12:begin address=16'h302e;value=8'h00;verify_mask=8'h00;end
            9'd13:begin address=16'h3000;value=8'h00;verify_mask=8'h00;end
            9'd14:begin address=16'h3004;value=8'hff;verify_mask=8'h00;end
            9'd15:begin address=16'h3002;value=8'h1c;verify_mask=8'h00;end
            9'd16:begin address=16'h3006;value=8'hc3;verify_mask=8'h00;end
            // Analog sensor baseline from STMicroelectronics BSD-3-Clause driver
            9'd17:begin address=16'h3630;value=8'h36;verify_mask=8'h00;end
            9'd18:begin address=16'h3631;value=8'h0e;verify_mask=8'h00;end
            9'd19:begin address=16'h3632;value=8'he2;verify_mask=8'h00;end
            9'd20:begin address=16'h3633;value=8'h12;verify_mask=8'h00;end
            9'd21:begin address=16'h3621;value=8'he0;verify_mask=8'h00;end
            9'd22:begin address=16'h3704;value=8'ha0;verify_mask=8'h00;end
            9'd23:begin address=16'h3703;value=8'h5a;verify_mask=8'h00;end
            9'd24:begin address=16'h3715;value=8'h78;verify_mask=8'h00;end
            9'd25:begin address=16'h3717;value=8'h01;verify_mask=8'h00;end
            9'd26:begin address=16'h370b;value=8'h60;verify_mask=8'h00;end
            9'd27:begin address=16'h3705;value=8'h1a;verify_mask=8'h00;end
            9'd28:begin address=16'h3905;value=8'h02;verify_mask=8'h00;end
            9'd29:begin address=16'h3906;value=8'h10;verify_mask=8'h00;end
            9'd30:begin address=16'h3901;value=8'h0a;verify_mask=8'h00;end
            9'd31:begin address=16'h3731;value=8'h12;verify_mask=8'h00;end
            9'd32:begin address=16'h3600;value=8'h08;verify_mask=8'h00;end
            9'd33:begin address=16'h3601;value=8'h33;verify_mask=8'h00;end
            9'd34:begin address=16'h302d;value=8'h60;verify_mask=8'h00;end
            9'd35:begin address=16'h3620;value=8'h52;verify_mask=8'h00;end
            9'd36:begin address=16'h371b;value=8'h20;verify_mask=8'h00;end
            9'd37:begin address=16'h3635;value=8'h13;verify_mask=8'h00;end
            9'd38:begin address=16'h3636;value=8'h03;verify_mask=8'h00;end
            9'd39:begin address=16'h3634;value=8'h40;verify_mask=8'h00;end
            9'd40:begin address=16'h3622;value=8'h01;verify_mask=8'h00;end
            9'd41:begin address=16'h3618;value=8'h00;verify_mask=8'h00;end
            9'd42:begin address=16'h3612;value=8'h29;verify_mask=8'h00;end
            9'd43:begin address=16'h3708;value=8'h64;verify_mask=8'h00;end
            9'd44:begin address=16'h3709;value=8'h52;verify_mask=8'h00;end
            9'd45:begin address=16'h370c;value=8'h03;verify_mask=8'h00;end
            // 1280x720 crop, HTS=1892, VTS=740, 2:1 sub-sampling
            9'd46:begin address=16'h3800;value=8'h00;verify_mask=8'h00;end
            9'd47:begin address=16'h3801;value=8'h00;verify_mask=8'h00;end
            9'd48:begin address=16'h3802;value=8'h00;verify_mask=8'h00;end
            9'd49:begin address=16'h3803;value=8'hfa;verify_mask=8'h00;end
            9'd50:begin address=16'h3804;value=8'h0a;verify_mask=8'h00;end
            9'd51:begin address=16'h3805;value=8'h3f;verify_mask=8'h00;end
            9'd52:begin address=16'h3806;value=8'h06;verify_mask=8'h00;end
            9'd53:begin address=16'h3807;value=8'ha9;verify_mask=8'h00;end
            9'd54:begin address=16'h3808;value=8'h05;verify_mask=8'hff;end
            9'd55:begin address=16'h3809;value=8'h00;verify_mask=8'hff;end
            9'd56:begin address=16'h380a;value=8'h02;verify_mask=8'hff;end
            9'd57:begin address=16'h380b;value=8'hd0;verify_mask=8'hff;end
            9'd58:begin address=16'h380c;value=8'h07;verify_mask=8'hff;end
            9'd59:begin address=16'h380d;value=8'h64;verify_mask=8'hff;end
            9'd60:begin address=16'h380e;value=8'h02;verify_mask=8'hff;end
            9'd61:begin address=16'h380f;value=8'he4;verify_mask=8'hff;end
            9'd62:begin address=16'h3810;value=8'h00;verify_mask=8'hff;end
            9'd63:begin address=16'h3811;value=8'h10;verify_mask=8'hff;end
            9'd64:begin address=16'h3812;value=8'h00;verify_mask=8'hff;end
            9'd65:begin address=16'h3813;value=8'h04;verify_mask=8'hff;end
            9'd66:begin address=16'h3814;value=8'h31;verify_mask=8'hff;end
            9'd67:begin address=16'h3815;value=8'h31;verify_mask=8'hff;end
            9'd68:begin address=16'h3820;value=8'h41;verify_mask=8'h07;end
            9'd69:begin address=16'h3821;value=8'h07;verify_mask=8'h07;end
            // RGB565 big-endian and free-running PCLK. HREF polarity is chosen
            // from this board's observed timing: 0x22 captured 1224 blanking
            // bytes and 721 segments, not 2560 active bytes and 720 lines.
            // Change only bit 1 (0x22 -> 0x20); do not also invert the RTL input.
            9'd70:begin address=16'h4300;value=8'h61;verify_mask=8'hff;end
            9'd71:begin address=16'h501f;value=8'h01;verify_mask=8'h07;end
            9'd72:begin address=16'h460b;value=8'h35;verify_mask=8'h00;end
            9'd73:begin address=16'h460c;value=8'h22;verify_mask=8'h02;end
            9'd74:begin address=16'h3824;value=8'h01;verify_mask=8'h1f;end
            9'd75:begin address=16'h471b;value=8'h02;verify_mask=8'h01;end
            9'd76:begin address=16'h471d;value=8'h00;verify_mask=8'h03;end
            9'd77:begin address=16'h4730;value=8'h00;verify_mask=8'h01;end
            9'd78:begin address=16'h4740;value=8'h20;verify_mask=8'h2f;end
            9'd79:begin address=16'h4741;value=8'h00;verify_mask=8'h07;end
            9'd80:begin address=16'h4745;value=8'h00;verify_mask=8'h07;end
            9'd81:begin address=16'h440e;value=8'h00;verify_mask=8'h00;end
            // Automatic exposure and gain; 100/120Hz band steps from 21MHz / HTS
            9'd82:begin address=16'h3a00;value=8'h78;verify_mask=8'h00;end
            9'd83:begin address=16'h3a13;value=8'h43;verify_mask=8'h00;end
            9'd84:begin address=16'h3a18;value=8'h00;verify_mask=8'h00;end
            9'd85:begin address=16'h3a19;value=8'hf8;verify_mask=8'h00;end
            9'd86:begin address=16'h3a02;value=8'h02;verify_mask=8'h00;end
            9'd87:begin address=16'h3a03;value=8'he0;verify_mask=8'h00;end
            9'd88:begin address=16'h3a14;value=8'h02;verify_mask=8'h00;end
            9'd89:begin address=16'h3a15;value=8'he0;verify_mask=8'h00;end
            9'd90:begin address=16'h3a08;value=8'h00;verify_mask=8'h00;end
            9'd91:begin address=16'h3a09;value=8'h6e;verify_mask=8'h00;end
            9'd92:begin address=16'h3a0a;value=8'h00;verify_mask=8'h00;end
            9'd93:begin address=16'h3a0b;value=8'h5c;verify_mask=8'h00;end
            9'd94:begin address=16'h3a0d;value=8'h08;verify_mask=8'h00;end
            9'd95:begin address=16'h3a0e;value=8'h06;verify_mask=8'h00;end
            9'd96:begin address=16'h3a0f;value=8'h30;verify_mask=8'h00;end
            9'd97:begin address=16'h3a10;value=8'h28;verify_mask=8'h00;end
            9'd98:begin address=16'h3a1b;value=8'h30;verify_mask=8'h00;end
            9'd99:begin address=16'h3a1e;value=8'h26;verify_mask=8'h00;end
            9'd100:begin address=16'h3a11;value=8'h60;verify_mask=8'h00;end
            9'd101:begin address=16'h3a1f;value=8'h14;verify_mask=8'h00;end
            9'd102:begin address=16'h3c01;value=8'h34;verify_mask=8'h00;end
            9'd103:begin address=16'h3c04;value=8'h28;verify_mask=8'h00;end
            9'd104:begin address=16'h3c05;value=8'h98;verify_mask=8'h00;end
            9'd105:begin address=16'h3c06;value=8'h00;verify_mask=8'h00;end
            9'd106:begin address=16'h3c07;value=8'h07;verify_mask=8'h00;end
            9'd107:begin address=16'h3c08;value=8'h00;verify_mask=8'h00;end
            9'd108:begin address=16'h3c09;value=8'h1c;verify_mask=8'h00;end
            9'd109:begin address=16'h3c0a;value=8'h9c;verify_mask=8'h00;end
            9'd110:begin address=16'h3c0b;value=8'h40;verify_mask=8'h00;end
            9'd111:begin address=16'h3503;value=8'h00;verify_mask=8'h03;end
            // Black level, ISP processing, automatic white balance
            9'd112:begin address=16'h4001;value=8'h02;verify_mask=8'h00;end
            9'd113:begin address=16'h4004;value=8'h02;verify_mask=8'h00;end
            9'd114:begin address=16'h4005;value=8'h1a;verify_mask=8'h00;end
            9'd115:begin address=16'h5000;value=8'ha7;verify_mask=8'h00;end
            9'd116:begin address=16'h5001;value=8'h83;verify_mask=8'h00;end
            9'd117:begin address=16'h3406;value=8'h00;verify_mask=8'h01;end
            9'd118:begin address=16'h5180;value=8'hff;verify_mask=8'h00;end
            9'd119:begin address=16'h5181;value=8'hf2;verify_mask=8'h00;end
            9'd120:begin address=16'h5182;value=8'h00;verify_mask=8'h00;end
            9'd121:begin address=16'h5183;value=8'h14;verify_mask=8'h00;end
            9'd122:begin address=16'h5184;value=8'h25;verify_mask=8'h00;end
            9'd123:begin address=16'h5185;value=8'h24;verify_mask=8'h00;end
            9'd124:begin address=16'h5186;value=8'h09;verify_mask=8'h00;end
            9'd125:begin address=16'h5187;value=8'h09;verify_mask=8'h00;end
            9'd126:begin address=16'h5188;value=8'h09;verify_mask=8'h00;end
            9'd127:begin address=16'h5189;value=8'h75;verify_mask=8'h00;end
            9'd128:begin address=16'h518a;value=8'h54;verify_mask=8'h00;end
            9'd129:begin address=16'h518b;value=8'he0;verify_mask=8'h00;end
            9'd130:begin address=16'h518c;value=8'hb2;verify_mask=8'h00;end
            9'd131:begin address=16'h518d;value=8'h42;verify_mask=8'h00;end
            9'd132:begin address=16'h518e;value=8'h3d;verify_mask=8'h00;end
            9'd133:begin address=16'h518f;value=8'h56;verify_mask=8'h00;end
            9'd134:begin address=16'h5190;value=8'h46;verify_mask=8'h00;end
            9'd135:begin address=16'h5191;value=8'hf8;verify_mask=8'h00;end
            9'd136:begin address=16'h5192;value=8'h04;verify_mask=8'h00;end
            9'd137:begin address=16'h5193;value=8'h70;verify_mask=8'h00;end
            9'd138:begin address=16'h5194;value=8'hf0;verify_mask=8'h00;end
            9'd139:begin address=16'h5195;value=8'hf0;verify_mask=8'h00;end
            9'd140:begin address=16'h5196;value=8'h03;verify_mask=8'h00;end
            9'd141:begin address=16'h5197;value=8'h01;verify_mask=8'h00;end
            9'd142:begin address=16'h5198;value=8'h04;verify_mask=8'h00;end
            9'd143:begin address=16'h5199;value=8'h12;verify_mask=8'h00;end
            9'd144:begin address=16'h519a;value=8'h04;verify_mask=8'h00;end
            9'd145:begin address=16'h519b;value=8'h00;verify_mask=8'h00;end
            9'd146:begin address=16'h519c;value=8'h06;verify_mask=8'h00;end
            9'd147:begin address=16'h519d;value=8'h82;verify_mask=8'h00;end
            9'd148:begin address=16'h519e;value=8'h38;verify_mask=8'h00;end
            // Color matrix, gamma and moderate sharpening (ST baseline)
            9'd149:begin address=16'h5381;value=8'h1e;verify_mask=8'h00;end
            9'd150:begin address=16'h5382;value=8'h5b;verify_mask=8'h00;end
            9'd151:begin address=16'h5383;value=8'h08;verify_mask=8'h00;end
            9'd152:begin address=16'h5384;value=8'h0a;verify_mask=8'h00;end
            9'd153:begin address=16'h5385;value=8'h7e;verify_mask=8'h00;end
            9'd154:begin address=16'h5386;value=8'h88;verify_mask=8'h00;end
            9'd155:begin address=16'h5387;value=8'h7c;verify_mask=8'h00;end
            9'd156:begin address=16'h5388;value=8'h6c;verify_mask=8'h00;end
            9'd157:begin address=16'h5389;value=8'h10;verify_mask=8'h00;end
            9'd158:begin address=16'h538a;value=8'h01;verify_mask=8'h00;end
            9'd159:begin address=16'h538b;value=8'h98;verify_mask=8'h00;end
            9'd160:begin address=16'h5480;value=8'h01;verify_mask=8'h00;end
            9'd161:begin address=16'h5481;value=8'h08;verify_mask=8'h00;end
            9'd162:begin address=16'h5482;value=8'h14;verify_mask=8'h00;end
            9'd163:begin address=16'h5483;value=8'h28;verify_mask=8'h00;end
            9'd164:begin address=16'h5484;value=8'h51;verify_mask=8'h00;end
            9'd165:begin address=16'h5485;value=8'h65;verify_mask=8'h00;end
            9'd166:begin address=16'h5486;value=8'h71;verify_mask=8'h00;end
            9'd167:begin address=16'h5487;value=8'h7d;verify_mask=8'h00;end
            9'd168:begin address=16'h5488;value=8'h87;verify_mask=8'h00;end
            9'd169:begin address=16'h5489;value=8'h91;verify_mask=8'h00;end
            9'd170:begin address=16'h548a;value=8'h9a;verify_mask=8'h00;end
            9'd171:begin address=16'h548b;value=8'haa;verify_mask=8'h00;end
            9'd172:begin address=16'h548c;value=8'hb8;verify_mask=8'h00;end
            9'd173:begin address=16'h548d;value=8'hcd;verify_mask=8'h00;end
            9'd174:begin address=16'h548e;value=8'hdd;verify_mask=8'h00;end
            9'd175:begin address=16'h548f;value=8'hea;verify_mask=8'h00;end
            9'd176:begin address=16'h5490;value=8'h1d;verify_mask=8'h00;end
            9'd177:begin address=16'h5300;value=8'h08;verify_mask=8'h00;end
            9'd178:begin address=16'h5301;value=8'h30;verify_mask=8'h00;end
            9'd179:begin address=16'h5302;value=8'h10;verify_mask=8'h00;end
            9'd180:begin address=16'h5303;value=8'h00;verify_mask=8'h00;end
            9'd181:begin address=16'h5304;value=8'h08;verify_mask=8'h00;end
            9'd182:begin address=16'h5305;value=8'h30;verify_mask=8'h00;end
            9'd183:begin address=16'h5306;value=8'h08;verify_mask=8'h00;end
            9'd184:begin address=16'h5307;value=8'h16;verify_mask=8'h00;end
            9'd185:begin address=16'h5309;value=8'h08;verify_mask=8'h00;end
            9'd186:begin address=16'h530a;value=8'h30;verify_mask=8'h00;end
            9'd187:begin address=16'h530b;value=8'h04;verify_mask=8'h00;end
            9'd188:begin address=16'h530c;value=8'h06;verify_mask=8'h00;end
            9'd189:begin address=16'h5580;value=8'h02;verify_mask=8'h00;end
            9'd190:begin address=16'h5583;value=8'h40;verify_mask=8'h00;end
            9'd191:begin address=16'h5584;value=8'h10;verify_mask=8'h00;end
            9'd192:begin address=16'h5589;value=8'h10;verify_mask=8'h00;end
            9'd193:begin address=16'h558a;value=8'h00;verify_mask=8'h00;end
            9'd194:begin address=16'h558b;value=8'hf8;verify_mask=8'h00;end
            // Lens correction baseline; module-specific optical calibration remains a hardware task
            9'd195:begin address=16'h5800;value=8'h23;verify_mask=8'h00;end
            9'd196:begin address=16'h5801;value=8'h14;verify_mask=8'h00;end
            9'd197:begin address=16'h5802;value=8'h0f;verify_mask=8'h00;end
            9'd198:begin address=16'h5803;value=8'h0f;verify_mask=8'h00;end
            9'd199:begin address=16'h5804;value=8'h12;verify_mask=8'h00;end
            9'd200:begin address=16'h5805;value=8'h26;verify_mask=8'h00;end
            9'd201:begin address=16'h5806;value=8'h0c;verify_mask=8'h00;end
            9'd202:begin address=16'h5807;value=8'h08;verify_mask=8'h00;end
            9'd203:begin address=16'h5808;value=8'h05;verify_mask=8'h00;end
            9'd204:begin address=16'h5809;value=8'h05;verify_mask=8'h00;end
            9'd205:begin address=16'h580a;value=8'h08;verify_mask=8'h00;end
            9'd206:begin address=16'h580b;value=8'h0d;verify_mask=8'h00;end
            9'd207:begin address=16'h580c;value=8'h08;verify_mask=8'h00;end
            9'd208:begin address=16'h580d;value=8'h03;verify_mask=8'h00;end
            9'd209:begin address=16'h580e;value=8'h00;verify_mask=8'h00;end
            9'd210:begin address=16'h580f;value=8'h00;verify_mask=8'h00;end
            9'd211:begin address=16'h5810;value=8'h03;verify_mask=8'h00;end
            9'd212:begin address=16'h5811;value=8'h09;verify_mask=8'h00;end
            9'd213:begin address=16'h5812;value=8'h07;verify_mask=8'h00;end
            9'd214:begin address=16'h5813;value=8'h03;verify_mask=8'h00;end
            9'd215:begin address=16'h5814;value=8'h00;verify_mask=8'h00;end
            9'd216:begin address=16'h5815;value=8'h01;verify_mask=8'h00;end
            9'd217:begin address=16'h5816;value=8'h03;verify_mask=8'h00;end
            9'd218:begin address=16'h5817;value=8'h08;verify_mask=8'h00;end
            9'd219:begin address=16'h5818;value=8'h0d;verify_mask=8'h00;end
            9'd220:begin address=16'h5819;value=8'h08;verify_mask=8'h00;end
            9'd221:begin address=16'h581a;value=8'h05;verify_mask=8'h00;end
            9'd222:begin address=16'h581b;value=8'h06;verify_mask=8'h00;end
            9'd223:begin address=16'h581c;value=8'h08;verify_mask=8'h00;end
            9'd224:begin address=16'h581d;value=8'h0e;verify_mask=8'h00;end
            9'd225:begin address=16'h581e;value=8'h29;verify_mask=8'h00;end
            9'd226:begin address=16'h581f;value=8'h17;verify_mask=8'h00;end
            9'd227:begin address=16'h5820;value=8'h11;verify_mask=8'h00;end
            9'd228:begin address=16'h5821;value=8'h11;verify_mask=8'h00;end
            9'd229:begin address=16'h5822;value=8'h15;verify_mask=8'h00;end
            9'd230:begin address=16'h5823;value=8'h28;verify_mask=8'h00;end
            9'd231:begin address=16'h5824;value=8'h46;verify_mask=8'h00;end
            9'd232:begin address=16'h5825;value=8'h26;verify_mask=8'h00;end
            9'd233:begin address=16'h5826;value=8'h08;verify_mask=8'h00;end
            9'd234:begin address=16'h5827;value=8'h26;verify_mask=8'h00;end
            9'd235:begin address=16'h5828;value=8'h64;verify_mask=8'h00;end
            9'd236:begin address=16'h5829;value=8'h26;verify_mask=8'h00;end
            9'd237:begin address=16'h582a;value=8'h24;verify_mask=8'h00;end
            9'd238:begin address=16'h582b;value=8'h22;verify_mask=8'h00;end
            9'd239:begin address=16'h582c;value=8'h24;verify_mask=8'h00;end
            9'd240:begin address=16'h582d;value=8'h24;verify_mask=8'h00;end
            9'd241:begin address=16'h582e;value=8'h06;verify_mask=8'h00;end
            9'd242:begin address=16'h582f;value=8'h22;verify_mask=8'h00;end
            9'd243:begin address=16'h5830;value=8'h40;verify_mask=8'h00;end
            9'd244:begin address=16'h5831;value=8'h42;verify_mask=8'h00;end
            9'd245:begin address=16'h5832;value=8'h24;verify_mask=8'h00;end
            9'd246:begin address=16'h5833;value=8'h26;verify_mask=8'h00;end
            9'd247:begin address=16'h5834;value=8'h24;verify_mask=8'h00;end
            9'd248:begin address=16'h5835;value=8'h22;verify_mask=8'h00;end
            9'd249:begin address=16'h5836;value=8'h22;verify_mask=8'h00;end
            9'd250:begin address=16'h5837;value=8'h26;verify_mask=8'h00;end
            9'd251:begin address=16'h5838;value=8'h44;verify_mask=8'h00;end
            9'd252:begin address=16'h5839;value=8'h24;verify_mask=8'h00;end
            9'd253:begin address=16'h583a;value=8'h26;verify_mask=8'h00;end
            9'd254:begin address=16'h583b;value=8'h28;verify_mask=8'h00;end
            9'd255:begin address=16'h583c;value=8'h42;verify_mask=8'h00;end
            9'd256:begin address=16'h583d;value=8'hce;verify_mask=8'h00;end
            // Test pattern selection and final stream enable
            9'd257:begin address=16'h503d;value=SENSOR_BARS ? 8'h80 : 8'h00;verify_mask=8'hff;end
            9'd258:begin address=16'h5025;value=8'h00;verify_mask=8'h00;end
            9'd259:begin address=16'h3008;value=8'h02;verify_mask=8'h40;end
            default:finished=1;
        endcase
    end
endmodule
`default_nettype wire


// BEGIN ov5640_setup.sv
`default_nettype none
// All delays are counted in the supplied reference clock. ready requires
// verified chip identity and every profile entry carrying a readback mask.
module ov5640_setup #(
    parameter CLOCK_HZ=50000000, parameter SENSOR_BARS=0,
    parameter RESET_CYCLES=CLOCK_HZ/500, parameter BOOT_CYCLES=CLOCK_HZ/50,
    parameter SOFT_CYCLES=CLOCK_HZ/100, parameter SETTLE_CYCLES=CLOCK_HZ/5,
    parameter MAX_ATTEMPTS=3
)(input wire clk,reset_n,
  output wire camera_reset_n,camera_pwdn,scl,inout wire sda,
  output reg ready,output reg [7:0] error_code,
  output reg [15:0] chip_id,failed_register);
    localparam HOLD=0,BOOT=1,ISSUE=2,WAIT=3,ADVANCE=4,DELAY=5,SETTLE=6,FINISHED=7,FAILED=8;
    reg [3:0] state;
    reg [31:0] delay_count;
    reg [8:0] index;
    reg [1:0] stage; // 0=ID high,1=ID low,2=profile write,3=profile readback
    reg [2:0] attempts;
    reg request;
    wire busy,done;
    wire [7:0] data_in,bus_error,profile_data,profile_mask;
    wire [15:0] profile_address;
    wire profile_end,profile_delay;
    wire operation_read=(stage!=2);
    wire [15:0] operation_address=(stage==0)?16'h300a:(stage==1)?16'h300b:profile_address;
    assign camera_reset_n=reset_n&&(state!=HOLD);
    assign camera_pwdn=1'b0;
    ov5640_profile #(.SENSOR_BARS(SENSOR_BARS)) profile(index,profile_address,
        profile_data,profile_mask,profile_delay,profile_end);
    sccb_master #(.CLOCK_HZ(CLOCK_HZ)) bus(clk,reset_n,request,operation_read,
        operation_address,profile_data,busy,done,data_in,bus_error,scl,sda);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=HOLD;delay_count<=0;index<=0;stage<=0;attempts<=0;
            request<=0;ready<=0;error_code<=0;chip_id<=0;failed_register<=0;
        end else begin
            request<=0;
            case(state)
                HOLD:if(delay_count==RESET_CYCLES-1)begin delay_count<=0;state<=BOOT;end
                     else delay_count<=delay_count+1'b1;
                BOOT:if(delay_count==BOOT_CYCLES-1)begin delay_count<=0;state<=ISSUE;end
                     else delay_count<=delay_count+1'b1;
                ISSUE:if(!busy)begin request<=1;state<=WAIT;end
                WAIT:if(done)begin
                    if(bus_error!=0)begin
                        if(attempts==MAX_ATTEMPTS-1)begin
                            error_code<=(bus_error==1)?8'h01:8'h02;
                            failed_register<=operation_address;state<=FAILED;
                        end else begin attempts<=attempts+1'b1;state<=ISSUE;end
                    end else if((stage==0&&data_in!=8'h56)||(stage==1&&data_in!=8'h40))begin
                        if(stage==0)chip_id[15:8]<=data_in;else chip_id[7:0]<=data_in;
                        if(attempts==MAX_ATTEMPTS-1)begin
                            error_code<=8'h03;failed_register<=operation_address;state<=FAILED;
                        end else begin attempts<=attempts+1'b1;state<=ISSUE;end
                    end else if(stage==3&&((data_in&profile_mask)!=(profile_data&profile_mask)))begin
                        if(attempts==MAX_ATTEMPTS-1)begin
                            error_code<=8'h04;failed_register<=operation_address;state<=FAILED;
                        end else begin
                            // Retry the write/read pair, rather than only rereading.
                            attempts<=attempts+1'b1;stage<=2;state<=ISSUE;
                        end
                    end else begin
                        case(stage)
                            0:begin chip_id[15:8]<=data_in;stage<=1;attempts<=0;state<=ISSUE;end
                            1:begin chip_id[7:0]<=data_in;stage<=2;attempts<=0;state<=ISSUE;end
                            2:if(profile_delay)begin delay_count<=0;state<=DELAY;end
                              else if(profile_mask!=0)begin stage<=3;state<=ISSUE;end
                              else state<=ADVANCE;
                            3:state<=ADVANCE;
                        endcase
                    end
                end
                ADVANCE:begin index<=index+1'b1;stage<=2;attempts<=0;state<=DELAY;delay_count<=32'hffffffff;end
                DELAY:if(delay_count==32'hffffffff)begin
                    if(profile_end)begin state<=SETTLE;delay_count<=0;end
                    else state<=ISSUE;
                end else if(delay_count==SOFT_CYCLES-1)state<=ADVANCE;
                else delay_count<=delay_count+1'b1;
                SETTLE:if(delay_count==SETTLE_CYCLES-1)begin ready<=1;state<=FINISHED;end
                       else delay_count<=delay_count+1'b1;
                FINISHED:;
                FAILED:ready<=0;
                default:begin ready<=0;error_code<=8'hff;state<=FAILED;end
            endcase
        end
    end
endmodule
`default_nettype wire

`default_nettype none
// Commit is a separate zero-payload packet emitted only at the END of a valid
// frame. Receiver must never display a frame without this final confirmation.
module camera_frontend #(parameter WIDTH=1280,HEIGHT=720,CHUNK=1280,WARMUP=4)(
 input wire clk,reset_n,configured,enabled,vsync,href,input wire [7:0] data,
 input wire slot_ready,output reg wr_en,wr_commit,
 output reg [10:0] wr_addr,output reg [7:0] wr_data,
 output reg [31:0] wr_frame,output reg [15:0] wr_index,wr_length,
 output reg wr_end,output reg [31:0] good_frames,bad_frames
);
 localparam PACKETS=WIDTH*HEIGHT*2/CHUNK;
 reg old_vs,old_href,started,bad;
 reg [15:0] warmup,row_bytes,rows,index,offset;
 reg [31:0] fid;
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin
   old_vs<=0;old_href<=0;started<=0;bad<=1;warmup<=0;
   row_bytes<=0;rows<=0;index<=0;offset<=0;fid<=0;
   wr_en<=0;wr_commit<=0;wr_addr<=0;wr_data<=0;wr_frame<=0;
   wr_index<=0;wr_length<=0;wr_end<=0;good_frames<=0;bad_frames<=0;
  end else begin
   old_vs<=vsync;old_href<=href;wr_en<=0;wr_commit<=0;
   if(!configured)begin started<=0;warmup<=0;bad<=1;end
   else if(vsync&&!old_vs)begin
    // Accept simultaneous HREF falling and VSYNC rising as end of last row.
    if(started&&!bad&&enabled&&slot_ready&&index==PACKETS&&offset==0&&
       !href&&((rows==HEIGHT&&!old_href)||
               (rows==HEIGHT-1&&old_href&&row_bytes==WIDTH*2)))begin
     wr_commit<=1;wr_frame<=fid;wr_index<=PACKETS;wr_length<=0;wr_end<=1;
     good_frames<=good_frames+1'b1;
    end else if(started)bad_frames<=bad_frames+1'b1;
    started<=0;row_bytes<=0;rows<=0;index<=0;offset<=0;fid<=fid+1'b1;
    bad<=1;
    if(warmup<WARMUP)warmup<=warmup+1'b1;
   end else if(!vsync&&old_vs)begin
    started<=warmup>=WARMUP&&enabled;
    bad<=!(warmup>=WARMUP&&enabled);
    row_bytes<=0;rows<=0;index<=0;offset<=0;
   end else if(started&&!vsync)begin
    if(!enabled)bad<=1;
    if(href)begin
     if(row_bytes!=16'hffff)row_bytes<=row_bytes+1'b1;
     if(row_bytes>=WIDTH*2||rows>=HEIGHT)bad<=1;
     if(!bad&&enabled)begin
      if(!slot_ready)bad<=1;
      else if(row_bytes<WIDTH*2&&rows<HEIGHT)begin
       wr_en<=1;wr_addr<=offset[10:0];wr_data<=data;
       if(offset==CHUNK-1)begin
        wr_commit<=1;wr_frame<=fid;wr_index<=index;wr_length<=CHUNK;wr_end<=0;
        offset<=0;index<=index+1'b1;
       end else offset<=offset+1'b1;
      end
     end
    end else if(old_href)begin
     if(row_bytes!=WIDTH*2)bad<=1;
     row_bytes<=0;
     if(rows!=16'hffff)rows<=rows+1'b1;
    end
   end
  end
endmodule
`default_nettype wire
