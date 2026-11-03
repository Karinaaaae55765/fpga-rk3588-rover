`timescale 1ns/1ps

// Timing-oriented PI controller for the car build.  P gain is 1 and I gain
// is 1/64, so the datapath is subtract/add/shift only.  It updates at the
// measured-speed strobe (20 ms in the full design), not every FPGA clock.
module PI_controller_fast #(
    parameter integer OUT_MAX=260,
    parameter integer I_MAX=200000,
    parameter integer DEAD_ZONE=120,
    parameter integer DUTY_MAX=360,
    parameter integer I_SHIFT=6
)(
    input wire clk,input wire rst_n,
    input wire signed [31:0] target_rpm,input wire signed [31:0] actual_rpm,
    input wire update_en,
    output reg [9:0] duty_pi,output reg [9:0] duty,
    output reg signed [31:0] error,output reg signed [31:0] integral
);
    reg signed [31:0] error_next,integral_candidate,integral_next;
    reg signed [32:0] control_next;
    reg [10:0] compensated;
    reg [9:0] duty_pi_next,duty_next;

    always @(*) begin
        error_next=target_rpm-actual_rpm;
        integral_candidate=integral+error_next;
        if(integral_candidate>I_MAX) integral_candidate=I_MAX;
        else if(integral_candidate< -I_MAX) integral_candidate=-I_MAX;
        integral_next=integral_candidate;
        control_next=error_next+(integral_candidate>>>I_SHIFT);
        duty_pi_next=0; duty_next=0; compensated=0;
        if(target_rpm==0) begin
            integral_next=0;
        end else if(control_next<=0) begin
            if(error_next<0) integral_next=integral;
        end else if(control_next>=OUT_MAX) begin
            duty_pi_next=OUT_MAX;
            compensated=OUT_MAX+DEAD_ZONE;
            duty_next=(compensated>=DUTY_MAX)?DUTY_MAX:compensated[9:0];
            if(error_next>0) integral_next=integral;
        end else begin
            duty_pi_next=control_next[9:0];
            compensated=control_next[9:0]+DEAD_ZONE;
            duty_next=(compensated>=DUTY_MAX)?DUTY_MAX:compensated[9:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin duty_pi<=0; duty<=0; error<=0; integral<=0; end
        else if(update_en) begin
            duty_pi<=duty_pi_next; duty<=duty_next;
            error<=error_next; integral<=integral_next;
        end
    end
endmodule
