`timescale 1ns/1ps

module motor_control #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer PWM_FREQ = 20_000
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [9:0] duty,
    input  wire       dir,

    output wire       pwm_out,
    output wire       in1,
    output wire       in2
);
    // PWM模块
    PWM #(
        .CLK_FREQ(CLK_FREQ),
        .PWM_FREQ(PWM_FREQ)
    ) u_pwm (
        .clk     (clk),
        .rst_n   (rst_n),
        .duty    (duty),
        .pwm_out (pwm_out)
    );
    // 方向模块
    DIR u_dir (
        .dir    (dir),
        .enable (duty != 10'd0),
        .in1    (in1),
        .in2    (in2)
    );
endmodule
