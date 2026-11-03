`timescale 1ns/1ps

module dual_motor_control #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer PWM_FREQ = 20_000,
    parameter integer COUNTS_PER_REV = 1456,
    parameter integer SAMPLE_TIME_MS = 10,

    parameter integer KP = 100,
    parameter integer KI = 10,
    parameter integer SCALE = 1000,
    parameter integer OUT_MAX = 1000,
    parameter integer I_MAX = 120000,
    parameter integer DEAD_ZONE = 0,
    parameter integer DUTY_MAX = 1000,
    parameter integer FAST_PI = 0
)(
    input wire clk,
    input wire rst_n,

    // 左右轮目标转速
    input wire signed [31:0] left_target_rpm,
    input wire signed [31:0] right_target_rpm,

    // 左右轮方向
    input wire left_dir,
    input wire right_dir,

    // 左右编码器
    input wire left_enc_a,
    input wire left_enc_b,
    input wire right_enc_a,
    input wire right_enc_b,

    // 左轮电机控制
    output wire left_pwm,
    output wire left_in1,
    output wire left_in2,

    // 右轮电机控制
    output wire right_pwm,
    output wire right_in1,
    output wire right_in2,

    // 左右轮反馈
    output wire signed [31:0] left_encoder_count,
    output wire signed [31:0] right_encoder_count,

    output wire signed [31:0] left_actual_rpm,
    output wire signed [31:0] right_actual_rpm,

    // 左右轮控制量
    output wire [9:0] left_duty,
    output wire [9:0] right_duty,

    output wire signed [31:0] left_error,
    output wire signed [31:0] right_error
);

    // 左轮
    single_motor_control #(
        .CLK_FREQ(CLK_FREQ),
        .PWM_FREQ(PWM_FREQ),
        .COUNTS_PER_REV(COUNTS_PER_REV),
        .SAMPLE_TIME_MS(SAMPLE_TIME_MS),
        .KP(KP),
        .KI(KI),
        .SCALE(SCALE),
        .OUT_MAX(OUT_MAX),
        .I_MAX(I_MAX),
        .DEAD_ZONE(DEAD_ZONE),
        .DUTY_MAX(DUTY_MAX), .FAST_PI(FAST_PI)
    ) u_left_motor (
        .clk(clk),
        .rst_n(rst_n),

        .target_rpm(left_target_rpm),
        .dir(left_dir),

        .enc_a(left_enc_a),
        .enc_b(left_enc_b),

        .pwm_out(left_pwm),
        .in1(left_in1),
        .in2(left_in2),

        .encoder_count(left_encoder_count),
        .actual_rpm(left_actual_rpm),

        .duty(left_duty),
        .duty_pi(),
        .error(left_error),
        .integral()
    );

    // 右轮
    single_motor_control #(
        .CLK_FREQ(CLK_FREQ),
        .PWM_FREQ(PWM_FREQ),
        .COUNTS_PER_REV(COUNTS_PER_REV),
        .SAMPLE_TIME_MS(SAMPLE_TIME_MS),
        .KP(KP),
        .KI(KI),
        .SCALE(SCALE),
        .OUT_MAX(OUT_MAX),
        .I_MAX(I_MAX),
        .DEAD_ZONE(DEAD_ZONE),
        .DUTY_MAX(DUTY_MAX), .FAST_PI(FAST_PI)
    ) u_right_motor (
        .clk(clk),
        .rst_n(rst_n),

        .target_rpm(right_target_rpm),
        .dir(right_dir),

        .enc_a(right_enc_a),
        .enc_b(right_enc_b),

        .pwm_out(right_pwm),
        .in1(right_in1),
        .in2(right_in2),

        .encoder_count(right_encoder_count),
        .actual_rpm(right_actual_rpm),

        .duty(right_duty),
        .duty_pi(),
        .error(right_error),
        .integral()
    );

endmodule
