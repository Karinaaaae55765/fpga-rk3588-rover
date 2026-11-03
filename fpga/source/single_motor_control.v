`timescale 1ns/1ps

module single_motor_control #(
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
    //============================================================
    // 目标转速
    //============================================================
    input wire signed [31:0] target_rpm,
    //============================================================
    // 电机方向
    //============================================================
    input wire dir,
    //============================================================
    // 编码器输入
    //============================================================
    input wire enc_a,
    input wire enc_b,
    //============================================================
    // 电机驱动输出
    //============================================================
    output wire pwm_out,
    output wire in1,
    output wire in2,
    //============================================================
    // 调试/观察信号
    //============================================================
    output wire signed [31:0] encoder_count,
    output wire signed [31:0] actual_rpm,
    output wire [9:0] duty,
    output wire [9:0] duty_pi,
    output wire signed [31:0] error,
    output wire signed [31:0] integral
);
    //============================================================
    // 1. 编码器计数
    //============================================================
    wire direction_feedback;
    quadrature_encoder #(
        .COUNT_WIDTH(32)
    ) u_encoder (
        .clk       (clk),
        .rst_n     (rst_n),
        .enc_a     (enc_a),
        .enc_b     (enc_b),
        .direction (direction_feedback),
        .count     (encoder_count)
    );
    //============================================================
    // 2. 速度测量
    //============================================================
    wire speed_update_en;
    // target_rpm is a speed magnitude; direction is carried separately by
    // dir.  Encoder polarity therefore must not turn reverse motion into a
    // permanently positive PI error.
    reg signed [31:0] actual_rpm_magnitude;
    speed_measure #(
        .CLK_FREQ        (CLK_FREQ),
        .SAMPLE_TIME_MS  (SAMPLE_TIME_MS),
        .COUNTS_PER_REV  (COUNTS_PER_REV)
    ) u_speed_measure (
        .clk       (clk),
        .rst_n     (rst_n),
        .count     (encoder_count),
        .rpm       (actual_rpm),
        .update_en (speed_update_en)
    );
    //============================================================
    // 3. PI更新脉冲延迟
    //
    // Edge N: speed_measure publishes RPM and its update strobe.
    // Edge N+1: capture the magnitude and delay the update strobe.
    // Edge N+2: PI consumes that registered magnitude. This preserves the
    // existing PI update cadence and breaks the absolute-value -> PI path.
    //============================================================
    reg pi_update_en;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pi_update_en <= 1'b0;
            actual_rpm_magnitude <= 32'sd0;
        end
        else begin
            pi_update_en <= speed_update_en;
            if (speed_update_en)
                actual_rpm_magnitude <= actual_rpm[31] ? -actual_rpm : actual_rpm;
        end
    end
    //============================================================
    // 4. PI速度控制
    //============================================================
    generate if (FAST_PI) begin : g_fast_pi
    PI_controller_fast #(
        .OUT_MAX(OUT_MAX), .I_MAX(I_MAX), .DEAD_ZONE(DEAD_ZONE),
        .DUTY_MAX(DUTY_MAX), .I_SHIFT(6)
    ) u_pi (
        .clk(clk), .rst_n(rst_n), .target_rpm(target_rpm),
        .actual_rpm(actual_rpm_magnitude), .update_en(pi_update_en),
        .duty_pi(duty_pi), .duty(duty), .error(error), .integral(integral)
    );
    end else begin : g_generic_pi
    PI_controller #(
        .KP        (KP),
        .KI        (KI),
        .SCALE     (SCALE),
        .OUT_MAX   (OUT_MAX),
        .I_MAX     (I_MAX),
        .DEAD_ZONE (DEAD_ZONE),
        .DUTY_MAX  (DUTY_MAX)
    ) u_pi (
        .clk        (clk),
        .rst_n      (rst_n),
        .target_rpm (target_rpm),
        .actual_rpm (actual_rpm_magnitude),
        .update_en  (pi_update_en),
        .duty_pi    (duty_pi),
        .duty       (duty),
        .error      (error),
        .integral   (integral)
    );
    end endgenerate
    //============================================================
    // 5. PWM + DIR + 电机驱动
    //============================================================
    motor_control #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ)
    ) u_motor_control (
        .clk     (clk),
        .rst_n   (rst_n),
        .duty    (duty),
        .dir     (dir),
        .pwm_out (pwm_out),
        .in1     (in1),
        .in2     (in2)
    );
endmodule
