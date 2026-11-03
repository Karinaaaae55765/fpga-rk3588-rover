`timescale 1ns/1ps

module PI_controller #(
    parameter integer KP         = 100,
    parameter integer KI         = 10,
    parameter integer SCALE      = 1000,

    // PI输出最大值
    parameter integer OUT_MAX    = 1000,

    // 积分限幅
    parameter integer I_MAX      = 120000,

    // 电机死区补偿
    parameter integer DEAD_ZONE  = 0,

    // PWM最终最大值
    parameter integer DUTY_MAX   = 1000
)(
    input wire clk,
    input wire rst_n,

    input wire signed [31:0] target_rpm,
    input wire signed [31:0] actual_rpm,

    // 例如10ms产生一次脉冲
    input wire update_en,

    // PI原始输出
    output reg [9:0] duty_pi,

    // 死区补偿后的最终PWM
    output reg [9:0] duty,

    // 调试信号
    output reg signed [31:0] error,
    output reg signed [31:0] integral
);

    // ==================================================
    // 中间变量
    // ==================================================

    reg signed [31:0] error_next;

    reg signed [31:0] integral_candidate;
    reg signed [31:0] integral_next;

    reg signed [63:0] p_term;
    reg signed [63:0] i_term;

    reg signed [63:0] control_candidate;
    reg signed [63:0] control_final;

    reg [9:0] duty_pi_candidate;
    reg [9:0] duty_pi_next;

    reg [10:0] duty_compensated_candidate;
    reg [10:0] duty_compensated_final;

    reg [9:0] duty_candidate;
    reg [9:0] duty_next;

    // 最终PWM是否饱和
    reg output_saturated;


    // ==================================================
    // 组合逻辑
    // ==================================================

    always @(*) begin

        // ------------------------------------------------
        // 默认值
        // ------------------------------------------------

        error_next = target_rpm - actual_rpm;

        integral_candidate = integral;
        integral_next = integral;

        p_term = 64'sd0;
        i_term = 64'sd0;

        control_candidate = 64'sd0;
        control_final = 64'sd0;

        duty_pi_candidate = 10'd0;
        duty_pi_next = 10'd0;

        duty_compensated_candidate = 11'd0;
        duty_compensated_final = 11'd0;

        duty_candidate = 10'd0;
        duty_next = 10'd0;

        output_saturated = 1'b0;


        // ==================================================
        // 情况1：目标速度为0
        // ==================================================

        if (target_rpm == 0) begin

            // 直接停止
            duty_pi_next = 10'd0;
            duty_next = 10'd0;

            // 清除积分
            integral_next = 32'sd0;

        end


        // ==================================================
        // 情况2：正常PI控制
        // ==================================================

        else begin

            // ------------------------------------------------
            // 1. 计算积分候选值
            // ------------------------------------------------

            integral_candidate = integral + error_next;


            // 积分限幅
            if (integral_candidate > I_MAX)

                integral_candidate = I_MAX;

            else if (integral_candidate < -I_MAX)

                integral_candidate = -I_MAX;


            // ------------------------------------------------
            // 2. P项
            // ------------------------------------------------

            p_term = error_next * KP;


            // ------------------------------------------------
            // 3. I项
            // ------------------------------------------------

            i_term = integral_candidate * KI;


            // ------------------------------------------------
            // 4. PI控制量
            // ------------------------------------------------

            control_candidate =
                (p_term + i_term) / SCALE;


            // ------------------------------------------------
            // 5. PI输出限幅
            // ------------------------------------------------

            if (control_candidate <= 0)

                duty_pi_candidate = 10'd0;

            else if (control_candidate >= OUT_MAX)

                duty_pi_candidate = OUT_MAX;

            else

                duty_pi_candidate =
                    control_candidate[9:0];


            // ------------------------------------------------
            // 6. 死区补偿
            // ------------------------------------------------

            if (duty_pi_candidate == 0) begin

                duty_compensated_candidate = 11'd0;

            end
            else begin

                duty_compensated_candidate =
                    duty_pi_candidate + DEAD_ZONE;

            end


            // ------------------------------------------------
            // 7. 最终PWM限幅
            // ------------------------------------------------

            if (duty_compensated_candidate >= DUTY_MAX) begin

                duty_candidate = DUTY_MAX;

                output_saturated = 1'b1;

            end
            else begin

                duty_candidate =
                    duty_compensated_candidate[9:0];

                output_saturated = 1'b0;

            end


            // ==================================================
            // 8. Anti-Windup
            // ==================================================

            /*
             * 最终PWM已经达到上限，
             * 而误差仍然要求增加PWM
             * → 冻结积分
             */

            if (output_saturated && (error_next > 0)) begin

                integral_next = integral;

            end

            /*
             * PI已经压到0，
             * 而误差还要求继续降低输出
             * → 冻结积分
             */

            else if ((duty_pi_candidate == 0) &&
                     (error_next < 0)) begin

                integral_next = integral;

            end

            else begin

                integral_next = integral_candidate;

            end


            // ==================================================
            // 9. 用最终积分重新计算PI
            // ==================================================

            i_term =
                integral_next * KI;

            control_final =
                (p_term + i_term) / SCALE;


            // ------------------------------------------------
            // PI最终输出
            // ------------------------------------------------

            if (control_final <= 0)

                duty_pi_next = 10'd0;

            else if (control_final >= OUT_MAX)

                duty_pi_next = OUT_MAX;

            else

                duty_pi_next =
                    control_final[9:0];


            // ------------------------------------------------
            // 最终死区补偿
            // ------------------------------------------------

            if (duty_pi_next == 0)

                duty_compensated_final = 11'd0;

            else

                duty_compensated_final =
                    duty_pi_next + DEAD_ZONE;


            // ------------------------------------------------
            // 最终限幅
            // ------------------------------------------------

            if (duty_compensated_final >= DUTY_MAX)

                duty_next = DUTY_MAX;

            else

                duty_next =
                    duty_compensated_final[9:0];

        end
    end


    // ==================================================
    // 时序逻辑
    // ==================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            duty     <= 10'd0;
            duty_pi  <= 10'd0;

            error    <= 32'sd0;
            integral <= 32'sd0;

        end

        else if (update_en) begin

            duty_pi  <= duty_pi_next;
            duty     <= duty_next;

            error    <= error_next;
            integral <= integral_next;

        end
    end

endmodule
