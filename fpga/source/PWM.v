`timescale 1ns / 1ps
module PWM #(
    parameter integer CLK_FREQ = 50_000_000,   // FPGA时钟频率：50MHz
    parameter integer PWM_FREQ = 20_000        // PWM频率：20kHz
)(
    input  wire        clk,
    input  wire        rst_n,
    // 占空比控制：
    // 0    -> 0%
    // 500  -> 50%
    // 1000 -> 100%
    input  wire [9:0]  duty,
    output reg         pwm_out
);
    // ============================================================
    // PWM周期计数值
    // 50MHz / 20kHz = 2500
    // 所以一个PWM周期包含2500个时钟
    // ============================================================
    localparam integer PERIOD_COUNT = CLK_FREQ / PWM_FREQ;
    // 根据PWM周期自动确定计数器位宽
    localparam integer COUNTER_WIDTH = $clog2(PERIOD_COUNT);
    reg [COUNTER_WIDTH-1:0] counter;
    wire [11:0] threshold_2500 =
        ({2'b00,duty} << 1) + ({2'b00,duty} >> 1);
    // ============================================================
    // PWM计数器
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
        end
        else begin
            if (counter >= PERIOD_COUNT - 1)
                counter <= 0;
            else
                counter <= counter + 1'b1;
        end
    end
    // ============================================================
    // PWM输出
    //
    // counter在：
    // 0 ~ 2499之间循环
    //
    // duty:
    // 0    -> 0%
    // 500  -> 50%
    // 1000 -> 100%
    //
    // 注意：这里将0~1000映射到0~PERIOD_COUNT
    // ============================================================
always @(*) begin
    if (duty == 10'd0) begin
        pwm_out = 1'b0;
    end
    else if (duty >= 10'd1000) begin
        pwm_out = 1'b1;
    end
    else begin
        // The car build uses 50 MHz / 20 kHz = 2500 clocks.  Expressing
        // 2500/1000 as 2 + 1/2 avoids inferring a general multiplier/divider
        // on the registered PI duty path.
        if (PERIOD_COUNT == 2500)
            pwm_out = (counter < threshold_2500);
        else
            pwm_out = (counter < (PERIOD_COUNT * duty / 1000));
    end
end
endmodule
