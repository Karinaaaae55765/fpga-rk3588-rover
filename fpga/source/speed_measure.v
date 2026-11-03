`timescale 1ns/1ps

module speed_measure #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SAMPLE_TIME_MS = 10,
    // Must be measured for the installed motor, gearbox and decode mode.
    // The default is only a historical placeholder, not a hardware claim.
    parameter integer COUNTS_PER_REV = 1456
)(
    input wire clk,
    input wire rst_n,
    // 编码器累计计数
    input wire signed [31:0] count,
    // 输出：轮子转速，单位 RPM
    output reg signed [31:0] rpm,
    // 测速更新脉冲
    output reg update_en
);
    //============================================================
    // 1. 测量窗口
    //============================================================
    localparam integer SAMPLE_COUNT =
        CLK_FREQ / 1000 * SAMPLE_TIME_MS;
    // Fixed-point RPM-per-count scale (Q16).  Constant multiplication plus a
    // shift maps efficiently to FPGA DSP resources; a general division here
    // created an unacceptably long count-to-RPM path.
    localparam signed [63:0] RPM_SCALE_Q16 =
        (64'sd60000 * 64'sd65536) /
        (COUNTS_PER_REV * SAMPLE_TIME_MS);
    //============================================================
    // 2. 测量窗口计时器
    //============================================================
    reg [31:0] sample_counter;
    //============================================================
    // 3. 上一次测量结束时的编码器计数
    //============================================================
    reg signed [31:0] last_count;
    //============================================================
    // 4. 本次测量窗口内的 count 增量
    //============================================================
    reg signed [63:0] count_diff;
    //============================================================
    // 5. RPM计算结果
    //============================================================
    reg signed [63:0] rpm_calculated;
    //============================================================
    // 6. count差值
    //============================================================
    always @(*) begin
        count_diff =
            {{32{count[31]}}, count} -
            {{32{last_count[31]}}, last_count};
    end
    //============================================================
    // 7. RPM计算
    //============================================================

    always @(*) begin
        rpm_calculated =
            (count_diff * RPM_SCALE_Q16) >>> 16;
    end
    //============================================================
    // 8. 速度测量
    //============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_counter <= 32'd0;
            last_count     <= 32'sd0;
            rpm            <= 32'sd0;
            update_en      <= 1'b0;
        end
        else begin
            // 默认没有新的测速结果
            update_en <= 1'b0;
            //====================================================
            // 尚未到达采样时间
            //====================================================
            if (sample_counter < SAMPLE_COUNT - 1) begin
                sample_counter <= sample_counter + 1'b1;
            end
            //====================================================
            // 到达采样时间
            //====================================================
            else begin
                sample_counter <= 32'd0;
                // 更新RPM
                rpm <= rpm_calculated[31:0];
                // 保存当前count
                last_count <= count;
                // 通知外部：新的RPM已经产生
                update_en <= 1'b1;
            end
        end
    end
endmodule
