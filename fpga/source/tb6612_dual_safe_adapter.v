`timescale 1ns/1ps

// Safety boundary for the photographed carrier board.  The carrier exposes
// two shared motor channels: PWM1/AIN1/AIN2 and PWM2/BIN1/BIN2.
module tb6612_dual_safe_adapter #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer DEADTIME_US = 2000,
    parameter integer CH1_REVERSED = 0,
    parameter integer CH2_REVERSED = 0
)(
    input wire clk, input wire rst_n, input wire enable,
    input wire ch1_pwm_in, input wire ch1_in1_in, input wire ch1_in2_in,
    input wire ch2_pwm_in, input wire ch2_in1_in, input wire ch2_in2_in,
    output wire driver_stby,
    output wire ch1_pwm, output wire ch1_in1, output wire ch1_in2,
    output wire ch2_pwm, output wire ch2_in1, output wire ch2_in2
);
    localparam integer DEADTIME_CYCLES_RAW =
        (CLK_FREQ / 1_000_000) * DEADTIME_US;
    localparam integer DEADTIME_CYCLES =
        (DEADTIME_CYCLES_RAW < 1) ? 1 : DEADTIME_CYCLES_RAW;

    wire ch1_req_in1 = CH1_REVERSED ? ch1_in2_in : ch1_in1_in;
    wire ch1_req_in2 = CH1_REVERSED ? ch1_in1_in : ch1_in2_in;
    wire ch2_req_in1 = CH2_REVERSED ? ch2_in2_in : ch2_in1_in;
    wire ch2_req_in2 = CH2_REVERSED ? ch2_in1_in : ch2_in2_in;
    wire [3:0] requested_direction = {
        ch2_req_in1, ch2_req_in2, ch1_req_in1, ch1_req_in2};

    reg [3:0] latched_direction;
    reg active;
    reg [31:0] deadtime_counter;
    wire direction_changed = requested_direction != latched_direction;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_direction <= 4'd0;
            active <= 1'b0;
            deadtime_counter <= 32'd0;
        end else if (!enable) begin
            active <= 1'b0;
            deadtime_counter <= DEADTIME_CYCLES - 1;
            latched_direction <= requested_direction;
        end else if (active && direction_changed) begin
            active <= 1'b0;
            deadtime_counter <= DEADTIME_CYCLES - 1;
            latched_direction <= requested_direction;
        end else if (!active) begin
            if (direction_changed) begin
                deadtime_counter <= DEADTIME_CYCLES - 1;
                latched_direction <= requested_direction;
            end else if (deadtime_counter != 0) begin
                deadtime_counter <= deadtime_counter - 1'b1;
            end else begin
                active <= 1'b1;
            end
        end
    end

    assign driver_stby = enable && active;
    assign ch1_pwm = driver_stby ? ch1_pwm_in : 1'b0;
    assign ch2_pwm = driver_stby ? ch2_pwm_in : 1'b0;
    assign ch1_in1 = driver_stby ? latched_direction[1] : 1'b0;
    assign ch1_in2 = driver_stby ? latched_direction[0] : 1'b0;
    assign ch2_in1 = driver_stby ? latched_direction[3] : 1'b0;
    assign ch2_in2 = driver_stby ? latched_direction[2] : 1'b0;
endmodule
