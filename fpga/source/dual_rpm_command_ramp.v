`timescale 1ns/1ps

// Slews speed set-points and forces both targets to zero before a direction
// change.  This keeps the PI loops closed while preserving the safe reversal
// behaviour previously provided by the open-loop duty ramp.
module dual_rpm_command_ramp #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UPDATE_MS = 10,
    parameter integer ACCEL_STEP_RPM = 3,
    parameter integer DECEL_STEP_RPM = 6
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire signed [31:0] target_left_rpm,
    input  wire signed [31:0] target_right_rpm,
    input  wire               target_left_dir,
    input  wire               target_right_dir,
    output reg  signed [31:0] left_rpm,
    output reg  signed [31:0] right_rpm,
    output reg                left_dir,
    output reg                right_dir
);
    localparam integer UPDATE_CYCLES_RAW =
        (CLK_FREQ / 1000) * UPDATE_MS;
    localparam integer UPDATE_CYCLES =
        (UPDATE_CYCLES_RAW < 1) ? 1 : UPDATE_CYCLES_RAW;

    reg [31:0] update_counter;
    wire direction_change =
        (left_dir != target_left_dir) ||
        (right_dir != target_right_dir);

    function signed [31:0] step_toward;
        input signed [31:0] current;
        input signed [31:0] target;
        input integer up_step;
        input integer down_step;
        begin
            if (current < target)
                step_toward = ((target-current) <= up_step) ? target :
                    current + up_step;
            else if (current > target)
                step_toward = ((current-target) <= down_step) ? target :
                    current - down_step;
            else
                step_toward = current;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            update_counter <= 0;
            left_rpm <= 0;
            right_rpm <= 0;
            left_dir <= 0;
            right_dir <= 0;
        end else if (!enable) begin
            update_counter <= 0;
            left_rpm <= 0;
            right_rpm <= 0;
        end else if (update_counter >= UPDATE_CYCLES-1) begin
            update_counter <= 0;
            if (direction_change) begin
                left_rpm <= step_toward(left_rpm, 0, DECEL_STEP_RPM,
                                        DECEL_STEP_RPM);
                right_rpm <= step_toward(right_rpm, 0, DECEL_STEP_RPM,
                                         DECEL_STEP_RPM);
                if (left_rpm == 0 && right_rpm == 0) begin
                    left_dir <= target_left_dir;
                    right_dir <= target_right_dir;
                end
            end else begin
                left_rpm <= step_toward(left_rpm, target_left_rpm,
                                        ACCEL_STEP_RPM, DECEL_STEP_RPM);
                right_rpm <= step_toward(right_rpm, target_right_rpm,
                                         ACCEL_STEP_RPM, DECEL_STEP_RPM);
            end
        end else begin
            update_counter <= update_counter + 1'b1;
        end
    end
endmodule

