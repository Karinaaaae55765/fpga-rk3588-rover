`timescale 1ns/1ps

module DIR(
    input  wire dir,
    input  wire enable,

    output reg in1,
    output reg in2
);
    always @(*) begin
        if (!enable) begin
            in1 = 1'b0;
            in2 = 1'b0;
        end
        else if (dir == 1'b0) begin
            // 正转
            in1 = 1'b1;
            in2 = 1'b0;
        end
        else begin
            // 反转
            in1 = 1'b0;
            in2 = 1'b1;
        end
    end
endmodule
