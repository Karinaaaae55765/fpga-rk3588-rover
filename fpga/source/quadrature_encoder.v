`timescale 1ns/1ps

module quadrature_encoder #(
    parameter integer COUNT_WIDTH = 32
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         enc_a,
    input  wire                         enc_b,

    output reg                          direction,
    output reg signed [COUNT_WIDTH-1:0] count
);
    //============================================================
    // 1. Encoder input synchronizer
    //    将编码器异步信号同步到 FPGA 时钟域
    //============================================================
    reg enc_a_ff1;
    reg enc_a_ff2;
    reg enc_b_ff1;
    reg enc_b_ff2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_ff1 <= 1'b0;
            enc_a_ff2 <= 1'b0;
            enc_b_ff1 <= 1'b0;
            enc_b_ff2 <= 1'b0;
        end
        else begin
            enc_a_ff1 <= enc_a;
            enc_a_ff2 <= enc_a_ff1;

            enc_b_ff1 <= enc_b;
            enc_b_ff2 <= enc_b_ff1;
        end
    end
    //============================================================
    // 2. Quadrature encoder state
    //============================================================
    reg [1:0] last_state;
    wire [1:0] current_state;
    assign current_state = {enc_a_ff2, enc_b_ff2};
    //============================================================
    // 3. Quadrature state transition decoding
    //
    //    00 -> 01 -> 11 -> 10 -> 00 : 正向
    //    00 -> 10 -> 11 -> 01 -> 00 : 反向
    //
    //    每一次合法状态变化计数一次
    //    因此属于 4 倍频计数
    //============================================================
    reg count_up;
    reg count_down;
    always @(*) begin
        count_up   = 1'b0;
        count_down = 1'b0;
        case ({last_state, current_state})
            //====================================================
            // Forward
            //====================================================
            4'b0001,    // 00 -> 01
            4'b0111,    // 01 -> 11
            4'b1110,    // 11 -> 10
            4'b1000:    // 10 -> 00
                count_up = 1'b1;
            //====================================================
            // Reverse
            //====================================================
            4'b0010,    // 00 -> 10
            4'b1011,    // 10 -> 11
            4'b1101,    // 11 -> 01
            4'b0100:    // 01 -> 00
                count_down = 1'b1;
            //====================================================
            // No movement / invalid transition
            //====================================================
            default: begin
                count_up   = 1'b0;
                count_down = 1'b0;
            end

        endcase
    end
    //============================================================
    // 4. Position counter and direction
    //============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_state <= 2'b00;

            count      <= {COUNT_WIDTH{1'b0}};
            direction  <= 1'b0;
        end
        else begin
            // 保存当前编码器状态
            last_state <= current_state;
            // 正向
            if (count_up) begin
                count     <= count + 1'b1;
                direction <= 1'b1;
            end
            // 反向
            else if (count_down) begin
                count     <= count - 1'b1;
                direction <= 1'b0;
            end
        end
    end
endmodule
