// Reused unchanged sensor modules; original source paths recorded in README.
// BEGIN uart_rx.v
`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        done,
    output reg        frame_error
);
    localparam integer BIT_CLKS  = (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE;
    localparam integer HALF_CLKS = BIT_CLKS / 2;
    localparam [1:0] IDLE  = 2'd0;
    localparam [1:0] START = 2'd1;
    localparam [1:0] DATA  = 2'd2;
    localparam [1:0] STOP  = 2'd3;

    (* ASYNC_REG="TRUE", syn_preserve=1 *) reg rx_d0, rx_d1, rx_d2;
    reg [1:0] state;
    reg [31:0] clk_count;
    reg [2:0] bit_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_d0 <= 1'b1;
            rx_d1 <= 1'b1;
            rx_d2 <= 1'b1;
        end else begin
            rx_d0 <= rx;
            rx_d1 <= rx_d0;
            rx_d2 <= rx_d1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            clk_count   <= 32'd0;
            bit_index   <= 3'd0;
            data        <= 8'h00;
            done        <= 1'b0;
            frame_error <= 1'b0;
        end else begin
            done        <= 1'b0;
            frame_error <= 1'b0;

            case (state)
                IDLE: begin
                    clk_count <= 32'd0;
                    bit_index <= 3'd0;
                    if (!rx_d2)
                        state <= START;
                end

                START: begin
                    if (clk_count == HALF_CLKS - 1) begin
                        clk_count <= 32'd0;
                        if (!rx_d2)
                            state <= DATA;
                        else
                            state <= IDLE;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                DATA: begin
                    if (clk_count == BIT_CLKS - 1) begin
                        clk_count       <= 32'd0;
                        data[bit_index] <= rx_d2;
                        if (bit_index == 3'd7) begin
                            bit_index <= 3'd0;
                            state     <= STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                STOP: begin
                    if (clk_count == BIT_CLKS - 1) begin
                        clk_count <= 32'd0;
                        state     <= IDLE;
                        if (rx_d2)
                            done <= 1'b1;
                        else
                            frame_error <= 1'b1;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule


// BEGIN uart_tx_v3.v
`timescale 1ns / 1ps

// Byte-oriented 8N1 UART transmitter.
// A byte is accepted only when valid && ready are high on the same clock edge.
module uart_tx_v3 #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       valid,
    output reg        ready,
    output reg        tx,
    output reg        done
);
    localparam integer BIT_CLKS = (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE;

    reg [31:0] clk_count;
    reg [3:0]  bit_index;
    reg [7:0]  data_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready        <= 1'b1;
            tx           <= 1'b1;
            done         <= 1'b0;
            clk_count    <= 32'd0;
            bit_index    <= 4'd0;
            data_latched <= 8'h00;
        end else begin
            done <= 1'b0;

            if (ready) begin
                tx        <= 1'b1;
                clk_count <= 32'd0;
                bit_index <= 4'd0;
                if (valid) begin
                    data_latched <= data;
                    ready        <= 1'b0;
                    tx           <= 1'b0;
                end
            end else if (clk_count == BIT_CLKS - 1) begin
                clk_count <= 32'd0;
                if (bit_index < 4'd8) begin
                    tx        <= data_latched[bit_index];
                    bit_index <= bit_index + 1'b1;
                end else if (bit_index == 4'd8) begin
                    tx        <= 1'b1;
                    bit_index <= 4'd9;
                end else begin
                    tx    <= 1'b1;
                    ready <= 1'b1;
                    done  <= 1'b1;
                end
            end else begin
                clk_count <= clk_count + 1'b1;
            end
        end
    end
endmodule


// BEGIN tfmini_s_parser.v
`timescale 1ns / 1ps

module tfmini_s_parser (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg  [15:0] distance_cm,
    output wire [15:0] distance_mm,
    output reg  [15:0] strength,
    output reg         frame_valid,
    output reg         measurement_valid,
    output reg         checksum_error
);
    localparam WAIT_HEAD_1 = 2'd0;
    localparam WAIT_HEAD_2 = 2'd1;
    localparam READ_FRAME  = 2'd2;

    reg [1:0] state;
    reg [3:0] byte_index;
    reg [7:0] checksum;
    reg [7:0] dist_l, dist_h;
    reg [7:0] strength_l, strength_h;

    assign distance_mm = (distance_cm << 3) + (distance_cm << 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= WAIT_HEAD_1;
            byte_index        <= 4'd0;
            checksum          <= 8'h00;
            dist_l            <= 8'h00;
            dist_h            <= 8'h00;
            strength_l        <= 8'h00;
            strength_h        <= 8'h00;
            distance_cm       <= 16'd0;
            strength          <= 16'd0;
            frame_valid       <= 1'b0;
            measurement_valid <= 1'b0;
            checksum_error    <= 1'b0;
        end else begin
            frame_valid       <= 1'b0;
            measurement_valid <= 1'b0;
            checksum_error    <= 1'b0;

            if (rx_valid) begin
                case (state)
                    WAIT_HEAD_1: begin
                        if (rx_data == 8'h59) begin
                            checksum <= 8'h59;
                            state    <= WAIT_HEAD_2;
                        end
                    end

                    WAIT_HEAD_2: begin
                        if (rx_data == 8'h59) begin
                            checksum   <= 8'hB2;
                            byte_index <= 4'd2;
                            state      <= READ_FRAME;
                        end else begin
                            state <= WAIT_HEAD_1;
                        end
                    end

                    READ_FRAME: begin
                        case (byte_index)
                            4'd2: begin
                                dist_l     <= rx_data;
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd3;
                            end
                            4'd3: begin
                                dist_h     <= rx_data;
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd4;
                            end
                            4'd4: begin
                                strength_l <= rx_data;
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd5;
                            end
                            4'd5: begin
                                strength_h <= rx_data;
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd6;
                            end
                            4'd6: begin
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd7;
                            end
                            4'd7: begin
                                checksum   <= checksum + rx_data;
                                byte_index <= 4'd8;
                            end
                            4'd8: begin
                                if (rx_data == checksum) begin
                                    distance_cm <= {dist_h, dist_l};
                                    strength    <= {strength_h, strength_l};
                                    frame_valid <= 1'b1;
                                    if ({dist_h, dist_l} != 16'hFFFF &&
                                        {dist_h, dist_l} != 16'hFFFE &&
                                        {dist_h, dist_l} != 16'hFFFC &&
                                        {strength_h, strength_l} >= 16'd100)
                                        measurement_valid <= 1'b1;
                                end else begin
                                    checksum_error <= 1'b1;
                                end
                                state      <= WAIT_HEAD_1;
                                byte_index <= 4'd0;
                                checksum   <= 8'h00;
                            end
                            default: begin
                                state <= WAIT_HEAD_1;
                                byte_index <= 4'd0;
                            end
                        endcase
                    end
                    default: state <= WAIT_HEAD_1;
                endcase
            end
        end
    end
endmodule


// BEGIN wt901_parser.v
`timescale 1ns / 1ps

module wt901_parser (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [7:0]         rx_data,
    input  wire               rx_valid,
    output reg  signed [15:0] roll_raw,
    output reg  signed [15:0] pitch_raw,
    output reg  signed [15:0] yaw_raw,
    output reg  signed [15:0] roll_cdeg,
    output reg  signed [15:0] pitch_cdeg,
    output reg  signed [15:0] yaw_cdeg,
    output reg                angle_valid,
    output reg                checksum_error
);
    localparam WAIT_HEAD = 2'd0;
    localparam WAIT_TYPE = 2'd1;
    localparam READ_DATA = 2'd2;

    reg [1:0] state;
    reg [3:0] byte_index;
    reg [7:0] frame_type;
    reg [7:0] checksum;
    reg [7:0] p0, p1, p2, p3, p4, p5, p6, p7;

    function signed [15:0] raw_to_cdeg;
        input signed [15:0] raw_angle;
        reg signed [31:0] scaled;
        reg signed [31:0] shifted;
        begin
            scaled = raw_angle * 32'sd18000;
            shifted = scaled >>> 15;
            raw_to_cdeg = shifted[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= WAIT_HEAD;
            byte_index     <= 4'd0;
            frame_type     <= 8'h00;
            checksum       <= 8'h00;
            p0 <= 0; p1 <= 0; p2 <= 0; p3 <= 0;
            p4 <= 0; p5 <= 0; p6 <= 0; p7 <= 0;
            roll_raw       <= 16'sd0;
            pitch_raw      <= 16'sd0;
            yaw_raw        <= 16'sd0;
            roll_cdeg      <= 16'sd0;
            pitch_cdeg     <= 16'sd0;
            yaw_cdeg       <= 16'sd0;
            angle_valid    <= 1'b0;
            checksum_error <= 1'b0;
        end else begin
            angle_valid    <= 1'b0;
            checksum_error <= 1'b0;

            if (rx_valid) begin
                case (state)
                    WAIT_HEAD: begin
                        if (rx_data == 8'h55) begin
                            checksum <= 8'h55;
                            state    <= WAIT_TYPE;
                        end
                    end

                    WAIT_TYPE: begin
                        if (rx_data >= 8'h50 && rx_data <= 8'h59) begin
                            frame_type <= rx_data;
                            checksum   <= checksum + rx_data;
                            byte_index <= 4'd2;
                            state      <= READ_DATA;
                        end else begin
                            state <= WAIT_HEAD;
                        end
                    end

                    READ_DATA: begin
                        if (byte_index < 4'd10) begin
                            case (byte_index)
                                4'd2: p0 <= rx_data;
                                4'd3: p1 <= rx_data;
                                4'd4: p2 <= rx_data;
                                4'd5: p3 <= rx_data;
                                4'd6: p4 <= rx_data;
                                4'd7: p5 <= rx_data;
                                4'd8: p6 <= rx_data;
                                4'd9: p7 <= rx_data;
                                default: ;
                            endcase
                            checksum   <= checksum + rx_data;
                            byte_index <= byte_index + 1'b1;
                        end else begin
                            if (rx_data == checksum) begin
                                if (frame_type == 8'h53) begin
                                    roll_raw   <= $signed({p1, p0});
                                    pitch_raw  <= $signed({p3, p2});
                                    yaw_raw    <= $signed({p5, p4});
                                    roll_cdeg  <= raw_to_cdeg($signed({p1, p0}));
                                    pitch_cdeg <= raw_to_cdeg($signed({p3, p2}));
                                    yaw_cdeg   <= raw_to_cdeg($signed({p5, p4}));
                                    angle_valid <= 1'b1;
                                end
                            end else begin
                                checksum_error <= 1'b1;
                            end
                            state      <= WAIT_HEAD;
                            byte_index <= 4'd0;
                            checksum   <= 8'h00;
                        end
                    end
                    default: state <= WAIT_HEAD;
                endcase
            end
        end
    end
endmodule

// BEGIN tof_outlier_gate.v
`timescale 1ns / 1ps

module tof_outlier_gate #(
    parameter MAX_STEP_MM  = 16'd1500,
    parameter REBASE_COUNT = 3
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] sample_in,
    input  wire        sample_valid,
    output reg  [15:0] sample_out,
    output reg         output_valid,
    output reg         rejected
);
    reg initialized;
    reg [15:0] candidate;
    reg [2:0] candidate_count;

    wire [15:0] main_diff = (sample_in >= sample_out) ?
                            (sample_in - sample_out) : (sample_out - sample_in);
    wire [15:0] candidate_diff = (sample_in >= candidate) ?
                                 (sample_in - candidate) : (candidate - sample_in);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            initialized     <= 1'b0;
            candidate       <= 16'd0;
            candidate_count <= 3'd0;
            sample_out      <= 16'd0;
            output_valid    <= 1'b0;
            rejected        <= 1'b0;
        end else begin
            output_valid <= 1'b0;
            rejected     <= 1'b0;

            if (sample_valid) begin
                if (!initialized) begin
                    initialized     <= 1'b1;
                    sample_out      <= sample_in;
                    output_valid    <= 1'b1;
                    candidate_count <= 3'd0;
                end else if (main_diff <= MAX_STEP_MM) begin
                    sample_out      <= sample_in;
                    output_valid    <= 1'b1;
                    candidate_count <= 3'd0;
                end else begin
                    rejected <= 1'b1;
                    if (candidate_count != 0 &&
                        candidate_diff <= (MAX_STEP_MM >> 2)) begin
                        if (candidate_count == REBASE_COUNT - 1) begin
                            sample_out      <= sample_in;
                            output_valid    <= 1'b1;
                            candidate_count <= 3'd0;
                        end else begin
                            candidate       <= sample_in;
                            candidate_count <= candidate_count + 1'b1;
                        end
                    end else begin
                        candidate       <= sample_in;
                        candidate_count <= 3'd1;
                    end
                end
            end
        end
    end
endmodule


// BEGIN moving_average4_unsigned.v
`timescale 1ns / 1ps

module moving_average4_unsigned (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] sample_in,
    input  wire        sample_valid,
    output reg  [15:0] average_out,
    output reg         average_valid
);
    reg [15:0] s0, s1, s2, s3;
    reg [2:0] sample_count;
    reg [17:0] sum_next;
    reg [17:0] average_next;

    always @(*) begin
        case (sample_count)
            3'd0: sum_next = {2'd0, sample_in};
            3'd1: sum_next = {2'd0, s0} + sample_in;
            3'd2: sum_next = {2'd0, s0} + s1 + sample_in;
            default: sum_next = {2'd0, s0} + s1 + s2 + sample_in;
        endcase

        case (sample_count)
            3'd0: average_next = sum_next;
            3'd1: average_next = sum_next >> 1;
            3'd2: average_next = sum_next / 3;
            default: average_next = sum_next >> 2;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0 <= 0; s1 <= 0; s2 <= 0; s3 <= 0;
            sample_count <= 0;
            average_out <= 0;
            average_valid <= 1'b0;
        end else begin
            average_valid <= 1'b0;
            if (sample_valid) begin
                s3 <= s2;
                s2 <= s1;
                s1 <= s0;
                s0 <= sample_in;

                case (sample_count)
                    3'd0: begin
                        average_out <= sample_in;
                        sample_count <= 3'd1;
                    end
                    3'd1: begin
                        average_out <= average_next[15:0];
                        sample_count <= 3'd2;
                    end
                    3'd2: begin
                        average_out <= average_next[15:0];
                        sample_count <= 3'd3;
                    end
                    default: begin
                        average_out <= average_next[15:0];
                        sample_count <= 3'd4;
                    end
                endcase
                average_valid <= 1'b1;
            end
        end
    end
endmodule

// BEGIN signed_iir_filter.v
`timescale 1ns / 1ps

module signed_iir_filter #(
    parameter SHIFT = 2
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] sample_in,
    input  wire               sample_valid,
    output reg  signed [15:0] filtered_out,
    output reg                output_valid
);
    reg initialized;
    reg signed [16:0] difference;
    reg signed [16:0] next_value;

    always @(*) begin
        difference = $signed({sample_in[15], sample_in}) -
                     $signed({filtered_out[15], filtered_out});
        next_value = $signed({filtered_out[15], filtered_out}) +
                     (difference >>> SHIFT);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            initialized  <= 1'b0;
            filtered_out <= 16'sd0;
            output_valid <= 1'b0;
        end else begin
            output_valid <= 1'b0;
            if (sample_valid) begin
                if (!initialized) begin
                    initialized  <= 1'b1;
                    filtered_out <= sample_in;
                end else begin
                    filtered_out <= next_value[15:0];
                end
                output_valid <= 1'b1;
            end
        end
    end
endmodule

// BEGIN angle_iir_filter.v
`timescale 1ns / 1ps

module angle_iir_filter #(
    parameter SHIFT = 2
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] angle_in_cdeg,
    input  wire               sample_valid,
    output reg  signed [15:0] angle_out_cdeg,
    output reg                output_valid
);
    reg initialized;
    reg signed [16:0] difference;
    reg signed [16:0] next_angle;

    always @(*) begin
        difference = $signed({angle_in_cdeg[15], angle_in_cdeg}) -
                     $signed({angle_out_cdeg[15], angle_out_cdeg});
        if (difference > 17'sd18000)
            difference = difference - 17'sd36000;
        else if (difference < -17'sd18000)
            difference = difference + 17'sd36000;

        next_angle = $signed({angle_out_cdeg[15], angle_out_cdeg}) +
                     (difference >>> SHIFT);
        if (next_angle >= 17'sd18000)
            next_angle = next_angle - 17'sd36000;
        else if (next_angle < -17'sd18000)
            next_angle = next_angle + 17'sd36000;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            initialized    <= 1'b0;
            angle_out_cdeg <= 16'sd0;
            output_valid   <= 1'b0;
        end else begin
            output_valid <= 1'b0;
            if (sample_valid) begin
                if (!initialized) begin
                    initialized    <= 1'b1;
                    angle_out_cdeg <= angle_in_cdeg;
                end else begin
                    angle_out_cdeg <= next_angle[15:0];
                end
                output_valid <= 1'b1;
            end
        end
    end
endmodule


// BEGIN perception_hub.v
`timescale 1ns / 1ps

module perception_hub #(
    parameter CLK_FREQ      = 50_000_000,
    parameter TOF_BAUD      = 115_200,
    parameter IMU_BAUD      = 9_600,
    parameter IMU_BAUD_ALT  = 115_200,
    parameter TOF_MAX_STEP  = 16'd1500,
    parameter ANGLE_LPF_SHIFT = 2
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tof_uart_rx,
    input  wire               imu_uart_rx,

    output wire [15:0]        front_distance_mm,
    output wire [15:0]        tof_strength,
    output wire               distance_valid,
    output wire signed [15:0] roll_cdeg,
    output wire signed [15:0] pitch_cdeg,
    output wire signed [15:0] yaw_cdeg,
    output wire               imu_valid,
    output reg  [15:0]        tof_error_count,
    output reg  [15:0]        imu_error_count
);
    wire [7:0] tof_byte;
    wire       tof_byte_valid;
    wire       tof_uart_error;
    wire [7:0] imu_byte;
    wire       imu_byte_valid;
    wire       imu_uart_error;
    wire [7:0] imu_byte_alt;
    wire       imu_byte_alt_valid;
    wire       imu_uart_alt_error;

    wire [15:0] tof_distance_cm_raw;
    wire [15:0] tof_distance_mm_raw;
    wire [15:0] tof_strength_raw;
    wire        tof_frame_valid;
    wire        tof_measurement_valid;
    wire        tof_checksum_error;

    wire [15:0] tof_gated;
    wire        tof_gated_valid;
    wire        tof_rejected;

    wire signed [15:0] roll_raw;
    wire signed [15:0] pitch_raw;
    wire signed [15:0] yaw_raw;
    wire signed [15:0] roll_cdeg_raw;
    wire signed [15:0] pitch_cdeg_raw;
    wire signed [15:0] yaw_cdeg_raw;
    wire               imu_angle_valid;
    wire               imu_checksum_error;
    wire signed [15:0] roll_raw_alt;
    wire signed [15:0] pitch_raw_alt;
    wire signed [15:0] yaw_raw_alt;
    wire signed [15:0] roll_cdeg_alt;
    wire signed [15:0] pitch_cdeg_alt;
    wire signed [15:0] yaw_cdeg_alt;
    wire               imu_angle_alt_valid;
    wire               imu_checksum_alt_error;
    wire signed [15:0] roll_cdeg_selected =
        imu_angle_valid ? roll_cdeg_raw : roll_cdeg_alt;
    wire signed [15:0] pitch_cdeg_selected =
        imu_angle_valid ? pitch_cdeg_raw : pitch_cdeg_alt;
    wire signed [15:0] yaw_cdeg_selected =
        imu_angle_valid ? yaw_cdeg_raw : yaw_cdeg_alt;
    wire               imu_angle_selected_valid =
        imu_angle_valid | imu_angle_alt_valid;
    wire               roll_filter_valid;
    wire               pitch_filter_valid;
    wire               yaw_filter_valid;
    // The decoder listening at the wrong baud naturally sees framing errors.
    // Count only errors from the baud last identified by a valid angle frame.
    // Before acquisition, the seen/fresh bits report missing data instead.
    reg imu_baud_known;
    reg imu_use_alt;
    wire selected_imu_error = imu_use_alt ?
        (imu_uart_alt_error || imu_checksum_alt_error) :
        (imu_uart_error || imu_checksum_error);

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(TOF_BAUD)) u_tof_uart (
        .clk(clk), .rst_n(rst_n), .rx(tof_uart_rx),
        .data(tof_byte), .done(tof_byte_valid), .frame_error(tof_uart_error)
    );

    tfmini_s_parser u_tof_parser (
        .clk(clk), .rst_n(rst_n),
        .rx_data(tof_byte), .rx_valid(tof_byte_valid),
        .distance_cm(tof_distance_cm_raw),
        .distance_mm(tof_distance_mm_raw),
        .strength(tof_strength_raw),
        .frame_valid(tof_frame_valid),
        .measurement_valid(tof_measurement_valid),
        .checksum_error(tof_checksum_error)
    );

    tof_outlier_gate #(.MAX_STEP_MM(TOF_MAX_STEP), .REBASE_COUNT(3)) u_tof_gate (
        .clk(clk), .rst_n(rst_n),
        .sample_in(tof_distance_mm_raw),
        .sample_valid(tof_measurement_valid),
        .sample_out(tof_gated),
        .output_valid(tof_gated_valid),
        .rejected(tof_rejected)
    );

    moving_average4_unsigned u_tof_average (
        .clk(clk), .rst_n(rst_n),
        .sample_in(tof_gated), .sample_valid(tof_gated_valid),
        .average_out(front_distance_mm), .average_valid(distance_valid)
    );

    assign tof_strength = tof_strength_raw;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(IMU_BAUD)) u_imu_uart (
        .clk(clk), .rst_n(rst_n), .rx(imu_uart_rx),
        .data(imu_byte), .done(imu_byte_valid), .frame_error(imu_uart_error)
    );

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(IMU_BAUD_ALT)) u_imu_uart_alt (
        .clk(clk), .rst_n(rst_n), .rx(imu_uart_rx),
        .data(imu_byte_alt), .done(imu_byte_alt_valid),
        .frame_error(imu_uart_alt_error)
    );

    wt901_parser u_imu_parser (
        .clk(clk), .rst_n(rst_n),
        .rx_data(imu_byte), .rx_valid(imu_byte_valid),
        .roll_raw(roll_raw), .pitch_raw(pitch_raw), .yaw_raw(yaw_raw),
        .roll_cdeg(roll_cdeg_raw),
        .pitch_cdeg(pitch_cdeg_raw),
        .yaw_cdeg(yaw_cdeg_raw),
        .angle_valid(imu_angle_valid),
        .checksum_error(imu_checksum_error)
    );

    wt901_parser u_imu_parser_alt (
        .clk(clk), .rst_n(rst_n),
        .rx_data(imu_byte_alt), .rx_valid(imu_byte_alt_valid),
        .roll_raw(roll_raw_alt), .pitch_raw(pitch_raw_alt),
        .yaw_raw(yaw_raw_alt), .roll_cdeg(roll_cdeg_alt),
        .pitch_cdeg(pitch_cdeg_alt), .yaw_cdeg(yaw_cdeg_alt),
        .angle_valid(imu_angle_alt_valid),
        .checksum_error(imu_checksum_alt_error)
    );

    signed_iir_filter #(.SHIFT(ANGLE_LPF_SHIFT)) u_roll_filter (
        .clk(clk), .rst_n(rst_n),
        .sample_in(roll_cdeg_selected),
        .sample_valid(imu_angle_selected_valid),
        .filtered_out(roll_cdeg), .output_valid(roll_filter_valid)
    );

    signed_iir_filter #(.SHIFT(ANGLE_LPF_SHIFT)) u_pitch_filter (
        .clk(clk), .rst_n(rst_n),
        .sample_in(pitch_cdeg_selected),
        .sample_valid(imu_angle_selected_valid),
        .filtered_out(pitch_cdeg), .output_valid(pitch_filter_valid)
    );

    angle_iir_filter #(.SHIFT(ANGLE_LPF_SHIFT)) u_yaw_filter (
        .clk(clk), .rst_n(rst_n),
        .angle_in_cdeg(yaw_cdeg_selected),
        .sample_valid(imu_angle_selected_valid),
        .angle_out_cdeg(yaw_cdeg), .output_valid(yaw_filter_valid)
    );

    assign imu_valid = roll_filter_valid & pitch_filter_valid & yaw_filter_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tof_error_count <= 16'd0;
            imu_error_count <= 16'd0;
            imu_baud_known <= 1'b0;
            imu_use_alt <= 1'b0;
        end else begin
            if (imu_angle_valid || imu_angle_alt_valid) begin
                imu_baud_known <= 1'b1;
                imu_use_alt <= !imu_angle_valid;
            end
            if ((tof_uart_error || tof_checksum_error || tof_rejected) &&
                tof_error_count != 16'hFFFF)
                tof_error_count <= tof_error_count + 1'b1;
            if (imu_baud_known && selected_imu_error &&
                !(imu_angle_valid || imu_angle_alt_valid) &&
                imu_error_count != 16'hFFFF)
                imu_error_count <= imu_error_count + 1'b1;
        end
    end
endmodule


// BEGIN sensor_telemetry_tx_v3.v
`timescale 1ns / 1ps

// Sensor telemetry v3
// Frame: A5 5A | version | type | payload_len | payload | CRC16 little-endian
// CRC-16/CCITT-FALSE covers version through the last payload byte.
module sensor_telemetry_tx_v3 #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer REPORT_HZ = 20,
    parameter integer FRESH_MS  = 500
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [15:0]        distance_mm,
    input  wire [15:0]        tof_strength,
    input  wire               distance_valid,
    input  wire signed [15:0] roll_cdeg,
    input  wire signed [15:0] pitch_cdeg,
    input  wire signed [15:0] yaw_cdeg,
    input  wire               imu_valid,
    input  wire [15:0]        tof_error_count,
    input  wire [15:0]        imu_error_count,
    output wire               uart_tx_pin,
    output reg                frame_done,
    output reg                frame_toggle
);
    localparam integer REPORT_CLKS = CLK_FREQ / REPORT_HZ;
    localparam integer MS_CLKS     = CLK_FREQ / 1000;
    localparam integer FRESH_CLKS  = (CLK_FREQ / 1000) * FRESH_MS;
    localparam [1:0] IDLE = 2'd0;
    localparam [1:0] SEND = 2'd1;
    localparam [1:0] WAIT = 2'd2;

    reg [1:0]  state;
    reg [31:0] report_count;
    reg [31:0] millisecond_count;
    reg [31:0] uptime_ms;
    reg [31:0] tof_age;
    reg [31:0] imu_age;
    reg        tof_seen;
    reg        imu_seen;

    reg [5:0]  byte_index;
    reg [15:0] frame_sequence;
    reg [15:0] sequence_latched;
    reg [7:0]  status_latched;
    reg [15:0] distance_latched;
    reg [15:0] strength_latched;
    reg signed [15:0] roll_latched;
    reg signed [15:0] pitch_latched;
    reg signed [15:0] yaw_latched;
    reg [15:0] tof_errors_latched;
    reg [15:0] imu_errors_latched;
    reg [31:0] uptime_latched;
    reg [15:0] crc_latched;

    wire tof_fresh = tof_seen && (tof_age < FRESH_CLKS);
    wire imu_fresh = imu_seen && (imu_age < FRESH_CLKS);
    wire [7:0] status_now = {
        2'b00,
        (imu_error_count != 16'd0),
        (tof_error_count != 16'd0),
        imu_fresh,
        tof_fresh,
        imu_seen,
        tof_seen
    };

    function [15:0] crc16_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer i;
        reg [15:0] crc_work;
        begin
            crc_work = crc_in ^ {data_in, 8'h00};
            for (i = 0; i < 8; i = i + 1) begin
                if (crc_work[15])
                    crc_work = (crc_work << 1) ^ 16'h1021;
                else
                    crc_work = crc_work << 1;
            end
            crc16_next = crc_work;
        end
    endfunction

    // CRC is updated once per accepted payload/header byte, avoiding a
    // 24-byte combinational CRC chain in the 50 MHz timing path.

    reg [7:0] tx_byte;
    always @(*) begin
        case (byte_index)
            6'd0:  tx_byte = 8'hA5;
            6'd1:  tx_byte = 8'h5A;
            6'd2:  tx_byte = 8'h01;
            6'd3:  tx_byte = 8'h10;
            6'd4:  tx_byte = 8'd21;
            6'd5:  tx_byte = sequence_latched[7:0];
            6'd6:  tx_byte = sequence_latched[15:8];
            6'd7:  tx_byte = status_latched;
            6'd8:  tx_byte = distance_latched[7:0];
            6'd9:  tx_byte = distance_latched[15:8];
            6'd10: tx_byte = strength_latched[7:0];
            6'd11: tx_byte = strength_latched[15:8];
            6'd12: tx_byte = roll_latched[7:0];
            6'd13: tx_byte = roll_latched[15:8];
            6'd14: tx_byte = pitch_latched[7:0];
            6'd15: tx_byte = pitch_latched[15:8];
            6'd16: tx_byte = yaw_latched[7:0];
            6'd17: tx_byte = yaw_latched[15:8];
            6'd18: tx_byte = tof_errors_latched[7:0];
            6'd19: tx_byte = tof_errors_latched[15:8];
            6'd20: tx_byte = imu_errors_latched[7:0];
            6'd21: tx_byte = imu_errors_latched[15:8];
            6'd22: tx_byte = uptime_latched[7:0];
            6'd23: tx_byte = uptime_latched[15:8];
            6'd24: tx_byte = uptime_latched[23:16];
            6'd25: tx_byte = uptime_latched[31:24];
            6'd26: tx_byte = crc_latched[7:0];
            default: tx_byte = crc_latched[15:8];
        endcase
    end

    wire tx_ready;
    wire tx_done;
    wire tx_valid = (state == SEND);

    uart_tx_v3 #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_byte), .valid(tx_valid),
        .ready(tx_ready), .tx(uart_tx_pin), .done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            millisecond_count <= 32'd0;
            uptime_ms         <= 32'd0;
            tof_age           <= FRESH_CLKS;
            imu_age           <= FRESH_CLKS;
            tof_seen          <= 1'b0;
            imu_seen          <= 1'b0;
        end else begin
            if (millisecond_count == MS_CLKS - 1) begin
                millisecond_count <= 32'd0;
                uptime_ms <= uptime_ms + 1'b1;
            end else begin
                millisecond_count <= millisecond_count + 1'b1;
            end

            if (distance_valid) begin
                tof_age  <= 32'd0;
                tof_seen <= 1'b1;
            end else if (tof_age < FRESH_CLKS) begin
                tof_age <= tof_age + 1'b1;
            end

            if (imu_valid) begin
                imu_age  <= 32'd0;
                imu_seen <= 1'b1;
            end else if (imu_age < FRESH_CLKS) begin
                imu_age <= imu_age + 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= IDLE;
            report_count        <= 32'd0;
            byte_index          <= 6'd0;
            frame_sequence      <= 16'd0;
            sequence_latched    <= 16'd0;
            status_latched      <= 8'd0;
            distance_latched    <= 16'd0;
            strength_latched    <= 16'd0;
            roll_latched        <= 16'sd0;
            pitch_latched       <= 16'sd0;
            yaw_latched         <= 16'sd0;
            tof_errors_latched  <= 16'd0;
            imu_errors_latched  <= 16'd0;
            uptime_latched      <= 32'd0;
            crc_latched         <= 16'hFFFF;
            frame_done          <= 1'b0;
            frame_toggle        <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            // Free-running schedule: frame start every REPORT_CLKS clocks.
            // Production settings leave >47 ms idle after each 28-byte frame.
            if (report_count == REPORT_CLKS - 1)
                report_count <= 32'd0;
            else
                report_count <= report_count + 1'b1;
            case (state)
                IDLE: begin
                    if (report_count == REPORT_CLKS - 1) begin
                        byte_index         <= 6'd0;
                        sequence_latched   <= frame_sequence;
                        status_latched     <= status_now;
                        distance_latched   <= distance_mm;
                        strength_latched   <= tof_strength;
                        roll_latched       <= roll_cdeg;
                        pitch_latched      <= pitch_cdeg;
                        yaw_latched        <= yaw_cdeg;
                        tof_errors_latched <= tof_error_count;
                        imu_errors_latched <= imu_error_count;
                        uptime_latched     <= uptime_ms;
                        crc_latched        <= 16'hFFFF;
                        frame_sequence     <= frame_sequence + 1'b1;
                        state              <= SEND;
                    end
                end

                SEND: begin
                    if (tx_ready) begin
                        if (byte_index >= 6'd2 && byte_index <= 6'd25)
                            crc_latched <= crc16_next(crc_latched, tx_byte);
                        state <= WAIT;
                    end
                end

                WAIT: begin
                    if (tx_done) begin
                        if (byte_index == 6'd27) begin
                            frame_done   <= 1'b1;
                            frame_toggle <= ~frame_toggle;
                            state        <= IDLE;
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            state      <= SEND;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
