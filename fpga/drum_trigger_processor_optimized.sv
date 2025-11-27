`timescale 1ns / 1ps

// Drum Trigger Processor - OPTIMIZED VERSION
// Time-multiplexes quaternion processing to reduce DSP usage
// Uses single shared quaternion_to_euler_dsp_optimized module (6 DSP blocks)
// Processes sensor 1 and sensor 2 alternately
// Total DSP usage: 6 blocks (fits in 8 limit)

module drum_trigger_processor_optimized (
    input  logic        clk,
    input  logic        rst_n,
    
    // BNO085 Sensor 1 Inputs (Right Hand)
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // BNO085 Sensor 2 Inputs (Left Hand)
    input  logic        quat2_valid,
    input  logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z,
    input  logic        gyro2_valid,
    input  logic signed [15:0] gyro2_x, gyro2_y, gyro2_z,
    
    // Button Inputs
    input  logic        calibrate_btn_pulse,
    input  logic        kick_btn_pulse,
    
    // Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,
    output logic        drum_hand
);

    // Time-multiplexing state machine
    typedef enum logic {
        PROCESS_SENSOR1,
        PROCESS_SENSOR2
    } sensor_state_t;
    
    sensor_state_t sensor_state;
    logic sensor_select;  // 0 = sensor 1, 1 = sensor 2
    
    // Muxed quaternion inputs (shared quaternion_to_euler_dsp)
    logic quat_mux_valid;
    logic signed [15:0] quat_mux_w, quat_mux_x, quat_mux_y, quat_mux_z;
    
    // Shared quaternion to Euler converter (SINGLE INSTANCE - 6 DSP blocks)
    logic quat_to_euler_valid;
    logic signed [31:0] roll_shared, pitch_shared, yaw_shared;
    
    quaternion_to_euler_dsp_optimized quat_to_euler_shared (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat_mux_valid),
        .quat_w(quat_mux_w),
        .quat_x(quat_mux_x),
        .quat_y(quat_mux_y),
        .quat_z(quat_mux_z),
        .valid_out(quat_to_euler_valid),
        .roll(roll_shared),
        .pitch(pitch_shared),
        .yaw(yaw_shared)
    );
    
    // Demux outputs to sensor-specific buffers
    logic quat1_to_euler_valid, quat2_to_euler_valid;
    logic signed [31:0] roll1, pitch1, yaw1;
    logic signed [31:0] roll2, pitch2, yaw2;
    
    // Buffer for sensor 1
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quat1_to_euler_valid <= 1'b0;
            roll1 <= 32'sd0;
            pitch1 <= 32'sd0;
            yaw1 <= 32'sd0;
        end else if (quat_to_euler_valid && !sensor_select) begin
            quat1_to_euler_valid <= 1'b1;
            roll1 <= roll_shared;
            pitch1 <= pitch_shared;
            yaw1 <= yaw_shared;
        end else begin
            quat1_to_euler_valid <= 1'b0;
        end
    end
    
    // Buffer for sensor 2
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quat2_to_euler_valid <= 1'b0;
            roll2 <= 32'sd0;
            pitch2 <= 32'sd0;
            yaw2 <= 32'sd0;
        end else if (quat_to_euler_valid && sensor_select) begin
            quat2_to_euler_valid <= 1'b1;
            roll2 <= roll_shared;
            pitch2 <= pitch_shared;
            yaw2 <= yaw_shared;
        end else begin
            quat2_to_euler_valid <= 1'b0;
        end
    end
    
    // Time-multiplexing control: Alternate between sensors
    // Simple round-robin: process sensor 1, then sensor 2, then sensor 1, etc.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_state <= PROCESS_SENSOR1;
            sensor_select <= 1'b0;
            quat_mux_valid <= 1'b0;
            quat_mux_w <= 16'sd0;
            quat_mux_x <= 16'sd0;
            quat_mux_y <= 16'sd0;
            quat_mux_z <= 16'sd0;
        end else begin
            case (sensor_state)
                PROCESS_SENSOR1: begin
                    if (quat1_valid) begin
                        // Mux sensor 1 inputs
                        quat_mux_w <= quat1_w;
                        quat_mux_x <= quat1_x;
                        quat_mux_y <= quat1_y;
                        quat_mux_z <= quat1_z;
                        quat_mux_valid <= 1'b1;
                        sensor_select <= 1'b0;
                        sensor_state <= PROCESS_SENSOR2;  // Next process sensor 2
                    end else if (quat2_valid) begin
                        // If sensor 1 not valid but sensor 2 is, process sensor 2
                        quat_mux_w <= quat2_w;
                        quat_mux_x <= quat2_x;
                        quat_mux_y <= quat2_y;
                        quat_mux_z <= quat2_z;
                        quat_mux_valid <= 1'b1;
                        sensor_select <= 1'b1;
                        // Stay in PROCESS_SENSOR1 state (will switch next cycle)
                    end else begin
                        quat_mux_valid <= 1'b0;
                    end
                end
                
                PROCESS_SENSOR2: begin
                    if (quat2_valid) begin
                        // Mux sensor 2 inputs
                        quat_mux_w <= quat2_w;
                        quat_mux_x <= quat2_x;
                        quat_mux_y <= quat2_y;
                        quat_mux_z <= quat2_z;
                        quat_mux_valid <= 1'b1;
                        sensor_select <= 1'b1;
                        sensor_state <= PROCESS_SENSOR1;  // Next process sensor 1
                    end else if (quat1_valid) begin
                        // If sensor 2 not valid but sensor 1 is, process sensor 1
                        quat_mux_w <= quat1_w;
                        quat_mux_x <= quat1_x;
                        quat_mux_y <= quat1_y;
                        quat_mux_z <= quat1_z;
                        quat_mux_valid <= 1'b1;
                        sensor_select <= 1'b0;
                        // Stay in PROCESS_SENSOR2 state (will switch next cycle)
                    end else begin
                        quat_mux_valid <= 1'b0;
                    end
                end
            endcase
        end
    end
    
    // ============================================
    // Sensor 1 Processing Pipeline (Right Hand)
    // ============================================
    
    logic yaw1_norm_valid;
    logic [31:0] yaw1_normalized;
    logic zone1_valid;
    logic [2:0] zone1_id;
    logic strike1_detected_pulse;
    logic strike1_active;
    logic strike1_synced;
    logic drum_sel1_valid;
    logic [3:0] drum_code1;
    
    // Yaw normalization with calibration (Sensor 1)
    yaw_normalizer yaw1_norm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat1_to_euler_valid),
        .yaw(yaw1),
        .calibrate_pulse(calibrate_btn_pulse),
        .valid_out(yaw1_norm_valid),
        .yaw_normalized(yaw1_normalized),
        .yaw_offset()
    );
    
    // Zone detection (Sensor 1 - Right Hand)
    drum_zone_detector zone1_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(yaw1_norm_valid),
        .yaw(yaw1_normalized),
        .is_left_hand(1'b0),
        .valid_out(zone1_valid),
        .zone_id(zone1_id)
    );
    
    // Strike detection (Sensor 1)
    strike_detector strike1_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(gyro1_valid),
        .gyro_y(gyro1_y),
        .strike_detected(strike1_detected_pulse),
        .printed_flag(strike1_active)
    );
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike1_synced <= 1'b0;
        end else begin
            strike1_synced <= strike1_active;
        end
    end
    
    logic drum_sel1_enable;
    assign drum_sel1_enable = zone1_valid && strike1_synced;
    
    drum_selector drum1_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel1_enable),
        .zone_id(zone1_id),
        .pitch(pitch1),
        .gyro_z(gyro1_z),
        .is_left_hand(1'b0),
        .valid_out(drum_sel1_valid),
        .drum_code(drum_code1)
    );
    
    // ============================================
    // Sensor 2 Processing Pipeline (Left Hand)
    // ============================================
    
    logic yaw2_norm_valid;
    logic [31:0] yaw2_normalized;
    logic zone2_valid;
    logic [2:0] zone2_id;
    logic strike2_detected_pulse;
    logic strike2_active;
    logic strike2_synced;
    logic drum_sel2_valid;
    logic [3:0] drum_code2;
    
    // Yaw normalization with calibration (Sensor 2)
    yaw_normalizer yaw2_norm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat2_to_euler_valid),
        .yaw(yaw2),
        .calibrate_pulse(calibrate_btn_pulse),
        .valid_out(yaw2_norm_valid),
        .yaw_normalized(yaw2_normalized),
        .yaw_offset()
    );
    
    // Zone detection (Sensor 2 - Left Hand)
    drum_zone_detector zone2_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(yaw2_norm_valid),
        .yaw(yaw2_normalized),
        .is_left_hand(1'b1),
        .valid_out(zone2_valid),
        .zone_id(zone2_id)
    );
    
    // Strike detection (Sensor 2)
    strike_detector strike2_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(gyro2_valid),
        .gyro_y(gyro2_y),
        .strike_detected(strike2_detected_pulse),
        .printed_flag(strike2_active)
    );
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike2_synced <= 1'b0;
        end else begin
            strike2_synced <= strike2_active;
        end
    end
    
    logic drum_sel2_enable;
    assign drum_sel2_enable = zone2_valid && strike2_synced;
    
    drum_selector drum2_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel2_enable),
        .zone_id(zone2_id),
        .pitch(pitch2),
        .gyro_z(gyro2_z),
        .is_left_hand(1'b1),
        .valid_out(drum_sel2_valid),
        .drum_code(drum_code2)
    );
    
    // ============================================
    // Output Mux: Combine both sensors
    // Priority: Kick button > Sensor 1 > Sensor 2
    // ============================================
    
    logic [2:0] trigger_hold_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger_valid <= 1'b0;
            drum_code <= 4'd0;
            drum_hand <= 1'b0;
            trigger_hold_counter <= 3'd0;
        end else begin
            if (kick_btn_pulse) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= 4'd2;
                drum_hand <= 1'b0;
                trigger_hold_counter <= 3'd5;
            end
            else if (drum_sel1_valid) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code1;
                drum_hand <= 1'b0;
                trigger_hold_counter <= 3'd5;
            end
            else if (drum_sel2_valid) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code2;
                drum_hand <= 1'b1;
                trigger_hold_counter <= 3'd5;
            end
            else if (trigger_hold_counter > 0) begin
                trigger_hold_counter <= trigger_hold_counter - 1;
            end else begin
                drum_trigger_valid <= 1'b0;
            end
        end
    end
    
endmodule

