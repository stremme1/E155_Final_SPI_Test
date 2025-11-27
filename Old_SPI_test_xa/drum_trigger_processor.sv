`timescale 1ns / 1ps

// Drum Trigger Processor - Top-Level Module
// Integrates all components to process BNO085 data and output drum codes
// Matches C code logic exactly
// Now supports dual sensors (right and left hand)

module drum_trigger_processor (
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
    
    // Button Inputs (EXACT from C code)
    input  logic        calibrate_btn_pulse,  // Button 2: Calibration (sets yaw offset for both sensors)
    input  logic        kick_btn_pulse,       // Button 1: Kick drum (outputs code "2")
    
    // Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,          // 0-7 (matches C code exactly)
    output logic        drum_hand            // 0=right, 1=left
);

    // ============================================
    // Sensor 1 Processing Pipeline (Right Hand)
    // ============================================
    
    logic quat1_to_euler_valid;
    logic signed [31:0] roll1, pitch1, yaw1;
    
    logic yaw1_norm_valid;
    logic [31:0] yaw1_normalized;
    
    logic zone1_valid;
    logic [2:0] zone1_id;
    
    logic strike1_detected_pulse;
    logic strike1_active;
    logic strike1_synced;
    
    logic drum_sel1_valid;
    logic [3:0] drum_code1;
    
    // Quaternion to Euler conversion (Sensor 1)
    quaternion_to_euler_dsp quat1_to_euler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat1_valid),
        .quat_w(quat1_w),
        .quat_x(quat1_x),
        .quat_y(quat1_y),
        .quat_z(quat1_z),
        .valid_out(quat1_to_euler_valid),
        .roll(roll1),
        .pitch(pitch1),
        .yaw(yaw1)
    );
    
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
        .is_left_hand(1'b0),  // Right hand
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
    
    // Synchronize strike with zone (Sensor 1)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike1_synced <= 1'b0;
        end else begin
            strike1_synced <= strike1_active;
        end
    end
    
    // Drum selection (Sensor 1)
    logic drum_sel1_enable;
    assign drum_sel1_enable = zone1_valid && strike1_synced;
    
    drum_selector drum1_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel1_enable),
        .zone_id(zone1_id),
        .pitch(pitch1),
        .gyro_z(gyro1_z),
        .is_left_hand(1'b0),  // Right hand
        .valid_out(drum_sel1_valid),
        .drum_code(drum_code1)
    );
    
    // ============================================
    // Sensor 2 Processing Pipeline (Left Hand)
    // ============================================
    
    logic quat2_to_euler_valid;
    logic signed [31:0] roll2, pitch2, yaw2;
    
    logic yaw2_norm_valid;
    logic [31:0] yaw2_normalized;
    
    logic zone2_valid;
    logic [2:0] zone2_id;
    
    logic strike2_detected_pulse;
    logic strike2_active;
    logic strike2_synced;
    
    logic drum_sel2_valid;
    logic [3:0] drum_code2;
    
    // Quaternion to Euler conversion (Sensor 2)
    quaternion_to_euler_dsp quat2_to_euler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat2_valid),
        .quat_w(quat2_w),
        .quat_x(quat2_x),
        .quat_y(quat2_y),
        .quat_z(quat2_z),
        .valid_out(quat2_to_euler_valid),
        .roll(roll2),
        .pitch(pitch2),
        .yaw(yaw2)
    );
    
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
        .is_left_hand(1'b1),  // Left hand
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
    
    // Synchronize strike with zone (Sensor 2)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike2_synced <= 1'b0;
        end else begin
            strike2_synced <= strike2_active;
        end
    end
    
    // Drum selection (Sensor 2)
    logic drum_sel2_enable;
    assign drum_sel2_enable = zone2_valid && strike2_synced;
    
    drum_selector drum2_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel2_enable),
        .zone_id(zone2_id),
        .pitch(pitch2),
        .gyro_z(gyro2_z),
        .is_left_hand(1'b1),  // Left hand
        .valid_out(drum_sel2_valid),
        .drum_code(drum_code2)
    );
    
    // ============================================
    // Output Mux: Combine both sensors
    // Priority: Kick button > Sensor 1 > Sensor 2
    // ============================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger_valid <= 1'b0;
            drum_code <= 4'd0;
            drum_hand <= 1'b0;
        end else begin
            // Kick button has highest priority (matches C code: button1 -> code "2")
            if (kick_btn_pulse) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= 4'd2;  // Kick drum
                drum_hand <= 1'b0;  // Right hand (arbitrary)
            end
            // Sensor 1 (Right Hand) trigger
            else if (drum_sel1_valid) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code1;
                drum_hand <= 1'b0;  // Right hand
            end
            // Sensor 2 (Left Hand) trigger
            else if (drum_sel2_valid) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code2;
                drum_hand <= 1'b1;  // Left hand
            end else begin
                drum_trigger_valid <= 1'b0;
            end
        end
    end
    
endmodule
