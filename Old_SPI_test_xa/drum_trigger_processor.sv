`timescale 1ns / 1ps

// Drum Trigger Processor - Top-Level Module
// Integrates all components to process BNO085 data and output drum codes
// Matches C code logic exactly

module drum_trigger_processor (
    input  logic        clk,
    input  logic        rst_n,
    
    // BNO085 Sensor Inputs (Right Hand - Sensor 1)
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // Button Inputs (EXACT from C code)
    input  logic        calibrate_btn_pulse,  // Button 2: Calibration (sets yaw offset)
    input  logic        kick_btn_pulse,       // Button 1: Kick drum (outputs code "2")
    
    // Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,          // 0-7 (matches C code exactly)
    output logic        drum_hand            // 0=right, 1=left (currently always 0)
);

    // Internal pipeline signals
    logic quat_to_euler_valid;
    logic signed [31:0] roll1, pitch1, yaw1;
    
    logic yaw_norm_valid;
    logic [31:0] yaw1_normalized;
    
    logic zone_valid;
    logic [2:0] zone_id;
    
    logic strike_valid;
    logic strike_detected;
    
    logic drum_sel_valid;
    logic [3:0] drum_code_from_selector;
    
    // Quaternion to Euler conversion
    quaternion_to_euler_dsp quat_to_euler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat1_valid),
        .quat_w(quat1_w),
        .quat_x(quat1_x),
        .quat_y(quat1_y),
        .quat_z(quat1_z),
        .valid_out(quat_to_euler_valid),
        .roll(roll1),
        .pitch(pitch1),
        .yaw(yaw1)
    );
    
    // Yaw normalization with calibration
    yaw_normalizer yaw_norm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat_to_euler_valid),
        .yaw(yaw1),
        .calibrate_pulse(calibrate_btn_pulse),
        .valid_out(yaw_norm_valid),
        .yaw_normalized(yaw1_normalized),
        .yaw_offset()
    );
    
    // Zone detection
    drum_zone_detector zone_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(yaw_norm_valid),
        .yaw(yaw1_normalized),
        .is_left_hand(1'b0),  // Right hand for sensor 1
        .valid_out(zone_valid),
        .zone_id(zone_id)
    );
    
    // Strike detection
    logic strike_detected_pulse;
    logic strike_active;  // High while strike is active
    
    strike_detector strike_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(gyro1_valid),
        .gyro_y(gyro1_y),
        .strike_detected(strike_detected_pulse),
        .printed_flag(strike_active)
    );
    
    // Synchronize strike with zone (strike must be active when zone is valid)
    logic strike_synced;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike_synced <= 1'b0;
        end else begin
            // Strike is active when printed_flag is set (gyro_y < -2500)
            strike_synced <= strike_active;
        end
    end
    
    // Drum selection (only when strike detected and zone valid)
    logic drum_sel_enable;
    assign drum_sel_enable = zone_valid && strike_synced;
    
    drum_selector drum_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel_enable),
        .zone_id(zone_id),
        .pitch(pitch1),
        .gyro_z(gyro1_z),
        .is_left_hand(1'b0),  // Right hand for sensor 1
        .valid_out(drum_sel_valid),
        .drum_code(drum_code_from_selector)
    );
    
    // Output mux: Kick button bypasses all processing (EXACT from C code)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger_valid <= 1'b0;
            drum_code <= 4'd0;
            drum_hand <= 1'b0;
        end else begin
            // Kick button has priority (matches C code: button1 -> code "2")
            if (kick_btn_pulse) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= 4'd2;  // Kick drum
                drum_hand <= 1'b0;  // Right hand
            end
            // Normal drum trigger from processing pipeline
            else if (drum_sel_valid) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code_from_selector;
                drum_hand <= 1'b0;  // Right hand
            end else begin
                drum_trigger_valid <= 1'b0;
            end
        end
    end
    
endmodule

