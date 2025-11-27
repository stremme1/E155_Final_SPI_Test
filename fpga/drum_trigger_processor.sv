`timescale 1ns / 1ps

// Drum Trigger Processor - Time-Multiplexed Version
// Processes dual sensors sequentially to reduce resource usage by ~50%
// Alternates between sensor 1 and sensor 2 each cycle

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
    
    // Button Inputs
    input  logic        calibrate_btn_pulse,  // Calibration (sets yaw offset)
    input  logic        kick_btn_pulse,       // Kick drum (outputs code "2")
    
    // Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,          // 0-7
    output logic        drum_hand            // 0=right, 1=left
);

    // Sensor select: alternates each cycle (0=sensor1, 1=sensor2)
    logic sensor_select;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_select <= 1'b0;
        end else begin
            sensor_select <= ~sensor_select;  // Toggle each cycle
        end
    end
    
    // Mux sensor inputs to single processing pipeline
    logic quat_valid_mux, gyro_valid_mux;
    logic signed [15:0] quat_w_mux, quat_x_mux, quat_y_mux, quat_z_mux;
    logic signed [15:0] gyro_x_mux, gyro_y_mux, gyro_z_mux;
    
    always_comb begin
        if (sensor_select == 1'b0) begin
            // Sensor 1 (Right Hand)
            quat_valid_mux = quat1_valid;
            quat_w_mux = quat1_w;
            quat_x_mux = quat1_x;
            quat_y_mux = quat1_y;
            quat_z_mux = quat1_z;
            gyro_valid_mux = gyro1_valid;
            gyro_x_mux = gyro1_x;
            gyro_y_mux = gyro1_y;
            gyro_z_mux = gyro1_z;
        end else begin
            // Sensor 2 (Left Hand)
            quat_valid_mux = quat2_valid;
            quat_w_mux = quat2_w;
            quat_x_mux = quat2_x;
            quat_y_mux = quat2_y;
            quat_z_mux = quat2_z;
            gyro_valid_mux = gyro2_valid;
            gyro_x_mux = gyro2_x;
            gyro_y_mux = gyro2_y;
            gyro_z_mux = gyro2_z;
        end
    end
    
    // ============================================
    // Single Processing Pipeline (Time-Multiplexed)
    // ============================================
    
    logic quat_to_euler_valid;
    logic signed [31:0] roll, pitch, yaw;
    
    logic yaw_norm_valid;
    logic [31:0] yaw_normalized;
    
    logic zone_valid;
    logic [2:0] zone_id;
    
    logic strike_detected_pulse;
    logic strike_active;
    logic strike_synced;
    
    logic drum_sel_valid;
    logic [3:0] drum_code_mux;
    
    // Quaternion to Euler conversion (SINGLE INSTANCE)
    quaternion_to_euler_dsp quat_to_euler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat_valid_mux),
        .quat_w(quat_w_mux),
        .quat_x(quat_x_mux),
        .quat_y(quat_y_mux),
        .quat_z(quat_z_mux),
        .valid_out(quat_to_euler_valid),
        .roll(roll),
        .pitch(pitch),
        .yaw(yaw)
    );
    
    // Yaw normalization (SINGLE INSTANCE)
    yaw_normalizer yaw_norm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(quat_to_euler_valid),
        .yaw(yaw),
        .calibrate_pulse(calibrate_btn_pulse),
        .is_left_hand(sensor_select),  // Use sensor_select to determine hand
        .valid_out(yaw_norm_valid),
        .yaw_normalized(yaw_normalized),
        .yaw_offset()
    );
    
    // Zone detection (SINGLE INSTANCE)
    drum_zone_detector zone_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(yaw_norm_valid),
        .yaw(yaw_normalized),
        .is_left_hand(sensor_select),  // Use sensor_select
        .valid_out(zone_valid),
        .zone_id(zone_id)
    );
    
    // Strike detection (SINGLE INSTANCE)
    strike_detector strike_detector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(gyro_valid_mux),
        .gyro_y(gyro_y_mux),
        .strike_detected(strike_detected_pulse),
        .printed_flag(strike_active)
    );
    
    // Synchronize strike with zone
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strike_synced <= 1'b0;
        end else begin
            strike_synced <= strike_active;
        end
    end
    
    // Drum selection (SINGLE INSTANCE)
    logic drum_sel_enable;
    assign drum_sel_enable = zone_valid && strike_synced;
    
    drum_selector drum_selector_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(drum_sel_enable),
        .zone_id(zone_id),
        .pitch(pitch),
        .gyro_z(gyro_z_mux),
        .is_left_hand(sensor_select),  // Use sensor_select
        .valid_out(drum_sel_valid),
        .drum_code(drum_code_mux)
    );
    
    // ============================================
    // Store Results Per Sensor
    // ============================================
    
    // Storage for sensor 1 results
    logic drum_sel1_valid_stored;
    logic [3:0] drum_code1_stored;
    logic sensor1_result_ready;
    
    // Storage for sensor 2 results
    logic drum_sel2_valid_stored;
    logic [3:0] drum_code2_stored;
    logic sensor2_result_ready;
    
    // Store results when pipeline completes (accounting for pipeline delay)
    // Pipeline delay: quat_to_euler (5 stages) + yaw_norm (3 stages) + zone (1) + strike sync (1) + selector (1) = ~11 cycles
    // We need to track which sensor the result belongs to
    logic [3:0] pipeline_sensor_tracker;  // Track sensor_select through pipeline
    
    // Track sensor_select through pipeline (simple shift register)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipeline_sensor_tracker <= 4'b0000;
        end else begin
            // Shift in current sensor_select when quat_valid_mux
            if (quat_valid_mux) begin
                pipeline_sensor_tracker <= {pipeline_sensor_tracker[2:0], sensor_select};
            end
        end
    end
    
    // Store results when drum_sel_valid (pipeline complete)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_sel1_valid_stored <= 1'b0;
            drum_code1_stored <= 4'd0;
            sensor1_result_ready <= 1'b0;
            drum_sel2_valid_stored <= 1'b0;
            drum_code2_stored <= 4'd0;
            sensor2_result_ready <= 1'b0;
        end else if (drum_sel_valid) begin
            // Use the tracked sensor_select from when this data entered pipeline
            if (pipeline_sensor_tracker[3] == 1'b0) begin
                // Result belongs to sensor 1
                drum_sel1_valid_stored <= 1'b1;
                drum_code1_stored <= drum_code_mux;
                sensor1_result_ready <= 1'b1;
            end else begin
                // Result belongs to sensor 2
                drum_sel2_valid_stored <= 1'b1;
                drum_code2_stored <= drum_code_mux;
                sensor2_result_ready <= 1'b1;
            end
        end else begin
            // Clear stored results when used
            if (sensor1_result_ready) begin
                drum_sel1_valid_stored <= 1'b0;
                sensor1_result_ready <= 1'b0;
            end
            if (sensor2_result_ready) begin
                drum_sel2_valid_stored <= 1'b0;
                sensor2_result_ready <= 1'b0;
            end
        end
    end
    
    // ============================================
    // Output Mux (Prioritization)
    // ============================================
    // Hold trigger valid until SPI slave acknowledges
    logic [2:0] trigger_hold_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger_valid <= 1'b0;
            drum_code <= 4'd0;
            drum_hand <= 1'b0;
            trigger_hold_counter <= 3'd0;
        end else begin
            // Kick button has highest priority
            if (kick_btn_pulse) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= 4'd2;  // Kick drum
                drum_hand <= 1'b0;  // Right hand (arbitrary)
                trigger_hold_counter <= 3'd5;  // Hold for 5 cycles
            end
            // Sensor 1 (Right Hand) has priority over Sensor 2 (Left Hand)
            else if (drum_sel1_valid_stored) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code1_stored;
                drum_hand <= 1'b0;  // Right hand
                trigger_hold_counter <= 3'd5;  // Hold for 5 cycles
            end
            else if (drum_sel2_valid_stored) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= drum_code2_stored;
                drum_hand <= 1'b1;  // Left hand
                trigger_hold_counter <= 3'd5;  // Hold for 5 cycles
            end
            // Hold valid for a few cycles, then clear
            else if (trigger_hold_counter > 0) begin
                trigger_hold_counter <= trigger_hold_counter - 1;
            end else begin
                drum_trigger_valid <= 1'b0;
            end
        end
    end
    
endmodule
