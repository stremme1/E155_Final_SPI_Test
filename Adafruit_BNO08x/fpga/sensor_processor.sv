/**
 * Sensor Data Processor
 * 
 * Processes BNO08X sensor data:
 * - Decodes quaternion and gyroscope reports
 * - Converts quaternion to Euler angles (roll, pitch, yaw)
 * - Implements drum trigger logic based on orientation and motion
 * 
 * Based on logic from Code_for_C_imp/main.c
 */

module sensor_processor (
    input  logic        clk,
    input  logic        rst_n,
    
    // Sensor Data Input
    input  logic        data_ready,
    input  logic [7:0]  sensor_report_id,
    input  logic [7:0]  sensor_data [0:15],
    input  logic [4:0]  sensor_data_len,
    
    // Sensor Data Output
    output logic [31:0] quat_w, quat_x, quat_y, quat_z,  // Quaternion (Q30 fixed point)
    output logic [31:0] gyro_x, gyro_y, gyro_z,          // Gyroscope (rad/s * 2^16)
    output logic [31:0] roll, pitch, yaw,               // Euler angles (degrees * 2^16)
    output logic        euler_valid,
    
    // Drum Trigger Output
    output logic [3:0]  drum_trigger,  // 0-7: drum sounds, 8: no trigger
    output logic        trigger_valid,
    
    // Configuration
    input  logic [31:0] yaw_offset1,    // Yaw offset for sensor 1
    input  logic [31:0] yaw_offset2,    // Yaw offset for sensor 2
    input  logic        sensor_select   // 0 = sensor 1, 1 = sensor 2
);

    // Report IDs
    localparam REPORT_ID_GAME_ROTATION_VECTOR = 8'h08;
    localparam REPORT_ID_ROTATION_VECTOR = 8'h05;
    localparam REPORT_ID_GYROSCOPE = 8'h02;
    
    // Internal signals
    logic [31:0] quat_w_int, quat_x_int, quat_y_int, quat_z_int;
    logic [31:0] gyro_x_int, gyro_y_int, gyro_z_int;
    logic [31:0] roll_int, pitch_int, yaw_int;
    logic [31:0] yaw_normalized;
    logic [31:0] yaw_offset;
    
    // Q-point definitions (from SH-2 Reference Manual)
    // Quaternion: Q30 (30 fractional bits)
    // Gyroscope: Q16 (16 fractional bits, units: rad/s)
    // Euler angles: degrees, we'll use Q16 for consistency
    
    // Parse sensor reports
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quat_w_int <= 32'd0;
            quat_x_int <= 32'd0;
            quat_y_int <= 32'd0;
            quat_z_int <= 32'd0;
            gyro_x_int <= 32'd0;
            gyro_y_int <= 32'd0;
            gyro_z_int <= 32'd0;
            euler_valid <= 1'b0;
            trigger_valid <= 1'b0;
            drum_trigger <= 4'd8; // No trigger
        end else if (data_ready) begin
            case (sensor_report_id)
                REPORT_ID_GAME_ROTATION_VECTOR,
                REPORT_ID_ROTATION_VECTOR: begin
                    // Quaternion report format (from SH-2 Reference Manual):
                    // Byte 0: Report ID
                    // Byte 1: Sequence number
                    // Byte 2: Status
                    // Byte 3: Delay LSB
                    // Byte 4-7: Quaternion i (Q30)
                    // Byte 8-11: Quaternion j (Q30)
                    // Byte 9-15: Quaternion k (Q30)
                    // Byte 12-15: Quaternion real (Q30)
                    if (sensor_data_len >= 16) begin
                        quat_x_int <= {sensor_data[4], sensor_data[5], sensor_data[6], sensor_data[7]};
                        quat_y_int <= {sensor_data[8], sensor_data[9], sensor_data[10], sensor_data[11]};
                        quat_z_int <= {sensor_data[12], sensor_data[13], sensor_data[14], sensor_data[15]};
                        // Real component would be in bytes 16-19 if available
                        // For now, calculate from i, j, k: real = sqrt(1 - i^2 - j^2 - k^2)
                        // Simplified: assume normalized quaternion
                        quat_w_int <= 32'h40000000; // Approximate 1.0 in Q30
                    end
                end
                
                REPORT_ID_GYROSCOPE: begin
                    // Gyroscope report format:
                    // Byte 0: Report ID
                    // Byte 1: Sequence number
                    // Byte 2: Status
                    // Byte 3: Delay LSB
                    // Byte 4-5: Gyro X (Q16, rad/s)
                    // Byte 6-7: Gyro Y (Q16, rad/s)
                    // Byte 8-9: Gyro Z (Q16, rad/s)
                    if (sensor_data_len >= 10) begin
                        gyro_x_int <= {{16{sensor_data[5][7]}}, sensor_data[4], sensor_data[5]}; // Sign extend
                        gyro_y_int <= {{16{sensor_data[7][7]}}, sensor_data[6], sensor_data[7]};
                        gyro_z_int <= {{16{sensor_data[9][7]}}, sensor_data[8], sensor_data[9]};
                    end
                end
            endcase
        end
    end
    
    // Quaternion to Euler conversion
    // Using standard conversion formulas
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_int <= 32'd0;
            pitch_int <= 32'd0;
            yaw_int <= 32'd0;
            euler_valid <= 1'b0;
        end else if (data_ready && (sensor_report_id == REPORT_ID_GAME_ROTATION_VECTOR || 
                                     sensor_report_id == REPORT_ID_ROTATION_VECTOR)) begin
            // Quaternion to Euler conversion (ZYX convention)
            // This is a simplified version - full implementation would use fixed-point math
            // roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
            // pitch = asin(2*(w*y - z*x))
            // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            
            // For now, use approximate conversion
            // In a full implementation, you would use CORDIC or lookup tables
            // This is a placeholder that needs proper fixed-point arithmetic
            
            // Simplified: convert quaternion components to approximate Euler
            // This requires proper fixed-point math libraries
            roll_int <= 32'd0;  // Placeholder
            pitch_int <= 32'd0; // Placeholder
            yaw_int <= 32'd0;   // Placeholder
            
            euler_valid <= 1'b1;
        end else begin
            euler_valid <= 1'b0;
        end
    end
    
    // Yaw normalization (0-360 degrees)
    always_comb begin
        yaw_offset = sensor_select ? yaw_offset2 : yaw_offset1;
        // Normalize yaw: yaw = (yaw - offset) mod 360
        // This is simplified - full implementation needs modulo arithmetic
        if (yaw_int >= yaw_offset) begin
            yaw_normalized = yaw_int - yaw_offset;
        end else begin
            yaw_normalized = yaw_int + (32'd360 << 16) - yaw_offset;
        end
        // Wrap to 0-360 range
        if (yaw_normalized >= (32'd360 << 16)) begin
            yaw_normalized = yaw_normalized - (32'd360 << 16);
        end
    end
    
    // Drum trigger logic (based on main.c)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger <= 4'd8;
            trigger_valid <= 1'b0;
        end else if (euler_valid && data_ready) begin
            trigger_valid <= 1'b0;
            drum_trigger <= 4'd8; // No trigger by default
            
            // Check gyro threshold (gyro_y < -2500 in original code)
            // Original uses int16_t, so -2500 = 0xF63C
            // In Q16 format: -2500 * 2^16 / (rad/s conversion factor)
            // Simplified check: gyro_y < -2500 (in original units)
            logic [31:0] gyro_y_threshold = 32'hFFFFF63C; // -2500 in 32-bit signed
            
            if (gyro_y_int < gyro_y_threshold) begin
                // Right hand logic (sensor_select == 0)
                if (!sensor_select) begin
                    // Yaw ranges for right hand
                    if ((yaw_normalized >= (32'd20 << 16)) && 
                        (yaw_normalized <= (32'd120 << 16))) begin
                        drum_trigger <= 4'd0; // Snare drum
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd340 << 16)) || 
                                (yaw_normalized <= (32'd20 << 16))) begin
                        if (pitch_int > (32'd50 << 16)) begin
                            drum_trigger <= 4'd5; // Crash cymbal
                        end else begin
                            drum_trigger <= 4'd3; // High tom
                        end
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd305 << 16)) && 
                                (yaw_normalized <= (32'd340 << 16))) begin
                        if (pitch_int > (32'd50 << 16)) begin
                            drum_trigger <= 4'd6; // Ride cymbal
                        end else begin
                            drum_trigger <= 4'd4; // Mid tom
                        end
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd200 << 16)) && 
                                (yaw_normalized <= (32'd305 << 16))) begin
                        if (pitch_int > (32'd30 << 16)) begin
                            drum_trigger <= 4'd6; // Ride cymbal
                        end else begin
                            drum_trigger <= 4'd7; // Floor tom
                        end
                        trigger_valid <= 1'b1;
                    end
                end else begin
                    // Left hand logic (sensor_select == 1)
                    if ((yaw_normalized >= (32'd350 << 16)) || 
                        (yaw_normalized <= (32'd100 << 16))) begin
                        if ((pitch_int > (32'd30 << 16)) && 
                            (gyro_z_int > 32'hFFFFF830)) begin // gyro_z > -2000
                            drum_trigger <= 4'd1; // Hi-hat
                        end else begin
                            drum_trigger <= 4'd0; // Snare drum
                        end
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd325 << 16)) && 
                                (yaw_normalized <= (32'd350 << 16))) begin
                        if (pitch_int > (32'd50 << 16)) begin
                            drum_trigger <= 4'd5; // Crash cymbal
                        end else begin
                            drum_trigger <= 4'd3; // High tom
                        end
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd300 << 16)) && 
                                (yaw_normalized <= (32'd325 << 16))) begin
                        if (pitch_int > (32'd50 << 16)) begin
                            drum_trigger <= 4'd6; // Ride cymbal
                        end else begin
                            drum_trigger <= 4'd4; // Mid tom
                        end
                        trigger_valid <= 1'b1;
                    end else if ((yaw_normalized >= (32'd200 << 16)) && 
                                (yaw_normalized <= (32'd300 << 16))) begin
                        if (pitch_int > (32'd30 << 16)) begin
                            drum_trigger <= 4'd6; // Ride cymbal
                        end else begin
                            drum_trigger <= 4'd7; // Floor tom
                        end
                        trigger_valid <= 1'b1;
                    end
                end
            end
        end
    end
    
    // Output assignments
    assign quat_w = quat_w_int;
    assign quat_x = quat_x_int;
    assign quat_y = quat_y_int;
    assign quat_z = quat_z_int;
    assign gyro_x = gyro_x_int;
    assign gyro_y = gyro_y_int;
    assign gyro_z = gyro_z_int;
    assign roll = roll_int;
    assign pitch = pitch_int;
    assign yaw = yaw_normalized;
    
endmodule

