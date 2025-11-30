`timescale 1ns / 1ps

// Sensor Packet Builder Module
// Latches complete quaternion and gyroscope readings atomically when valid flags pulse
// Provides stable, complete sensor data to SPI slave module
//
// Problem: BNO085 controller updates sensor fields incrementally during SHTP parsing:
//   - quat_x updated at byte 5, quat_y at byte 7, quat_z at byte 9, quat_w at byte 11
//   - If SPI slave snapshots during parsing, it gets mixed old/new values
//
// Solution: This module latches complete readings atomically when valid flags pulse
//   - All quaternion components latched together when quat_valid pulses
//   - All gyroscope components latched together when gyro_valid pulses
//   - Values remain stable until next complete reading arrives

module sensor_packet_builder (
    input  logic        clk,
    input  logic        rst_n,
    
    // Inputs from BNO085 controller (may change incrementally during parsing)
    input  logic        quat_valid,  // Pulses when complete quaternion is ready
    input  logic signed [15:0] quat_w, quat_x, quat_y, quat_z,
    
    input  logic        gyro_valid,  // Pulses when complete gyroscope is ready
    input  logic signed [15:0] gyro_x, gyro_y, gyro_z,
    
    // Outputs to SPI slave (stable, complete readings)
    output logic        quat1_valid,  // Stays high once latched
    output logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    
    output logic        gyro1_valid,  // Stays high once latched
    output logic signed [15:0] gyro1_x, gyro1_y, gyro1_z
);

    // Latch complete quaternion atomically when quat_valid pulses
    // All 4 components are captured together to ensure consistency
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quat1_valid <= 1'b0;
            quat1_w <= 16'd0;
            quat1_x <= 16'd0;
            quat1_y <= 16'd0;
            quat1_z <= 16'd0;
        end else begin
            // Latch complete quaternion when valid flag pulses
            // Once latched, values remain stable until next complete reading
            if (quat_valid) begin
                quat1_w <= quat_w;
                quat1_x <= quat_x;
                quat1_y <= quat_y;
                quat1_z <= quat_z;
                quat1_valid <= 1'b1;  // Set and stay high (sticky)
            end
            // Note: quat1_valid is sticky - once set, it remains high
            // This ensures SPI slave can reliably capture the valid flag
        end
    end
    
    // Latch complete gyroscope atomically when gyro_valid pulses
    // All 3 components are captured together to ensure consistency
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gyro1_valid <= 1'b0;
            gyro1_x <= 16'd0;
            gyro1_y <= 16'd0;
            gyro1_z <= 16'd0;
        end else begin
            // Latch complete gyroscope when valid flag pulses
            // Once latched, values remain stable until next complete reading
            if (gyro_valid) begin
                gyro1_x <= gyro_x;
                gyro1_y <= gyro_y;
                gyro1_z <= gyro_z;
                gyro1_valid <= 1'b1;  // Set and stay high (sticky)
            end
            // Note: gyro1_valid is sticky - once set, it remains high
            // This ensures SPI slave can reliably capture the valid flag
        end
    end

endmodule

