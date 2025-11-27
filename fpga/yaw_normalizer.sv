`timescale 1ns / 1ps

// Yaw Normalizer Module
// Normalizes yaw angle to 0-360 degrees range with calibration offset
// Matches C code: normalizeYaw(yaw - yawOffset)

module yaw_normalizer (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [31:0] yaw,        // Q16.15 format, degrees
    input  logic        calibrate_pulse,   // Calibration button pulse
    output logic        valid_out,
    output logic [31:0] yaw_normalized,    // 0-360 degrees, Q16.15
    output logic signed [31:0] yaw_offset   // Current offset (for debug)
);

    // Constants for normalization (360 degrees in Q16.15 format)
    // 360 * 2^15 = 360 * 32768 = 11,796,480
    localparam [31:0] DEG_360 = 32'd11796480;  // 360.0 in Q16.15
    
    // Yaw offset register (calibration value)
    logic signed [31:0] yaw_offset_reg;
    
    // Pipeline registers
    logic signed [31:0] yaw_adjusted;
    logic signed [31:0] yaw_mod;
    
    // Calibration: capture current yaw as offset (matches C code: yawOffset1 = yaw1)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_offset_reg <= 32'sd0;
        end else if (calibrate_pulse && valid_in) begin
            yaw_offset_reg <= yaw;  // Set offset to current yaw
        end
    end
    
    assign yaw_offset = yaw_offset_reg;
    
    // Stage 1: Subtract offset (matches C code: yaw1 - yawOffset1)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_adjusted <= 32'sd0;
        end else if (valid_in) begin
            yaw_adjusted <= yaw - yaw_offset_reg;
        end
    end
    
    // Stage 2: Normalize to 0-360 (matches C code: normalizeYaw function)
    // normalizeYaw: yaw = fmod(yaw, 360.0); if (yaw < 0) yaw += 360.0;
    logic signed [31:0] yaw_final;
    
    // Calculate modulo: yaw_adjusted mod DEG_360
    // Use proper modulo calculation that handles signed numbers correctly
    always_comb begin
        logic signed [31:0] yaw_temp;
        logic signed [31:0] yaw_abs;
        logic [31:0] num_wraps;
        
        yaw_temp = yaw_adjusted;
        
        // Handle negative: add 360 until positive (matches C: if (yaw < 0) yaw += 360.0)
        if (yaw_temp < 0) begin
            // For negative values, calculate how many 360s to add
            // Use unsigned division for the absolute value
            yaw_abs = -yaw_temp;  // Make positive for division
            num_wraps = yaw_abs / DEG_360;
            // Add (num_wraps + 1) * DEG_360 to ensure result is positive
            yaw_temp = yaw_temp + ((num_wraps + 1) * DEG_360);
        end
        
        // Handle >= 360: subtract 360 until in range (matches C: fmod(yaw, 360.0))
        if (yaw_temp >= DEG_360) begin
            // Calculate how many 360s to subtract
            num_wraps = yaw_temp / DEG_360;
            yaw_temp = yaw_temp - (num_wraps * DEG_360);
        end
        
        // Final check: ensure 0-360 range (shouldn't be needed but safety check)
        if (yaw_temp < 0) begin
            yaw_final = yaw_temp + DEG_360;
        end else if (yaw_temp >= DEG_360) begin
            yaw_final = yaw_temp - DEG_360;
        end else begin
            yaw_final = yaw_temp;
        end
    end
    
    // Pipeline stage 2 output
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_normalized <= 32'sd0;
            valid_out <= 1'b0;
        end else begin
            yaw_normalized <= yaw_final;
            // valid_out is delayed by one cycle from valid_in (pipeline stage)
            valid_out <= valid_in;
        end
    end
    
endmodule

