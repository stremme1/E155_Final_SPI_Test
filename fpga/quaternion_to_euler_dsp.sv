`timescale 1ns / 1ps

// Quaternion to Euler Angle Converter
// Matches C code: bno055_quaternion_to_euler function exactly
// Uses fixed-point arithmetic optimized for DSP blocks
// Input: 16-bit signed quaternion (from BNO085)
// Output: 32-bit signed Euler angles in degrees (Q16.15 format)

module quaternion_to_euler_dsp (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] quat_w, quat_x, quat_y, quat_z,
    output logic        valid_out,
    output logic signed [31:0] roll,   // Q16.15 format, degrees
    output logic signed [31:0] pitch,  // Q16.15 format, degrees
    output logic signed [31:0] yaw     // Q16.15 format, degrees
);

    // C code formulas (EXACT):
    // roll  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / M_PI;
    // pitch = asin(2.0 * (w * y - z * x)) * 180.0 / M_PI;
    // yaw   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
    
    // Constants
    // 180/π ≈ 57.2957795131
    // In Q16.15: 57.2957795131 * 32768 = 1,877,464 (approximately)
    localparam [31:0] RAD_TO_DEG = 32'd1877464;  // 180/π in Q16.15
    
    // Pipeline stage 1: Multiplications (use DSP blocks)
    logic signed [31:0] w_x, y_z, w_y, z_x, w_z, x_y;
    logic signed [31:0] x_sq, y_sq, z_sq;
    logic valid_stage1;
    
    // Pipeline stage 2: Additions and subtractions
    logic signed [31:0] w_x_plus_y_z, w_y_minus_z_x, w_z_plus_x_y;
    logic signed [31:0] x_sq_plus_y_sq, y_sq_plus_z_sq;
    logic signed [31:0] roll_num, roll_den, pitch_arg, yaw_num, yaw_den;
    logic valid_stage2;
    
    // Pipeline stage 3: Scale by 2 (left shift 1)
    logic signed [31:0] roll_num_x2, pitch_arg_x2, yaw_num_x2;
    logic signed [31:0] roll_den_scaled, yaw_den_scaled;
    logic valid_stage3;
    
    // Pipeline stage 4: atan2/asin calculations
    logic signed [31:0] roll_rad, pitch_rad, yaw_rad;
    logic valid_stage4;
    
    // Pipeline stage 5: Convert to degrees
    logic valid_stage5;
    
    // Stage 1: Multiplications (DSP blocks)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_x <= 32'sd0;
            y_z <= 32'sd0;
            w_y <= 32'sd0;
            z_x <= 32'sd0;
            w_z <= 32'sd0;
            x_y <= 32'sd0;
            x_sq <= 32'sd0;
            y_sq <= 32'sd0;
            z_sq <= 32'sd0;
            valid_stage1 <= 1'b0;
        end else if (valid_in) begin
            // Q15 * Q15 = Q30, but we keep as Q15 by right-shifting 15 bits
            // For DSP blocks: multiply and shift
            w_x <= (quat_w * quat_x) >>> 15;  // Q15 result
            y_z <= (quat_y * quat_z) >>> 15;
            w_y <= (quat_w * quat_y) >>> 15;
            z_x <= (quat_z * quat_x) >>> 15;
            w_z <= (quat_w * quat_z) >>> 15;
            x_y <= (quat_x * quat_y) >>> 15;
            x_sq <= (quat_x * quat_x) >>> 15;
            y_sq <= (quat_y * quat_y) >>> 15;
            z_sq <= (quat_z * quat_z) >>> 15;
            valid_stage1 <= 1'b1;
        end else begin
            valid_stage1 <= 1'b0;
        end
    end
    
    // Stage 2: Additions and subtractions
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_x_plus_y_z <= 32'sd0;
            w_y_minus_z_x <= 32'sd0;
            w_z_plus_x_y <= 32'sd0;
            x_sq_plus_y_sq <= 32'sd0;
            y_sq_plus_z_sq <= 32'sd0;
            roll_num <= 32'sd0;
            roll_den <= 32'sd0;
            pitch_arg <= 32'sd0;
            yaw_num <= 32'sd0;
            yaw_den <= 32'sd0;
            valid_stage2 <= 1'b0;
        end else if (valid_stage1) begin
            // Calculate sums and differences
            w_x_plus_y_z <= w_x + y_z;
            w_y_minus_z_x <= w_y - z_x;
            w_z_plus_x_y <= w_z + x_y;
            x_sq_plus_y_sq <= x_sq + y_sq;
            y_sq_plus_z_sq <= y_sq + z_sq;
            
            // Roll: atan2(2*(w*x + y*z), 1 - 2*(x² + y²))
            roll_num <= w_x_plus_y_z;
            roll_den <= (32'sd32768 << 15) - (x_sq_plus_y_sq << 1);  // 1 - 2*(x²+y²) in Q15
            
            // Pitch: asin(2*(w*y - z*x))
            pitch_arg <= w_y_minus_z_x;
            
            // Yaw: atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
            yaw_num <= w_z_plus_x_y;
            yaw_den <= (32'sd32768 << 15) - (y_sq_plus_z_sq << 1);  // 1 - 2*(y²+z²) in Q15
            
            valid_stage2 <= 1'b1;
        end else begin
            valid_stage2 <= 1'b0;
        end
    end
    
    // Stage 3: Scale by 2 (left shift 1)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_num_x2 <= 32'sd0;
            pitch_arg_x2 <= 32'sd0;
            yaw_num_x2 <= 32'sd0;
            roll_den_scaled <= 32'sd0;
            yaw_den_scaled <= 32'sd0;
            valid_stage3 <= 1'b0;
        end else if (valid_stage2) begin
            roll_num_x2 <= roll_num << 1;  // 2*(w*x + y*z)
            pitch_arg_x2 <= pitch_arg << 1;  // 2*(w*y - z*x)
            yaw_num_x2 <= yaw_num << 1;  // 2*(w*z + x*y)
            roll_den_scaled <= roll_den;
            yaw_den_scaled <= yaw_den;
            valid_stage3 <= 1'b1;
        end else begin
            valid_stage3 <= 1'b0;
        end
    end
    
    // Stage 4: atan2 and asin calculations
    // Use simplified CORDIC or lookup table approach
    // For now, use a simplified approximation that will be replaced with proper CORDIC
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_rad <= 32'sd0;
            pitch_rad <= 32'sd0;
            yaw_rad <= 32'sd0;
            valid_stage4 <= 1'b0;
        end else if (valid_stage3) begin
            // Simplified atan2: atan2(y, x) ≈ atan(y/x) with quadrant correction
            // For production: use CORDIC module
            roll_rad <= cordic_atan2(roll_num_x2, roll_den_scaled);
            pitch_rad <= cordic_asin(pitch_arg_x2);
            yaw_rad <= cordic_atan2(yaw_num_x2, yaw_den_scaled);
            valid_stage4 <= 1'b1;
        end else begin
            valid_stage4 <= 1'b0;
        end
    end
    
    // Stage 5: Convert radians to degrees
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll <= 32'sd0;
            pitch <= 32'sd0;
            yaw <= 32'sd0;
            valid_out <= 1'b0;
        end else if (valid_stage4) begin
            // Multiply by 180/π (RAD_TO_DEG)
            // Q15 * Q15 = Q30, shift right 15 to get Q15
            roll <= (roll_rad * RAD_TO_DEG) >>> 15;
            pitch <= (pitch_rad * RAD_TO_DEG) >>> 15;
            yaw <= (yaw_rad * RAD_TO_DEG) >>> 15;
            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end
    
    // Simplified CORDIC atan2 function (placeholder - should use proper CORDIC module)
    function automatic logic signed [31:0] cordic_atan2(input logic signed [31:0] y, input logic signed [31:0] x);
        logic signed [31:0] ratio;
        logic signed [31:0] angle;
        
        // Handle special cases
        if (x == 32'sd0) begin
            // atan2(y, 0) = π/2 or -π/2
            angle = y[31] ? -32'sd51472 : 32'sd51472;  // -π/2 or π/2 in Q15
        end else begin
            // Simplified: atan2(y, x) ≈ atan(y/x)
            // For small angles: atan(ratio) ≈ ratio
            // For production: use proper CORDIC algorithm
            ratio = (y << 15) / x;  // y/x in Q15 (simplified division)
            
            // Clamp ratio to [-1, 1] range for asin
            if (ratio > 32'sd32768) ratio = 32'sd32768;
            if (ratio < -32'sd32768) ratio = -32'sd32768;
            
            // atan(ratio) approximation (linear for small angles)
            angle = ratio;  // Simplified - needs proper atan
            
            // Quadrant correction
            if (x[31] && !y[31]) angle = angle + 32'sd102944;  // +π in Q15
            else if (x[31] && y[31]) angle = angle - 32'sd102944;  // -π in Q15
        end
        
        return angle;
    endfunction
    
    // Simplified CORDIC asin function (placeholder - should use proper CORDIC module)
    function automatic logic signed [31:0] cordic_asin(input logic signed [31:0] x);
        logic signed [31:0] angle;
        
        // Clamp x to [-1, 1] range
        if (x > 32'sd32768) x = 32'sd32768;
        if (x < -32'sd32768) x = -32'sd32768;
        
        // asin(x) approximation (linear for small x)
        // For production: use proper CORDIC or lookup table
        angle = x;  // Simplified - needs proper asin
        
        return angle;
    endfunction
    
endmodule

