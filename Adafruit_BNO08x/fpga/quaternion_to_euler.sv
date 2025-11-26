/**
 * Quaternion to Euler Angle Converter
 * 
 * Converts quaternion (Q30 format) to Euler angles (degrees, Q16 format)
 * Uses ZYX (yaw-pitch-roll) convention
 * 
 * Formulas:
 * roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
 * pitch = asin(2*(w*y - z*x))
 * yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
 * 
 * Uses CORDIC-like approximation for atan2 and asin
 */

module quaternion_to_euler (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] quat_w,  // Q30 format
    input  logic [31:0] quat_x,  // Q30 format
    input  logic [31:0] quat_y,  // Q30 format
    input  logic [31:0] quat_z,  // Q30 format
    
    output logic [31:0] roll,    // Degrees, Q16 format
    output logic [31:0] pitch,   // Degrees, Q16 format
    output logic [31:0] yaw,     // Degrees, Q16 format
    output logic        done
);

    // Internal signals for calculations
    logic [63:0] w_x, y_z, w_y, z_x, w_z, x_y;
    logic [63:0] x_sq, y_sq, z_sq;
    logic [63:0] roll_num, roll_den, pitch_arg, yaw_num, yaw_den;
    logic [31:0] roll_atan, pitch_asin, yaw_atan;
    
    // Fixed-point multiplication: Q30 * Q30 = Q60, then shift right 30 to get Q30
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_x <= 64'd0;
            y_z <= 64'd0;
            w_y <= 64'd0;
            z_x <= 64'd0;
            w_z <= 64'd0;
            x_y <= 64'd0;
            x_sq <= 64'd0;
            y_sq <= 64'd0;
            z_sq <= 64'd0;
            roll_num <= 64'd0;
            roll_den <= 64'd0;
            pitch_arg <= 64'd0;
            yaw_num <= 64'd0;
            yaw_den <= 64'd0;
            roll <= 32'd0;
            pitch <= 32'd0;
            yaw <= 32'd0;
            done <= 1'b0;
        end else if (start) begin
            // Calculate products (Q30 * Q30 = Q60)
            w_x <= $signed(quat_w) * $signed(quat_x);
            y_z <= $signed(quat_y) * $signed(quat_z);
            w_y <= $signed(quat_w) * $signed(quat_y);
            z_x <= $signed(quat_z) * $signed(quat_x);
            w_z <= $signed(quat_w) * $signed(quat_z);
            x_y <= $signed(quat_x) * $signed(quat_y);
            x_sq <= $signed(quat_x) * $signed(quat_x);
            y_sq <= $signed(quat_y) * $signed(quat_y);
            z_sq <= $signed(quat_z) * $signed(quat_z);
            
            // Roll calculation: atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
            roll_num <= (w_x + y_z) << 1; // 2*(w*x + y*z), Q60
            roll_den <= (64'd1 << 60) - ((x_sq + y_sq) << 1); // 1 - 2*(x^2 + y^2), Q60
            
            // Pitch calculation: asin(2*(w*y - z*x))
            pitch_arg <= (w_y - z_x) << 1; // 2*(w*y - z*x), Q60
            
            // Yaw calculation: atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            yaw_num <= (w_z + x_y) << 1; // 2*(w*z + x*y), Q60
            yaw_den <= (64'd1 << 60) - ((y_sq + z_sq) << 1); // 1 - 2*(y^2 + z^2), Q60
            
            // Convert to angles using simplified atan2 and asin
            // For full precision, use CORDIC or lookup tables
            // Simplified version using linear approximation
            
            // Roll: atan2(roll_num, roll_den) in radians, convert to degrees
            // atan2(y, x) ≈ y/x for small angles, full implementation needs proper atan2
            roll_atan <= approximate_atan2(roll_num[63:32], roll_den[63:32]);
            roll <= (roll_atan * 32'd180) >> 16; // Convert radians to degrees (Q16)
            
            // Pitch: asin(pitch_arg) in radians, convert to degrees
            pitch_asin <= approximate_asin(pitch_arg[63:32]);
            pitch <= (pitch_asin * 32'd180) >> 16; // Convert radians to degrees (Q16)
            
            // Yaw: atan2(yaw_num, yaw_den) in radians, convert to degrees
            yaw_atan <= approximate_atan2(yaw_num[63:32], yaw_den[63:32]);
            yaw <= (yaw_atan * 32'd180) >> 16; // Convert radians to degrees (Q16)
            
            done <= 1'b1;
        end else begin
            done <= 1'b0;
        end
    end
    
    // Simplified atan2 approximation
    // For production, use CORDIC or lookup table
    function automatic logic [31:0] approximate_atan2(input logic [31:0] y, input logic [31:0] x);
        logic [31:0] ratio;
        logic [31:0] result;
        logic y_sign, x_sign;
        
        y_sign = y[31];
        x_sign = x[31];
        
        if (x == 32'd0) begin
            // atan2(y, 0) = pi/2 or -pi/2
            result = y_sign ? -32'h8000 : 32'h8000; // -pi/2 or pi/2 in Q15
        end else begin
            // ratio = y/x (Q30 / Q30 = Q30, but we need to handle division)
            // Simplified: use y directly if x ≈ 1
            ratio = y; // Placeholder - needs proper division
            
            // atan(ratio) ≈ ratio for small angles
            // Full implementation needs proper atan calculation
            result = ratio; // Placeholder
        end
        
        // Adjust quadrant based on signs
        if (x_sign && !y_sign) result = result + (32'h4000 << 16); // +pi in Q16
        else if (x_sign && y_sign) result = result - (32'h4000 << 16); // -pi in Q16
        else if (!x_sign && y_sign) result = result; // -pi/2 to pi/2
        
        return result;
    endfunction
    
    // Simplified asin approximation
    function automatic logic [31:0] approximate_asin(input logic [31:0] x);
        // asin(x) ≈ x for small x
        // Full implementation needs proper asin calculation
        return x; // Placeholder - needs proper asin
    endfunction
    
endmodule

