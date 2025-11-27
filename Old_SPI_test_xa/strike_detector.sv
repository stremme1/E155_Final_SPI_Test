`timescale 1ns / 1ps

// Strike Detector Module
// Detects drum strikes using gyroscope Y-axis threshold
// Matches C code logic: gyro_y < -2500 with printedForGyro flag

module strike_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] gyro_y,
    output logic        strike_detected,  // Single-cycle pulse when strike occurs
    output logic        printed_flag      // High while printed (prevents retrigger)
);

    // Threshold: -2500 (matches C code: gyro1_y < -2500)
    localparam signed [15:0] STRIKE_THRESHOLD = -16'sd2500;
    
    // State machine (matches C code's printedForGyro flag)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            printed_flag <= 1'b0;
        end else if (valid_in) begin
            // C code logic:
            // if (gyro_y < -2500 && !printedForGyro) { trigger, printedForGyro = true }
            // else if (gyro_y >= -2500 && printedForGyro) { printedForGyro = false }
            
            if (gyro_y < STRIKE_THRESHOLD && !printed_flag) begin
                // Strike detected, set flag to prevent retrigger
                printed_flag <= 1'b1;
            end else if (gyro_y >= STRIKE_THRESHOLD && printed_flag) begin
                // Gyro returned above threshold, reset flag
                printed_flag <= 1'b0;
            end
        end
    end
    
    // Strike detected pulse: rising edge of printed_flag
    logic printed_flag_prev;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            printed_flag_prev <= 1'b0;
        end else begin
            printed_flag_prev <= printed_flag;
        end
    end
    
    assign strike_detected = valid_in && printed_flag && !printed_flag_prev;
    
endmodule

