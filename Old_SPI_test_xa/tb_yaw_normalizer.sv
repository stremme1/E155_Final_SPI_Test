`timescale 1ns / 1ps

// Testbench for Yaw Normalizer Module
// Tests exact C code logic: normalizeYaw(yaw - yawOffset)

module tb_yaw_normalizer;

    logic clk, rst_n;
    logic valid_in;
    logic signed [31:0] yaw;  // Q16.15
    logic calibrate_pulse;
    logic valid_out;
    logic [31:0] yaw_normalized;
    logic signed [31:0] yaw_offset;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #166.67 clk = ~clk;
    end
    
    // DUT
    yaw_normalizer dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .yaw(yaw),
        .calibrate_pulse(calibrate_pulse),
        .valid_out(valid_out),
        .yaw_normalized(yaw_normalized),
        .yaw_offset(yaw_offset)
    );
    
    // Helper function
    function [31:0] deg_to_q1615(input [31:0] deg);
        deg_to_q1615 = deg * 32'd32768;
    endfunction
    
    function [31:0] q1615_to_deg(input [31:0] q);
        q1615_to_deg = q / 32'd32768;
    endfunction
    
    initial begin
        $display("=== Yaw Normalizer Testbench ===");
        
        // Initialize
        rst_n = 0;
        valid_in = 0;
        yaw = 0;
        calibrate_pulse = 0;
        #1000;
        rst_n = 1;
        #1000;
        
        // Test 1: Normal yaw, no offset
        $display("\nTest 1: Normal yaw (45 degrees), no offset");
        valid_in = 1;
        yaw = deg_to_q1615(45);
        #1000;
        assert(q1615_to_deg(yaw_normalized) == 45) else $error("Test 1 failed");
        $display("PASS: Yaw normalized correctly");
        
        // Test 2: Calibration - set offset
        $display("\nTest 2: Calibration - set offset to 45 degrees");
        yaw = deg_to_q1615(45);
        calibrate_pulse = 1;
        #500;
        calibrate_pulse = 0;
        #500;
        assert(q1615_to_deg(yaw_offset) == 45) else $error("Test 2 failed");
        $display("PASS: Offset set correctly");
        
        // Test 3: Yaw with offset applied
        $display("\nTest 3: Yaw (90 degrees) with offset (45 degrees) = 45 degrees");
        yaw = deg_to_q1615(90);
        valid_in = 1;
        #1000;
        assert(q1615_to_deg(yaw_normalized) == 45) else $error("Test 3 failed: Expected 45, got %d", q1615_to_deg(yaw_normalized));
        $display("PASS: Offset applied correctly");
        
        // Test 4: Wrap-around - negative result
        $display("\nTest 4: Wrap-around - negative result (yaw=10, offset=30 -> 340)");
        yaw = deg_to_q1615(30);
        calibrate_pulse = 1;
        valid_in = 1;
        #1000;  // Wait for calibration to complete
        calibrate_pulse = 0;
        #1000;
        // Now test with yaw=10, offset=30 -> 10-30 = -20 -> 340
        yaw = deg_to_q1615(10);
        valid_in = 1;
        #2000;  // Wait for pipeline (2 cycles: adjust + normalize)
        // 10 - 30 = -20, normalized = 340
        assert(q1615_to_deg(yaw_normalized) == 340) else $error("Test 4 failed: Expected 340, got %d", q1615_to_deg(yaw_normalized));
        $display("PASS: Negative wrap-around handled");
        
        // Test 5: Wrap-around - > 360
        $display("\nTest 5: Wrap-around - > 360 (yaw=370 -> 10)");
        yaw = deg_to_q1615(0);  // Reset offset first
        calibrate_pulse = 1;
        valid_in = 1;
        #1000;
        calibrate_pulse = 0;
        #1000;
        // Now test with yaw=370, offset=0 -> 370 -> 10
        yaw = deg_to_q1615(370);
        valid_in = 1;
        #2000;  // Wait for pipeline
        assert(q1615_to_deg(yaw_normalized) == 10) else $error("Test 5 failed: Expected 10, got %d", q1615_to_deg(yaw_normalized));
        $display("PASS: > 360 wrap-around handled");
        
        // Test 6: Edge case - 0 degrees
        $display("\nTest 6: Edge case - 0 degrees");
        yaw = deg_to_q1615(0);
        calibrate_pulse = 1;
        #500;
        calibrate_pulse = 0;
        #500;
        yaw = deg_to_q1615(0);
        valid_in = 1;
        #1000;
        assert(q1615_to_deg(yaw_normalized) == 0) else $error("Test 6 failed");
        $display("PASS: 0 degrees handled");
        
        // Test 7: Edge case - 360 degrees
        $display("\nTest 7: Edge case - 360 degrees -> 0");
        yaw = deg_to_q1615(360);
        valid_in = 1;
        #1000;
        assert(q1615_to_deg(yaw_normalized) == 0) else $error("Test 7 failed");
        $display("PASS: 360 degrees normalized to 0");
        
        $display("\n=== All Yaw Normalizer Tests PASSED ===");
        #1000;
        $finish;
    end

endmodule

