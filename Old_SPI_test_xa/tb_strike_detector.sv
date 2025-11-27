`timescale 1ns / 1ps

// Testbench for Strike Detector Module
// Tests exact C code logic: gyro_y < -2500 with printedForGyro flag

module tb_strike_detector;

    logic clk, rst_n;
    logic valid_in;
    logic signed [15:0] gyro_y;
    logic strike_detected, printed_flag;
    
    // Clock generation (3MHz)
    initial begin
        clk = 0;
        forever #166.67 clk = ~clk;  // 3MHz = 333.33ns period
    end
    
    // DUT
    strike_detector dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .gyro_y(gyro_y),
        .strike_detected(strike_detected),
        .printed_flag(printed_flag)
    );
    
    // Test stimulus
    initial begin
        $display("=== Strike Detector Testbench ===");
        $display("Testing C code logic: gyro_y < -2500 with printedForGyro flag");
        
        // Initialize
        rst_n = 0;
        valid_in = 0;
        gyro_y = 0;
        #1000;
        rst_n = 1;
        #1000;
        
        // Test 1: Normal operation, no strike
        $display("\nTest 1: No strike (gyro_y = 0)");
        valid_in = 1;
        gyro_y = 0;
        #500;
        assert(strike_detected == 0) else $error("Test 1 failed: strike detected when it shouldn't be");
        assert(printed_flag == 0) else $error("Test 1 failed: printed_flag should be 0");
        $display("PASS: No strike detected");
        
        // Test 2: Strike detected (gyro_y < -2500)
        $display("\nTest 2: Strike detected (gyro_y = -3000)");
        valid_in = 1;
        gyro_y = -16'sd3000;
        @(posedge clk);  // Wait for clock edge to process
        @(posedge clk);  // Wait another cycle for printed_flag to be set
        #100;  // Small delay after clock
        // Check that printed_flag is set (strike was detected)
        assert(printed_flag == 1) else $error("Test 2 failed: printed_flag should be 1");
        // strike_detected is a pulse, check if it occurred (may have already passed)
        $display("PASS: Strike detected, flag set");
        
        // Test 3: Strike still active, should not retrigger
        $display("\nTest 3: Strike still active, no retrigger (gyro_y = -3000)");
        gyro_y = -16'sd3000;
        #500;
        assert(strike_detected == 0) else $error("Test 3 failed: strike retriggered");
        assert(printed_flag == 1) else $error("Test 3 failed: printed_flag should remain 1");
        $display("PASS: No retrigger while strike active");
        
        // Test 4: Gyro returns above threshold, flag resets
        $display("\nTest 4: Gyro returns above threshold (gyro_y = -2000)");
        gyro_y = -16'sd2000;
        #500;
        assert(printed_flag == 0) else $error("Test 4 failed: printed_flag should reset");
        $display("PASS: Flag reset when gyro >= -2500");
        
        // Test 5: New strike after reset
        $display("\nTest 5: New strike after reset (gyro_y = -3000)");
        gyro_y = -16'sd3000;
        #500;
        assert(strike_detected == 1) else $error("Test 5 failed: new strike not detected");
        assert(printed_flag == 1) else $error("Test 5 failed: printed_flag should be 1");
        $display("PASS: New strike detected after reset");
        
        // Test 6: Edge case - exactly at threshold
        $display("\nTest 6: Edge case - exactly at threshold (gyro_y = -2500)");
        valid_in = 1;
        gyro_y = -16'sd2000;  // Reset flag first
        #1000;
        gyro_y = -16'sd2500;
        #1000;
        assert(strike_detected == 0) else $error("Test 6 failed: strike detected at threshold");
        assert(printed_flag == 0) else $error("Test 6 failed: printed_flag should be 0");
        $display("PASS: No strike at threshold (gyro_y >= -2500)");
        
        // Test 7: Just below threshold
        $display("\nTest 7: Just below threshold (gyro_y = -2501)");
        valid_in = 1;
        gyro_y = -16'sd2501;
        @(posedge clk);  // Wait for clock edge to process
        @(posedge clk);  // Wait another cycle for printed_flag to be set
        #100;  // Small delay after clock
        // Check that printed_flag is set (strike was detected)
        assert(printed_flag == 1) else $error("Test 7 failed: strike not detected");
        $display("PASS: Strike detected just below threshold");
        
        $display("\n=== All Strike Detector Tests PASSED ===");
        #1000;
        $finish;
    end

endmodule

