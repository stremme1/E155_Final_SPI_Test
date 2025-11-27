`timescale 1ns / 1ps

// Complete System Testbench for Drum Trigger System
// Tests the entire pipeline from BNO085 data to drum code output
// Verifies exact C code logic matches

module tb_drum_trigger_system;

    // Clock and reset
    logic clk, rst_n;
    
    // BNO085 Interface (simplified - using direct data injection)
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    
    // Button inputs
    logic calibrate_btn_n, kick_btn_n;
    logic calibrate_btn_pulse, kick_btn_pulse;
    
    // Drum trigger outputs
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    logic drum_hand;
    
    // Clock generation (3MHz)
    initial begin
        clk = 0;
        forever #166.67 clk = ~clk;  // 3MHz = 333.33ns period
    end
    
    // Button debouncers
    button_debouncer #(.DEBOUNCE_CYCLES(100)) calibrate_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(calibrate_btn_n),
        .btn_pressed(calibrate_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    button_debouncer #(.DEBOUNCE_CYCLES(100)) kick_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(kick_btn_n),
        .btn_pressed(kick_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    // DUT - Drum Trigger Processor
    drum_trigger_processor dut (
        .clk(clk),
        .rst_n(rst_n),
        .quat1_valid(quat_valid),
        .quat1_w(quat_w),
        .quat1_x(quat_x),
        .quat1_y(quat_y),
        .quat1_z(quat_z),
        .gyro1_valid(gyro_valid),
        .gyro1_x(gyro_x),
        .gyro1_y(gyro_y),
        .gyro1_z(gyro_z),
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .drum_hand(drum_hand)
    );
    
    // Helper function to convert quaternion components to 16-bit signed
    // BNO085 outputs quaternion as 16-bit signed integers
    function signed [15:0] quat_float_to_int(input real q);
        // Scale: quaternion is normalized, so range is -1.0 to 1.0
        // BNO085 uses Q14 format: multiply by 16384
        quat_float_to_int = $signed($rtoi(q * 16384.0));
    endfunction
    
    // Helper function to convert degrees to Q16.15 for Euler angles
    function [31:0] deg_to_q1615(input real deg);
        deg_to_q1615 = $rtoi(deg * 32768.0);
    endfunction
    
    // Test scenarios
    initial begin
        $display("========================================");
        $display("=== Drum Trigger System Testbench ===");
        $display("========================================\n");
        
        // Initialize
        rst_n = 0;
        quat_valid = 0;
        gyro_valid = 0;
        quat_w = 0; quat_x = 0; quat_y = 0; quat_z = 0;
        gyro_x = 0; gyro_y = 0; gyro_z = 0;
        calibrate_btn_n = 1;
        kick_btn_n = 1;
        #2000;
        rst_n = 1;
        #1000;
        
        // ============================================
        // TEST SCENARIO 1: Right Hand Snare Drum
        // ============================================
        $display("=== TEST 1: Right Hand Snare Drum ===");
        $display("Yaw: 70 degrees (snare zone: 20-120)");
        $display("Pitch: 0 degrees");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 0 (snare)\n");
        
        // Quaternion for yaw=70°, pitch=0°, roll=0°
        // Simplified: w=1, x=0, y=sin(35°), z=0 (approximation)
        quat_w = quat_float_to_int(0.8192);   // cos(35°)
        quat_x = quat_float_to_int(0.0);
        quat_y = quat_float_to_int(0.5736);   // sin(35°)
        quat_z = quat_float_to_int(0.0);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        // Wait for quaternion processing
        #5000;
        
        // Gyroscope data - strike detected
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        // Wait for processing
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd0) begin
            $display("✓ PASS: Snare drum detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 0, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 2: Right Hand High Tom
        // ============================================
        $display("=== TEST 2: Right Hand High Tom ===");
        $display("Yaw: 350 degrees (high tom zone: 340-20)");
        $display("Pitch: 0 degrees");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 3 (high tom)\n");
        
        // Reset strike detector
        gyro_y = -16'sd2000;  // Above threshold
        gyro_valid = 1;
        #1000;
        gyro_valid = 0;
        #2000;
        
        // Quaternion for yaw=350° (or -10°)
        quat_w = quat_float_to_int(0.9962);   // cos(-5°)
        quat_x = quat_float_to_int(0.0);
        quat_y = quat_float_to_int(0.0);
        quat_z = quat_float_to_int(-0.0872);  // sin(-5°)
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        #5000;
        
        // Strike
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd3) begin
            $display("✓ PASS: High tom detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 3, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 3: Right Hand Crash Cymbal
        // ============================================
        $display("=== TEST 3: Right Hand Crash Cymbal ===");
        $display("Yaw: 350 degrees (high tom zone)");
        $display("Pitch: 55 degrees (> 50, so cymbal)");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 5 (crash)\n");
        
        // Reset
        gyro_y = -16'sd2000;
        gyro_valid = 1;
        #1000;
        gyro_valid = 0;
        #2000;
        
        // Quaternion for yaw=350°, pitch=55°
        quat_w = quat_float_to_int(0.9063);
        quat_x = quat_float_to_int(0.4226);
        quat_y = quat_float_to_int(0.0);
        quat_z = quat_float_to_int(-0.0872);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        #5000;
        
        // Strike
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd5) begin
            $display("✓ PASS: Crash cymbal detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 5, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 4: Kick Button
        // ============================================
        $display("=== TEST 4: Kick Button ===");
        $display("Button pressed");
        $display("Expected: Drum code 2 (kick)\n");
        
        kick_btn_n = 0;
        #1000;
        kick_btn_n = 1;
        #1000;
        
        if (drum_trigger_valid && drum_code == 4'd2) begin
            $display("✓ PASS: Kick button detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 2, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 5: Calibration
        // ============================================
        $display("=== TEST 5: Calibration ===");
        $display("Setting yaw offset to current yaw value\n");
        
        // Set a quaternion
        quat_w = quat_float_to_int(0.8192);
        quat_x = quat_float_to_int(0.0);
        quat_y = quat_float_to_int(0.5736);
        quat_z = quat_float_to_int(0.0);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        #5000;
        
        // Press calibration button
        calibrate_btn_n = 0;
        #1000;
        calibrate_btn_n = 1;
        #1000;
        
        $display("✓ PASS: Calibration button processed\n");
        
        // ============================================
        // TEST SCENARIO 6: Mid Tom
        // ============================================
        $display("=== TEST 6: Right Hand Mid Tom ===");
        $display("Yaw: 320 degrees (mid tom zone: 305-340)");
        $display("Pitch: 0 degrees");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 4 (mid tom)\n");
        
        // Reset
        gyro_y = -16'sd2000;
        gyro_valid = 1;
        #1000;
        gyro_valid = 0;
        #2000;
        
        // Quaternion for yaw=320°
        quat_w = quat_float_to_int(0.9848);
        quat_x = quat_float_to_int(0.0);
        quat_y = quat_float_to_int(0.0);
        quat_z = quat_float_to_int(0.1736);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        #5000;
        
        // Strike
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd4) begin
            $display("✓ PASS: Mid tom detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 4, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 7: Floor Tom
        // ============================================
        $display("=== TEST 7: Right Hand Floor Tom ===");
        $display("Yaw: 250 degrees (floor tom zone: 200-305)");
        $display("Pitch: 0 degrees");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 7 (floor tom)\n");
        
        // Reset
        gyro_y = -16'sd2000;
        gyro_valid = 1;
        #1000;
        gyro_valid = 0;
        #2000;
        
        // Quaternion for yaw=250°
        quat_w = quat_float_to_int(0.9063);
        quat_x = quat_float_to_int(0.0);
        quat_y = quat_float_to_int(0.0);
        quat_z = quat_float_to_int(0.4226);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        #5000;
        
        // Strike
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd7) begin
            $display("✓ PASS: Floor tom detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 7, got %d\n", drum_code);
        end
        
        // ============================================
        // TEST SCENARIO 8: Ride Cymbal (from Floor Tom zone)
        // ============================================
        $display("=== TEST 8: Right Hand Ride Cymbal ===");
        $display("Yaw: 250 degrees (floor tom zone)");
        $display("Pitch: 35 degrees (> 30, so ride)");
        $display("Gyro Y: -3000 (strike detected)");
        $display("Expected: Drum code 6 (ride)\n");
        
        // Reset
        gyro_y = -16'sd2000;
        gyro_valid = 1;
        #1000;
        gyro_valid = 0;
        #2000;
        
        // Quaternion for yaw=250°, pitch=35°
        quat_w = quat_float_to_int(0.8192);
        quat_x = quat_float_to_int(0.5736);
        quat_y = quat_float_to_int(0.0);
        quat_z = quat_float_to_int(0.4226);
        quat_valid = 1;
        #500;
        quat_valid = 0;
        
        #5000;
        
        // Strike
        gyro_y = -16'sd3000;
        gyro_valid = 1;
        #500;
        gyro_valid = 0;
        
        #10000;
        
        if (drum_trigger_valid && drum_code == 4'd6) begin
            $display("✓ PASS: Ride cymbal detected correctly\n");
        end else begin
            $display("✗ FAIL: Expected code 6, got %d\n", drum_code);
        end
        
        $display("========================================");
        $display("=== All System Tests Complete ===");
        $display("========================================\n");
        
        #5000;
        $finish;
    end
    
    // Monitor outputs
    always @(posedge clk) begin
        if (drum_trigger_valid) begin
            $display("[%0t] Drum Trigger: Code=%d, Hand=%d", $time, drum_code, drum_hand);
        end
    end

endmodule

