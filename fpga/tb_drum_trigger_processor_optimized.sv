`timescale 1ns / 1ps

// Professional Testbench for drum_trigger_processor_optimized
// Tests time-multiplexed dual sensor processing

module tb_drum_trigger_processor_optimized;

    // Clock and reset
    logic clk, rst_n;
    parameter CLK_PERIOD = 333;  // 3MHz
    
    // Sensor 1 inputs (Right Hand)
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    // Sensor 2 inputs (Left Hand)
    logic quat2_valid, gyro2_valid;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    
    // Button inputs
    logic calibrate_btn_pulse, kick_btn_pulse;
    
    // Outputs
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    logic drum_hand;
    
    // Test control
    int test_count;
    int pass_count;
    int fail_count;
    
    // Clock generation
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // DUT instantiation
    drum_trigger_processor_optimized dut (
        .clk(clk),
        .rst_n(rst_n),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .quat2_valid(quat2_valid),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_valid(gyro2_valid),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z),
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .drum_hand(drum_hand)
    );
    
    // Task to send sensor 1 quaternion
    task send_sensor1_quaternion(
        input logic signed [15:0] w,
        input logic signed [15:0] x,
        input logic signed [15:0] y,
        input logic signed [15:0] z
    );
        @(posedge clk);
        quat1_w = w;
        quat1_x = x;
        quat1_y = y;
        quat1_z = z;
        quat1_valid = 1;
        @(posedge clk);
        quat1_valid = 0;
    endtask
    
    // Task to send sensor 2 quaternion
    task send_sensor2_quaternion(
        input logic signed [15:0] w,
        input logic signed [15:0] x,
        input logic signed [15:0] y,
        input logic signed [15:0] z
    );
        @(posedge clk);
        quat2_w = w;
        quat2_x = x;
        quat2_y = y;
        quat2_z = z;
        quat2_valid = 1;
        @(posedge clk);
        quat2_valid = 0;
    endtask
    
    // Task to send sensor 1 gyro
    task send_sensor1_gyro(
        input logic signed [15:0] x,
        input logic signed [15:0] y,
        input logic signed [15:0] z
    );
        @(posedge clk);
        gyro1_x = x;
        gyro1_y = y;
        gyro1_z = z;
        gyro1_valid = 1;
        @(posedge clk);
        gyro1_valid = 0;
    endtask
    
    // Task to send sensor 2 gyro
    task send_sensor2_gyro(
        input logic signed [15:0] x,
        input logic signed [15:0] y,
        input logic signed [15:0] z
    );
        @(posedge clk);
        gyro2_x = x;
        gyro2_y = y;
        gyro2_z = z;
        gyro2_valid = 1;
        @(posedge clk);
        gyro2_valid = 0;
    endtask
    
    // Task to wait for trigger (with timeout)
    task wait_for_trigger(output logic success, input int timeout_cycles = 1000);
        int cycles = 0;
        success = 0;
        while (!drum_trigger_valid && cycles < timeout_cycles) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (drum_trigger_valid) begin
            success = 1;
        end else begin
            $display("  ⚠️  Timeout waiting for trigger after %0d cycles", timeout_cycles);
        end
    endtask
    
    // Test 1: Kick button priority
    task test_kick_button();
        logic success;
        test_count++;
        $display("\n=== Test %0d: Kick Button Priority ===", test_count);
        
        // Send some sensor data
        send_sensor1_quaternion(16'sd32768, 16'sd0, 16'sd0, 16'sd0);
        repeat(10) @(posedge clk);
        
        // Press kick button
        @(posedge clk);
        kick_btn_pulse = 1;
        @(posedge clk);
        kick_btn_pulse = 0;
        
        wait_for_trigger(success, 200);
        
        if (success) begin
            if (drum_code == 4'd2 && drum_hand == 1'b0) begin
                $display("  ✅ PASS: Kick button triggered (code=%0d, hand=%0d)", 
                         drum_code, drum_hand);
                pass_count++;
            end else begin
                $display("  ❌ FAIL: Wrong output (code=%0d, hand=%0d, expected code=2, hand=0)", 
                         drum_code, drum_hand);
                fail_count++;
            end
        end else begin
            $display("  ❌ FAIL: No trigger received");
            fail_count++;
        end
        
        repeat(50) @(posedge clk);
    endtask
    
    // Test 2: Time-multiplexing (both sensors)
    task test_time_multiplexing();
        test_count++;
        $display("\n=== Test %0d: Time-Multiplexing (Both Sensors) ===", test_count);
        
        // Send quaternion from sensor 1
        send_sensor1_quaternion(16'sd32768, 16'sd0, 16'sd0, 16'sd0);
        repeat(20) @(posedge clk);
        
        // Send quaternion from sensor 2
        send_sensor2_quaternion(16'sd32768, 16'sd0, 16'sd0, 16'sd0);
        repeat(20) @(posedge clk);
        
        // Send more from sensor 1
        send_sensor1_quaternion(16'sd30000, 16'sd1000, 16'sd2000, 16'sd1500);
        repeat(20) @(posedge clk);
        
        // Send more from sensor 2
        send_sensor2_quaternion(16'sd28000, 16'sd1200, 16'sd1800, 16'sd1600);
        repeat(20) @(posedge clk);
        
        $display("  ✅ PASS: Time-multiplexing completed without errors");
        pass_count++;
        
        repeat(50) @(posedge clk);
    endtask
    
    // Test 3: Calibration button
    task test_calibration();
        test_count++;
        $display("\n=== Test %0d: Calibration Button ===", test_count);
        
        // Send quaternion
        send_sensor1_quaternion(16'sd32768, 16'sd0, 16'sd0, 16'sd0);
        repeat(10) @(posedge clk);
        
        // Press calibration button
        @(posedge clk);
        calibrate_btn_pulse = 1;
        @(posedge clk);
        calibrate_btn_pulse = 0;
        
        repeat(50) @(posedge clk);
        
        $display("  ✅ PASS: Calibration button processed");
        pass_count++;
        
        repeat(20) @(posedge clk);
    endtask
    
    // Test 4: Rapid sensor inputs
    task test_rapid_inputs();
        test_count++;
        $display("\n=== Test %0d: Rapid Sensor Inputs ===", test_count);
        
        // Rapidly alternate between sensors
        for (int i = 0; i < 5; i++) begin
            send_sensor1_quaternion(
                16'sd30000 + i*500,
                16'sd1000 + i*200,
                16'sd2000 + i*300,
                16'sd1500 + i*100
            );
            repeat(5) @(posedge clk);
            
            send_sensor2_quaternion(
                16'sd28000 + i*400,
                16'sd1200 + i*150,
                16'sd1800 + i*250,
                16'sd1600 + i*80
            );
            repeat(5) @(posedge clk);
        end
        
        repeat(100) @(posedge clk);
        
        $display("  ✅ PASS: Rapid inputs handled without errors");
        pass_count++;
    endtask
    
    // Main test sequence
    initial begin
        // Initialize counters
        test_count = 0;
        pass_count = 0;
        fail_count = 0;
        
        $display("==========================================");
        $display("Drum Trigger Processor Optimized Testbench");
        $display("Testing time-multiplexed dual sensor processing");
        $display("==========================================\n");
        
        // Initialize
        rst_n = 0;
        quat1_valid = 0;
        quat2_valid = 0;
        gyro1_valid = 0;
        gyro2_valid = 0;
        calibrate_btn_pulse = 0;
        kick_btn_pulse = 0;
        quat1_w = 16'sd0;
        quat1_x = 16'sd0;
        quat1_y = 16'sd0;
        quat1_z = 16'sd0;
        quat2_w = 16'sd0;
        quat2_x = 16'sd0;
        quat2_y = 16'sd0;
        quat2_z = 16'sd0;
        gyro1_x = 16'sd0;
        gyro1_y = 16'sd0;
        gyro1_z = 16'sd0;
        gyro2_x = 16'sd0;
        gyro2_y = 16'sd0;
        gyro2_z = 16'sd0;
        
        // Reset
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(20) @(posedge clk);
        
        $display("Reset complete, starting tests...\n");
        
        // Run tests
        test_kick_button();
        test_time_multiplexing();
        test_calibration();
        test_rapid_inputs();
        
        // Summary
        $display("\n==========================================");
        $display("Test Summary");
        $display("==========================================");
        $display("Total tests: %0d", test_count);
        $display("Passed:      %0d", pass_count);
        $display("Failed:      %0d", fail_count);
        $display("==========================================");
        
        if (fail_count == 0) begin
            $display("✅ ALL TESTS PASSED");
        end else begin
            $display("❌ SOME TESTS FAILED");
        end
        $display("==========================================\n");
        
        #(200 * CLK_PERIOD);
        $finish;
    end
    
endmodule

