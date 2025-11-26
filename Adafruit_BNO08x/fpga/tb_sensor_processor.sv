/**
 * Testbench for Sensor Processor Module
 * 
 * Tests quaternion parsing, Euler conversion, and drum trigger logic
 */

`timescale 1ns / 1ps

module tb_sensor_processor;

    // Parameters
    parameter CLK_PERIOD = 333; // 3MHz = 333ns period
    
    // Signals
    logic        clk;
    logic        rst_n;
    logic        data_ready;
    logic [7:0]  sensor_report_id;
    logic [7:0]  sensor_data [0:15];
    logic [4:0]  sensor_data_len;
    logic [31:0] quat_w, quat_x, quat_y, quat_z;
    logic [31:0] gyro_x, gyro_y, gyro_z;
    logic [31:0] roll, pitch, yaw;
    logic        euler_valid;
    logic [3:0]  drum_trigger;
    logic        trigger_valid;
    logic [31:0] yaw_offset1;
    logic [31:0] yaw_offset2;
    logic        sensor_select;
    
    // Testbench signals
    logic [7:0]  test_data [0:15];
    
    // Instantiate DUT
    sensor_processor dut (
        .clk(clk),
        .rst_n(rst_n),
        .data_ready(data_ready),
        .sensor_report_id(sensor_report_id),
        .sensor_data(sensor_data),
        .sensor_data_len(sensor_data_len),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .roll(roll),
        .pitch(pitch),
        .yaw(yaw),
        .euler_valid(euler_valid),
        .drum_trigger(drum_trigger),
        .trigger_valid(trigger_valid),
        .yaw_offset1(yaw_offset1),
        .yaw_offset2(yaw_offset2),
        .sensor_select(sensor_select)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // Test tasks
    task reset_dut();
        rst_n = 0;
        data_ready = 0;
        sensor_report_id = 0;
        sensor_data_len = 0;
        yaw_offset1 = 0;
        yaw_offset2 = 0;
        sensor_select = 0;
        for (int i = 0; i < 16; i++) begin
            sensor_data[i] = 0;
        end
        #(10 * CLK_PERIOD);
        rst_n = 1;
        #(5 * CLK_PERIOD);
    endtask
    
    task send_quaternion_report(
        input logic [31:0] q_i,
        input logic [31:0] q_j,
        input logic [31:0] q_k,
        input logic [31:0] q_real
    );
        $display("[%0t] Sending Quaternion Report", $time);
        
        sensor_report_id = 8'h08; // Game Rotation Vector
        sensor_data_len = 19;
        
        // SHTP header (simulated as already parsed)
        sensor_data[0] = 8'h08; // Report ID
        sensor_data[1] = 8'h01; // Sequence
        sensor_data[2] = 8'h03; // Status
        sensor_data[3] = 8'h00; // Delay LSB
        
        // Quaternion i (Q30) - little endian
        sensor_data[4] = q_i[7:0];
        sensor_data[5] = q_i[15:8];
        sensor_data[6] = q_i[23:16];
        sensor_data[7] = q_i[31:24];
        
        // Quaternion j (Q30)
        sensor_data[8] = q_j[7:0];
        sensor_data[9] = q_j[15:8];
        sensor_data[10] = q_j[23:16];
        sensor_data[11] = q_j[31:24];
        
        // Quaternion k (Q30)
        sensor_data[12] = q_k[7:0];
        sensor_data[13] = q_k[15:8];
        sensor_data[14] = q_k[23:16];
        sensor_data[15] = q_k[31:24];
        
        @(posedge clk);
        data_ready = 1;
        @(posedge clk);
        data_ready = 0;
        #(10 * CLK_PERIOD);
    endtask
    
    task send_gyroscope_report(
        input logic [15:0] g_x,
        input logic [15:0] g_y,
        input logic [15:0] g_z
    );
        $display("[%0t] Sending Gyroscope Report", $time);
        
        sensor_report_id = 8'h02; // Calibrated Gyroscope
        sensor_data_len = 10;
        
        sensor_data[0] = 8'h02; // Report ID
        sensor_data[1] = 8'h01; // Sequence
        sensor_data[2] = 8'h03; // Status
        sensor_data[3] = 8'h00; // Delay LSB
        
        // Gyro X (Q16, little endian)
        sensor_data[4] = g_x[7:0];
        sensor_data[5] = g_x[15:8];
        
        // Gyro Y (Q16, little endian)
        sensor_data[6] = g_y[7:0];
        sensor_data[7] = g_y[15:8];
        
        // Gyro Z (Q16, little endian)
        sensor_data[8] = g_z[7:0];
        sensor_data[9] = g_z[15:8];
        
        @(posedge clk);
        data_ready = 1;
        @(posedge clk);
        data_ready = 0;
        #(10 * CLK_PERIOD);
    endtask
    
    // Main test sequence
    initial begin
        $display("=========================================");
        $display("Sensor Processor Testbench");
        $display("=========================================");
        
        // Initialize
        reset_dut();
        
        // Test 1: Quaternion parsing
        $display("\n--- Test 1: Quaternion Parsing ---");
        // Send quaternion with i=0.1, j=0.2, k=0.3, real=0.9 (in Q30)
        send_quaternion_report(
            32'h0CCCCCCD, // 0.1 in Q30
            32'h1999999A, // 0.2 in Q30
            32'h26666666, // 0.3 in Q30
            32'h73333333  // 0.9 in Q30
        );
        
        wait(euler_valid);
        $display("[%0t] Quaternion parsed: i=0x%08X j=0x%08X k=0x%08X", 
                 $time, quat_x, quat_y, quat_z);
        #(10 * CLK_PERIOD);
        
        // Test 2: Gyroscope parsing
        $display("\n--- Test 2: Gyroscope Parsing ---");
        // Send gyro with negative Y (trigger condition)
        send_gyroscope_report(
            16'h0000,  // X = 0
            16'hF63C,  // Y = -2500 (trigger threshold)
            16'h0000   // Z = 0
        );
        
        #(10 * CLK_PERIOD);
        $display("[%0t] Gyroscope parsed: x=0x%04X y=0x%04X z=0x%04X", 
                 $time, gyro_x[15:0], gyro_y[15:0], gyro_z[15:0]);
        
        // Test 3: Drum trigger - Right hand, Yaw 20-120, Snare
        $display("\n--- Test 3: Drum Trigger - Right Hand Snare ---");
        sensor_select = 0; // Right hand
        yaw_offset1 = 0;
        
        // Send quaternion for yaw = 60 degrees
        // This is a simplified test - actual quaternion would need proper calculation
        send_quaternion_report(
            32'h00000000,
            32'h00000000,
            32'h2AAAAAAA, // Simplified quaternion for ~60 deg yaw
            32'h40000000  // Real component
        );
        
        // Send gyro with negative Y
        send_gyroscope_report(16'h0000, 16'hF63C, 16'h0000);
        
        #(10 * CLK_PERIOD);
        if (trigger_valid) begin
            $display("[%0t] ✓ Drum trigger: %0d", $time, drum_trigger);
        end else begin
            $display("[%0t] ✗ No trigger", $time);
        end
        
        // Test 4: Drum trigger - Left hand, Hi-hat
        $display("\n--- Test 4: Drum Trigger - Left Hand Hi-hat ---");
        sensor_select = 1; // Left hand
        yaw_offset2 = 0;
        
        // Send quaternion for yaw = 10 degrees, pitch = 40 degrees
        send_quaternion_report(
            32'h00000000,
            32'h2AAAAAAA, // Pitch component
            32'h0AAAAAAA, // Yaw component
            32'h40000000
        );
        
        // Send gyro with negative Y but positive Z
        send_gyroscope_report(16'h0000, 16'hF63C, 16'hF830); // Z > -2000
        
        #(10 * CLK_PERIOD);
        if (trigger_valid) begin
            $display("[%0t] ✓ Drum trigger: %0d (expected 1 for hi-hat)", 
                     $time, drum_trigger);
        end else begin
            $display("[%0t] ✗ No trigger", $time);
        end
        
        // Test 5: Yaw offset
        $display("\n--- Test 5: Yaw Offset ---");
        sensor_select = 0;
        yaw_offset1 = 32'd45 << 16; // 45 degrees offset
        
        send_quaternion_report(
            32'h00000000,
            32'h00000000,
            32'h2AAAAAAA, // ~60 deg yaw
            32'h40000000
        );
        
        send_gyroscope_report(16'h0000, 16'hF63C, 16'h0000);
        
        #(10 * CLK_PERIOD);
        $display("[%0t] Yaw with offset: 0x%08X", $time, yaw);
        
        $display("\n=========================================");
        $display("All Tests Complete");
        $display("=========================================");
        #(100 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor signals
    initial begin
        $monitor("[%0t] DATA_RDY=%b EULER_VALID=%b TRIGGER=%b TRIG_VALID=%b YAW=0x%08X", 
                 $time, data_ready, euler_valid, drum_trigger, trigger_valid, yaw);
    end

endmodule

