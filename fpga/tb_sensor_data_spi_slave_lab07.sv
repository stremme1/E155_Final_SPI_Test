/**
 * Testbench for Sensor Data SPI Slave
 * Based on Lab07 aes_spi_tb pattern
 * Tests 32-byte packet transmission
 */

`timescale 1ns / 1ps

module tb_sensor_data_spi_slave_lab07;

    localparam CLK_PERIOD = 333;  // 3MHz FPGA clock
    localparam SCK_PERIOD = 200;   // 5MHz SPI clock
    
    logic clk, sck, sdi, sdo, load, done;
    logic [7:0] data_bytes [0:31];
    logic data_ready, data_ack;
    
    int test_passed = 0;
    int test_failed = 0;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    initial begin
        sck = 0;
        forever #(SCK_PERIOD/2) sck = ~sck;
    end
    
    // DUT
    sensor_data_spi_slave dut (
        .clk(clk),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .load(load),
        .done(done),
        .data_bytes(data_bytes),
        .data_ready(data_ready),
        .data_ack(data_ack)
    );
    
    // Global packet storage
    logic [7:0] received_packet [0:31];
    
    // Task to read 32-byte packet (following Lab07 pattern)
    task mcu_read_packet;
        int i, j;
        logic [7:0] byte_data;
        
        // Wait for DONE (like Lab07 waits for done)
        load = 0;
        while (!done) #(CLK_PERIOD);
        
        $display("[%0t] MCU: DONE detected, starting SPI read", $time);
        
        // Read 32 bytes (256 bits) - MCU samples on rising edge in SPI Mode 0
        for (i = 0; i < 32; i = i + 1) begin
            byte_data = 8'h00;
            // Read 8 bits MSB first
            for (j = 7; j >= 0; j = j - 1) begin
                @(posedge sck);  // Wait for rising edge
                #1;  // Small delay after edge
                byte_data[j] = sdo;
            end
            received_packet[i] = byte_data;
            $display("[%0t] MCU: Byte %0d = 0x%02X", $time, i, byte_data);
        end
        
        // Acknowledge by pulling LOAD high (then low)
        load = 1;
        #(10 * CLK_PERIOD);
        load = 0;
        #(10 * CLK_PERIOD);
        
        $display("[%0t] MCU: Packet read complete", $time);
    endtask
    
    // Test 1: Known data pattern (0x00, 0x01, ..., 0x1F)
    task test_known_data;
        int i;
        logic test_pass;
        
        $display("\n=== Test 1: Known Data Pattern (0x00-0x1F) ===");
        
        // Set known data FIRST (before asserting data_ready)
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = i;
        end
        #(CLK_PERIOD);  // Ensure data_bytes is stable
        
        // Now assert data_ready
        data_ready = 1;
        #(CLK_PERIOD);
        data_ready = 0;
        
        // Wait for data to be ready and done to assert
        #(20 * CLK_PERIOD);
        
        // Read packet
        mcu_read_packet();
        
        // Verify
        test_pass = 1;
        for (i = 0; i < 32; i = i + 1) begin
            if (received_packet[i] !== i) begin
                $display("  ERROR: Byte %0d: expected 0x%02X, got 0x%02X", i, i, received_packet[i]);
                test_pass = 0;
            end
        end
        
        if (test_pass) begin
            $display("  ✅ PASSED: All 32 bytes match");
            test_passed++;
        end else begin
            $display("  ❌ FAILED: Data mismatch");
            test_failed++;
        end
    endtask
    
    // Test 2: All 0xFF
    task test_all_ones;
        int i;
        logic test_pass;
        
        $display("\n=== Test 2: All 0xFF Pattern ===");
        
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = 8'hFF;
        end
        data_ready = 1;
        #(CLK_PERIOD);
        data_ready = 0;
        
        #(10 * CLK_PERIOD);
        mcu_read_packet();
        
        test_pass = 1;
        for (i = 0; i < 32; i = i + 1) begin
            if (received_packet[i] !== 8'hFF) begin
                $display("  ERROR: Byte %0d: expected 0xFF, got 0x%02X", i, received_packet[i]);
                test_pass = 0;
            end
        end
        
        if (test_pass) begin
            $display("  ✅ PASSED: All bytes are 0xFF");
            test_passed++;
        end else begin
            $display("  ❌ FAILED");
            test_failed++;
        end
    endtask
    
    // Test 3: Sensor data format (realistic values)
    task test_sensor_format;
        int i;
        logic test_pass;
        
        $display("\n=== Test 3: Realistic Sensor Data Format ===");
        
        // Initialize all to zero
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = 0;
        end
        
        // Set specific sensor values
        data_bytes[0] = 8'h12;  // quat1_w MSB
        data_bytes[1] = 8'h34;  // quat1_w LSB
        data_bytes[10] = 8'hFE; // gyro1_y MSB
        data_bytes[11] = 8'hDC; // gyro1_y LSB
        data_bytes[14] = 8'h03; // flags: both valid
        data_bytes[30] = 8'h01; // kick button
        data_bytes[31] = 8'h00; // no calibrate
        
        data_ready = 1;
        #(CLK_PERIOD);
        data_ready = 0;
        
        #(10 * CLK_PERIOD);
        mcu_read_packet();
        
        test_pass = 1;
        if (received_packet[0] !== 8'h12 || received_packet[1] !== 8'h34) begin
            $display("  ERROR: Quat1_w: got 0x%02X%02X", received_packet[0], received_packet[1]);
            test_pass = 0;
        end
        if (received_packet[10] !== 8'hFE || received_packet[11] !== 8'hDC) begin
            $display("  ERROR: Gyro1_y: got 0x%02X%02X", received_packet[10], received_packet[11]);
            test_pass = 0;
        end
        if (received_packet[14] !== 8'h03) begin
            $display("  ERROR: Flags: got 0x%02X", received_packet[14]);
            test_pass = 0;
        end
        if (received_packet[30] !== 8'h01) begin
            $display("  ERROR: Kick button: got 0x%02X", received_packet[30]);
            test_pass = 0;
        end
        
        if (test_pass) begin
            $display("  ✅ PASSED: Sensor data format correct");
            test_passed++;
        end else begin
            $display("  ❌ FAILED");
            test_failed++;
        end
    endtask
    
    // Test 4: Multiple packets
    task test_multiple_packets;
        int i, j;
        logic test_pass;
        
        $display("\n=== Test 4: Multiple Packets (5 packets) ===");
        
        test_pass = 1;
        for (j = 0; j < 5; j = j + 1) begin
            // Set data with packet number
            for (i = 0; i < 32; i = i + 1) begin
                data_bytes[i] = j * 32 + i;
            end
            data_ready = 1;
            #(CLK_PERIOD);
            data_ready = 0;
            
            #(10 * CLK_PERIOD);
            mcu_read_packet();
            
            // Verify
            for (i = 0; i < 32 && test_pass; i = i + 1) begin
                if (received_packet[i] !== (j * 32 + i)) begin
                    $display("  ERROR: Packet %0d, Byte %0d: expected 0x%02X, got 0x%02X", 
                             j, i, j * 32 + i, received_packet[i]);
                    test_pass = 0;
                end
            end
        end
        
        if (test_pass) begin
            $display("  ✅ PASSED: All 5 packets correct");
            test_passed++;
        end else begin
            $display("  ❌ FAILED");
            test_failed++;
        end
    endtask
    
    // Main test sequence
    initial begin
        // Initialize
        sdi = 0;
        load = 0;
        data_ready = 0;
        for (int i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = 0;
        end
        
        #(100 * CLK_PERIOD);
        
        // Run tests
        test_known_data();
        #(100 * CLK_PERIOD);
        
        test_all_ones();
        #(100 * CLK_PERIOD);
        
        test_sensor_format();
        #(100 * CLK_PERIOD);
        
        test_multiple_packets();
        #(100 * CLK_PERIOD);
        
        // Summary
        $display("\n========================================");
        $display("Test Summary:");
        $display("  Passed: %0d", test_passed);
        $display("  Failed: %0d", test_failed);
        $display("========================================\n");
        
        if (test_failed == 0) begin
            $display("✅ ALL TESTS PASSED!");
        end else begin
            $display("❌ SOME TESTS FAILED");
        end
        
        #(1000 * CLK_PERIOD);
        $finish;
    end
    
endmodule

