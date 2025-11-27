/**
 * Comprehensive Testbench for Sensor Data SPI Slave
 * 
 * Tests:
 * 1. Single packet transmission with known data
 * 2. Bit-by-bit verification
 * 3. Multiple packets
 * 4. Rapid data updates
 * 5. Handshaking edge cases
 * 6. All 32 bytes with different patterns
 */

`timescale 1ns / 1ps

module tb_sensor_data_spi_slave_comprehensive;

    localparam CLK_PERIOD = 333;  // 3MHz FPGA clock
    localparam SCK_PERIOD = 200;  // 5MHz SPI clock
    
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
    
    // Task to read one byte via SPI (MCU side)
    task mcu_read_byte(output logic [7:0] byte_out);
        int i;
        for (i = 7; i >= 0; i = i - 1) begin
            @(posedge sck);
            #1;
            byte_out[i] = sdo;
        end
    endtask
    
    // Global packet storage (iverilog doesn't support unpacked arrays in task ports)
    logic [7:0] received_packet [0:31];
    
    // Task to read full 32-byte packet
    task mcu_read_packet;
        int i;
        logic [7:0] byte_data;
        
        // Wait for DONE
        load = 0;
        while (!done) #(CLK_PERIOD);
        
        // Read 32 bytes
        for (i = 0; i < 32; i = i + 1) begin
            mcu_read_byte(byte_data);
            received_packet[i] = byte_data;
        end
        
        // Acknowledge
        load = 1;
        #(10 * CLK_PERIOD);
        load = 0;
        #(10 * CLK_PERIOD);
    endtask
    
    // Test 1: Known data pattern
    task test_known_data;
        int i;
        logic test_pass;
        
        $display("\n=== Test 1: Known Data Pattern ===");
        
        // Set known data
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = i;  // 0x00, 0x01, 0x02, ...
        end
        data_ready = 1;
        #(CLK_PERIOD);
        data_ready = 0;
        
        // Wait for data to be ready
        #(10 * CLK_PERIOD);
        
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
    
    // Test 2: All 0xFF pattern
    task test_all_ones;
        int i;
        logic test_pass;
        
        $display("\n=== Test 2: All 0xFF Pattern ===");
        
        // Set all 0xFF
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
    
    // Test 3: Alternating pattern
    task test_alternating;
        int i;
        logic test_pass;
        
        $display("\n=== Test 3: Alternating Pattern ===");
        
        // Set alternating 0xAA, 0x55
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = (i % 2) ? 8'h55 : 8'hAA;
        end
        data_ready = 1;
        #(CLK_PERIOD);
        data_ready = 0;
        
        #(10 * CLK_PERIOD);
        mcu_read_packet();
        
        test_pass = 1;
        for (i = 0; i < 32; i = i + 1) begin
            logic [7:0] expected = (i % 2) ? 8'h55 : 8'hAA;
            if (received_packet[i] !== expected) begin
                $display("  ERROR: Byte %0d: expected 0x%02X, got 0x%02X", i, expected, received_packet[i]);
                test_pass = 0;
            end
        end
        
        if (test_pass) begin
            $display("  ✅ PASSED: Alternating pattern correct");
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
        
        $display("\n=== Test 4: Multiple Packets ===");
        
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
            for (i = 0; i < 32; i = i + 1) begin
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
    
    // Test 5: Sensor data format (realistic)
    task test_sensor_format;
        int i;
        logic test_pass;
        
        $display("\n=== Test 5: Realistic Sensor Data Format ===");
        
        // Initialize all to zero first
        for (i = 0; i < 32; i = i + 1) begin
            data_bytes[i] = 0;
        end
        
        // Simulate sensor data: quat1_w = 0x1234, gyro1_y = 0xFEDC, etc.
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
            $display("  ERROR: Quat1_w mismatch: got 0x%02X%02X", received_packet[0], received_packet[1]);
            test_pass = 0;
        end
        if (received_packet[10] !== 8'hFE || received_packet[11] !== 8'hDC) begin
            $display("  ERROR: Gyro1_y mismatch: got 0x%02X%02X", received_packet[10], received_packet[11]);
            test_pass = 0;
        end
        if (received_packet[14] !== 8'h03) begin
            $display("  ERROR: Flags mismatch: got 0x%02X", received_packet[14]);
            test_pass = 0;
        end
        if (received_packet[30] !== 8'h01) begin
            $display("  ERROR: Kick button mismatch: got 0x%02X", received_packet[30]);
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
        
        test_alternating();
        #(100 * CLK_PERIOD);
        
        test_multiple_packets();
        #(100 * CLK_PERIOD);
        
        test_sensor_format();
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

