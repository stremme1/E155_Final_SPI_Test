`timescale 1ns / 1ps

// Testbench for Arduino SPI Slave Module
// Tests SPI slave operation for receiving 16-byte packets from Arduino
// Verifies packet reception, header validation, and data mapping to MCU interface
// SPI Mode 0 (CPOL=0, CPHA=0) at 100kHz (10us period)

module tb_arduino_spi_slave;

    // Clock
    logic clk;
    
    // Arduino SPI interface (Arduino is master, FPGA is slave)
    logic cs_n;      // Chip select from Arduino (active low)
    logic sck;       // SPI clock from Arduino
    logic sdi;       // SPI data in (MOSI from Arduino)
    
    // DUT outputs
    logic initialized;
    logic error;
    logic quat1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic gyro1_valid;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;  // 3MHz = 333ns period
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // Arduino SPI clock - 100kHz (10us period = 10000ns)
    // SCK starts idle low (SPI Mode 0: CPOL=0)
    parameter SCK_PERIOD = 10000;  // 100kHz = 10us period
    initial begin
        sck = 0;  // Idle low for SPI Mode 0
    end
    
    // DUT
    arduino_spi_slave dut (
        .clk(clk),
        .cs_n(cs_n),
        .sck(sck),
        .sdi(sdi),
        .initialized(initialized),
        .error(error),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z)
    );
    
    // Access internal signals for debugging
    wire [7:0] packet_buffer_0 = dut.packet_buffer[0];
    wire [7:0] packet_buffer_1 = dut.packet_buffer[1];
    wire [7:0] packet_buffer_2 = dut.packet_buffer[2];
    wire [3:0] byte_count = dut.byte_count;
    wire [2:0] bit_count = dut.bit_count;
    
    // Test tracking
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    
    // Helper task to check and report
    task check_test(input string test_name, input logic condition);
        test_count = test_count + 1;
        if (condition) begin
            $display("[PASS] %s", test_name);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] %s", test_name);
            fail_count = fail_count + 1;
        end
    endtask
    
    // Helper task to wait for clock domain crossing
    task wait_cdc;
        #(CLK_PERIOD * 20);  // Wait 20 clock cycles for CDC and packet processing
    endtask
    
    // Task to send a byte via SPI (MSB first, Mode 0)
    // Assumes CS is already low
    task send_spi_byte(input [7:0] data);
        integer i;
        for (i = 7; i >= 0; i = i - 1) begin
            // Set data before rising edge (setup time)
            sdi = data[i];
            #(SCK_PERIOD/4);  // Setup time
            sck = 1;  // Rising edge - FPGA samples here
            #(SCK_PERIOD/2);  // Hold time
            sck = 0;  // Falling edge
            #(SCK_PERIOD/4);  // Recovery time
        end
    endtask
    
    // Task to send a complete 16-byte packet (inline version to avoid unpacked array issues)
    task send_packet_inline;
        input [7:0] b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15;
        // CS falling edge
        cs_n = 1;
        #(SCK_PERIOD * 2);
        cs_n = 0;
        #(SCK_PERIOD);
        
        // Send all 16 bytes
        send_spi_byte(b0);
        send_spi_byte(b1);
        send_spi_byte(b2);
        send_spi_byte(b3);
        send_spi_byte(b4);
        send_spi_byte(b5);
        send_spi_byte(b6);
        send_spi_byte(b7);
        send_spi_byte(b8);
        send_spi_byte(b9);
        send_spi_byte(b10);
        send_spi_byte(b11);
        send_spi_byte(b12);
        send_spi_byte(b13);
        send_spi_byte(b14);
        send_spi_byte(b15);
        
        // CS rising edge (transaction complete)
        #(SCK_PERIOD);
        cs_n = 1;
        #(SCK_PERIOD * 2);
    endtask
    
    // Main test sequence
    initial begin
        $dumpfile("tb_arduino_spi_slave.vcd");
        $dumpvars(0, tb_arduino_spi_slave);
        
        $display("========================================");
        $display("Arduino SPI Slave Testbench");
        $display("========================================");
        $display("");
        
        // Initialize
        cs_n = 1;
        sck = 0;
        sdi = 0;
        
        #(10 * CLK_PERIOD);
        
        // ========================================
        // TEST 1: Valid Packet Reception
        // ========================================
        $display("=== Test 1: Valid Packet Reception ===");
        
        // Test 1.1: Valid packet with header 0xAA
        begin
            // Header = 0xAA
            // Roll = 1000 (0.01° resolution = 10.00°)
            // Pitch = -500 (-5.00°)
            // Yaw = 2000 (20.00°)
            // Gyro X = 100, Y = -200, Z = 50
            // Flags: both valid (0x03)
            send_packet_inline(
                8'hAA,  // Header
                8'h03, 8'hE8,  // Roll = 1000
                8'hFE, 8'h0C,  // Pitch = -500
                8'h07, 8'hD0,  // Yaw = 2000
                8'h00, 8'h64,  // Gyro X = 100
                8'hFF, 8'h38,  // Gyro Y = -200
                8'h00, 8'h32,  // Gyro Z = 50
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            // Wait for CS rising edge to be detected and packet to be processed
            // Wait for CS to go high, then wait for synchronization and processing
            wait(cs_n == 1'b1);  // Wait for CS to go high
            repeat(15) @(posedge clk);  // Wait for CDC synchronization (need more cycles for first packet)
            
            // Check outputs
            check_test("Test 1.1a: Header valid → initialized = 1", initialized == 1'b1);
            check_test("Test 1.1b: Header valid → error = 0", error == 1'b0);
            check_test("Test 1.1c: Quat W = 16384 (Q14 format)", quat1_w == 16'd16384);
            check_test("Test 1.1d: Quat X = Roll (1000)", quat1_x == 16'd1000);
            check_test("Test 1.1e: Quat Y = Pitch (-500)", quat1_y == -16'd500);
            check_test("Test 1.1f: Quat Z = Yaw (2000)", quat1_z == 16'd2000);
            check_test("Test 1.1g: Gyro X = 100", gyro1_x == 16'd100);
            check_test("Test 1.1h: Gyro Y = -200", gyro1_y == -16'd200);
            check_test("Test 1.1i: Gyro Z = 50", gyro1_z == 16'd50);
            check_test("Test 1.1j: Quat valid = 1 (from flags[0])", quat1_valid == 1'b1);
            check_test("Test 1.1k: Gyro valid = 1 (from flags[1])", gyro1_valid == 1'b1);
        end
        
        $display("");
        
        // ========================================
        // TEST 2: Invalid Header Detection
        // ========================================
        $display("=== Test 2: Invalid Header Detection ===");
        
        // Test 2.1: Invalid header (0x55 instead of 0xAA)
        begin
            send_packet_inline(
                8'h55,  // Invalid header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h00,  // Flags
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 2.1a: Invalid header → initialized = 0", initialized == 1'b0);
            check_test("Test 2.1b: Invalid header → error = 1", error == 1'b1);
        end
        
        // Test 2.2: Valid header restores status
        begin
            send_packet_inline(
                8'hAA,  // Valid header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 2.2a: Valid header restores initialized = 1", initialized == 1'b1);
            check_test("Test 2.2b: Valid header clears error = 0", error == 1'b0);
        end
        
        $display("");
        
        // ========================================
        // TEST 3: Flag Mapping
        // ========================================
        $display("=== Test 3: Flag Mapping ===");
        
        // Test 3.1: Only Euler valid (flag = 0x01)
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h01,  // Flags: only Euler valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 3.1a: Flag 0x01 → quat_valid = 1", quat1_valid == 1'b1);
            check_test("Test 3.1b: Flag 0x01 → gyro_valid = 0", gyro1_valid == 1'b0);
        end
        
        // Test 3.2: Only Gyro valid (flag = 0x02)
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h02,  // Flags: only Gyro valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 3.2a: Flag 0x02 → quat_valid = 0", quat1_valid == 1'b0);
            check_test("Test 3.2b: Flag 0x02 → gyro_valid = 1", gyro1_valid == 1'b1);
        end
        
        // Test 3.3: Both valid (flag = 0x03)
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 3.3a: Flag 0x03 → quat_valid = 1", quat1_valid == 1'b1);
            check_test("Test 3.3b: Flag 0x03 → gyro_valid = 1", gyro1_valid == 1'b1);
        end
        
        // Test 3.4: Neither valid (flag = 0x00)
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z
                8'h00,  // Flags: neither valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 3.4a: Flag 0x00 → quat_valid = 0", quat1_valid == 1'b0);
            check_test("Test 3.4b: Flag 0x00 → gyro_valid = 0", gyro1_valid == 1'b0);
        end
        
        $display("");
        
        // ========================================
        // TEST 4: Edge Cases
        // ========================================
        $display("=== Test 4: Edge Cases ===");
        
        // Test 4.1: Maximum positive values
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h7F, 8'hFF,  // Roll = 32767
                8'h7F, 8'hFF,  // Pitch = 32767
                8'h7F, 8'hFF,  // Yaw = 32767
                8'h7F, 8'hFF,  // Gyro X = 32767
                8'h7F, 8'hFF,  // Gyro Y = 32767
                8'h7F, 8'hFF,  // Gyro Z = 32767
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 4.1a: Max positive Roll (32767)", quat1_x == 16'd32767);
            check_test("Test 4.1b: Max positive Pitch (32767)", quat1_y == 16'd32767);
            check_test("Test 4.1c: Max positive Yaw (32767)", quat1_z == 16'd32767);
            check_test("Test 4.1d: Max positive Gyro X (32767)", gyro1_x == 16'd32767);
        end
        
        // Test 4.2: Maximum negative values
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h80, 8'h00,  // Roll = -32768
                8'h80, 8'h00,  // Pitch = -32768
                8'h80, 8'h00,  // Yaw = -32768
                8'h80, 8'h00,  // Gyro X = -32768
                8'h80, 8'h00,  // Gyro Y = -32768
                8'h80, 8'h00,  // Gyro Z = -32768
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 4.2a: Max negative Roll (-32768)", quat1_x == -16'd32768);
            check_test("Test 4.2b: Max negative Pitch (-32768)", quat1_y == -16'd32768);
            check_test("Test 4.2c: Max negative Yaw (-32768)", quat1_z == -16'd32768);
            check_test("Test 4.2d: Max negative Gyro X (-32768)", gyro1_x == -16'd32768);
        end
        
        // Test 4.3: Zero values
        begin
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Roll, Pitch, Yaw = 0
                8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,  // Gyro X, Y, Z = 0
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            
            check_test("Test 4.3a: Zero Roll", quat1_x == 16'd0);
            check_test("Test 4.3b: Zero Pitch", quat1_y == 16'd0);
            check_test("Test 4.3c: Zero Yaw", quat1_z == 16'd0);
            check_test("Test 4.3d: Zero Gyro X", gyro1_x == 16'd0);
        end
        
        $display("");
        
        // ========================================
        // TEST 5: Multiple Packets
        // ========================================
        $display("=== Test 5: Multiple Packets ===");
        
        // Test 5.1: Send multiple packets and verify data updates
        begin
            // First packet: Roll = 100
            send_packet_inline(
                8'hAA,  // Header
                8'h00, 8'h64,  // Roll = 100
                8'h00, 8'h00,  // Pitch = 0
                8'h00, 8'h00,  // Yaw = 0
                8'h00, 8'h00,  // Gyro X = 0
                8'h00, 8'h00,  // Gyro Y = 0
                8'h00, 8'h00,  // Gyro Z = 0
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            check_test("Test 5.1a: First packet Roll = 100", quat1_x == 16'd100);
            
            // Second packet: Roll = 500
            send_packet_inline(
                8'hAA,  // Header
                8'h01, 8'hF4,  // Roll = 500
                8'h00, 8'h00,  // Pitch = 0
                8'h00, 8'h00,  // Yaw = 0
                8'h00, 8'h00,  // Gyro X = 0
                8'h00, 8'h00,  // Gyro Y = 0
                8'h00, 8'h00,  // Gyro Z = 0
                8'h03,  // Flags: both valid
                8'h00, 8'h00   // Reserved
            );
            wait_cdc();
            check_test("Test 5.1b: Second packet Roll = 500", quat1_x == 16'd500);
        end
        
        $display("");
        
        // ========================================
        // Summary
        // ========================================
        $display("========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Total Tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        $display("========================================");
        
        if (fail_count == 0) begin
            $display("ALL TESTS PASSED");
        end else begin
            $display("SOME TESTS FAILED");
        end
        $display("========================================");
        
        #(10 * CLK_PERIOD);
        $finish;
    end
    
endmodule

