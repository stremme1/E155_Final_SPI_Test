`timescale 1ns / 1ps

// Unit Tests for SPI Slave MCU - Debug packet assembly issues
// Tests individual components to isolate the problem

module tb_spi_slave_unit_tests;

    // Clock
    logic clk;
    logic cs_n;
    logic sck;
    
    // Sensor data inputs
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    // Outputs
    logic sdo;
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // MCU SPI clock - NOT continuous! Only toggles during spiSendReceive calls
    // In real MCU, SCK only toggles when spiSendReceive() is called
    // For this test, we'll manually control SCK to simulate MCU behavior
    parameter SCK_PERIOD = 1000;  // 1MHz (1000ns period)
    // SCK starts idle low (SPI Mode 0: CPOL=0)
    initial begin
        sck = 0;
    end
    // No continuous clock - SCK is controlled manually in the test
    
    // DUT
    spi_slave_mcu dut (
        .clk(clk),
        .cs_n(cs_n),
        .sck(sck),
        .sdi(1'b0),
        .sdo(sdo),
        .initialized(1'b1),
        .error(1'b0),
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
    
    // Access internal signals
    wire [7:0] packet_buffer_0 = dut.packet_buffer[0];
    wire [7:0] packet_buffer_1 = dut.packet_buffer[1];
    wire [7:0] packet_buffer_2 = dut.packet_buffer[2];
    wire signed [15:0] quat1_w_snap = dut.quat1_w_snap;
    wire signed [15:0] quat1_x_snap = dut.quat1_x_snap;
    wire [127:0] tx_packet = dut.tx_packet;
    wire [7:0] shift_out = dut.shift_out;
    wire [2:0] bit_count = dut.bit_count;
    
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    logic [7:0] received_byte;
    integer j;
    
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
    
    initial begin
        $dumpfile("tb_spi_slave_unit_tests.vcd");
        $dumpvars(0, tb_spi_slave_unit_tests);
        
        $display("========================================");
        $display("SPI Slave Unit Tests - Debug Packet Assembly");
        $display("========================================");
        $display("");
        
        // Initialize
        cs_n = 1'b1;
        quat1_valid = 1'b1;
        gyro1_valid = 1'b1;
        quat1_w = 16'h1234;
        quat1_x = 16'h5678;
        quat1_y = 16'h9ABC;
        quat1_z = 16'hDEF0;
        gyro1_x = 16'h1111;
        gyro1_y = 16'h2222;
        gyro1_z = 16'h3333;
        
        #(10 * CLK_PERIOD);
        
        // ========================================
        // UNIT TEST 1: Check snapshot before CS goes low
        // ========================================
        $display("=== Unit Test 1: Snapshot Before CS Low ===");
        $display("[DEBUG] quat1_w_snap = %d (0x%04X)", quat1_w_snap, quat1_w_snap);
        $display("[DEBUG] quat1_x_snap = %d (0x%04X)", quat1_x_snap, quat1_x_snap);
        $display("[DEBUG] packet_buffer[0] = 0x%02X", packet_buffer_0);
        $display("[DEBUG] packet_buffer[1] = 0x%02X", packet_buffer_1);
        $display("[DEBUG] packet_buffer[2] = 0x%02X", packet_buffer_2);
        check_test("UT1.1: Header byte before CS low", packet_buffer_0 == 8'hAA);
        $display("");
        
        // ========================================
        // UNIT TEST 2: Check snapshot immediately after CS goes low
        // ========================================
        $display("=== Unit Test 2: Snapshot After CS Low ===");
        cs_n = 1'b0;
        #(1);  // Minimal delay to allow snapshot capture
        $display("[DEBUG] After CS low - quat1_w_snap = %d (0x%04X)", quat1_w_snap, quat1_w_snap);
        $display("[DEBUG] After CS low - quat1_x_snap = %d (0x%04X)", quat1_x_snap, quat1_x_snap);
        $display("[DEBUG] After CS low - packet_buffer[0] = 0x%02X", packet_buffer_0);
        $display("[DEBUG] After CS low - packet_buffer[1] = 0x%02X", packet_buffer_1);
        $display("[DEBUG] After CS low - packet_buffer[2] = 0x%02X", packet_buffer_2);
        $display("[DEBUG] After CS low - shift_out = 0x%02X", shift_out);
        check_test("UT2.1: Header byte after CS low", packet_buffer_0 == 8'hAA);
        check_test("UT2.2: Snapshot captured quat1_w", quat1_w_snap == 16'h1234);
        check_test("UT2.3: Packet buffer[1] = quat1_w MSB", packet_buffer_1 == 8'h12);
        check_test("UT2.4: Packet buffer[2] = quat1_w LSB", packet_buffer_2 == 8'h34);
        check_test("UT2.5: Shift_out = header", shift_out == 8'hAA);
        $display("");
        
        // ========================================
        // UNIT TEST 3: Check after clock cycles
        // ========================================
        $display("=== Unit Test 3: After Clock Cycles ===");
        #(10 * CLK_PERIOD);
        $display("[DEBUG] After 10 clk cycles - quat1_w_snap = %d (0x%04X)", quat1_w_snap, quat1_w_snap);
        $display("[DEBUG] After 10 clk cycles - packet_buffer[1] = 0x%02X", packet_buffer_1);
        $display("[DEBUG] After 10 clk cycles - packet_buffer[2] = 0x%02X", packet_buffer_2);
        check_test("UT3.1: Packet buffer stable", packet_buffer_1 == 8'h12);
        $display("");
        
        // ========================================
        // UNIT TEST 4: Check tx_packet assembly
        // ========================================
        $display("=== Unit Test 4: TX Packet Assembly ===");
        $display("[DEBUG] tx_packet[127:120] = 0x%02X (should be 0xAA)", tx_packet[127:120]);
        $display("[DEBUG] tx_packet[119:112] = 0x%02X (should be 0x12)", tx_packet[119:112]);
        $display("[DEBUG] tx_packet[111:104] = 0x%02X (should be 0x34)", tx_packet[111:104]);
        check_test("UT4.1: tx_packet[127:120] = 0xAA", tx_packet[127:120] == 8'hAA);
        check_test("UT4.2: tx_packet[119:112] = 0x12", tx_packet[119:112] == 8'h12);
        check_test("UT4.3: tx_packet[111:104] = 0x34", tx_packet[111:104] == 8'h34);
        $display("");
        
        // ========================================
        // UNIT TEST 5: Read first byte via SPI (simulating MCU spiSendReceive)
        // ========================================
        $display("=== Unit Test 5: Read First Byte via SPI ===");
        received_byte = 8'h00;
        
        // Wait for setup - ensure CS is stable
        #(5 * CLK_PERIOD);
        
        // Simulate MCU spiSendReceive(0x00) - generates 8 SCK cycles
        // SPI Mode 0: CPOL=0 (idle low), CPHA=0 (sample on rising edge)
        // First bit is ready when CS goes low, MCU samples on first rising edge
        // SCK is already low (idle), so first edge is rising
        for (j = 7; j >= 0; j = j - 1) begin
            // Rising edge - MCU samples this bit
            sck = 1'b1;
            #(10);  // Small delay for sampling
            received_byte[j] = sdo;
            #(SCK_PERIOD/2 - 10);
            $display("[DEBUG] Bit %0d: sdo=%b, received_byte=0x%02X, shift_out=0x%02X, bit_count=%0d", 
                     j, sdo, received_byte, shift_out, bit_count);
            
            // Falling edge - FPGA shifts to prepare next bit (except after last bit)
            if (j > 0) begin
                sck = 1'b0;
                #(SCK_PERIOD/2);
            end
        end
        
        // SCK returns to idle low after byte complete
        sck = 1'b0;
        #(SCK_PERIOD/2);
        
        $display("[DEBUG] Received byte = 0x%02X", received_byte);
        check_test("UT5.1: Received first byte = 0xAA", received_byte == 8'hAA);
        $display("");
        
        cs_n = 1'b1;
        #(10 * CLK_PERIOD);
        
        // ========================================
        // Summary
        // ========================================
        $display("========================================");
        $display("Unit Test Summary");
        $display("========================================");
        $display("Total Tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        $display("========================================");
        
        #(100 * CLK_PERIOD);
        $finish;
    end
    
endmodule

