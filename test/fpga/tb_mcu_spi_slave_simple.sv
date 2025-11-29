`timescale 1ns / 1ps

// Simplified MCU SPI Slave Testbench
// Tests packet buffer assembly and DONE/LOAD handshaking without SPI transmission
// Uses direct access to internal signals via hierarchical reference

module tb_mcu_spi_slave_simple;

    // Clock
    logic clk;
    
    // SPI interface (not used for transmission, but needed for module instantiation)
    logic sck = 0;      // SPI clock (not used in this test)
    logic sdi = 0;      // SPI data in (not used)
    logic sdo;          // SPI data out (not checked in this test)
    logic load;         // Load signal from MCU
    logic done;         // Done signal to MCU
    
    // Sensor data inputs (single sensor only)
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // DUT (single sensor only)
    spi_slave_mcu dut (
        .clk(clk),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .load(load),
        .done(done),
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
    
    // Access packet_buffer via hierarchical reference (16-byte packet)
    wire [7:0] packet_buffer [0:15];
    assign packet_buffer[0] = dut.packet_buffer[0];
    assign packet_buffer[1] = dut.packet_buffer[1];
    assign packet_buffer[2] = dut.packet_buffer[2];
    assign packet_buffer[3] = dut.packet_buffer[3];
    assign packet_buffer[4] = dut.packet_buffer[4];
    assign packet_buffer[5] = dut.packet_buffer[5];
    assign packet_buffer[6] = dut.packet_buffer[6];
    assign packet_buffer[7] = dut.packet_buffer[7];
    assign packet_buffer[8] = dut.packet_buffer[8];
    assign packet_buffer[9] = dut.packet_buffer[9];
    assign packet_buffer[10] = dut.packet_buffer[10];
    assign packet_buffer[11] = dut.packet_buffer[11];
    assign packet_buffer[12] = dut.packet_buffer[12];
    assign packet_buffer[13] = dut.packet_buffer[13];
    assign packet_buffer[14] = dut.packet_buffer[14];
    assign packet_buffer[15] = dut.packet_buffer[15];
    
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
    
    // Helper task to wait for combinational logic
    task wait_comb;
        #(CLK_PERIOD * 2);  // Wait 2 clock cycles for combinational logic
    endtask
    
    // Main test sequence
    initial begin
        $dumpfile("tb_mcu_spi_slave_simple.vcd");
        $dumpvars(0, tb_mcu_spi_slave_simple);
        
        $display("========================================");
        $display("MCU SPI Slave Simplified Testbench");
        $display("========================================");
        $display("");
        
        // Initialize
        load = 0;
        quat1_valid = 0;
        gyro1_valid = 0;
        quat1_w = 0; quat1_x = 0; quat1_y = 0; quat1_z = 0;
        gyro1_x = 0; gyro1_y = 0; gyro1_z = 0;
        
        #(10 * CLK_PERIOD);
        
        // ========================================
        // TEST 1: Packet Buffer Assembly
        // ========================================
        $display("=== Test 1: Packet Buffer Assembly ===");
        
        // Test 1.1: Header byte
        quat1_w = 16'h1234;
        wait_comb();
        check_test("Test 1.1: Header byte = 0xAA", packet_buffer[0] == 8'hAA);
        
        // Test 1.2: Sensor 1 quaternion MSB/LSB order
        quat1_w = 16'h1234;
        quat1_x = 16'h5678;
        quat1_y = 16'h9ABC;
        quat1_z = 16'hDEF0;
        wait_comb();
        check_test("Test 1.2a: Sensor 1 quat W MSB/LSB", 
                   packet_buffer[1] == 8'h12 && packet_buffer[2] == 8'h34);
        check_test("Test 1.2b: Sensor 1 quat X MSB/LSB", 
                   packet_buffer[3] == 8'h56 && packet_buffer[4] == 8'h78);
        check_test("Test 1.2c: Sensor 1 quat Y MSB/LSB", 
                   packet_buffer[5] == 8'h9A && packet_buffer[6] == 8'hBC);
        check_test("Test 1.2d: Sensor 1 quat Z MSB/LSB", 
                   packet_buffer[7] == 8'hDE && packet_buffer[8] == 8'hF0);
        
        // Test 1.3: Sensor 1 gyroscope MSB/LSB order
        gyro1_x = 16'h1111;
        gyro1_y = 16'h2222;
        gyro1_z = 16'h3333;
        wait_comb();
        check_test("Test 1.3a: Sensor 1 gyro X MSB/LSB", 
                   packet_buffer[9] == 8'h11 && packet_buffer[10] == 8'h11);
        check_test("Test 1.3b: Sensor 1 gyro Y MSB/LSB", 
                   packet_buffer[11] == 8'h22 && packet_buffer[12] == 8'h22);
        check_test("Test 1.3c: Sensor 1 gyro Z MSB/LSB", 
                   packet_buffer[13] == 8'h33 && packet_buffer[14] == 8'h33);
        
        // Test 1.4: Sensor 1 flags
        quat1_valid = 1;
        gyro1_valid = 0;
        wait_comb();
        check_test("Test 1.4a: Sensor 1 flags - quat valid only", 
                   packet_buffer[15] == 8'h01);
        quat1_valid = 0;
        gyro1_valid = 1;
        wait_comb();
        check_test("Test 1.4b: Sensor 1 flags - gyro valid only", 
                   packet_buffer[15] == 8'h02);
        quat1_valid = 1;
        gyro1_valid = 1;
        wait_comb();
        check_test("Test 1.4c: Sensor 1 flags - both valid", 
                   packet_buffer[15] == 8'h03);
        
        $display("");
        
        // ========================================
        // TEST 2: DONE Signal Assertion
        // ========================================
        $display("=== Test 2: DONE Signal Assertion ===");
        
        // Reset valid signals
        quat1_valid = 0;
        gyro1_valid = 0;
        load = 0;
        #(5 * CLK_PERIOD);
        
        // Test 2.1: DONE = 0 when no valid data
        check_test("Test 2.1: DONE = 0 when no valid data", done == 1'b0);
        
        // Test 2.2: DONE = 1 when quat1_valid asserted
        quat1_valid = 1;
        #(5 * CLK_PERIOD);
        check_test("Test 2.2: DONE = 1 when quat1_valid", done == 1'b1);
        
        // Test 2.3: DONE = 1 when gyro1_valid asserted
        quat1_valid = 0;
        gyro1_valid = 1;
        #(5 * CLK_PERIOD);
        check_test("Test 2.3: DONE = 1 when gyro1_valid", done == 1'b1);
        
        // Test 2.4: DONE = 1 when both valid
        quat1_valid = 1;
        gyro1_valid = 1;
        #(5 * CLK_PERIOD);
        check_test("Test 2.4: DONE = 1 when both valid", done == 1'b1);
        
        $display("");
        
        // ========================================
        // TEST 3: LOAD Acknowledgment
        // ========================================
        $display("=== Test 3: LOAD Acknowledgment ===");
        
        // Test 3.1: DONE stays high after data becomes valid
        quat1_valid = 1;
        #(5 * CLK_PERIOD);
        check_test("Test 3.1: DONE stays high after data valid", done == 1'b1);
        
        // Test 3.2: DONE clears when LOAD goes high (edge detection)
        load = 1;
        #(5 * CLK_PERIOD);
        check_test("Test 3.2: DONE clears when LOAD goes high", done == 1'b0);
        
        // Test 3.3: DONE stays low after acknowledgment until new data
        load = 0;
        #(5 * CLK_PERIOD);
        check_test("Test 3.3: DONE stays low after LOAD cleared", done == 1'b0);
        
        // Test 3.4: New data sets DONE high again
        // First, ensure valid goes low and stays low long enough for state to update
        // After test 3.3, quat1_valid is still 1, so we need to clear it first
        quat1_valid = 0;
        // Wait for multiple clock cycles to ensure has_valid_prev updates to 0
        // Need to wait at least 2 cycles: one for has_valid to become 0, one for has_valid_prev to update
        repeat(10) @(posedge clk);
        // Verify that done is low (no valid data) and has_valid_prev has updated
        if (done != 1'b0) begin
            $display("  [WARN] DONE should be 0 after quat1_valid goes low, but it's %b", done);
        end
        // Now set valid high - this should trigger new data detection
        // The transition from 0 to 1 should be detected on the next clock edge
        quat1_valid = 1;
        // Wait for multiple clock cycles to ensure data_ready_reg updates to 1
        // Need to wait at least 2 cycles: one for has_valid to become 1, one for data_ready_reg to update
        repeat(10) @(posedge clk);
        check_test("Test 3.4: New data sets DONE high again", done == 1'b1);
        
        // Test 3.5: Multiple LOAD pulses don't cause issues
        load = 1;
        #(2 * CLK_PERIOD);
        load = 0;
        #(2 * CLK_PERIOD);
        load = 1;
        #(2 * CLK_PERIOD);
        load = 0;
        #(5 * CLK_PERIOD);
        check_test("Test 3.5: Multiple LOAD pulses handled correctly", done == 1'b0);
        
        $display("");
        
        // ========================================
        // TEST 4: Edge Cases
        // ========================================
        $display("=== Test 4: Edge Cases ===");
        
        // Test 4.1: All zeros
        quat1_w = 16'h0000;
        quat1_x = 16'h0000;
        quat1_y = 16'h0000;
        quat1_z = 16'h0000;
        gyro1_x = 16'h0000;
        gyro1_y = 16'h0000;
        gyro1_z = 16'h0000;
        wait_comb();
        check_test("Test 4.1: All zeros handled correctly", 
                   packet_buffer[1] == 8'h00 && packet_buffer[2] == 8'h00 &&
                   packet_buffer[9] == 8'h00 && packet_buffer[10] == 8'h00);
        
        // Test 4.2: Maximum positive value (0x7FFF)
        quat1_w = 16'h7FFF;
        wait_comb();
        check_test("Test 4.2: Max positive value (0x7FFF)", 
                   packet_buffer[1] == 8'h7F && packet_buffer[2] == 8'hFF);
        
        // Test 4.3: Maximum negative value (0x8000)
        quat1_w = 16'h8000;
        wait_comb();
        check_test("Test 4.3: Max negative value (0x8000)", 
                   packet_buffer[1] == 8'h80 && packet_buffer[2] == 8'h00);
        
        // Test 4.4: Negative value (-1 = 0xFFFF)
        quat1_w = 16'hFFFF;
        wait_comb();
        check_test("Test 4.4: Negative value (-1 = 0xFFFF)", 
                   packet_buffer[1] == 8'hFF && packet_buffer[2] == 8'hFF);
        
        // Test 4.5: Only quat valid
        quat1_valid = 1;
        gyro1_valid = 0;
        wait_comb();
        check_test("Test 4.5: Only quat valid flag", packet_buffer[15] == 8'h01);
        
        // Test 4.6: Only gyro valid
        quat1_valid = 0;
        gyro1_valid = 1;
        wait_comb();
        check_test("Test 4.6: Only gyro valid flag", packet_buffer[15] == 8'h02);
        
        // Test 4.7: Both quat and gyro valid
        quat1_valid = 1;
        gyro1_valid = 1;
        wait_comb();
        check_test("Test 4.7: Both quat and gyro valid flags", 
                   packet_buffer[15] == 8'h03);
        
        // Test 4.8: Rapid data updates
        quat1_w = 16'h1111;
        wait_comb();
        quat1_w = 16'h2222;
        wait_comb();
        quat1_w = 16'h3333;
        wait_comb();
        check_test("Test 4.8: Rapid data updates handled", 
                   packet_buffer[1] == 8'h33 && packet_buffer[2] == 8'h33);
        
        $display("");
        
        // ========================================
        // TEST 5: Byte Order Verification
        // ========================================
        $display("=== Test 5: Byte Order Verification ===");
        
        // Test 5.1: Known value 0x1234
        quat1_w = 16'h1234;
        wait_comb();
        check_test("Test 5.1: 0x1234 → MSB=0x12, LSB=0x34", 
                   packet_buffer[1] == 8'h12 && packet_buffer[2] == 8'h34);
        
        // Test 5.2: Known value 0xABCD
        quat1_w = 16'hABCD;
        wait_comb();
        check_test("Test 5.2: 0xABCD → MSB=0xAB, LSB=0xCD", 
                   packet_buffer[1] == 8'hAB && packet_buffer[2] == 8'hCD);
        
        // Test 5.3: Negative value -1 (0xFFFF)
        quat1_w = 16'hFFFF;
        wait_comb();
        check_test("Test 5.3: -1 (0xFFFF) → MSB=0xFF, LSB=0xFF", 
                   packet_buffer[1] == 8'hFF && packet_buffer[2] == 8'hFF);
        
        // Test 5.4: Zero value (0x0000)
        quat1_w = 16'h0000;
        wait_comb();
        check_test("Test 5.4: 0x0000 → MSB=0x00, LSB=0x00", 
                   packet_buffer[1] == 8'h00 && packet_buffer[2] == 8'h00);
        
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

