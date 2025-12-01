`timescale 1ns / 1ps

// SHTP to SPI Integration Testbench
// Tests that SPI slave correctly captures and outputs SHTP sensor data
// Verifies pulse valid signals are not lost and data is correctly formatted
// Per BNO08X datasheet Section 1.3.5.2 for sensor report formats

module tb_shtp_to_spi_integration;

    // Clock
    logic clk;
    
    // SPI interface (MCU is master, CS-based protocol)
    logic cs_n;      // Chip select from MCU (active low)
    logic sck;       // SPI clock from MCU
    logic sdi;       // SPI data in (MOSI - not used in read-only mode)
    logic sdo;       // SPI data out (MISO to MCU)
    
    // Sensor data inputs (simulating BNO085 controller outputs)
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    logic initialized, error;
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // MCU SPI clock - NOT continuous! Only toggles during SPI transactions
    // SCK starts idle low (SPI Mode 0: CPOL=0)
    parameter SCK_PERIOD = 1000;  // 1MHz (1000ns period)
    initial begin
        sck = 0;  // Idle low for SPI Mode 0
    end
    
    // DUT
    spi_slave_mcu dut (
        .clk(clk),
        .cs_n(cs_n),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
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
    wire quat1_valid_snap = dut.quat1_valid_snap;
    wire gyro1_valid_snap = dut.gyro1_valid_snap;
    wire signed [15:0] quat1_w_snap = dut.quat1_w_snap;
    wire signed [15:0] quat1_x_snap = dut.quat1_x_snap;
    wire signed [15:0] quat1_y_snap = dut.quat1_y_snap;
    wire signed [15:0] quat1_z_snap = dut.quat1_z_snap;
    wire signed [15:0] gyro1_x_snap = dut.gyro1_x_snap;
    wire signed [15:0] gyro1_y_snap = dut.gyro1_y_snap;
    wire signed [15:0] gyro1_z_snap = dut.gyro1_z_snap;
    wire initialized_snap = dut.initialized_snap;
    wire error_snap = dut.error_snap;
    wire [7:0] packet_buffer_15 = dut.packet_buffer[15];
    
    // Test variables
    logic [7:0] received_packet [0:15];
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
    
    // ========================================================================
    // BNO085 Controller Simulator Tasks
    // ========================================================================
    // These tasks simulate the BNO085 controller outputting sensor data
    // with pulse valid signals (high for ONE clock cycle only)
    
    // Task: Output quaternion data with valid pulse
    // Per datasheet: Rotation Vector report (Report ID=0x05)
    // Data format: X, Y, Z, W as 16-bit signed values (little-endian in SHTP)
    task output_quaternion(
        input signed [15:0] w, x, y, z
    );
        begin
            // Set data values
            quat1_w <= w;
            quat1_x <= x;
            quat1_y <= y;
            quat1_z <= z;
            
            // Pulse valid signal (high for ONE clock cycle)
            quat1_valid <= 1'b1;
            @(posedge clk);
            quat1_valid <= 1'b0;
            
            $display("[%0t] BNO085: Quaternion output - w=%d x=%d y=%d z=%d (valid pulse)", 
                     $time, w, x, y, z);
        end
    endtask
    
    // Task: Output gyroscope data with valid pulse
    // Per datasheet Figure 1-34: Calibrated gyroscope report (Report ID=0x02)
    // Data format: X, Y, Z as 16-bit signed values (little-endian in SHTP)
    task output_gyroscope(
        input signed [15:0] x, y, z
    );
        begin
            // Set data values
            gyro1_x <= x;
            gyro1_y <= y;
            gyro1_z <= z;
            
            // Pulse valid signal (high for ONE clock cycle)
            gyro1_valid <= 1'b1;
            @(posedge clk);
            gyro1_valid <= 1'b0;
            
            $display("[%0t] BNO085: Gyroscope output - x=%d y=%d z=%d (valid pulse)", 
                     $time, x, y, z);
        end
    endtask
    
    // ========================================================================
    // MCU SPI Master Task
    // ========================================================================
    // Task: MCU reads 16-byte packet via SPI Mode 0
    // SPI Mode 0: CPOL=0 (idle low), CPHA=0 (sample on rising edge)
    task mcu_read_packet;
        integer i, j;
        begin
            $display("[%0t] MCU: Starting SPI transaction - CS goes low", $time);
            
            // CS goes low to start transaction
            cs_n = 1'b0;
            #(5 * CLK_PERIOD);  // Setup time for first bit
            
            // Read 16 bytes (128 bits total)
            // SPI Mode 0: CPOL=0 (idle low), CPHA=0 (sample on rising, change on falling)
            for (i = 0; i < 16; i = i + 1) begin
                received_packet[i] = 8'h00;
                
                // Read 8 bits MSB first (SPI Mode 0: SCK starts low, first edge is rising)
                for (j = 7; j >= 0; j = j - 1) begin
                    // Rising edge - MCU samples
                    sck = 1'b1;
                    #(10);  // Small delay for sampling
                    received_packet[i][j] = sdo;
                    #(SCK_PERIOD/2 - 10);
                    
                    // Falling edge - FPGA shifts to prepare next bit (except after last bit)
                    if (j > 0 || i < 15) begin
                        sck = 1'b0;
                        #(SCK_PERIOD/2);
                    end
                end
                
                $display("[%0t] MCU: Byte[%0d] = 0x%02X", $time, i, received_packet[i]);
            end
            
            // CS goes high to end transaction
            sck = 1'b0;  // Ensure SCK is low before CS goes high
            #(SCK_PERIOD);
            cs_n = 1'b1;
            #(5 * CLK_PERIOD);
            
            $display("[%0t] MCU: Packet complete - CS high", $time);
        end
    endtask
    
    // Task: Verify received packet matches expected values
    task verify_packet(
        input signed [15:0] exp_quat_w, exp_quat_x, exp_quat_y, exp_quat_z,
        input signed [15:0] exp_gyro_x, exp_gyro_y, exp_gyro_z,
        input exp_quat_valid, exp_gyro_valid, exp_init, exp_error
    );
        integer i;
        logic [7:0] exp_flags;
        logic signed [15:0] recv_quat_w, recv_quat_x, recv_quat_y, recv_quat_z;
        logic signed [15:0] recv_gyro_x, recv_gyro_y, recv_gyro_z;
        logic recv_quat_valid, recv_gyro_valid, recv_init, recv_error;
        begin
            // Verify header
            check_test("Packet header = 0xAA", received_packet[0] == 8'hAA);
            
            // Extract quaternion (MSB,LSB format)
            recv_quat_w = {received_packet[1], received_packet[2]};
            recv_quat_x = {received_packet[3], received_packet[4]};
            recv_quat_y = {received_packet[5], received_packet[6]};
            recv_quat_z = {received_packet[7], received_packet[8]};
            
            // Extract gyroscope (MSB,LSB format)
            recv_gyro_x = {received_packet[9], received_packet[10]};
            recv_gyro_y = {received_packet[11], received_packet[12]};
            recv_gyro_z = {received_packet[13], received_packet[14]};
            
            // Extract flags
            recv_quat_valid = received_packet[15][0];
            recv_gyro_valid = received_packet[15][1];
            recv_init = received_packet[15][2];
            recv_error = received_packet[15][3];
            exp_flags = {4'h0, exp_error, exp_init, exp_gyro_valid, exp_quat_valid};
            
            // Verify quaternion
            check_test("Quaternion W", recv_quat_w == exp_quat_w);
            check_test("Quaternion X", recv_quat_x == exp_quat_x);
            check_test("Quaternion Y", recv_quat_y == exp_quat_y);
            check_test("Quaternion Z", recv_quat_z == exp_quat_z);
            
            // Verify gyroscope
            check_test("Gyroscope X", recv_gyro_x == exp_gyro_x);
            check_test("Gyroscope Y", recv_gyro_y == exp_gyro_y);
            check_test("Gyroscope Z", recv_gyro_z == exp_gyro_z);
            
            // Verify flags
            check_test("Flags byte", received_packet[15] == exp_flags);
            check_test("Quat valid flag", recv_quat_valid == exp_quat_valid);
            check_test("Gyro valid flag", recv_gyro_valid == exp_gyro_valid);
            check_test("Init flag", recv_init == exp_init);
            check_test("Error flag", recv_error == exp_error);
        end
    endtask
    
    // ========================================================================
    // Test Cases
    // ========================================================================
    
    initial begin
        // Initialize signals
        cs_n = 1'b1;
        sck = 1'b0;
        sdi = 1'b0;
        quat1_valid = 1'b0;
        gyro1_valid = 1'b0;
        quat1_w = 16'h0000;
        quat1_x = 16'h0000;
        quat1_y = 16'h0000;
        quat1_z = 16'h0000;
        gyro1_x = 16'h0000;
        gyro1_y = 16'h0000;
        gyro1_z = 16'h0000;
        initialized = 1'b1;
        error = 1'b0;
        
        // Wait for initial reset
        #(10 * CLK_PERIOD);
        
        $display("\n========================================");
        $display("SHTP to SPI Integration Testbench");
        $display("========================================\n");
        
        // ====================================================================
        // TC1: Valid quaternion pulse when CS is high
        // ====================================================================
        $display("[TC1] Valid quaternion pulse when CS is high");
        $display("----------------------------------------");
        
        // Output quaternion data (valid pulse)
        output_quaternion(16'd1000, 16'd2000, 16'd3000, 16'd4000);
        
        // Wait a few clocks to ensure snapshot is captured
        #(10 * CLK_PERIOD);
        
        // Verify snapshot captured the data
        $display("[%0t] Snapshot check: quat1_w_snap=%d (expected 1000)", $time, quat1_w_snap);
        check_test("TC1: Snapshot captured quat W", quat1_w_snap == 16'd1000);
        check_test("TC1: Snapshot captured quat X", quat1_x_snap == 16'd2000);
        check_test("TC1: Snapshot captured quat Y", quat1_y_snap == 16'd3000);
        check_test("TC1: Snapshot captured quat Z", quat1_z_snap == 16'd4000);
        check_test("TC1: Snapshot captured quat_valid", quat1_valid_snap == 1'b1);
        
        // Read packet and verify
        mcu_read_packet();
        verify_packet(16'd1000, 16'd2000, 16'd3000, 16'd4000,
                      16'h0000, 16'h0000, 16'h0000,
                      1'b1, 1'b0, 1'b1, 1'b0);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC2: Valid gyroscope pulse when CS is high
        // ====================================================================
        $display("[TC2] Valid gyroscope pulse when CS is high");
        $display("----------------------------------------");
        
        // Output gyroscope data (valid pulse)
        output_gyroscope(16'd500, 16'd600, 16'd700);
        
        // Wait a few clocks to ensure snapshot is captured
        #(10 * CLK_PERIOD);
        
        // Verify snapshot captured the data
        $display("[%0t] Snapshot check: gyro1_x_snap=%d (expected 500)", $time, gyro1_x_snap);
        check_test("TC2: Snapshot captured gyro X", gyro1_x_snap == 16'd500);
        check_test("TC2: Snapshot captured gyro Y", gyro1_y_snap == 16'd600);
        check_test("TC2: Snapshot captured gyro Z", gyro1_z_snap == 16'd700);
        check_test("TC2: Snapshot captured gyro_valid", gyro1_valid_snap == 1'b1);
        
        // Read packet and verify
        mcu_read_packet();
        verify_packet(16'd1000, 16'd2000, 16'd3000, 16'd4000,  // Previous quat still there
                      16'd500, 16'd600, 16'd700,
                      1'b1, 1'b1, 1'b1, 1'b0);  // Both valid now
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC3: Valid pulse during SPI transaction (should be captured on next CS high)
        // ====================================================================
        $display("[TC3] Valid pulse during SPI transaction");
        $display("----------------------------------------");
        
        // Start SPI transaction
        cs_n = 1'b0;
        #(5 * CLK_PERIOD);
        
        // Output new quaternion data DURING transaction (should not affect current transaction)
        output_quaternion(16'd1111, 16'd2222, 16'd3333, 16'd4444);
        
        // Complete current transaction (should still have old data)
        cs_n = 1'b1;
        #(5 * CLK_PERIOD);
        
        // Read packet - should have OLD data (snapshot frozen during transaction)
        mcu_read_packet();
        verify_packet(16'd1000, 16'd2000, 16'd3000, 16'd4000,  // Old quat
                      16'd500, 16'd600, 16'd700,  // Old gyro
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        // Wait for snapshot to update (CS is high now)
        #(10 * CLK_PERIOD);
        
        // Verify snapshot now has new data
        check_test("TC3: Snapshot updated after CS high", quat1_w_snap == 16'd1111);
        
        // Read packet again - should have NEW data
        mcu_read_packet();
        verify_packet(16'd1111, 16'd2222, 16'd3333, 16'd4444,  // New quat
                      16'd500, 16'd600, 16'd700,  // Old gyro still
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC4: Multiple valid pulses (latest data should be captured)
        // ====================================================================
        $display("[TC4] Multiple valid pulses - latest data captured");
        $display("----------------------------------------");
        
        // Output multiple quaternion updates
        output_quaternion(16'd100, 16'd200, 16'd300, 16'd400);
        #(5 * CLK_PERIOD);
        output_quaternion(16'd200, 16'd300, 16'd400, 16'd500);
        #(5 * CLK_PERIOD);
        output_quaternion(16'd300, 16'd400, 16'd500, 16'd600);
        
        // Wait for snapshot to capture latest
        #(10 * CLK_PERIOD);
        
        // Verify snapshot has LATEST data
        check_test("TC4: Snapshot has latest quat W", quat1_w_snap == 16'd300);
        check_test("TC4: Snapshot has latest quat X", quat1_x_snap == 16'd400);
        
        // Read packet and verify latest data
        mcu_read_packet();
        verify_packet(16'd300, 16'd400, 16'd500, 16'd600,  // Latest quat
                      16'd500, 16'd600, 16'd700,  // Previous gyro
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC5: Valid pulse immediately before CS goes low
        // ====================================================================
        $display("[TC5] Valid pulse immediately before CS goes low");
        $display("----------------------------------------");
        
        // Output new quaternion
        output_quaternion(16'd9999, 16'd8888, 16'd7777, 16'd6666);
        
        // Immediately start SPI transaction (within same clock cycle or next)
        #(1 * CLK_PERIOD);
        cs_n = 1'b0;
        #(5 * CLK_PERIOD);
        
        // Verify snapshot was captured (should have new data or old, depending on timing)
        $display("[%0t] Snapshot check: quat1_w_snap=%d", $time, quat1_w_snap);
        
        // Complete transaction
        cs_n = 1'b1;
        #(10 * CLK_PERIOD);
        
        // Verify snapshot now has new data
        check_test("TC5: Snapshot updated after CS high", quat1_w_snap == 16'd9999);
        
        // Read packet and verify
        mcu_read_packet();
        verify_packet(16'd9999, 16'd8888, 16'd7777, 16'd6666,  // New quat
                      16'd500, 16'd600, 16'd700,  // Previous gyro
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC6: Continuous data updates (100Hz simulation)
        // ====================================================================
        $display("[TC6] Continuous data updates (100Hz simulation)");
        $display("----------------------------------------");
        
        // Simulate 100Hz updates (10ms = 10,000,000ns at 3MHz = 30,000 clocks)
        // But for simulation speed, use shorter intervals
        begin
            integer update_count;
            for (update_count = 0; update_count < 5; update_count = update_count + 1) begin
            output_quaternion(16'd1000 + update_count, 
                            16'd2000 + update_count, 
                            16'd3000 + update_count, 
                            16'd4000 + update_count);
                output_gyroscope(16'd100 + update_count,
                                16'd200 + update_count,
                                16'd300 + update_count);
                #(100 * CLK_PERIOD);  // Simulate ~33us between updates (much faster than real 10ms)
            end
        end
        
        // Wait for snapshot to capture latest
        #(10 * CLK_PERIOD);
        
        // Verify snapshot has latest data
        check_test("TC6: Snapshot has latest quat W", quat1_w_snap == 16'd1004);
        check_test("TC6: Snapshot has latest gyro X", gyro1_x_snap == 16'd104);
        
        // Read packet and verify latest data
        mcu_read_packet();
        verify_packet(16'd1004, 16'd2004, 16'd3004, 16'd4004,  // Latest quat
                      16'd104, 16'd204, 16'd304,  // Latest gyro
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // TC7: Verify flags byte includes all bits correctly
        // ====================================================================
        $display("[TC7] Verify flags byte includes all bits correctly");
        $display("----------------------------------------");
        
        // Test with error flag set
        error = 1'b1;
        #(10 * CLK_PERIOD);
        
        // Read packet and verify flags
        mcu_read_packet();
        check_test("TC7: Flags byte bit 3 (error) = 1", received_packet[15][3] == 1'b1);
        check_test("TC7: Flags byte bit 2 (init) = 1", received_packet[15][2] == 1'b1);
        check_test("TC7: Flags byte bit 1 (gyro_valid) = 1", received_packet[15][1] == 1'b1);
        check_test("TC7: Flags byte bit 0 (quat_valid) = 1", received_packet[15][0] == 1'b1);
        
        // Test with initialized = 0
        initialized = 1'b0;
        error = 1'b0;
        quat1_valid = 1'b0;  // Clear valid flags
        gyro1_valid = 1'b0;
        #(10 * CLK_PERIOD);
        
        // Read packet and verify flags
        mcu_read_packet();
        check_test("TC7: Flags byte with init=0, error=0, valid=0", received_packet[15] == 8'h00);
        
        // Restore
        initialized = 1'b1;
        #(10 * CLK_PERIOD);
        
        $display("");
        
        // ====================================================================
        // TC8: Verify data format (MSB,LSB per byte, signed 16-bit values)
        // ====================================================================
        $display("[TC8] Verify data format (MSB,LSB, signed 16-bit)");
        $display("----------------------------------------");
        
        // Test with negative values (signed 16-bit)
        output_quaternion(-16'd1000, -16'd2000, 16'd3000, -16'd4000);
        output_gyroscope(-16'd500, 16'd600, -16'd700);
        
        #(10 * CLK_PERIOD);
        
        // Read packet and verify
        mcu_read_packet();
        verify_packet(-16'd1000, -16'd2000, 16'd3000, -16'd4000,
                      -16'd500, 16'd600, -16'd700,
                      1'b1, 1'b1, 1'b1, 1'b0);
        
        // Verify byte order (MSB,LSB)
        // For -1000 (0xFC18): MSB=0xFC, LSB=0x18
        check_test("TC8: Quat W MSB correct", received_packet[1] == 8'hFC);
        check_test("TC8: Quat W LSB correct", received_packet[2] == 8'h18);
        
        // For 3000 (0x0BB8): MSB=0x0B, LSB=0xB8
        check_test("TC8: Quat Y MSB correct", received_packet[5] == 8'h0B);
        check_test("TC8: Quat Y LSB correct", received_packet[6] == 8'hB8);
        
        #(10 * CLK_PERIOD);
        $display("");
        
        // ====================================================================
        // Test Summary
        // ====================================================================
        $display("\n========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Total tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        
        if (fail_count == 0) begin
            $display("\n*** ALL TESTS PASSED ***");
        end else begin
            $display("\n*** SOME TESTS FAILED ***");
        end
        
        $display("\n========================================\n");
        
        #(100 * CLK_PERIOD);
        $finish;
    end

endmodule

