`timescale 1ns / 1ps

/**
 * Integration Testbench: SHTP Reports -> SPI Slave Output
 * 
 * Tests the complete data path:
 * 1. Mock BNO085 sends SHTP rotation vector and gyroscope reports
 * 2. BNO085 controller parses the reports
 * 3. SPI slave captures the data
 * 4. MCU reads the data via SPI and verifies it matches the SHTP input
 * 
 * This testbench ensures that sensor data flows correctly from SHTP to SPI output.
 */

module tb_shtp_to_spi_integration;

    // Clock and reset
    logic clk, rst_n;
    parameter CLK_PERIOD = 333; // 3MHz
    
    // BNO085 SPI Interface
    logic sclk, mosi, miso1, cs_n1, ps0_wake1, int1;
    
    // MCU SPI Interface
    logic mcu_sck, mcu_sdi, mcu_sdo, mcu_cs_n;
    
    // Clock generation
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // Reset generation
    initial begin
        rst_n = 0;
        #(CLK_PERIOD * 10);
        rst_n = 1;
        $display("[TEST] Reset released at time %0t", $time);
    end
    
    // ============================================
    // Mock BNO085 Sensor
    // ============================================
    mock_bno085 sensor_model (
        .clk(clk),
        .rst_n(rst_n),
        .ps0_wake(ps0_wake1),
        .cs_n(cs_n1),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1),
        .int_n(int1)
    );
    
    // ============================================
    // SPI Master for BNO085
    // ============================================
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    
    spi_master #(.CLK_DIV(2)) spi_master_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi1_start),
        .tx_valid(spi1_tx_valid),
        .tx_data(spi1_tx_data),
        .tx_ready(spi1_tx_ready),
        .rx_valid(spi1_rx_valid),
        .rx_data(spi1_rx_data),
        .busy(spi1_busy),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1)
    );
    
    // ============================================
    // BNO085 Controller
    // ============================================
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    logic initialized1, error1;
    
    bno085_controller bno085_ctrl_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi1_start),
        .spi_tx_valid(spi1_tx_valid),
        .spi_tx_data(spi1_tx_data),
        .spi_tx_ready(spi1_tx_ready),
        .spi_rx_valid(spi1_rx_valid),
        .spi_rx_data(spi1_rx_data),
        .spi_busy(spi1_busy),
        .cs_n(cs_n1),
        .ps0_wake(ps0_wake1),
        .int_n(int1),
        .quat_valid(quat1_valid),
        .quat_w(quat1_w),
        .quat_x(quat1_x),
        .quat_y(quat1_y),
        .quat_z(quat1_z),
        .gyro_valid(gyro1_valid),
        .gyro_x(gyro1_x),
        .gyro_y(gyro1_y),
        .gyro_z(gyro1_z),
        .initialized(initialized1),
        .error(error1)
    );
    
    // ============================================
    // SPI Slave (MCU Interface)
    // ============================================
    spi_slave_mcu spi_slave_inst (
        .clk(clk),
        .cs_n(mcu_cs_n),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .initialized(initialized1),
        .error(error1),
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
    
    // ============================================
    // MCU SPI Master Simulation
    // ============================================
    // Simulates MCU reading 16-byte packet via SPI Mode 0
    // Modifies received_packet array directly
    // Based on working testbench pattern from tb_spi_slave_mcu_debug.sv
    task mcu_read_packet;
        integer i, j;
        parameter SCK_PERIOD = 1000; // 1MHz SPI clock
        begin
            // CS goes low to start transaction
            mcu_cs_n = 1'b0;
            mcu_sck = 1'b0; // SCK starts low (SPI Mode 0)
            #(CLK_PERIOD * 10); // Setup time - wait for FPGA to set up first byte
            
            // Read 16 bytes (128 bits total)
            // SPI Mode 0: MSB first, sample on rising edge
            for (i = 0; i < 16; i = i + 1) begin
                received_packet[i] = 8'h00;
                
                // Read 8 bits MSB first (SPI Mode 0: SCK starts low, first edge is rising)
                for (j = 7; j >= 0; j = j - 1) begin
                    // Rising edge - MCU samples
                    mcu_sck = 1'b1;
                    #(SCK_PERIOD/4);  // Small delay for sampling
                    received_packet[i][j] = mcu_sdo;
                    #(SCK_PERIOD/2 - SCK_PERIOD/4);
                    
                    // Falling edge - FPGA shifts to prepare next bit (except after last bit)
                    if (j > 0 || i < 15) begin
                        mcu_sck = 1'b0;
                        #(SCK_PERIOD/2);
                    end
                end
            end
            
            // CS goes high to end transaction
            mcu_sck = 1'b0; // Return SCK to idle low
            #(CLK_PERIOD);
            mcu_cs_n = 1'b1;
            #(CLK_PERIOD * 2);
        end
    endtask
    
    // ============================================
    // Test Variables
    // ============================================
    logic [7:0] received_packet [0:15];
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    
    // Expected values from SHTP reports
    logic signed [15:0] expected_quat_w, expected_quat_x, expected_quat_y, expected_quat_z;
    logic signed [15:0] expected_gyro_x, expected_gyro_y, expected_gyro_z;
    
    // Received data from SPI
    logic signed [15:0] received_quat_w, received_quat_x, received_quat_y, received_quat_z;
    logic signed [15:0] received_gyro_x, received_gyro_y, received_gyro_z;
    logic [7:0] flags;
    logic quat_valid_flag, gyro_valid_flag, init_flag, error_flag;
    
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
    
    // Helper to print packet
    task print_packet;
        integer i;
        begin
            $write("Packet: ");
            for (i = 0; i < 16; i = i + 1) begin
                $write("0x%02x ", received_packet[i]);
            end
            $write("\n");
        end
    endtask
    
    // ============================================
    // Main Test Sequence
    // ============================================
    initial begin
        // Initialize SPI signals
        mcu_cs_n = 1'b1;
        mcu_sck = 1'b0;
        mcu_sdi = 1'b0;
        
        $display("\n==========================================");
        $display("SHTP to SPI Integration Test");
        $display("==========================================\n");
        
        // Wait for initialization to complete
        $display("[TEST] Step 1: Waiting for BNO085 initialization...");
        wait(initialized1 == 1'b1);
        $display("[TEST] BNO085 initialized at time %0t", $time);
        #(CLK_PERIOD * 100); // Wait a bit more for stability
        
        // Test 1: Send Rotation Vector report and verify SPI output
        $display("\n[TEST] Step 2: Sending Rotation Vector Report");
        expected_quat_w = 16'h4000;  // 1.0 in Q14 format
        expected_quat_x = 16'h0000;
        expected_quat_y = 16'h0000;
        expected_quat_z = 16'h0000;
        
        // Wait for controller to be in WAIT_DATA state
        // WAIT_DATA state value is 7 (from bno085_controller enum)
        // Use a timeout to avoid infinite wait
        fork
            begin
                // Wait for valid data or timeout
                wait(quat1_valid == 1'b1 || gyro1_valid == 1'b1 || initialized1 == 1'b1);
            end
            begin
                #(CLK_PERIOD * 10_000_000); // 10M cycle timeout
                $display("[WARNING] Timeout waiting for controller state");
            end
        join_any
        #(CLK_PERIOD * 100);
        
        // Ensure INT is high before sending
        if (int1 == 1'b0) begin
            wait(int1 == 1'b1);
            #(CLK_PERIOD * 100);
        end
        
        $display("  Sending Rotation Vector: W=0x%04h X=0x%04h Y=0x%04h Z=0x%04h",
                 expected_quat_w, expected_quat_x, expected_quat_y, expected_quat_z);
        
        // Send the report
        sensor_model.send_rotation_vector(
            expected_quat_x, expected_quat_y, expected_quat_z, expected_quat_w
        );
        
        // Wait for controller to parse the report
        wait(quat1_valid == 1'b1);
        $display("[TEST] Quaternion received by controller");
        $display("  W=0x%04h X=0x%04h Y=0x%04h Z=0x%04h", 
                 quat1_w, quat1_x, quat1_y, quat1_z);
        
        // Verify controller received correct data
        check_test("Quaternion W matches", quat1_w == expected_quat_w);
        check_test("Quaternion X matches", quat1_x == expected_quat_x);
        check_test("Quaternion Y matches", quat1_y == expected_quat_y);
        check_test("Quaternion Z matches", quat1_z == expected_quat_z);
        
        // Wait for SPI slave to capture snapshot (need multiple clock cycles for CDC)
        // Also ensure CS is high so snapshot can update
        // Ensure CS is high (from previous transaction if any)
        mcu_cs_n = 1'b1;
        #(CLK_PERIOD * 500); // Wait for snapshot to update (multiple cycles for CDC)
        
        // Read packet via SPI
        $display("\n[TEST] Step 3: Reading packet via MCU SPI");
        mcu_read_packet();
        print_packet();
        
        // Verify header
        check_test("Header byte is 0xAA", received_packet[0] == 8'hAA);
        
        // Verify quaternion data (MSB,LSB format)
        received_quat_w = {received_packet[1], received_packet[2]};
        received_quat_x = {received_packet[3], received_packet[4]};
        received_quat_y = {received_packet[5], received_packet[6]};
        received_quat_z = {received_packet[7], received_packet[8]};
        
        $display("  Received Quaternion: W=0x%04h X=0x%04h Y=0x%04h Z=0x%04h",
                 received_quat_w, received_quat_x, received_quat_y, received_quat_z);
        
        check_test("SPI Quaternion W matches", received_quat_w == expected_quat_w);
        check_test("SPI Quaternion X matches", received_quat_x == expected_quat_x);
        check_test("SPI Quaternion Y matches", received_quat_y == expected_quat_y);
        check_test("SPI Quaternion Z matches", received_quat_z == expected_quat_z);
        
        // Verify flags byte
        flags = received_packet[15];
        quat_valid_flag = flags[0];
        gyro_valid_flag = flags[1];
        init_flag = flags[2];
        error_flag = flags[3];
        
        $display("  Flags byte: 0x%02x (quat_valid=%d, gyro_valid=%d, init=%d, error=%d)",
                 flags, quat_valid_flag, gyro_valid_flag, init_flag, error_flag);
        
        check_test("Quaternion valid flag set", quat_valid_flag == 1'b1);
        check_test("Initialized flag set", init_flag == 1'b1);
        check_test("Error flag clear", error_flag == 1'b0);
        
        // Test 2: Send Gyroscope report and verify SPI output
        $display("\n[TEST] Step 4: Sending Gyroscope Report");
        expected_gyro_x = 16'd100;
        expected_gyro_y = 16'd200;
        expected_gyro_z = 16'd300;
        
        // Wait for controller to be ready (gyro valid will be set when report is processed)
        // Use a timeout to avoid infinite wait
        fork
            begin
                wait(gyro1_valid == 1'b1 || quat1_valid == 1'b0); // Wait for gyro or quat to clear
            end
            begin
                #(CLK_PERIOD * 10_000_000); // 10M cycle timeout
                $display("[WARNING] Timeout waiting for controller to be ready");
            end
        join_any
        #(CLK_PERIOD * 100);
        
        // Ensure INT is high before sending
        if (int1 == 1'b0) begin
            wait(int1 == 1'b1);
            #(CLK_PERIOD * 100);
        end
        
        $display("  Sending Gyroscope: X=%d Y=%d Z=%d",
                 expected_gyro_x, expected_gyro_y, expected_gyro_z);
        
        // Send the report (need to check mock_bno085 signature)
        // For now, assume it takes x, y, z as inputs
        sensor_model.send_gyroscope(expected_gyro_x, expected_gyro_y, expected_gyro_z);
        
        // Wait for controller to parse the report
        wait(gyro1_valid == 1'b1);
        $display("[TEST] Gyroscope received by controller");
        $display("  X=%d Y=%d Z=%d", gyro1_x, gyro1_y, gyro1_z);
        
        // Verify controller received correct data
        check_test("Gyroscope X matches", gyro1_x == expected_gyro_x);
        check_test("Gyroscope Y matches", gyro1_y == expected_gyro_y);
        check_test("Gyroscope Z matches", gyro1_z == expected_gyro_z);
        
        // Wait for SPI slave to capture snapshot (need multiple clock cycles for CDC)
        // Ensure CS is high so snapshot can update
        mcu_cs_n = 1'b1;
        #(CLK_PERIOD * 500); // Wait for snapshot to update (multiple cycles for CDC)
        
        // Read packet via SPI
        $display("\n[TEST] Step 5: Reading packet via MCU SPI (with gyro data)");
        mcu_read_packet();
        print_packet();
        
        // Verify gyroscope data (MSB,LSB format)
        received_gyro_x = {received_packet[9], received_packet[10]};
        received_gyro_y = {received_packet[11], received_packet[12]};
        received_gyro_z = {received_packet[13], received_packet[14]};
        
        $display("  Received Gyroscope: X=%d Y=%d Z=%d",
                 received_gyro_x, received_gyro_y, received_gyro_z);
        
        check_test("SPI Gyroscope X matches", received_gyro_x == expected_gyro_x);
        check_test("SPI Gyroscope Y matches", received_gyro_y == expected_gyro_y);
        check_test("SPI Gyroscope Z matches", received_gyro_z == expected_gyro_z);
        
        // Verify flags byte
        flags = received_packet[15];
        quat_valid_flag = flags[0];
        gyro_valid_flag = flags[1];
        init_flag = flags[2];
        error_flag = flags[3];
        
        $display("  Flags byte: 0x%02x (quat_valid=%d, gyro_valid=%d, init=%d, error=%d)",
                 flags, quat_valid_flag, gyro_valid_flag, init_flag, error_flag);
        
        check_test("Gyroscope valid flag set", gyro_valid_flag == 1'b1);
        check_test("Quaternion valid flag still set", quat_valid_flag == 1'b1);
        
        // Test 3: Verify data persists across multiple SPI reads
        $display("\n[TEST] Step 6: Verifying data persists across multiple SPI reads");
        #(CLK_PERIOD * 100);
        
        mcu_read_packet();
        received_quat_w = {received_packet[1], received_packet[2]};
        received_gyro_x = {received_packet[9], received_packet[10]};
        
        check_test("Quaternion persists", received_quat_w == expected_quat_w);
        check_test("Gyroscope persists", received_gyro_x == expected_gyro_x);
        
        // Print summary
        $display("\n==========================================");
        $display("Test Summary");
        $display("==========================================");
        $display("Total tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        
        if (fail_count == 0) begin
            $display("\n[SUCCESS] All tests passed!");
        end else begin
            $display("\n[FAILURE] Some tests failed!");
        end
        
        $finish;
    end
    
    // Timeout
    initial begin
        #(CLK_PERIOD * 50_000_000); // 50M cycles timeout
        $display("\n[ERROR] Test timeout!");
        $finish;
    end

endmodule

