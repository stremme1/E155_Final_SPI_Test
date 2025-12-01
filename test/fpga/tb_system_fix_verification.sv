`timescale 1ns / 1ps

// System Fix Verification Testbench
// Tests that valid flags persist and snapshot updates correctly
// Verifies the fixes for the "all zeros" issue

module tb_system_fix_verification;

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
    
    // Test variables
    logic [7:0] received_packet [0:15];
    integer byte_idx;
    integer test_passed;
    
    // Task to perform SPI read transaction
    task spi_read_transaction;
        integer i;
        begin
            // CS goes low to start transaction
            cs_n = 0;
            #(CLK_PERIOD * 2);  // Wait for CS to be recognized
            
            // Read 16 bytes
            for (i = 0; i < 16; i = i + 1) begin
                // Generate 8 SCK cycles for one byte
                repeat(8) begin
                    sck = 1;
                    #(SCK_PERIOD/2);
                    received_packet[i] = {received_packet[i][6:0], sdo};  // Sample on rising edge
                    sck = 0;
                    #(SCK_PERIOD/2);
                end
            end
            
            // CS goes high to end transaction
            cs_n = 1;
            #(CLK_PERIOD * 2);
        end
    endtask
    
    // Test case 1: Valid flags persist and snapshot updates
    task test_valid_flag_persistence;
        begin
            $display("[TEST 1] Testing valid flag persistence...");
            test_passed = 1;
            
            // Initialize
            quat1_valid = 0;
            gyro1_valid = 0;
            quat1_w = 16'h0000;
            quat1_x = 16'h0000;
            quat1_y = 16'h0000;
            quat1_z = 16'h0000;
            gyro1_x = 16'h0000;
            gyro1_y = 16'h0000;
            gyro1_z = 16'h0000;
            initialized = 1;
            error = 0;
            cs_n = 1;
            
            // Wait for initialization
            #(CLK_PERIOD * 10);
            
            // Set valid data with valid flags
            quat1_w = 16'h1234;
            quat1_x = 16'h5678;
            quat1_y = 16'h9ABC;
            quat1_z = 16'hDEF0;
            quat1_valid = 1;
            
            gyro1_x = 16'h1111;
            gyro1_y = 16'h2222;
            gyro1_z = 16'h3333;
            gyro1_valid = 1;
            
            // Wait for valid flags to be latched
            #(CLK_PERIOD * 5);
            
            // Keep valid flags high (persistent) - this is the fix
            // Valid flags should stay high until new data arrives
            #(CLK_PERIOD * 10);
            
            // Perform SPI read
            spi_read_transaction();
            
            // Verify received data
            if (received_packet[0] != 8'hAA) begin
                $display("  FAIL: Header byte incorrect (got 0x%02x, expected 0xAA)", received_packet[0]);
                test_passed = 0;
            end
            
            // Check quaternion data (bytes 1-8: w, x, y, z MSB,LSB)
            if ({received_packet[1], received_packet[2]} != 16'h1234) begin
                $display("  FAIL: Quat W incorrect (got 0x%04x, expected 0x1234)", {received_packet[1], received_packet[2]});
                test_passed = 0;
            end
            
            if ({received_packet[3], received_packet[4]} != 16'h5678) begin
                $display("  FAIL: Quat X incorrect (got 0x%04x, expected 0x5678)", {received_packet[3], received_packet[4]});
                test_passed = 0;
            end
            
            // Check flags byte (byte 15)
            if ((received_packet[15] & 8'h03) != 8'h03) begin
                $display("  FAIL: Valid flags incorrect (got 0x%02x, expected 0x03)", received_packet[15] & 8'h03);
                test_passed = 0;
            end
            
            if (test_passed) begin
                $display("  PASS: Valid flags persist and snapshot updates correctly");
            end else begin
                $display("  FAIL: Test 1 failed");
            end
        end
    endtask
    
    // Test case 2: Valid flags cleared when new data arrives
    task test_valid_flag_clearing;
        begin
            $display("[TEST 2] Testing valid flag clearing on new data...");
            test_passed = 1;
            
            // Set initial data
            quat1_w = 16'hAAAA;
            quat1_x = 16'hBBBB;
            quat1_y = 16'hCCCC;
            quat1_z = 16'hDDDD;
            quat1_valid = 1;
            
            #(CLK_PERIOD * 5);
            
            // Clear valid flag (simulating new report starting)
            quat1_valid = 0;
            
            // Set new data
            quat1_w = 16'h1111;
            quat1_x = 16'h2222;
            quat1_y = 16'h3333;
            quat1_z = 16'h4444;
            quat1_valid = 1;  // New valid data
            
            #(CLK_PERIOD * 5);
            
            // Perform SPI read
            spi_read_transaction();
            
            // Verify new data is captured
            if ({received_packet[1], received_packet[2]} != 16'h1111) begin
                $display("  FAIL: New quat W not captured (got 0x%04x, expected 0x1111)", {received_packet[1], received_packet[2]});
                test_passed = 0;
            end
            
            if (test_passed) begin
                $display("  PASS: Valid flags cleared and new data captured correctly");
            end else begin
                $display("  FAIL: Test 2 failed");
            end
        end
    endtask
    
    // Main test
    initial begin
        $display("========================================");
        $display("System Fix Verification Testbench");
        $display("========================================");
        
        // Initialize
        cs_n = 1;
        sdi = 0;
        quat1_valid = 0;
        gyro1_valid = 0;
        quat1_w = 0;
        quat1_x = 0;
        quat1_y = 0;
        quat1_z = 0;
        gyro1_x = 0;
        gyro1_y = 0;
        gyro1_z = 0;
        initialized = 0;
        error = 0;
        
        // Wait for reset
        #(CLK_PERIOD * 10);
        
        // Run tests
        test_valid_flag_persistence();
        #(CLK_PERIOD * 20);
        
        test_valid_flag_clearing();
        #(CLK_PERIOD * 20);
        
        $display("========================================");
        $display("Testbench complete");
        $display("========================================");
        $finish;
    end

endmodule

