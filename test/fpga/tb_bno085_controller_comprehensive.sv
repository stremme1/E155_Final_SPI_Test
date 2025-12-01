`timescale 1ns / 1ps

// Comprehensive Testbench for BNO085 Controller
// Tests initialization sequence, response reading, and report parsing
// Per BNO08X datasheet Section 1.3

module tb_bno085_controller_comprehensive;

    // Clock and reset
    logic clk, rst_n;
    
    // SPI interface
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    logic cs_n;
    logic ps0_wake;
    logic int_n;
    
    // Sensor data outputs
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    
    // Status
    logic initialized, error;
    
    // SPI master signals
    logic sclk, mosi, miso;
    
    // Clock generation (3MHz)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // SPI Master instance
    spi_master #(.CLK_DIV(2)) spi_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi_start),
        .tx_valid(spi_tx_valid),
        .tx_data(spi_tx_data),
        .tx_ready(spi_tx_ready),
        .rx_valid(spi_rx_valid),
        .rx_data(spi_rx_data),
        .busy(spi_busy),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso)
    );
    
    // BNO085 Controller DUT
    bno085_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi_start),
        .spi_tx_valid(spi_tx_valid),
        .spi_tx_data(spi_tx_data),
        .spi_tx_ready(spi_tx_ready),
        .spi_rx_valid(spi_rx_valid),
        .spi_rx_data(spi_rx_data),
        .spi_busy(spi_busy),
        .cs_n(cs_n),
        .ps0_wake(ps0_wake),
        .int_n(int_n),
        .quat_valid(quat_valid),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_valid(gyro_valid),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .initialized(initialized),
        .error(error)
    );
    
    // ========================================================================
    // Mock BNO085 Sensor
    // ========================================================================
    // Simulates BNO085 behavior per datasheet:
    // - Responds to initialization commands
    // - Sends Get Feature Response (0xFC) after Set Feature
    // - Sends sensor reports on Channel 3 or 5
    
    logic [7:0] mock_response [0:255];
    integer mock_response_len = 0;
    integer mock_response_ptr = 0;
    logic [7:0] mock_tx_byte = 0;
    integer mock_tx_bit_cnt = 0;
    logic mock_cs_prev = 1;
    
    // Track initialization commands
    integer init_cmd_count = 0;
    integer response_sent_count = 0;
    
    // INT handling - assert after responses are queued or after reset/wake
    logic int_assert_pending = 0;
    logic [18:0] reset_int_delay = 0;
    logic ps0_wake_prev = 1;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_n <= 1'b1;
            int_assert_pending <= 1'b0;
            mock_cs_prev <= 1'b1;
            reset_int_delay <= 0;
            ps0_wake_prev <= 1;
        end else begin
            mock_cs_prev <= cs_n;
            ps0_wake_prev <= ps0_wake;
            
            // After reset, assert INT after delay (sensor ready)
            if (reset_int_delay < 19'd100_000) begin
                reset_int_delay <= reset_int_delay + 1;
            end else if (reset_int_delay == 19'd100_000) begin
                int_n <= 1'b0; // Assert INT to indicate sensor is ready
                reset_int_delay <= reset_int_delay + 1;
            end
            
            // Assert INT when PS0/WAKE goes low (wake signal)
            if (ps0_wake_prev == 1 && ps0_wake == 0) begin
                // Wake signal - assert INT after delay
                int_assert_pending <= 1'b1;
            end
            
            // INT assertion is handled in the posedge cs_n block above
            // This block just handles deassertion
            
            // Deassert INT when CS goes low (per datasheet 6.5.4)
            if (cs_n == 0) begin
                int_n <= 1'b1;
                int_assert_pending <= 1'b0;
            end
        end
    end
    
    // SPI Receive - sample MOSI on rising edge (Mode 3)
    logic [7:0] mock_rx_byte = 0;
    integer mock_rx_bit_cnt = 0;
    integer mock_rx_byte_count = 0;
    logic [7:0] mock_rx_buffer [0:31];
    integer mock_rx_ptr = 0;
    logic [15:0] mock_received_length = 0;
    logic [7:0] mock_received_channel = 0;
    logic [7:0] mock_received_seq = 0;
    
    always @(posedge sclk) begin
        if (!cs_n) begin
            mock_rx_byte = {mock_rx_byte[6:0], mosi};
            mock_rx_bit_cnt = mock_rx_bit_cnt + 1;
            
            if (mock_rx_bit_cnt == 8) begin
                mock_rx_bit_cnt = 0;
                mock_rx_buffer[mock_rx_ptr] = mock_rx_byte;
                mock_rx_byte_count = mock_rx_byte_count + 1;
                
                // Parse SHTP header
                if (mock_rx_byte_count == 1) begin
                    mock_received_length[7:0] = mock_rx_byte;
                end else if (mock_rx_byte_count == 2) begin
                    mock_received_length[15:8] = mock_rx_byte;
                    mock_received_length[15] = 0; // Clear continuation bit
                end else if (mock_rx_byte_count == 3) begin
                    mock_received_channel = mock_rx_byte;
                end else if (mock_rx_byte_count == 4) begin
                    mock_received_seq = mock_rx_byte;
                end else if (mock_rx_byte_count == 5) begin
                    // Now we have the first payload byte (command/report ID)
                    $display("[%0t] MOCK: Received payload byte 0: %02h", $time, mock_rx_byte);
                    $display("[%0t] MOCK: Buffer[0-4]: %02h %02h %02h %02h %02h", 
                             $time, mock_rx_buffer[0], mock_rx_buffer[1], mock_rx_buffer[2], 
                             mock_rx_buffer[3], mock_rx_buffer[4]);
                    $display("[%0t] MOCK: Channel=%02h, preparing response...", $time, mock_received_channel);
                    mock_prepare_response();
                    $display("[%0t] MOCK: Response prepared, len=%0d", $time, mock_response_len);
                end
                
                // Increment pointer after storing byte
                mock_rx_ptr = mock_rx_ptr + 1;
            end
        end
    end
    
    // SPI Transmit - shift MISO on falling edge (Mode 3)
    // Per datasheet: Mode 3 (CPOL=1, CPHA=1) - data is sampled on rising edge, so we shift on falling edge
    always @(negedge sclk) begin
        if (!cs_n) begin
            if (mock_response_len > 0 && mock_response_ptr < mock_response_len) begin
                miso = mock_tx_byte[7];
                mock_tx_bit_cnt = mock_tx_bit_cnt + 1;
                
                if (mock_tx_bit_cnt == 8) begin
                    $display("[%0t] MOCK: Transmitted byte %0d: %02h", $time, mock_response_ptr, mock_tx_byte);
                    mock_tx_bit_cnt = 0;
                    mock_response_ptr = mock_response_ptr + 1;
                    if (mock_response_ptr < mock_response_len) begin
                        mock_tx_byte = mock_response[mock_response_ptr];
                    end else begin
                        mock_tx_byte = 8'h00;
                        $display("[%0t] MOCK: All %0d bytes transmitted", $time, mock_response_len);
                    end
                end else begin
                    mock_tx_byte = {mock_tx_byte[6:0], 1'b0};
                end
            end else begin
                miso = 1'b0;
            end
        end
    end
    
    // Track when we should assert INT
    logic int_should_assert = 0;
    
    // Track if we received a full command (4+ bytes) and have response ready
    logic command_received = 0;
    
    // Reset on CS high
    always @(posedge cs_n) begin
        // If we just finished receiving a command and have a response ready, mark INT for assertion
        if (mock_response_len > 0 && command_received) begin
            int_should_assert = 1'b1;
            $display("[%0t] MOCK: CS high, response ready (len=%0d, cmd_received=%0d), will assert INT", 
                     $time, mock_response_len, command_received);
        end
        
        command_received = 0;
        mock_rx_ptr = 0;
        mock_rx_byte_count = 0;
        mock_rx_bit_cnt = 0;
        mock_rx_byte = 0;
        mock_received_length = 0;
        
        // If response was fully sent, prepare for next
        if (mock_response_ptr >= mock_response_len && mock_response_len > 0) begin
            $display("[%0t] MOCK: Response fully sent (ptr=%0d, len=%0d)", 
                     $time, mock_response_ptr, mock_response_len);
            mock_response_len = 0;
            mock_response_ptr = 0;
            response_sent_count = response_sent_count + 1;
            int_should_assert = 1'b0;
        end else if (mock_response_len > 0) begin
            // Response not fully sent - reload first byte for next transaction
            $display("[%0t] MOCK: Response partially sent (ptr=%0d, len=%0d), reloading", 
                     $time, mock_response_ptr, mock_response_len);
            mock_tx_byte = mock_response[0];
            mock_response_ptr = 0;
            mock_tx_bit_cnt = 0;
        end
    end
    
    // Reset transmit state when CS goes low (start of new transaction)
    always @(negedge cs_n) begin
        if (mock_response_len > 0) begin
            // Reload first byte when CS goes low to start transmitting
            mock_tx_byte = mock_response[0];
            mock_response_ptr = 0;
            mock_tx_bit_cnt = 0;
            miso = mock_response[0][7]; // Set MSB immediately (Mode 3: data ready before first clock)
            $display("[%0t] MOCK: CS low, starting to transmit response (len=%0d, first_byte=%02h)", 
                     $time, mock_response_len, mock_response[0]);
        end
    end
    
    // Assert INT when marked (in clocked domain)
    always_ff @(posedge clk) begin
        if (int_should_assert && cs_n == 1) begin
            int_n <= 1'b0;
            int_should_assert <= 1'b0;
        end
    end
    
    // Debug: Monitor INT assertion
    always @(negedge int_n) begin
        $display("[%0t] MOCK: INT asserted (response_len=%0d)", $time, mock_response_len);
    end
    
    // Prepare response based on received command
    task mock_prepare_response;
        integer i;
        begin
            $display("[%0t] MOCK: prepare_response: channel=%02h, buffer[4]=%02h, rx_ptr=%0d", 
                     $time, mock_received_channel, mock_rx_buffer[4], mock_rx_ptr);
            if (mock_received_channel == 8'h02) begin // Channel 2 (Control)
                // Product ID Request (0xF9) - first payload byte is at buffer[4] (5th byte, 0-indexed)
                if (mock_rx_buffer[4] == 8'hF9) begin
                    $display("[%0t] MOCK: Received Product ID Request", $time);
                    mock_response[0] = 8'h11; // Length LSB (17 bytes)
                    mock_response[1] = 8'h00; // Length MSB
                    mock_response[2] = 8'h02; // Channel
                    mock_response[3] = mock_received_seq; // Sequence
                    mock_response[4] = 8'hF8; // Report ID (Product ID Response)
                    mock_response[5] = 8'h00; // Reset Cause
                    mock_response[6] = 8'h03; // SW Version Major
                    mock_response[7] = 8'h02; // SW Version Minor
                    for (i = 8; i < 17; i = i + 1) begin
                        mock_response[i] = 8'h00; // Part number, build, etc.
                    end
                    mock_response_len = 17;
                    mock_response_ptr = 0;
                    mock_tx_byte = mock_response[0];
                    init_cmd_count = init_cmd_count + 1;
                    command_received = 1;
                    $display("[%0t] MOCK: Product ID Response prepared, len=%0d", $time, mock_response_len);
                end
                // Set Feature (0xFD)
                else if (mock_rx_buffer[4] == 8'hFD) begin
                    $display("[%0t] MOCK: Received Set Feature (Report ID=0x%02h)", $time, mock_rx_buffer[5]);
                    // Get Feature Response (0xFC) per datasheet Figure 1-30
                    mock_response[0] = 8'h05; // Length LSB (5 bytes)
                    mock_response[1] = 8'h00; // Length MSB
                    mock_response[2] = 8'h02; // Channel
                    mock_response[3] = mock_received_seq; // Sequence
                    mock_response[4] = 8'hFC; // Get Feature Response
                    mock_response_len = 5;
                    mock_response_ptr = 0;
                    mock_tx_byte = mock_response[0];
                    init_cmd_count = init_cmd_count + 1;
                    command_received = 1;
                    $display("[%0t] MOCK: Set Feature Response prepared, len=%0d", $time, mock_response_len);
                end else begin
                    $display("[%0t] MOCK: Unknown command 0x%02h on channel %02h", 
                             $time, mock_rx_buffer[4], mock_received_channel);
                end
            end else begin
                $display("[%0t] MOCK: Wrong channel %02h (expected 02)", $time, mock_received_channel);
            end
        end
    endtask
    
    // Task to send Rotation Vector report (Channel 3)
    // Per datasheet: Rotation Vector is 12 bytes payload (Report ID, Seq, Status, Delay, X, Y, Z, W)
    // Format: X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB, W LSB, W MSB
    task mock_send_rotation_vector;
        input [15:0] w, x, y, z;
        integer i;
        begin
            $display("[%0t] MOCK: Queuing Rotation Vector: W=%04h X=%04h Y=%04h Z=%04h", $time, w, x, y, z);
            // SHTP Header (4 bytes)
            mock_response[0] = 8'h10; // Length LSB (16 bytes total: 4 header + 12 payload)
            mock_response[1] = 8'h00; // Length MSB
            mock_response[2] = 8'h03; // Channel 3 (Input sensor reports)
            mock_response[3] = 8'h00; // Sequence
            // Payload (12 bytes per datasheet Section 1.3.5.2)
            mock_response[4] = 8'h05; // Report ID (Rotation Vector)
            mock_response[5] = 8'h00; // Sequence number
            mock_response[6] = 8'h03; // Status (Accuracy high)
            mock_response[7] = 8'h00; // Delay LSB
            // Quaternion data (little-endian, 16-bit each) - per datasheet X, Y, Z, W order
            mock_response[8] = x[7:0];  // X LSB
            mock_response[9] = x[15:8]; // X MSB
            mock_response[10] = y[7:0]; // Y LSB
            mock_response[11] = y[15:8]; // Y MSB
            mock_response[12] = z[7:0]; // Z LSB
            mock_response[13] = z[15:8]; // Z MSB
            mock_response[14] = w[7:0];  // W LSB
            mock_response[15] = w[15:8]; // W MSB
            mock_response_len = 16;
            mock_response_ptr = 0;
            mock_tx_byte = mock_response[0];
            // INT will be asserted by the always_ff block when CS goes high
        end
    endtask
    
    // Task to send Gyroscope report (Channel 3)
    task mock_send_gyroscope;
        input [15:0] x, y, z;
        integer i;
        begin
            $display("[%0t] MOCK: Queuing Gyroscope: X=%04h Y=%04h Z=%04h", $time, x, y, z);
            // SHTP Header (4 bytes)
            mock_response[0] = 8'h0A; // Length LSB (10 bytes total: 4 header + 6 payload)
            mock_response[1] = 8'h00; // Length MSB
            mock_response[2] = 8'h03; // Channel 3 (Input sensor reports)
            mock_response[3] = 8'h00; // Sequence
            // Payload (6 bytes per datasheet Figure 1-34)
            mock_response[4] = 8'h02; // Report ID (Calibrated Gyroscope)
            mock_response[5] = 8'h00; // Sequence number
            mock_response[6] = 8'h03; // Status (Accuracy high)
            mock_response[7] = 8'h00; // Delay LSB
            // Gyroscope data (little-endian, 16-bit each)
            mock_response[8] = x[7:0];  // X LSB
            mock_response[9] = x[15:8]; // X MSB
            mock_response[10] = y[7:0]; // Y LSB
            mock_response[11] = y[15:8]; // Y MSB
            mock_response[12] = z[7:0]; // Z LSB
            mock_response[13] = z[15:8]; // Z MSB
            mock_response_len = 14;
            mock_response_ptr = 0;
            mock_tx_byte = mock_response[0];
            // INT will be asserted by the always_ff block when CS goes high
        end
    endtask
    
    // ========================================================================
    // Test Sequence
    // ========================================================================
    
    integer test_pass = 0;
    integer test_fail = 0;
    
    initial begin
        $dumpfile("tb_bno085_controller_comprehensive.vcd");
        $dumpvars(0, tb_bno085_controller_comprehensive);
        
        $display("========================================");
        $display("BNO085 Controller Comprehensive Testbench");
        $display("========================================\n");
        
        // Initialize
        rst_n = 0;
        int_n = 1;
        #1000;
        rst_n = 1;
        $display("[%0t] Reset released", $time);
        
        // Test 1: Wait for initialization
        $display("\n[TEST 1] Waiting for initialization...");
        fork
            begin
                wait(initialized == 1);
                $display("[%0t] PASS: Controller initialized", $time);
                test_pass = test_pass + 1;
                
                if (init_cmd_count == 3) begin
                    $display("[%0t] PASS: All 3 initialization commands sent", $time);
                    test_pass = test_pass + 1;
                end else begin
                    $display("[%0t] FAIL: Expected 3 commands, got %0d", $time, init_cmd_count);
                    test_fail = test_fail + 1;
                end
                
                if (response_sent_count >= 3) begin
                    $display("[%0t] PASS: All responses read (%0d)", $time, response_sent_count);
                    test_pass = test_pass + 1;
                end else begin
                    $display("[%0t] FAIL: Expected 3 responses read, got %0d", $time, response_sent_count);
                    test_fail = test_fail + 1;
                end
            end
            begin
                #500000000; // 500ms timeout (increased to allow full initialization)
                $display("[%0t] FAIL: Timeout waiting for initialization", $time);
                $display("[%0t] DEBUG: State=%0d, init_cmd_count=%0d, response_sent_count=%0d", 
                         $time, dut.state, init_cmd_count, response_sent_count);
                test_fail = test_fail + 1;
                $finish;
            end
        join_any
        
        // Wait for controller to be in WAIT_DATA state
        wait(dut.state == 9); // WAIT_DATA
        #10000; // Wait a bit more
        
        // Test 2: Rotation Vector Report
        $display("\n[TEST 2] Testing Rotation Vector Report...");
        mock_send_rotation_vector(16'h4000, 16'h0000, 16'h0000, 16'h0000); // W=1.0, X=0, Y=0, Z=0
        
        fork
            begin
                wait(quat_valid == 1);
                $display("[%0t] PASS: quat_valid asserted", $time);
                test_pass = test_pass + 1;
                
                if (quat_w == 16'h4000 && quat_x == 16'h0000 && quat_y == 16'h0000 && quat_z == 16'h0000) begin
                    $display("[%0t] PASS: Quaternion data correct: W=%04h X=%04h Y=%04h Z=%04h", 
                             $time, quat_w, quat_x, quat_y, quat_z);
                    test_pass = test_pass + 1;
                end else begin
                    $display("[%0t] FAIL: Quaternion data incorrect: W=%04h X=%04h Y=%04h Z=%04h (expected W=4000)", 
                             $time, quat_w, quat_x, quat_y, quat_z);
                    test_fail = test_fail + 1;
                end
            end
            begin
                #5000000; // 5ms timeout
                $display("[%0t] FAIL: Timeout waiting for quat_valid", $time);
                test_fail = test_fail + 1;
            end
        join_any
        
        // Wait for next report
        wait(dut.state == 9); // WAIT_DATA
        #10000;
        
        // Test 3: Gyroscope Report
        $display("\n[TEST 3] Testing Gyroscope Report...");
        mock_send_gyroscope(16'd100, 16'd200, 16'd300);
        
        fork
            begin
                wait(gyro_valid == 1);
                $display("[%0t] PASS: gyro_valid asserted", $time);
                test_pass = test_pass + 1;
                
                if (gyro_x == 16'd100 && gyro_y == 16'd200 && gyro_z == 16'd300) begin
                    $display("[%0t] PASS: Gyroscope data correct: X=%04h Y=%04h Z=%04h", 
                             $time, gyro_x, gyro_y, gyro_z);
                    test_pass = test_pass + 1;
                end else begin
                    $display("[%0t] FAIL: Gyroscope data incorrect: X=%04h Y=%04h Z=%04h (expected 0064 00C8 012C)", 
                             $time, gyro_x, gyro_y, gyro_z);
                    test_fail = test_fail + 1;
                end
            end
            begin
                #5000000; // 5ms timeout
                $display("[%0t] FAIL: Timeout waiting for gyro_valid", $time);
                test_fail = test_fail + 1;
            end
        join_any
        
        // Test 4: Multiple reports
        $display("\n[TEST 4] Testing multiple reports...");
        wait(dut.state == 9);
        #10000;
        
        mock_send_rotation_vector(16'h2000, 16'h3000, 16'h4000, 16'h5000);
        wait(quat_valid == 1);
        if (quat_w == 16'h2000 && quat_x == 16'h3000 && quat_y == 16'h4000 && quat_z == 16'h5000) begin
            $display("[%0t] PASS: Second quaternion correct", $time);
            test_pass = test_pass + 1;
        end else begin
            $display("[%0t] FAIL: Second quaternion incorrect", $time);
            test_fail = test_fail + 1;
        end
        
        // Summary
        $display("\n========================================");
        $display("Test Summary");
        $display("========================================");
        $display("PASS: %0d", test_pass);
        $display("FAIL: %0d", test_fail);
        $display("========================================\n");
        
        if (test_fail == 0) begin
            $display("*** ALL TESTS PASSED ***");
        end else begin
            $display("*** SOME TESTS FAILED ***");
        end
        
        #10000;
        $finish;
    end
    
    // Monitor state changes
    logic [3:0] prev_state = 0;
    initial begin
        prev_state = 0;
        forever begin
            @(posedge clk);
            if (dut.state != prev_state) begin
                $display("[%0t] STATE: %0d -> %0d", $time, prev_state, dut.state);
                prev_state = dut.state;
            end
        end
    end
    
endmodule

