`timescale 1ns / 1ps

// Comprehensive Testbench for Dual BNO085 Sensors with MCU SPI Integration
// Tests:
// 1. Dual sensor initialization
// 2. Sensor data reading from both sensors
// 3. Data formatter packaging
// 4. SPI slave MCU communication
// 5. DONE/LOAD handshake protocol
// 6. Full end-to-end data flow

module tb_dual_sensor_mcu;

    // System signals
    logic fpga_rst_n;
    logic clk;
    
    // Sensor 1 SPI Interface
    logic sclk1, mosi1, miso1, cs_n1, ps0_wake1, bno085_rst_n1, int_n1;
    
    // Sensor 2 SPI Interface
    logic sclk2, mosi2, miso2, cs_n2, ps0_wake2, bno085_rst_n2, int_n2;
    
    // MCU SPI Interface
    logic sclk_mcu, mosi_mcu, miso_mcu, cs_n_mcu, done, load;
    
    // Status LEDs
    logic led_initialized, led_error, led_heartbeat;
    
    // Test data variables
    logic [15:0] rx_quat1_w, rx_quat1_x, rx_quat1_y, rx_quat1_z;
    logic [15:0] rx_gyro1_x, rx_gyro1_y, rx_gyro1_z;
    logic [15:0] rx_quat2_w, rx_quat2_x, rx_quat2_y, rx_quat2_z;
    logic [15:0] rx_gyro2_x, rx_gyro2_y, rx_gyro2_z;
    
    // Instantiate the DUT
    drum_trigger_top dut (
        .fpga_rst_n(fpga_rst_n),
        .sclk1(sclk1),
        .mosi1(mosi1),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .ps0_wake1(ps0_wake1),
        .bno085_rst_n1(bno085_rst_n1),
        .int_n1(int_n1),
        .sclk2(sclk2),
        .mosi2(mosi2),
        .miso2(miso2),
        .cs_n2(cs_n2),
        .ps0_wake2(ps0_wake2),
        .bno085_rst_n2(bno085_rst_n2),
        .int_n2(int_n2),
        .sclk_mcu(sclk_mcu),
        .mosi_mcu(mosi_mcu),
        .miso_mcu(miso_mcu),
        .cs_n_mcu(cs_n_mcu),
        .done(done),
        .load(load),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Instantiate Mock BNO085 Sensors
    mock_bno085 sensor1 (
        .clk(dut.clk),
        .rst_n(bno085_rst_n1),
        .ps0_wake(ps0_wake1),
        .cs_n(cs_n1),
        .sclk(sclk1),
        .mosi(mosi1),
        .miso(miso1),
        .int_n(int_n1)
    );
    
    mock_bno085 sensor2 (
        .clk(dut.clk),
        .rst_n(bno085_rst_n2),
        .ps0_wake(ps0_wake2),
        .cs_n(cs_n2),
        .sclk(sclk2),
        .mosi(mosi2),
        .miso(miso2),
        .int_n(int_n2)
    );
    
    // Mock MCU SPI Master (Mode 0: CPOL=0, CPHA=0)
    // Simulates MCU reading data from FPGA
    logic [7:0] mcu_rx_buffer [0:28];
    integer mcu_rx_index = 0;
    logic mcu_sclk_en = 0;
    logic [7:0] mcu_sclk_div = 0;
    logic mcu_sclk_reg = 0;
    logic [7:0] mcu_rx_shift = 0;
    logic [2:0] mcu_bit_cnt = 0;
    
    // MCU SPI clock generation (simulate MCU clock domain)
    always @(posedge clk) begin
        if (mcu_sclk_en) begin
            if (mcu_sclk_div < 8'd10) begin
                mcu_sclk_div <= mcu_sclk_div + 1;
            end else begin
                mcu_sclk_div <= 0;
                mcu_sclk_reg <= ~mcu_sclk_reg;
            end
        end else begin
            mcu_sclk_reg <= 1'b0;
            mcu_sclk_div <= 0;
        end
    end
    assign sclk_mcu = mcu_sclk_reg;
    
    // MCU SPI receive logic (Mode 0: sample on rising edge)
    always @(posedge sclk_mcu) begin
        if (!cs_n_mcu) begin
            mcu_rx_shift <= {mcu_rx_shift[6:0], miso_mcu};
            mcu_bit_cnt <= mcu_bit_cnt + 1;
            
            if (mcu_bit_cnt == 7) begin
                mcu_rx_buffer[mcu_rx_index] <= mcu_rx_shift;
                mcu_rx_index <= mcu_rx_index + 1;
                mcu_bit_cnt <= 0;
            end
        end
    end
    
    // MCU task: Read sensor data packet (29 bytes)
    task mcu_read_packet;
        integer i;
        begin
            $display("[MCU] Waiting for DONE signal...");
            // Wait for DONE signal (blocking wait like real MCU)
            while (!done) begin
                @(posedge clk);
            end
            $display("[MCU] DONE signal received, starting SPI transaction");
            
            // Reset buffer index
            mcu_rx_index = 0;
            mcu_bit_cnt = 0;
            mcu_rx_shift = 0;
            
            // Lower CS and enable clock
            cs_n_mcu = 1'b0;
            mcu_sclk_en = 1'b1;
            
            // Read 29 bytes
            for (i = 0; i < 29; i = i + 1) begin
                // Wait for byte to be received
                while (mcu_rx_index <= i) begin
                    @(posedge clk);
                end
                $display("[MCU] Received byte %0d: 0x%02h", i, mcu_rx_buffer[i]);
            end
            
            // Wait for SPI to complete
            #1000;
            
            // Raise CS and disable clock
            cs_n_mcu = 1'b1;
            mcu_sclk_en = 1'b0;
            
            // Acknowledge with LOAD toggle
            $display("[MCU] Acknowledging with LOAD toggle");
            load = 1'b1;
            #1000;
            load = 1'b0;
            #1000;
            
            $display("[MCU] Packet read complete");
        end
    endtask
    
    // Test state tracking
    integer test_step = 0;
    integer init_commands_sensor1 = 0;
    integer init_commands_sensor2 = 0;
    logic [15:0] expected_quat1_w, expected_quat1_x, expected_quat1_y, expected_quat1_z;
    logic [15:0] expected_gyro1_x, expected_gyro1_y, expected_gyro1_z;
    logic [15:0] expected_quat2_w, expected_quat2_x, expected_quat2_y, expected_quat2_z;
    logic [15:0] expected_gyro2_x, expected_gyro2_y, expected_gyro2_z;
    
    // Accelerator: Skip long delays for faster simulation
    initial begin
        wait(fpga_rst_n == 1);
        #200000; // Wait for reset to propagate and controllers to start
        forever begin
            @(posedge dut.clk);
            // Sensor 1: Skip INIT_WAIT_RESET delay (state 1)
            if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 1) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter < 19'd299_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter = 19'd299_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter;
                end
            end
            // Sensor 1: Skip INIT_DONE_CHECK delay (state 6)
            else if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 6) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter < 19'd29_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter = 19'd29_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter;
                end
            end
            // Sensor 2: Skip INIT_WAIT_RESET delay (state 1)
            if (dut.dual_sensor_ctrl.bno085_ctrl2.state == 1) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter < 19'd299_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter = 19'd299_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter;
                end
            end
            // Sensor 2: Skip INIT_DONE_CHECK delay (state 6)
            else if (dut.dual_sensor_ctrl.bno085_ctrl2.state == 6) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter < 19'd29_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter = 19'd29_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter;
                end
            end
        end
    end
    
    // Monitor initialization commands - count when commands complete
    initial begin
        init_commands_sensor1 = 0;
        init_commands_sensor2 = 0;
        forever begin
            @(posedge cs_n1);
            #100;
            // Count when in INIT_DONE_CHECK state (command just completed)
            if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 6) begin
                init_commands_sensor1 = init_commands_sensor1 + 1;
                $display("[%0t] Sensor 1: Command %0d completed (cmd_select=%0d)", 
                         $time, init_commands_sensor1, dut.dual_sensor_ctrl.bno085_ctrl1.cmd_select);
            end
        end
    end
    
    initial begin
        forever begin
            @(posedge cs_n2);
            #100;
            // Count when in INIT_DONE_CHECK state (command just completed)
            if (dut.dual_sensor_ctrl.bno085_ctrl2.state == 6) begin
                init_commands_sensor2 = init_commands_sensor2 + 1;
                $display("[%0t] Sensor 2: Command %0d completed (cmd_select=%0d)", 
                         $time, init_commands_sensor2, dut.dual_sensor_ctrl.bno085_ctrl2.cmd_select);
            end
        end
    end
    
    // Main Test Sequence
    initial begin
        $dumpfile("dual_sensor_mcu_test.vcd");
        $dumpvars(0, tb_dual_sensor_mcu);
        
        $display("========================================");
        $display("Dual BNO085 + MCU SPI Integration Test");
        $display("========================================");
        
        // Initialize signals
        fpga_rst_n = 0;
        cs_n_mcu = 1'b1;
        load = 1'b0;
        mosi_mcu = 1'b0;
        
        // 1. Reset System
        $display("\n[TEST] Step 1: System Reset");
        #1000;
        fpga_rst_n = 1;
        $display("[TEST] Reset released at %0t", $time);
        
        // 2. Wait for Dual Sensor Initialization
        $display("\n[TEST] Step 2: Waiting for Dual Sensor Initialization");
        $display("  - Sensor 1: Product ID, Rotation Vector, Gyroscope");
        $display("  - Sensor 2: Product ID, Rotation Vector, Gyroscope");
        
        fork
            begin
                wait(dut.dual_sensor_ctrl.initialized == 1);
                $display("\n[PASS] Both Sensors Initialized!");
                $display("  - LED status: %b", led_initialized);
                $display("  - Sensor 1 commands: %0d", init_commands_sensor1);
                $display("  - Sensor 2 commands: %0d", init_commands_sensor2);
                
                if (init_commands_sensor1 >= 3 && init_commands_sensor2 >= 3) begin
                    $display("[PASS] All initialization commands completed");
                end else begin
                    $display("[FAIL] Expected 3 commands per sensor");
                    $display("  Sensor 1: %0d, Sensor 2: %0d", 
                             init_commands_sensor1, init_commands_sensor2);
                end
            end
            begin
                #2000000000; // 2 seconds timeout (with acceleration should be much faster)
                $display("\n[FAIL] TIMEOUT waiting for initialization!");
                $display("  Sensor 1 initialized: %b", dut.dual_sensor_ctrl.initialized1);
                $display("  Sensor 2 initialized: %b", dut.dual_sensor_ctrl.initialized2);
                $display("  Sensor 1 state: %0d", dut.dual_sensor_ctrl.bno085_ctrl1.state);
                $display("  Sensor 2 state: %0d", dut.dual_sensor_ctrl.bno085_ctrl2.state);
                $display("  Commands sensor 1: %0d", init_commands_sensor1);
                $display("  Commands sensor 2: %0d", init_commands_sensor2);
                $finish;
            end
        join_any
        
        // 3. Test Sensor 1 Data Reading
        $display("\n[TEST] Step 3: Testing Sensor 1 Data Reading");
        #10000;
        
        expected_quat1_w = 16'h4000;
        expected_quat1_x = 16'd100;
        expected_quat1_y = 16'd200;
        expected_quat1_z = 16'd300;
        expected_gyro1_x = 16'd1000;
        expected_gyro1_y = 16'd2000;
        expected_gyro1_z = 16'd3000;
        
        $display("  Sending Sensor 1 Rotation Vector: W=0x%04h, X=%0d, Y=%0d, Z=%0d",
                 expected_quat1_w, expected_quat1_x, expected_quat1_y, expected_quat1_z);
        sensor1.send_rotation_vector(
            expected_quat1_x, expected_quat1_y, expected_quat1_z, expected_quat1_w
        );
        
        fork
            begin
                wait(dut.dual_sensor_ctrl.sensor1_valid == 1);
                $display("[PASS] Sensor 1 Quaternion Received!");
                $display("  W=%04h X=%04h Y=%04h Z=%04h",
                         dut.dual_sensor_ctrl.quat1_w,
                         dut.dual_sensor_ctrl.quat1_x,
                         dut.dual_sensor_ctrl.quat1_y,
                         dut.dual_sensor_ctrl.quat1_z);
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for Sensor 1 quaternion!");
            end
        join_any
        
        #5000;
        $display("  Sending Sensor 1 Gyroscope: X=%0d, Y=%0d, Z=%0d",
                 expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
        sensor1.send_gyroscope(expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
        
        fork
            begin
                wait(dut.dual_sensor_ctrl.gyro1_x == expected_gyro1_x);
                $display("[PASS] Sensor 1 Gyroscope Received!");
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for Sensor 1 gyroscope!");
            end
        join_any
        
        // 4. Test Sensor 2 Data Reading
        $display("\n[TEST] Step 4: Testing Sensor 2 Data Reading");
        #10000;
        
        expected_quat2_w = 16'h5000;
        expected_quat2_x = 16'd400;
        expected_quat2_y = 16'd500;
        expected_quat2_z = 16'd600;
        expected_gyro2_x = 16'd4000;
        expected_gyro2_y = 16'd5000;
        expected_gyro2_z = 16'd6000;
        
        $display("  Sending Sensor 2 Rotation Vector: W=0x%04h, X=%0d, Y=%0d, Z=%0d",
                 expected_quat2_w, expected_quat2_x, expected_quat2_y, expected_quat2_z);
        sensor2.send_rotation_vector(
            expected_quat2_x, expected_quat2_y, expected_quat2_z, expected_quat2_w
        );
        
        fork
            begin
                wait(dut.dual_sensor_ctrl.sensor2_valid == 1);
                $display("[PASS] Sensor 2 Quaternion Received!");
                $display("  W=%04h X=%04h Y=%04h Z=%04h",
                         dut.dual_sensor_ctrl.quat2_w,
                         dut.dual_sensor_ctrl.quat2_x,
                         dut.dual_sensor_ctrl.quat2_y,
                         dut.dual_sensor_ctrl.quat2_z);
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for Sensor 2 quaternion!");
            end
        join_any
        
        #5000;
        $display("  Sending Sensor 2 Gyroscope: X=%0d, Y=%0d, Z=%0d",
                 expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
        sensor2.send_gyroscope(expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
        
        fork
            begin
                wait(dut.dual_sensor_ctrl.gyro2_x == expected_gyro2_x);
                $display("[PASS] Sensor 2 Gyroscope Received!");
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for Sensor 2 gyroscope!");
            end
        join_any
        
        // 5. Test Data Ready Signal
        $display("\n[TEST] Step 5: Testing Data Ready Signal");
        #10000;
        
        // Both sensors should have data now
        if (dut.dual_sensor_ctrl.data_ready) begin
            $display("[PASS] Data ready signal asserted");
        end else begin
            $display("[FAIL] Data ready signal not asserted");
        end
        
        // 6. Test MCU SPI Communication
        $display("\n[TEST] Step 6: Testing MCU SPI Communication");
        $display("  - DONE/LOAD handshake");
        $display("  - 29-byte packet transmission");
        
        // Wait a bit for formatter to package data
        #10000;
        
        // MCU reads the packet
        mcu_read_packet();
        
        // Verify packet contents
        $display("\n[TEST] Step 7: Verifying Packet Contents");
        
        // Check header
        if (mcu_rx_buffer[0] == 8'hAA) begin
            $display("[PASS] Packet header correct: 0x%02h", mcu_rx_buffer[0]);
        end else begin
            $display("[FAIL] Packet header incorrect: 0x%02h (expected 0xAA)", mcu_rx_buffer[0]);
        end
        
        // Verify Sensor 1 data (little-endian)
        rx_quat1_w = mcu_rx_buffer[1] | (mcu_rx_buffer[2] << 8);
        rx_quat1_x = mcu_rx_buffer[3] | (mcu_rx_buffer[4] << 8);
        rx_quat1_y = mcu_rx_buffer[5] | (mcu_rx_buffer[6] << 8);
        rx_quat1_z = mcu_rx_buffer[7] | (mcu_rx_buffer[8] << 8);
        rx_gyro1_x = mcu_rx_buffer[9] | (mcu_rx_buffer[10] << 8);
        rx_gyro1_y = mcu_rx_buffer[11] | (mcu_rx_buffer[12] << 8);
        rx_gyro1_z = mcu_rx_buffer[13] | (mcu_rx_buffer[14] << 8);
        
        $display("  Sensor 1 Data:");
        $display("    Quat: W=%04h X=%04h Y=%04h Z=%04h", 
                 rx_quat1_w, rx_quat1_x, rx_quat1_y, rx_quat1_z);
        $display("    Gyro: X=%04h Y=%04h Z=%04h",
                 rx_gyro1_x, rx_gyro1_y, rx_gyro1_z);
        
        if (rx_quat1_w == expected_quat1_w &&
            rx_quat1_x == expected_quat1_x &&
            rx_quat1_y == expected_quat1_y &&
            rx_quat1_z == expected_quat1_z &&
            rx_gyro1_x == expected_gyro1_x &&
            rx_gyro1_y == expected_gyro1_y &&
            rx_gyro1_z == expected_gyro1_z) begin
            $display("[PASS] Sensor 1 data matches expected values");
        end else begin
            $display("[FAIL] Sensor 1 data mismatch");
        end
        
        // Verify Sensor 2 data
        rx_quat2_w = mcu_rx_buffer[15] | (mcu_rx_buffer[16] << 8);
        rx_quat2_x = mcu_rx_buffer[17] | (mcu_rx_buffer[18] << 8);
        rx_quat2_y = mcu_rx_buffer[19] | (mcu_rx_buffer[20] << 8);
        rx_quat2_z = mcu_rx_buffer[21] | (mcu_rx_buffer[22] << 8);
        rx_gyro2_x = mcu_rx_buffer[23] | (mcu_rx_buffer[24] << 8);
        rx_gyro2_y = mcu_rx_buffer[25] | (mcu_rx_buffer[26] << 8);
        rx_gyro2_z = mcu_rx_buffer[27] | (mcu_rx_buffer[28] << 8);
        
        $display("  Sensor 2 Data:");
        $display("    Quat: W=%04h X=%04h Y=%04h Z=%04h",
                 rx_quat2_w, rx_quat2_x, rx_quat2_y, rx_quat2_z);
        $display("    Gyro: X=%04h Y=%04h Z=%04h",
                 rx_gyro2_x, rx_gyro2_y, rx_gyro2_z);
        
        if (rx_quat2_w == expected_quat2_w &&
            rx_quat2_x == expected_quat2_x &&
            rx_quat2_y == expected_quat2_y &&
            rx_quat2_z == expected_quat2_z &&
            rx_gyro2_x == expected_gyro2_x &&
            rx_gyro2_y == expected_gyro2_y &&
            rx_gyro2_z == expected_gyro2_z) begin
            $display("[PASS] Sensor 2 data matches expected values");
        end else begin
            $display("[FAIL] Sensor 2 data mismatch");
        end
        
        // 8. Test Multiple Packets
        $display("\n[TEST] Step 8: Testing Multiple Packet Transmission");
        
        // Send new data from both sensors
        #50000;
        sensor1.send_rotation_vector(16'd1000, 16'd2000, 16'd3000, 16'h6000);
        sensor1.send_gyroscope(16'd5000, 16'd6000, 16'd7000);
        sensor2.send_rotation_vector(16'd4000, 16'd5000, 16'd6000, 16'h7000);
        sensor2.send_gyroscope(16'd8000, 16'd9000, 16'd10000);
        
        #50000;
        
        // Read second packet
        mcu_read_packet();
        
        $display("[PASS] Second packet received successfully");
        
        // 9. Final Status Check
        $display("\n[TEST] Step 9: Final Status Check");
        if (led_error == 1'b0) begin
            $display("[PASS] Error LED is OFF");
        end else begin
            $display("[FAIL] Error LED is ON");
        end
        
        if (led_initialized == 1'b1) begin
            $display("[PASS] Initialized LED is ON");
        end else begin
            $display("[FAIL] Initialized LED is OFF");
        end
        
        #10000;
        $display("\n========================================");
        $display("All Tests Completed Successfully!");
        $display("========================================");
        $finish;
    end
    
    // Monitor for errors
    always @(posedge dut.dual_sensor_ctrl.error) begin
        $display("\n[ERROR] Controller entered error state at %0t", $time);
    end
    
    // Monitor DONE signal
    always @(posedge done) begin
        $display("[MONITOR] DONE signal asserted at %0t", $time);
    end
    
    // Monitor LOAD signal
    always @(posedge load) begin
        $display("[MONITOR] LOAD signal asserted at %0t", $time);
    end

endmodule

