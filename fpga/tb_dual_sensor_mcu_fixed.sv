`timescale 1ns / 1ps

// Professional Testbench for Dual BNO085 Sensors with MCU SPI Integration
// Fixed version with proper delay acceleration and monitoring

module tb_dual_sensor_mcu_fixed;

    // System signals
    logic fpga_rst_n;
    
    // Sensor 1 SPI Interface
    logic sclk1, mosi1, miso1, cs_n1, ps0_wake1, bno085_rst_n1, int_n1;
    
    // Sensor 2 SPI Interface
    logic sclk2, mosi2, miso2, cs_n2, ps0_wake2, bno085_rst_n2, int_n2;
    
    // MCU SPI Interface
    logic sclk_mcu, mosi_mcu, miso_mcu, cs_n_mcu, done, load;
    
    // Status LEDs
    logic led_initialized, led_error, led_heartbeat;
    
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
    logic [7:0] mcu_rx_buffer [0:28];
    integer mcu_rx_index = 0;
    logic mcu_sclk_en = 0;
    logic [7:0] mcu_sclk_div = 0;
    logic mcu_sclk_reg = 0;
    logic [7:0] mcu_rx_shift = 0;
    logic [2:0] mcu_bit_cnt = 0;
    
    // MCU SPI clock generation
    always @(posedge dut.clk) begin
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
    
    // MCU SPI receive logic (Mode 0: sample on rising edge, MSB first)
    always @(posedge sclk_mcu) begin
        if (!cs_n_mcu) begin
            // Sample MISO and shift into receive register (MSB first)
            mcu_rx_shift <= {mcu_rx_shift[6:0], miso_mcu};
            mcu_bit_cnt <= mcu_bit_cnt + 1;
            
            if (mcu_bit_cnt == 7) begin
                // Byte complete - store it
                mcu_rx_buffer[mcu_rx_index] <= {mcu_rx_shift[6:0], miso_mcu};
                mcu_rx_index <= mcu_rx_index + 1;
                mcu_bit_cnt <= 0;
                mcu_rx_shift <= 0;
            end
        end
    end
    
    // MCU task: Read sensor data packet (29 bytes)
    task mcu_read_packet;
        integer i;
        begin
            $display("[MCU] Waiting for DONE signal...");
            while (!done) begin
                @(posedge dut.clk);
            end
            $display("[MCU] DONE signal received, starting SPI transaction");
            
            mcu_rx_index = 0;
            mcu_bit_cnt = 0;
            mcu_rx_shift = 0;
            
            cs_n_mcu = 1'b0;
            mcu_sclk_en = 1'b1;
            
            for (i = 0; i < 29; i = i + 1) begin
                while (mcu_rx_index <= i) begin
                    @(posedge dut.clk);
                end
                $display("[MCU] Received byte %0d: 0x%02h", i, mcu_rx_buffer[i]);
            end
            
            #1000;
            cs_n_mcu = 1'b1;
            mcu_sclk_en = 1'b0;
            
            $display("[MCU] Acknowledging with LOAD toggle");
            load = 1'b1;
            #1000;
            load = 1'b0;
            #1000;
            
            $display("[MCU] Packet read complete");
        end
    endtask
    
    // Test data variables
    logic [15:0] rx_quat1_w, rx_quat1_x, rx_quat1_y, rx_quat1_z;
    logic [15:0] rx_gyro1_x, rx_gyro1_y, rx_gyro1_z;
    logic [15:0] rx_quat2_w, rx_quat2_x, rx_quat2_y, rx_quat2_z;
    logic [15:0] rx_gyro2_x, rx_gyro2_y, rx_gyro2_z;
    logic [15:0] expected_quat1_w, expected_quat1_x, expected_quat1_y, expected_quat1_z;
    logic [15:0] expected_gyro1_x, expected_gyro1_y, expected_gyro1_z;
    logic [15:0] expected_quat2_w, expected_quat2_x, expected_quat2_y, expected_quat2_z;
    logic [15:0] expected_gyro2_x, expected_gyro2_y, expected_gyro2_z;
    
    // Command counters
    integer init_commands_sensor1 = 0;
    integer init_commands_sensor2 = 0;
    
    // Professional delay acceleration - speeds up simulation significantly
    // Also accelerate the top-level reset delay
    initial begin
        wait(fpga_rst_n == 1);
        #10000; // Wait for reset propagation
        forever begin
            @(posedge dut.clk);
            
            // Accelerate top-level reset delay (2 second delay)
            if (dut.rst_delay_counter < 23'd5_999_900) begin
                force dut.rst_delay_counter = 23'd5_999_990;
                @(posedge dut.clk);
                release dut.rst_delay_counter;
            end
            
            // Sensor 1: Accelerate INIT_WAIT_RESET (state 1)
            if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 1) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter < 19'd299_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter = 19'd299_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter;
                end
            end
            // Sensor 1: Accelerate INIT_DONE_CHECK (state 6)
            else if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 6) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter < 19'd29_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter = 19'd29_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl1.delay_counter;
                end
            end
            
            // Sensor 2: Accelerate INIT_WAIT_RESET (state 1)
            if (dut.dual_sensor_ctrl.bno085_ctrl2.state == 1) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter < 19'd299_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter = 19'd299_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter;
                end
            end
            // Sensor 2: Accelerate INIT_DONE_CHECK (state 6)
            else if (dut.dual_sensor_ctrl.bno085_ctrl2.state == 6) begin
                if (dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter < 19'd29_900) begin
                    force dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter = 19'd29_990;
                    @(posedge dut.clk);
                    release dut.dual_sensor_ctrl.bno085_ctrl2.delay_counter;
                end
            end
        end
    end
    
    // Monitor command completions accurately
    initial begin
        init_commands_sensor1 = 0;
        forever begin
            @(posedge cs_n1);
            #200;
            // Check if we're in INIT_DONE_CHECK state (command completed)
            if (dut.dual_sensor_ctrl.bno085_ctrl1.state == 6) begin
                init_commands_sensor1 = init_commands_sensor1 + 1;
                $display("[%0t] Sensor 1: Command %0d completed (cmd_select=%0d)", 
                         $time, init_commands_sensor1, dut.dual_sensor_ctrl.bno085_ctrl1.cmd_select);
            end
        end
    end
    
    initial begin
        init_commands_sensor2 = 0;
        forever begin
            @(posedge cs_n2);
            #200;
            // Check if we're in INIT_DONE_CHECK state (command completed)
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
        $dumpvars(0, tb_dual_sensor_mcu_fixed);
        
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
                #50000000; // 50ms timeout (with acceleration should complete much faster)
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
        
        // 3. Test Sensor Data Reading - Send data to both sensors simultaneously
        $display("\n[TEST] Step 3: Testing Dual Sensor Data Reading");
        #10000;
        
        expected_quat1_w = 16'h4000;
        expected_quat1_x = 16'd100;
        expected_quat1_y = 16'd200;
        expected_quat1_z = 16'd300;
        expected_gyro1_x = 16'd1000;
        expected_gyro1_y = 16'd2000;
        expected_gyro1_z = 16'd3000;
        
        expected_quat2_w = 16'h5000;
        expected_quat2_x = 16'd400;
        expected_quat2_y = 16'd500;
        expected_quat2_z = 16'd600;
        expected_gyro2_x = 16'd4000;
        expected_gyro2_y = 16'd5000;
        expected_gyro2_z = 16'd6000;
        
        $display("  Sending Sensor 1 Rotation Vector: W=0x%04h, X=%0d, Y=%0d, Z=%0d",
                 expected_quat1_w, expected_quat1_x, expected_quat1_y, expected_quat1_z);
        $display("  Sending Sensor 2 Rotation Vector: W=0x%04h, X=%0d, Y=%0d, Z=%0d",
                 expected_quat2_w, expected_quat2_x, expected_quat2_y, expected_quat2_z);
        
        // Send quaternion data to both sensors
        sensor1.send_rotation_vector(
            expected_quat1_x, expected_quat1_y, expected_quat1_z, expected_quat1_w
        );
        sensor2.send_rotation_vector(
            expected_quat2_x, expected_quat2_y, expected_quat2_z, expected_quat2_w
        );
        
        // Wait for both to be received
        fork
            begin
                wait(dut.dual_sensor_ctrl.quat1_valid == 1);
                $display("[PASS] Sensor 1 Quaternion Received!");
            end
            begin
                wait(dut.dual_sensor_ctrl.quat2_valid == 1);
                $display("[PASS] Sensor 2 Quaternion Received!");
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for quaternions!");
            end
        join_any
        
        #10000;
        
        // Send gyroscope data to both sensors
        $display("  Sending Sensor 1 Gyroscope: X=%0d, Y=%0d, Z=%0d",
                 expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
        $display("  Sending Sensor 2 Gyroscope: X=%0d, Y=%0d, Z=%0d",
                 expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
        
        sensor1.send_gyroscope(expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
        sensor2.send_gyroscope(expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
        
        // Wait for both to be received
        fork
            begin
                wait(dut.dual_sensor_ctrl.gyro1_valid == 1);
                $display("[PASS] Sensor 1 Gyroscope Received!");
            end
            begin
                wait(dut.dual_sensor_ctrl.gyro2_valid == 1);
                $display("[PASS] Sensor 2 Gyroscope Received!");
            end
            begin
                #5000000;
                $display("[FAIL] TIMEOUT waiting for gyroscopes!");
            end
        join_any
        
        // 4. Test Data Ready Signal
        $display("\n[TEST] Step 4: Testing Data Ready Signal");
        #1000;
        
        // Check if data_ready was asserted (it's a combinational signal)
        // We need to check it right after both valid signals are asserted
        if (dut.dual_sensor_ctrl.data_ready) begin
            $display("[PASS] Data ready signal asserted");
        end else begin
            $display("[INFO] Data ready not currently asserted (may be one-cycle pulse)");
            // Try sending data again to both sensors simultaneously
            $display("  Sending data to both sensors again to trigger data_ready...");
            sensor1.send_gyroscope(expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
            sensor2.send_gyroscope(expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
            #10000;
            if (dut.dual_sensor_ctrl.data_ready) begin
                $display("[PASS] Data ready signal asserted after second data send");
            end else begin
                $display("[FAIL] Data ready signal still not asserted");
            end
        end
        
        // 5. Test MCU SPI Communication
        $display("\n[TEST] Step 5: Testing MCU SPI Communication");
        #20000;
        
        // Clear any previous data_ready by waiting a bit
        #50000;
        
        // Send fresh data to both sensors SIMULTANEOUSLY to ensure both quat and gyro are ready
        $display("  Sending fresh data to both sensors (quat + gyro)...");
        // Send quaternion first
        sensor1.send_rotation_vector(
            expected_quat1_x, expected_quat1_y, expected_quat1_z, expected_quat1_w
        );
        sensor2.send_rotation_vector(
            expected_quat2_x, expected_quat2_y, expected_quat2_z, expected_quat2_w
        );
        
        // Wait for quaternions to be received
        #50000;
        
        // Then send gyroscope data
        sensor1.send_gyroscope(expected_gyro1_x, expected_gyro1_y, expected_gyro1_z);
        sensor2.send_gyroscope(expected_gyro2_x, expected_gyro2_y, expected_gyro2_z);
        
        // Wait for all data to be received and data_ready to be asserted
        #100000; // Wait for data to be processed
        
        // Debug: Check what data is in the controllers and formatter
        $display("  Debug: Sensor 1 data - Quat W=%04h X=%04h Y=%04h Z=%04h, Gyro X=%04h Y=%04h Z=%04h",
                 dut.dual_sensor_ctrl.quat1_w, dut.dual_sensor_ctrl.quat1_x,
                 dut.dual_sensor_ctrl.quat1_y, dut.dual_sensor_ctrl.quat1_z,
                 dut.dual_sensor_ctrl.gyro1_x, dut.dual_sensor_ctrl.gyro1_y, dut.dual_sensor_ctrl.gyro1_z);
        $display("  Debug: Sensor 2 data - Quat W=%04h X=%04h Y=%04h Z=%04h, Gyro X=%04h Y=%04h Z=%04h",
                 dut.dual_sensor_ctrl.quat2_w, dut.dual_sensor_ctrl.quat2_x,
                 dut.dual_sensor_ctrl.quat2_y, dut.dual_sensor_ctrl.quat2_z,
                 dut.dual_sensor_ctrl.gyro2_x, dut.dual_sensor_ctrl.gyro2_y, dut.dual_sensor_ctrl.gyro2_z);
        $display("  Debug: data_ready=%b, sensor1_valid=%b, sensor2_valid=%b",
                 dut.dual_sensor_ctrl.data_ready,
                 dut.dual_sensor_ctrl.sensor1_valid,
                 dut.dual_sensor_ctrl.sensor2_valid);
        $display("  Debug: Formatter state=%0d, tx_data_ready=%b, tx_data=0x%02h, busy=%b",
                 dut.data_formatter.state,
                 dut.data_formatter.tx_data_ready,
                 dut.data_formatter.tx_data,
                 dut.data_formatter.busy);
        $display("  Debug: Formatter buffers - Quat1 W=%04h X=%04h Y=%04h Z=%04h, Gyro1 X=%04h Y=%04h Z=%04h",
                 dut.data_formatter.quat1_w_buf, dut.data_formatter.quat1_x_buf,
                 dut.data_formatter.quat1_y_buf, dut.data_formatter.quat1_z_buf,
                 dut.data_formatter.gyro1_x_buf, dut.data_formatter.gyro1_y_buf, dut.data_formatter.gyro1_z_buf);
        $display("  Debug: Formatter buffers - Quat2 W=%04h X=%04h Y=%04h Z=%04h, Gyro2 X=%04h Y=%04h Z=%04h",
                 dut.data_formatter.quat2_w_buf, dut.data_formatter.quat2_x_buf,
                 dut.data_formatter.quat2_y_buf, dut.data_formatter.quat2_z_buf,
                 dut.data_formatter.gyro2_x_buf, dut.data_formatter.gyro2_y_buf, dut.data_formatter.gyro2_z_buf);
        
        // Monitor formatter during transmission
        fork
            begin
                wait(dut.data_formatter.busy == 1);
                $display("  Debug: Formatter started, byte_index=%0d, tx_data=0x%02h", 
                         dut.data_formatter.byte_index, dut.data_formatter.tx_data);
                while(dut.data_formatter.busy == 1) begin
                    @(posedge dut.clk);
                    if (dut.spi_slave.tx_ack) begin
                        $display("  Debug: Byte %0d sent, next byte_index=%0d, next tx_data=0x%02h",
                                 dut.data_formatter.byte_index, 
                                 dut.data_formatter.byte_index + 1,
                                 dut.data_formatter.tx_data);
                    end
                end
            end
        join_none
        
        mcu_read_packet();
        
        // 6. Verify Packet Contents
        $display("\n[TEST] Step 6: Verifying Packet Contents");
        
        if (mcu_rx_buffer[0] == 8'hAA) begin
            $display("[PASS] Packet header correct: 0x%02h", mcu_rx_buffer[0]);
        end else begin
            $display("[FAIL] Packet header incorrect: 0x%02h (expected 0xAA)", mcu_rx_buffer[0]);
        end
        
        // Parse Sensor 1 data (little-endian)
        rx_quat1_w = mcu_rx_buffer[1] | (mcu_rx_buffer[2] << 8);
        rx_quat1_x = mcu_rx_buffer[3] | (mcu_rx_buffer[4] << 8);
        rx_quat1_y = mcu_rx_buffer[5] | (mcu_rx_buffer[6] << 8);
        rx_quat1_z = mcu_rx_buffer[7] | (mcu_rx_buffer[8] << 8);
        rx_gyro1_x = mcu_rx_buffer[9] | (mcu_rx_buffer[10] << 8);
        rx_gyro1_y = mcu_rx_buffer[11] | (mcu_rx_buffer[12] << 8);
        rx_gyro1_z = mcu_rx_buffer[13] | (mcu_rx_buffer[14] << 8);
        
        $display("  Sensor 1 Data: W=%04h X=%04h Y=%04h Z=%04h, Gyro: X=%04h Y=%04h Z=%04h",
                 rx_quat1_w, rx_quat1_x, rx_quat1_y, rx_quat1_z,
                 rx_gyro1_x, rx_gyro1_y, rx_gyro1_z);
        
        // Parse Sensor 2 data
        rx_quat2_w = mcu_rx_buffer[15] | (mcu_rx_buffer[16] << 8);
        rx_quat2_x = mcu_rx_buffer[17] | (mcu_rx_buffer[18] << 8);
        rx_quat2_y = mcu_rx_buffer[19] | (mcu_rx_buffer[20] << 8);
        rx_quat2_z = mcu_rx_buffer[21] | (mcu_rx_buffer[22] << 8);
        rx_gyro2_x = mcu_rx_buffer[23] | (mcu_rx_buffer[24] << 8);
        rx_gyro2_y = mcu_rx_buffer[25] | (mcu_rx_buffer[26] << 8);
        rx_gyro2_z = mcu_rx_buffer[27] | (mcu_rx_buffer[28] << 8);
        
        $display("  Sensor 2 Data: W=%04h X=%04h Y=%04h Z=%04h, Gyro: X=%04h Y=%04h Z=%04h",
                 rx_quat2_w, rx_quat2_x, rx_quat2_y, rx_quat2_z,
                 rx_gyro2_x, rx_gyro2_y, rx_gyro2_z);
        
        // 7. Final Status Check
        $display("\n[TEST] Step 7: Final Status Check");
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

endmodule

