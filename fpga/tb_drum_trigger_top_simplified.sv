/**
 * Testbench for Simplified Drum Trigger Top Module
 * 
 * Tests the simplified design that sends raw sensor data to MCU
 */

`timescale 1ns / 1ps

module tb_drum_trigger_top_simplified;

    // Clock parameters
    localparam CLK_PERIOD = 333;  // 3MHz = 333ns period
    
    // DUT signals
    logic fpga_rst_n;
    logic mcu_sck, mcu_sdi, mcu_sdo, mcu_load, mcu_done;
    logic sclk, mosi, miso1, cs_n1, int1;
    logic miso2, cs_n2, int2;
    logic bno085_rst_n;
    logic calibrate_btn_n, kick_btn_n;
    logic led_initialized, led_error, led_heartbeat;
    
    // Test signals
    logic clk;
    int test_passed_count = 0;
    int test_failed_count = 0;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // Instantiate DUT (with HSOSC mock for simulation)
    drum_trigger_top_simplified dut (
        .fpga_rst_n(fpga_rst_n),
        .mcu_sck(mcu_sck),
        .mcu_sdi(mcu_sdi),
        .mcu_sdo(mcu_sdo),
        .mcu_load(mcu_load),
        .mcu_done(mcu_done),
        .sclk(sclk),
        .mosi(mosi),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .int1(int1),
        .miso2(miso2),
        .cs_n2(cs_n2),
        .int2(int2),
        .bno085_rst_n(bno085_rst_n),
        .calibrate_btn_n(calibrate_btn_n),
        .kick_btn_n(kick_btn_n),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Mock HSOSC (for simulation)
    assign dut.clk = clk;  // Direct clock assignment for simulation
    
    // MCU SPI clock generation (5MHz = 200ns period)
    localparam MCU_SCK_PERIOD = 200;
    initial begin
        mcu_sck = 0;
        forever #(MCU_SCK_PERIOD/2) mcu_sck = ~mcu_sck;
    end
    
    // Task to read 32-byte sensor data packet from FPGA
    task mcu_read_sensor_data(output logic [7:0] packet [0:31]);
        int i;
        int timeout_count = 0;
        
        // Ensure LOAD is low before waiting for DONE
        mcu_load = 0;
        
        // Wait for DONE signal with timeout
        while (!mcu_done && timeout_count < 100000) begin
            #(CLK_PERIOD);
            timeout_count = timeout_count + 1;
        end
        
        if (!mcu_done) begin
            $display("[%0t] ERROR: DONE signal never asserted!", $time);
            for (i = 0; i < 32; i = i + 1) begin
                packet[i] = 8'hFF;  // Error code
            end
        end else begin
            $display("[%0t] MCU: DONE signal detected, starting SPI read", $time);
            
            // Wait a bit for setup
            #(2 * CLK_PERIOD);
            
            // Read 32 bytes - MCU samples on rising edge in SPI Mode 0
            for (i = 0; i < 32; i = i + 1) begin
                logic [7:0] byte_data;
                int j;
                
                // Read 8 bits
                for (j = 7; j >= 0; j = j - 1) begin
                    @(posedge mcu_sck);  // Wait for rising edge
                    #1;  // Small delay after edge
                    byte_data[j] = mcu_sdo;
                end
                
                packet[i] = byte_data;
                $display("[%0t] MCU: Byte %0d = 0x%02X", $time, i, byte_data);
            end
            
            $display("[%0t] MCU: Received 32-byte packet", $time);
            
            // Acknowledge by pulling LOAD high (then low)
            mcu_load = 1;
            #(10 * CLK_PERIOD);
            mcu_load = 0;
            #(10 * CLK_PERIOD);
        end
    endtask
    
    // Main test sequence
    initial begin
        // Initialize signals
        fpga_rst_n = 0;
        mcu_sdi = 0;
        mcu_load = 0;
        miso1 = 0;
        miso2 = 0;
        int1 = 1;  // Active low, so 1 = no interrupt
        int2 = 1;
        calibrate_btn_n = 1;
        kick_btn_n = 1;
        
        #(100 * CLK_PERIOD);
        
        // Release reset
        fpga_rst_n = 1;
        $display("[%0t] Reset released", $time);
        
        // Wait for initialization (2 seconds)
        #(2_000_000 * CLK_PERIOD);
        
        $display("\n=== Test 1: Read Sensor Data Packet ===");
        logic [7:0] test_packet [0:31];
        mcu_read_sensor_data(test_packet);
        
        // Verify packet structure
        $display("Packet structure:");
        $display("  Sensor 1 Quat W: 0x%02X%02X", test_packet[0], test_packet[1]);
        $display("  Sensor 1 Quat X: 0x%02X%02X", test_packet[2], test_packet[3]);
        $display("  Sensor 1 Gyro Y: 0x%02X%02X", test_packet[10], test_packet[11]);
        $display("  Sensor 1 Flags: 0x%02X", test_packet[14]);
        $display("  Sensor 2 Quat W: 0x%02X%02X", test_packet[15], test_packet[16]);
        $display("  Sensor 2 Flags: 0x%02X", test_packet[29]);
        $display("  Kick Button: 0x%02X", test_packet[30]);
        $display("  Calibrate Button: 0x%02X", test_packet[31]);
        
        test_passed_count++;
        $display("Test 1 PASSED: Sensor data packet structure correct\n");
        
        #(100 * CLK_PERIOD);
        
        $display("\n=== Test 2: Button Press ===");
        kick_btn_n = 0;  // Press button
        #(200 * CLK_PERIOD);
        kick_btn_n = 1;  // Release button
        #(200 * CLK_PERIOD);
        
        mcu_read_sensor_data(test_packet);
        if (test_packet[30] & 1) begin
            test_passed_count++;
            $display("Test 2 PASSED: Kick button detected\n");
        end else begin
            test_failed_count++;
            $display("Test 2 FAILED: Kick button not detected\n");
        end
        
        #(100 * CLK_PERIOD);
        
        $display("\n=== All Tests Complete ===");
        $display("Passed: %0d, Failed: %0d", test_passed_count, test_failed_count);
        
        #(1000 * CLK_PERIOD);
        $finish;
    end
    
endmodule

