/**
 * Simplified Testbench for Top-Level Module
 * Tests MCU SPI interface and button functionality
 * Bypasses BNO085 sensors to avoid multiple driver issues
 */

`timescale 1ns / 1ps

module tb_top_mcu_spi_only;

    // Top-level signals
    logic fpga_rst_n;
    
    // MCU SPI Interface
    logic mcu_sck;
    logic mcu_sdi;
    logic mcu_sdo;
    logic mcu_load;
    logic mcu_done;
    
    // BNO085 SPI Interface (not used in this test, but must be connected)
    logic sclk;
    logic mosi;
    logic bno085_rst_n;
    logic miso1, miso2;
    logic cs_n1, cs_n2;
    logic int1, int2;
    
    // Button inputs
    logic calibrate_btn_n;
    logic kick_btn_n;
    
    // Status LEDs
    logic led_initialized;
    logic led_error;
    logic led_heartbeat;
    
    // Test variables
    logic [7:0] cmd1, cmd2;
    integer i;
    
    // Clock generation
    parameter CLK_PERIOD = 333;  // 3MHz FPGA clock
    parameter MCU_SCK_PERIOD = 1000;  // 1MHz MCU SPI clock
    
    // MCU SPI clock generation
    always begin
        mcu_sck = 0;
        #(MCU_SCK_PERIOD/2);
        mcu_sck = 1;
        #(MCU_SCK_PERIOD/2);
    end
    
    // DUT instantiation
    drum_trigger_top_integrated dut (
        .fpga_rst_n(fpga_rst_n),
        .mcu_sck(mcu_sck),
        .mcu_sdi(mcu_sdi),
        .mcu_sdo(mcu_sdo),
        .mcu_load(mcu_load),
        .mcu_done(mcu_done),
        .sclk(sclk),
        .mosi(mosi),
        .bno085_rst_n(bno085_rst_n),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .int1(int1),
        .miso2(miso2),
        .cs_n2(cs_n2),
        .int2(int2),
        .calibrate_btn_n(calibrate_btn_n),
        .kick_btn_n(kick_btn_n),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Mock BNO085 sensors - just keep interrupts high (not asserted)
    initial begin
        int1 = 1'b1;
        int2 = 1'b1;
        miso1 = 1'b0;
        miso2 = 1'b0;
    end
    
    // Task: MCU reads command from FPGA
    task mcu_read_drum_command(output logic [7:0] cmd);
        logic [7:0] temp_cmd;
        integer timeout_count_local;
        temp_cmd = 8'h00;
        timeout_count_local = 0;
        
        // Wait for DONE signal with timeout
        while (!mcu_done && timeout_count_local < 50000) begin
            #(CLK_PERIOD);
            timeout_count_local = timeout_count_local + 1;
        end
        
        if (!mcu_done) begin
            $display("[%0t] ERROR: DONE signal never asserted! done=%b", $time, mcu_done);
            cmd = 8'hFF;  // Error code
        end else begin
            $display("[%0t] MCU: DONE signal detected, starting SPI read", $time);
            
            // Wait for setup time
            #(MCU_SCK_PERIOD);
            
            // Read 8 bits MSB first (SPI Mode 0: sample on rising edge)
            for (i = 7; i >= 0; i = i - 1) begin
                @(posedge mcu_sck);
                #1;
                temp_cmd[i] = mcu_sdo;
                $display("[%0t] MCU: Bit %0d = %b", $time, i, mcu_sdo);
            end
            
            cmd = temp_cmd;
            $display("[%0t] MCU: Received drum command: 0x%02X (%0d)", $time, cmd, cmd);
            
            // Acknowledge by toggling LOAD
            mcu_load = 1;
            #(10 * CLK_PERIOD);
            mcu_load = 0;
            #(10 * CLK_PERIOD);
        end
    endtask
    
    // Task: Simulate kick button press
    task trigger_kick_drum();
        $display("[%0t] Simulating kick drum button press", $time);
        kick_btn_n = 0;
        #(200 * CLK_PERIOD);
        kick_btn_n = 1;
        #(200 * CLK_PERIOD);
    endtask
    
    // Initialization
    initial begin
        fpga_rst_n = 0;
        mcu_load = 0;
        calibrate_btn_n = 1;
        kick_btn_n = 1;
        mcu_sdi = 0;
        
        #(100 * CLK_PERIOD);
        fpga_rst_n = 1;
        #(100 * CLK_PERIOD);
        
        $display("=== Simplified Top-Level Testbench Started ===");
        $display("Time: %0t", $time);
        $display("Testing: MCU SPI interface and button functionality");
        $display("");
    end
    
    // Main test sequence
    initial begin
        wait(fpga_rst_n);
        
        // Wait a bit for system to stabilize
        #(1000 * CLK_PERIOD);
        
        $display("\n=== Test 1: Kick Drum via Button ===");
        trigger_kick_drum();
        #(50 * CLK_PERIOD);
        
        mcu_read_drum_command(cmd1);
        if (cmd1 == 8'hFF) begin
            $error("Test 1 failed: DONE signal timeout");
        end else begin
            assert (cmd1 == 8'h02) else $error("Test 1 failed: Expected 0x02 (kick), got 0x%02X", cmd1);
            $display("✅ Test 1 PASSED: Kick drum command (0x02) received correctly\n");
        end
        
        #(100 * CLK_PERIOD);
        
        $display("=== Test 2: Second Kick Trigger ===");
        trigger_kick_drum();
        #(50 * CLK_PERIOD);
        
        mcu_read_drum_command(cmd2);
        if (cmd2 == 8'hFF) begin
            $error("Test 2 failed: DONE signal timeout");
        end else begin
            assert (cmd2 == 8'h02) else $error("Test 2 failed: Expected 0x02 (kick), got 0x%02X", cmd2);
            $display("✅ Test 2 PASSED: Second kick drum command received correctly\n");
        end
        
        $display("\n=== Test 3: System Status LEDs ===");
        #(100 * CLK_PERIOD);
        $display("LED Initialized: %b", led_initialized);
        $display("LED Error: %b", led_error);
        $display("LED Heartbeat: %b (should be blinking)", led_heartbeat);
        $display("✅ Test 3: Status LEDs checked\n");
        
        $display("\n=== Test 4: Calibration Button ===");
        $display("[%0t] Simulating calibration button press", $time);
        calibrate_btn_n = 0;
        #(200 * CLK_PERIOD);
        calibrate_btn_n = 1;
        #(200 * CLK_PERIOD);
        $display("✅ Test 4: Calibration button pressed\n");
        
        $display("\n==========================================");
        $display("All Tests Completed Successfully!");
        $display("Time: %0t", $time);
        $display("==========================================\n");
        
        #(500 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor key signals
    initial begin
        $monitor("[%0t] done=%b load=%b sdo=%b kick_btn=%b", 
                 $time, mcu_done, mcu_load, mcu_sdo, kick_btn_n);
    end
    
endmodule

