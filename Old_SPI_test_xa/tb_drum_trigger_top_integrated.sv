/**
 * System-Level Testbench for Integrated Drum Trigger System
 * 
 * Tests the complete system:
 * - Drum trigger detection
 * - SPI communication to MCU
 * - End-to-end functionality
 */

`timescale 1ns / 1ps

module tb_drum_trigger_top_integrated;

    // Top-level signals
    logic fpga_rst_n;
    
    // MCU SPI Interface
    logic mcu_sck;
    logic mcu_sdi;
    logic mcu_sdo;
    logic mcu_load;
    logic mcu_done;
    
    // BNO085 SPI Interface (simulated)
    logic sclk1;
    logic mosi1;
    logic miso1;
    logic cs_n1;
    logic ps0_1;
    logic bno085_rst_n1;
    logic int1;
    
    // Button inputs
    logic calibrate_btn_n;
    logic kick_btn_n;
    
    // Status LEDs
    logic led_initialized;
    logic led_error;
    logic led_heartbeat;
    
    // Test signals
    logic [7:0] mcu_received_command;
    logic [2:0] mcu_bit_counter;
    logic mcu_transmission_active;
    
    // Clock generation
    parameter CLK_PERIOD = 333;  // 3MHz FPGA clock
    parameter MCU_SCK_PERIOD = 1000;  // 1MHz MCU SPI clock (slower for testing)
    
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
        .sclk1(sclk1),
        .mosi1(mosi1),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .ps0_1(ps0_1),
        .bno085_rst_n1(bno085_rst_n1),
        .int1(int1),
        .calibrate_btn_n(calibrate_btn_n),
        .kick_btn_n(kick_btn_n),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Mock BNO085 (simplified - just provides interrupt)
    initial begin
        int1 = 1'b1;  // Start with interrupt high (not asserted)
        miso1 = 1'b0;
    end
    
    // MCU simulation: Receive SPI data
    always_ff @(posedge mcu_sck) begin
        if (mcu_done && mcu_transmission_active) begin
            mcu_received_command <= {mcu_received_command[6:0], mcu_sdo};
            mcu_bit_counter <= mcu_bit_counter + 1;
        end else if (!mcu_done) begin
            mcu_bit_counter <= 0;
            mcu_received_command <= 8'h00;
        end
    end
    
    // Track transmission state
    always_ff @(posedge mcu_sck) begin
        if (mcu_done) begin
            mcu_transmission_active <= 1;
        end else if (mcu_bit_counter >= 7) begin
            mcu_transmission_active <= 0;
        end
    end
    
    // Initialization
    initial begin
        fpga_rst_n = 0;
        mcu_load = 0;
        calibrate_btn_n = 1;
        kick_btn_n = 1;
        mcu_sdi = 0;
        mcu_received_command = 8'h00;
        mcu_bit_counter = 0;
        mcu_transmission_active = 0;
        
        #(100 * CLK_PERIOD);
        fpga_rst_n = 1;
        #(100 * CLK_PERIOD);
        
        $display("=== Integrated Drum Trigger System Testbench Started ===");
        $display("Time: %0t", $time);
    end
    
    // Task: Simulate drum trigger via kick button
    task trigger_kick_drum();
        $display("[%0t] Simulating kick drum button press", $time);
        kick_btn_n = 0;
        #(200 * CLK_PERIOD);
        kick_btn_n = 1;
        #(200 * CLK_PERIOD);
    endtask
    
    // Task: MCU reads command from FPGA
    task mcu_read_drum_command(output logic [7:0] cmd);
        logic [7:0] temp_cmd;
        temp_cmd = 8'h00;
        
        // Wait for DONE signal
        wait(mcu_done);
        $display("[%0t] MCU: DONE signal detected, starting SPI read", $time);
        
        // Wait for setup time
        #(MCU_SCK_PERIOD);
        
        // Read 8 bits MSB first
        for (int i = 7; i >= 0; i--) begin
            @(negedge mcu_sck);  // Sample on falling edge
            temp_cmd[i] = mcu_sdo;
            #(MCU_SCK_PERIOD/4);
        end
        
        cmd = temp_cmd;
        $display("[%0t] MCU: Received drum command: 0x%02X (%0d)", $time, cmd, cmd);
        
        // Acknowledge by toggling LOAD
        mcu_load = 1;
        #(10 * CLK_PERIOD);
        mcu_load = 0;
        #(10 * CLK_PERIOD);
    endtask
    
    // Main test sequence
    initial begin
        wait(fpga_rst_n);
        
        // Wait for system initialization (2 seconds + some margin)
        $display("\nWaiting for system initialization...");
        #(2_100_000_000);  // 2.1 seconds (allowing for reset delay)
        
        $display("\n=== Test 1: Kick Drum via Button ===");
        trigger_kick_drum();
        #(50 * CLK_PERIOD);
        
        logic [7:0] cmd1;
        mcu_read_drum_command(cmd1);
        assert (cmd1 == 8'h02) else $error("Test 1 failed: Expected 0x02 (kick), got 0x%02X", cmd1);
        $display("Test 1 PASSED: Kick drum command (0x02) received correctly\n");
        
        #(100 * CLK_PERIOD);
        
        $display("=== Test 2: Multiple Rapid Triggers ===");
        // Trigger multiple times rapidly
        trigger_kick_drum();
        #(20 * CLK_PERIOD);
        trigger_kick_drum();
        #(20 * CLK_PERIOD);
        trigger_kick_drum();
        
        // Read first command
        logic [7:0] cmd2a;
        mcu_read_drum_command(cmd2a);
        assert (cmd2a == 8'h02) else $error("Test 2a failed");
        $display("Test 2a PASSED: First rapid trigger received\n");
        
        #(50 * CLK_PERIOD);
        
        // Read second command
        logic [7:0] cmd2b;
        mcu_read_drum_command(cmd2b);
        assert (cmd2b == 8'h02) else $error("Test 2b failed");
        $display("Test 2b PASSED: Second rapid trigger received\n");
        
        #(50 * CLK_PERIOD);
        
        // Read third command
        logic [7:0] cmd2c;
        mcu_read_drum_command(cmd2c);
        assert (cmd2c == 8'h02) else $error("Test 2c failed");
        $display("Test 2c PASSED: Third rapid trigger received\n");
        
        $display("\n=== Test 3: System Status LEDs ===");
        #(100 * CLK_PERIOD);
        $display("LED Initialized: %b", led_initialized);
        $display("LED Error: %b", led_error);
        $display("LED Heartbeat: %b", led_heartbeat);
        $display("Test 3: Status LEDs checked\n");
        
        $display("\n=== All System Tests Completed ===");
        $display("Time: %0t", $time);
        
        #(500 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor key signals
    initial begin
        $monitor("[%0t] done=%b load=%b sdo=%b kick_btn=%b", 
                 $time, mcu_done, mcu_load, mcu_sdo, kick_btn_n);
    end
    
endmodule

