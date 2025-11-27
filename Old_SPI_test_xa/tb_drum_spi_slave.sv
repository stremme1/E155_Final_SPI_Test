/**
 * Testbench for Drum SPI Slave Module
 * 
 * Tests SPI communication protocol and drum command transmission
 * Simulates MCU master behavior
 */

`timescale 1ns / 1ps

module tb_drum_spi_slave;

    // Clock
    logic clk;
    
    // SPI interface
    logic sck;
    logic sdi;
    logic sdo;
    logic load;
    logic done;
    
    // Drum trigger interface
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    logic command_sent;
    
    // Test signals
    logic [7:0] received_command;
    logic [2:0] bit_count;
    logic transmission_complete;
    
    // Test variables (must be declared at module level for iverilog)
    logic [7:0] cmd1, cmd2, cmd3, cmd, cmd5a, cmd5b, cmd5c;
    logic [7:0] temp_cmd;
    integer timeout_count, i;
    
    // SPI clock period constant (for use in tasks)
    localparam SCK_HALF_PERIOD = 500;  // Half period for manual clock control
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;  // 3MHz = 333ns period
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // SPI clock - controlled manually in testbench (like Lab07)
    // Start with clock low
    initial begin
        sck = 0;
    end
    
    // DUT instantiation
    drum_spi_slave dut (
        .clk(clk),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .load(load),
        .done(done),
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .command_sent(command_sent)
    );
    
    // MCU master simulation: Receive SPI data (not used - we read in task instead)
    // The task mcu_read_command handles the actual reading
    
    // Initialization
    initial begin
        load = 0;
        drum_trigger_valid = 0;
        drum_code = 4'h0;
        sdi = 0;
        received_command = 8'h00;
        bit_count = 0;
        
        #(10 * CLK_PERIOD);
        
        $display("=== Drum SPI Slave Testbench Started ===");
        $display("Time: %0t", $time);
    end
    
    // Test task: Simulate drum trigger
    task trigger_drum(input logic [3:0] code);
        $display("[%0t] Triggering drum code: %0d", $time, code);
        drum_code = code;
        drum_trigger_valid = 1;
        #(2 * CLK_PERIOD);
        drum_trigger_valid = 0;
        #(2 * CLK_PERIOD);
    endtask
    
    // Test task: MCU reads command
    task mcu_read_command(output logic [7:0] cmd);
        temp_cmd = 8'h00;
        timeout_count = 0;
        
        // Wait for DONE signal with timeout
        while (!done && timeout_count < 10000) begin
            #(CLK_PERIOD);
            timeout_count = timeout_count + 1;
        end
        
        if (!done) begin
            $display("[%0t] ERROR: DONE signal never asserted! done=%b", $time, done);
            cmd = 8'hFF;  // Error code
        end else begin
            $display("[%0t] MCU: DONE signal detected, starting SPI read", $time);
            
            // Initialize temp_cmd
            temp_cmd = 8'h00;
            
            // Wait a bit for setup - ensure shift register is loaded
            #(2 * CLK_PERIOD);
            
            // Read 8 bits - Following Lab07 testbench pattern exactly
            // Lab07: sck = 1; #1; cyphertext[383-i] = sdo; #4; sck = 0;
            // Read MSB first (bits 7 down to 0)
            for (i = 7; i >= 0; i = i - 1) begin
                // Generate SPI clock (like Lab07 testbench)
                sck = 1'b1;
                #(SCK_HALF_PERIOD/10);  // Small delay (like Lab07's #1)
                temp_cmd[i] = sdo;  // Sample sdo (like Lab07: cyphertext[383-i] = sdo)
                #(SCK_HALF_PERIOD*2);  // Complete clock high period (like Lab07's #4)
                sck = 1'b0;
                #(SCK_HALF_PERIOD);  // Hold low
                $display("[%0t] MCU: Bit %0d = %b", $time, i, sdo);
            end
            
            cmd = temp_cmd;
            $display("[%0t] MCU: Received command: 0x%02X (%0d)", $time, cmd, cmd);
            
            // Acknowledge by pulling LOAD high (then low)
            load = 1;
            #(10 * CLK_PERIOD);
            load = 0;
            #(10 * CLK_PERIOD);
        end
    endtask
    
    // Main test sequence
    initial begin
        #(20 * CLK_PERIOD);
        
        $display("\n=== Test 1: Snare Drum (code 0) ===");
        trigger_drum(4'h0);
        #(10 * CLK_PERIOD);
        
        mcu_read_command(cmd1);
        assert (cmd1 == 8'h00) else $error("Test 1 failed: Expected 0x00, got 0x%02X", cmd1);
        $display("Test 1 PASSED: Snare drum command received correctly\n");
        
        #(20 * CLK_PERIOD);
        
        $display("=== Test 2: Kick Drum (code 2) ===");
        trigger_drum(4'h2);
        #(10 * CLK_PERIOD);
        
        mcu_read_command(cmd2);
        assert (cmd2 == 8'h02) else $error("Test 2 failed: Expected 0x02, got 0x%02X", cmd2);
        $display("Test 2 PASSED: Kick drum command received correctly\n");
        
        #(20 * CLK_PERIOD);
        
        $display("=== Test 3: Crash Cymbal (code 5) ===");
        trigger_drum(4'h5);
        #(10 * CLK_PERIOD);
        
        mcu_read_command(cmd3);
        assert (cmd3 == 8'h05) else $error("Test 3 failed: Expected 0x05, got 0x%02X", cmd3);
        $display("Test 3 PASSED: Crash cymbal command received correctly\n");
        
        #(20 * CLK_PERIOD);
        
        $display("=== Test 4: All Drum Codes (0-7) ===");
        begin : test4_loop
            integer i;
            for (i = 0; i < 8; i = i + 1) begin
                trigger_drum(i[3:0]);
                #(10 * CLK_PERIOD);
                
                mcu_read_command(cmd);
            assert (cmd == {4'h0, i[3:0]}) else 
                $error("Test 4.%0d failed: Expected 0x%02X, got 0x%02X", i, {4'h0, i[3:0]}, cmd);
                $display("Test 4.%0d PASSED: Code %0d received correctly", i, i);
                
                #(20 * CLK_PERIOD);
            end
        end
        $display("Test 4 PASSED: All drum codes transmitted correctly\n");
        
        #(20 * CLK_PERIOD);
        
        $display("=== Test 5: Rapid Triggers ===");
        // Test that rapid triggers update the command (latest trigger wins)
        // Trigger three drums rapidly - only the latest should be captured
        trigger_drum(4'h1);
        #(5 * CLK_PERIOD);
        trigger_drum(4'h3);
        #(5 * CLK_PERIOD);
        trigger_drum(4'h6);  // Latest trigger should overwrite previous ones
        
        // Read command - should get the latest trigger (code 6)
        mcu_read_command(cmd5a);
        assert (cmd5a == 8'h06) else $error("Test 5a failed: Expected 0x06 (latest trigger), got 0x%02X", cmd5a);
        $display("Test 5a PASSED: Latest rapid trigger (code 6) received correctly\n");
        
        #(10 * CLK_PERIOD);
        
        // Trigger another command after acknowledgment
        trigger_drum(4'h2);
        mcu_read_command(cmd5b);
        assert (cmd5b == 8'h02) else $error("Test 5b failed: Expected 0x02, got 0x%02X", cmd5b);
        $display("Test 5b PASSED: Second trigger after acknowledgment received correctly\n");
        
        #(10 * CLK_PERIOD);
        
        // Trigger one more
        trigger_drum(4'h4);
        mcu_read_command(cmd5c);
        assert (cmd5c == 8'h04) else $error("Test 5c failed: Expected 0x04, got 0x%02X", cmd5c);
        $display("Test 5c PASSED: Third trigger after acknowledgment received correctly\n");
        
        $display("\n=== All Tests PASSED ===");
        $display("Time: %0t", $time);
        #(100 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor signals (commented out to reduce output, uncomment for debugging)
    // initial begin
    //     $monitor("[%0t] done=%b load=%b sdo=%b command_sent=%b drum_code=%0d", 
    //              $time, done, load, sdo, command_sent, drum_code);
    // end
    
endmodule

