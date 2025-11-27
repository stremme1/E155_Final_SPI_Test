/**
 * Signal Path Verification Testbench
 * 
 * Tests the critical signal path without the long reset delay:
 * Button -> Debouncer -> drum_trigger_valid -> drum_spi_slave -> done
 * 
 * This bypasses the 6M cycle reset delay to verify the actual logic works
 */

`timescale 1ns / 1ps

module tb_signal_path_verification;

    // Clock and reset
    logic clk, rst_n;
    
    // Button inputs
    logic kick_btn_n;
    logic kick_btn_pulse;
    
    // Drum trigger signals
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    
    // SPI slave signals
    logic sck, sdi, sdo, load, done;
    logic command_sent;
    
    // Test variables
    logic [7:0] received_cmd;
    integer i;
    integer test_passed = 0;
    integer test_failed = 0;
    
    // Clock generation (3MHz)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // SPI clock - only run when needed (not continuously)
    parameter SCK_PERIOD = 1000;
    logic sck_enable = 0;
    
    // Generate SPI clock only when enabled
    always begin
        if (sck_enable) begin
            sck = 0;
            #(SCK_PERIOD/2);
            sck = 1;
            #(SCK_PERIOD/2);
        end else begin
            sck = 0;
            #(SCK_PERIOD);
        end
    end
    
    // Button debouncer (with reduced debounce for faster testing)
    button_debouncer #(.DEBOUNCE_CYCLES(100)) kick_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(kick_btn_n),
        .btn_pressed(kick_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    // Drum trigger processor (simplified - just button logic)
    // Hold trigger valid for multiple cycles to ensure SPI slave latches it
    logic [2:0] trigger_hold_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_trigger_valid <= 1'b0;
            drum_code <= 4'd0;
            trigger_hold_counter <= 3'd0;
        end else begin
            if (kick_btn_pulse) begin
                drum_trigger_valid <= 1'b1;
                drum_code <= 4'd2;  // Kick drum
                trigger_hold_counter <= 3'd5;  // Hold for 5 cycles
            end else if (trigger_hold_counter > 0) begin
                // Keep valid high while counter is counting down
                trigger_hold_counter <= trigger_hold_counter - 1;
            end else if (command_sent) begin
                // Clear when SPI slave acknowledges
                drum_trigger_valid <= 1'b0;
            end
        end
    end
    
    // SPI slave
    drum_spi_slave spi_slave (
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
    
    // Task: MCU reads command
    task mcu_read_command(output logic [7:0] cmd);
        logic [7:0] temp_cmd;
        integer timeout = 0;
        
        $display("[%0t] Waiting for DONE signal...", $time);
        while (!done && timeout < 10000) begin
            #(CLK_PERIOD);
            timeout = timeout + 1;
        end
        
        if (!done) begin
            $display("[%0t] ERROR: DONE never asserted! done=%b", $time, done);
            cmd = 8'hFF;
        end else begin
            $display("[%0t] DONE asserted! Starting SPI read...", $time);
            
            // Enable SPI clock and wait for it to stabilize
            sck_enable = 1;
            #(SCK_PERIOD);
            
            // Read 8 bits MSB first
            for (i = 7; i >= 0; i = i - 1) begin
                @(posedge sck);
                #1;  // Small delay after edge
                temp_cmd[i] = sdo;
                $display("[%0t] Bit %0d: sdo=%b", $time, i, sdo);
            end
            
            // Disable SPI clock
            sck_enable = 0;
            
            cmd = temp_cmd;
            $display("[%0t] Received: 0x%02X", $time, cmd);
            
            // Acknowledge
            load = 1;
            #(10 * CLK_PERIOD);
            load = 0;
            #(10 * CLK_PERIOD);
        end
    endtask
    
    // Main test
    initial begin
        rst_n = 0;
        kick_btn_n = 1;
        load = 0;
        sdi = 0;
        
        #(100 * CLK_PERIOD);
        rst_n = 1;
        #(1000 * CLK_PERIOD);
        
        $display("=== Signal Path Verification Test ===");
        $display("Testing: Button -> Debouncer -> drum_trigger_valid -> SPI Slave -> done");
        $display("");
        
        // Test 1: Button press should trigger done signal
        $display("=== Test 1: Button Press Triggers DONE ===");
        $display("[%0t] Pressing button...", $time);
        kick_btn_n = 0;
        #(500 * CLK_PERIOD);  // Hold long enough for debouncer
        kick_btn_n = 1;
        #(500 * CLK_PERIOD);
        
        $display("[%0t] Button released, checking signals...", $time);
        $display("  kick_btn_pulse: %b", kick_btn_pulse);
        $display("  drum_trigger_valid: %b", drum_trigger_valid);
        $display("  drum_code: %0d", drum_code);
        $display("  done: %b", done);
        
        if (drum_trigger_valid && drum_code == 4'd2) begin
            $display("✅ drum_trigger_valid asserted correctly");
            test_passed = test_passed + 1;
        end else begin
            $display("❌ drum_trigger_valid not asserted correctly");
            test_failed = test_failed + 1;
        end
        
        // Wait for done signal
        #(1000 * CLK_PERIOD);
        
        if (done) begin
            $display("✅ DONE signal asserted!");
            test_passed = test_passed + 1;
        end else begin
            $display("❌ DONE signal NOT asserted!");
            $display("  drum_trigger_valid: %b", drum_trigger_valid);
            $display("  done: %b", done);
            test_failed = test_failed + 1;
        end
        
        // Test 2: Read command via SPI
        if (done) begin
            $display("");
            $display("=== Test 2: SPI Read ===");
            mcu_read_command(received_cmd);
            
            if (received_cmd == 8'h02) begin
                $display("✅ Correct command received: 0x02 (kick)");
                test_passed = test_passed + 1;
            end else begin
                $display("❌ Wrong command: expected 0x02, got 0x%02X", received_cmd);
                test_failed = test_failed + 1;
            end
        end
        
        // Summary
        $display("");
        $display("==========================================");
        $display("Test Results:");
        $display("  Passed: %0d", test_passed);
        $display("  Failed: %0d", test_failed);
        if (test_failed == 0) begin
            $display("✅ ALL SIGNAL PATH TESTS PASSED!");
            $display("✅ System logic is working correctly!");
        end else begin
            $display("❌ Some signal path tests failed");
        end
        $display("==========================================");
        
        #(1000 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor key signals
    initial begin
        $monitor("[%0t] btn=%b btn_pulse=%b valid=%b code=%0d done=%b", 
                 $time, kick_btn_n, kick_btn_pulse, drum_trigger_valid, drum_code, done);
    end
    
endmodule

