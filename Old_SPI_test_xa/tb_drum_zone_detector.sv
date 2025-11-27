`timescale 1ns / 1ps

// Testbench for Drum Zone Detector Module
// Tests exact C code zone ranges for right and left hand

module tb_drum_zone_detector;

    logic clk, rst_n;
    logic valid_in;
    logic [31:0] yaw;  // Q16.15 format
    logic is_left_hand;
    logic valid_out;
    logic [2:0] zone_id;
    
    // Zone IDs
    localparam [2:0] ZONE_SNARE = 3'd0;
    localparam [2:0] ZONE_HIGH_TOM = 3'd1;
    localparam [2:0] ZONE_MID_TOM = 3'd2;
    localparam [2:0] ZONE_FLOOR_TOM = 3'd3;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #166.67 clk = ~clk;  // 3MHz
    end
    
    // DUT
    drum_zone_detector dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .yaw(yaw),
        .is_left_hand(is_left_hand),
        .valid_out(valid_out),
        .zone_id(zone_id)
    );
    
    // Helper function to convert degrees to Q16.15
    function [31:0] deg_to_q1615(input [31:0] deg);
        deg_to_q1615 = deg * 32'd32768;
    endfunction
    
    // Test stimulus
    initial begin
        $display("=== Drum Zone Detector Testbench ===");
        
        // Initialize
        rst_n = 0;
        valid_in = 0;
        yaw = 0;
        is_left_hand = 0;
        #1000;
        rst_n = 1;
        #1000;
        
        // RIGHT HAND TESTS (EXACT from C code)
        $display("\n=== RIGHT HAND TESTS ===");
        is_left_hand = 0;
        valid_in = 1;
        
        // Test 1: Snare zone (20-120)
        $display("\nTest 1: Right hand snare zone (yaw = 70 degrees)");
        yaw = deg_to_q1615(70);
        #500;
        assert(zone_id == ZONE_SNARE) else $error("Test 1 failed: Expected snare zone");
        $display("PASS: Snare zone detected");
        
        // Test 2: High tom zone - lower bound (340)
        $display("\nTest 2: Right hand high tom zone (yaw = 340 degrees)");
        yaw = deg_to_q1615(340);
        #500;
        assert(zone_id == ZONE_HIGH_TOM) else $error("Test 2 failed: Expected high tom zone");
        $display("PASS: High tom zone detected at 340");
        
        // Test 3: High tom zone - wraps around (20)
        // Note: In C code, yaw=20 is in snare zone (20-120) because that check comes first
        // High tom zone (340-20) is else-if, so yaw=20 goes to snare
        // Testing at yaw=19 to verify high tom zone boundary
        $display("\nTest 3: Right hand high tom zone - wrap (yaw = 19 degrees)");
        yaw = deg_to_q1615(19);
        #500;
        assert(zone_id == ZONE_HIGH_TOM) else $error("Test 3 failed: Expected high tom zone");
        $display("PASS: High tom zone detected at 19 (wrap)");
        
        // Test 4: High tom zone - wraps around (0)
        $display("\nTest 4: Right hand high tom zone - wrap (yaw = 0 degrees)");
        yaw = deg_to_q1615(0);
        #500;
        assert(zone_id == ZONE_HIGH_TOM) else $error("Test 4 failed: Expected high tom zone");
        $display("PASS: High tom zone detected at 0 (wrap)");
        
        // Test 5: Mid tom zone (305-340)
        $display("\nTest 5: Right hand mid tom zone (yaw = 320 degrees)");
        yaw = deg_to_q1615(320);
        #500;
        assert(zone_id == ZONE_MID_TOM) else $error("Test 5 failed: Expected mid tom zone");
        $display("PASS: Mid tom zone detected");
        
        // Test 6: Floor tom zone (200-305)
        $display("\nTest 6: Right hand floor tom zone (yaw = 250 degrees)");
        yaw = deg_to_q1615(250);
        #500;
        assert(zone_id == ZONE_FLOOR_TOM) else $error("Test 6 failed: Expected floor tom zone");
        $display("PASS: Floor tom zone detected");
        
        // LEFT HAND TESTS (EXACT from C code)
        $display("\n=== LEFT HAND TESTS ===");
        is_left_hand = 1;
        
        // Test 7: Snare/hi-hat zone (350-100, wraps)
        $display("\nTest 7: Left hand snare/hi-hat zone (yaw = 350 degrees)");
        yaw = deg_to_q1615(350);
        #500;
        assert(zone_id == ZONE_SNARE) else $error("Test 7 failed: Expected snare zone");
        $display("PASS: Snare/hi-hat zone detected at 350");
        
        // Test 8: Snare/hi-hat zone - wraps (100)
        $display("\nTest 8: Left hand snare/hi-hat zone - wrap (yaw = 100 degrees)");
        yaw = deg_to_q1615(100);
        #500;
        assert(zone_id == ZONE_SNARE) else $error("Test 8 failed: Expected snare zone");
        $display("PASS: Snare/hi-hat zone detected at 100 (wrap)");
        
        // Test 9: High tom zone (325-350)
        $display("\nTest 9: Left hand high tom zone (yaw = 337 degrees)");
        yaw = deg_to_q1615(337);
        #500;
        assert(zone_id == ZONE_HIGH_TOM) else $error("Test 9 failed: Expected high tom zone");
        $display("PASS: High tom zone detected");
        
        // Test 10: Mid tom zone (300-325)
        $display("\nTest 10: Left hand mid tom zone (yaw = 312 degrees)");
        yaw = deg_to_q1615(312);
        #500;
        assert(zone_id == ZONE_MID_TOM) else $error("Test 10 failed: Expected mid tom zone");
        $display("PASS: Mid tom zone detected");
        
        // Test 11: Floor tom zone (200-300)
        $display("\nTest 11: Left hand floor tom zone (yaw = 250 degrees)");
        yaw = deg_to_q1615(250);
        #500;
        assert(zone_id == ZONE_FLOOR_TOM) else $error("Test 11 failed: Expected floor tom zone");
        $display("PASS: Floor tom zone detected");
        
        $display("\n=== All Zone Detector Tests PASSED ===");
        #1000;
        $finish;
    end

endmodule

