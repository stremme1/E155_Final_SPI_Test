`timescale 1ns / 1ps

// Testbench for Drum Selector Module
// Tests exact C code drum selection logic

module tb_drum_selector;

    logic clk, rst_n;
    logic valid_in;
    logic [2:0] zone_id;
    logic signed [31:0] pitch;  // Q16.15
    logic signed [15:0] gyro_z;
    logic is_left_hand;
    logic valid_out;
    logic [3:0] drum_code;
    
    // Zone IDs
    localparam [2:0] ZONE_SNARE = 3'd0;
    localparam [2:0] ZONE_HIGH_TOM = 3'd1;
    localparam [2:0] ZONE_MID_TOM = 3'd2;
    localparam [2:0] ZONE_FLOOR_TOM = 3'd3;
    
    // Drum codes
    localparam [3:0] CODE_SNARE = 4'd0;
    localparam [3:0] CODE_HI_HAT = 4'd1;
    localparam [3:0] CODE_HIGH_TOM = 4'd3;
    localparam [3:0] CODE_MID_TOM = 4'd4;
    localparam [3:0] CODE_CRASH = 4'd5;
    localparam [3:0] CODE_RIDE = 4'd6;
    localparam [3:0] CODE_FLOOR_TOM = 4'd7;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #166.67 clk = ~clk;
    end
    
    // DUT
    drum_selector dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .zone_id(zone_id),
        .pitch(pitch),
        .gyro_z(gyro_z),
        .is_left_hand(is_left_hand),
        .valid_out(valid_out),
        .drum_code(drum_code)
    );
    
    // Helper function
    function [31:0] deg_to_q1615(input [31:0] deg);
        deg_to_q1615 = deg * 32'd32768;
    endfunction
    
    initial begin
        $display("=== Drum Selector Testbench ===");
        
        // Initialize
        rst_n = 0;
        valid_in = 0;
        zone_id = 0;
        pitch = 0;
        gyro_z = 0;
        is_left_hand = 0;
        #1000;
        rst_n = 1;
        #1000;
        
        // RIGHT HAND TESTS (EXACT from C code)
        $display("\n=== RIGHT HAND TESTS ===");
        is_left_hand = 0;
        valid_in = 1;
        
        // Test 1: Snare zone - always snare
        $display("\nTest 1: Right hand snare zone -> snare");
        zone_id = ZONE_SNARE;
        pitch = deg_to_q1615(0);
        #500;
        assert(drum_code == CODE_SNARE) else $error("Test 1 failed");
        $display("PASS: Snare zone -> snare");
        
        // Test 2: High tom zone, pitch <= 50 -> high tom
        $display("\nTest 2: Right hand high tom, pitch=40 -> high tom");
        zone_id = ZONE_HIGH_TOM;
        pitch = deg_to_q1615(40);
        #500;
        assert(drum_code == CODE_HIGH_TOM) else $error("Test 2 failed");
        $display("PASS: High tom zone, low pitch -> high tom");
        
        // Test 3: High tom zone, pitch > 50 -> crash
        $display("\nTest 3: Right hand high tom, pitch=55 -> crash");
        zone_id = ZONE_HIGH_TOM;
        pitch = deg_to_q1615(55);
        #500;
        assert(drum_code == CODE_CRASH) else $error("Test 3 failed");
        $display("PASS: High tom zone, high pitch -> crash");
        
        // Test 4: Mid tom zone, pitch <= 50 -> mid tom
        $display("\nTest 4: Right hand mid tom, pitch=40 -> mid tom");
        zone_id = ZONE_MID_TOM;
        pitch = deg_to_q1615(40);
        #500;
        assert(drum_code == CODE_MID_TOM) else $error("Test 4 failed");
        $display("PASS: Mid tom zone, low pitch -> mid tom");
        
        // Test 5: Mid tom zone, pitch > 50 -> ride
        $display("\nTest 5: Right hand mid tom, pitch=55 -> ride");
        zone_id = ZONE_MID_TOM;
        pitch = deg_to_q1615(55);
        #500;
        assert(drum_code == CODE_RIDE) else $error("Test 5 failed");
        $display("PASS: Mid tom zone, high pitch -> ride");
        
        // Test 6: Floor tom zone, pitch <= 30 -> floor tom
        $display("\nTest 6: Right hand floor tom, pitch=20 -> floor tom");
        zone_id = ZONE_FLOOR_TOM;
        pitch = deg_to_q1615(20);
        #500;
        assert(drum_code == CODE_FLOOR_TOM) else $error("Test 6 failed");
        $display("PASS: Floor tom zone, low pitch -> floor tom");
        
        // Test 7: Floor tom zone, pitch > 30 -> ride
        $display("\nTest 7: Right hand floor tom, pitch=35 -> ride");
        zone_id = ZONE_FLOOR_TOM;
        pitch = deg_to_q1615(35);
        #500;
        assert(drum_code == CODE_RIDE) else $error("Test 7 failed");
        $display("PASS: Floor tom zone, high pitch -> ride");
        
        // LEFT HAND TESTS (EXACT from C code)
        $display("\n=== LEFT HAND TESTS ===");
        is_left_hand = 1;
        
        // Test 8: Snare zone, pitch <= 30 -> snare
        $display("\nTest 8: Left hand snare zone, pitch=20 -> snare");
        zone_id = ZONE_SNARE;
        pitch = deg_to_q1615(20);
        gyro_z = -16'sd3000;  // Below threshold
        #500;
        assert(drum_code == CODE_SNARE) else $error("Test 8 failed");
        $display("PASS: Snare zone, low pitch -> snare");
        
        // Test 9: Snare zone, pitch > 30, gyro_z <= -2000 -> snare
        $display("\nTest 9: Left hand snare zone, pitch=35, gyro_z=-2500 -> snare");
        zone_id = ZONE_SNARE;
        pitch = deg_to_q1615(35);
        gyro_z = -16'sd2500;  // At or below threshold
        #500;
        assert(drum_code == CODE_SNARE) else $error("Test 9 failed");
        $display("PASS: Snare zone, high pitch but low gyro_z -> snare");
        
        // Test 10: Snare zone, pitch > 30, gyro_z > -2000 -> hi-hat
        $display("\nTest 10: Left hand snare zone, pitch=35, gyro_z=-1500 -> hi-hat");
        zone_id = ZONE_SNARE;
        pitch = deg_to_q1615(35);
        gyro_z = -16'sd1500;  // Above threshold
        #500;
        assert(drum_code == CODE_HI_HAT) else $error("Test 10 failed");
        $display("PASS: Snare zone, high pitch and high gyro_z -> hi-hat");
        
        // Test 11: High tom zone, pitch <= 50 -> high tom
        $display("\nTest 11: Left hand high tom, pitch=40 -> high tom");
        zone_id = ZONE_HIGH_TOM;
        pitch = deg_to_q1615(40);
        #500;
        assert(drum_code == CODE_HIGH_TOM) else $error("Test 11 failed");
        $display("PASS: High tom zone, low pitch -> high tom");
        
        // Test 12: High tom zone, pitch > 50 -> crash
        $display("\nTest 12: Left hand high tom, pitch=55 -> crash");
        zone_id = ZONE_HIGH_TOM;
        pitch = deg_to_q1615(55);
        #500;
        assert(drum_code == CODE_CRASH) else $error("Test 12 failed");
        $display("PASS: High tom zone, high pitch -> crash");
        
        // Test 13: Mid tom zone, pitch <= 50 -> mid tom
        $display("\nTest 13: Left hand mid tom, pitch=40 -> mid tom");
        zone_id = ZONE_MID_TOM;
        pitch = deg_to_q1615(40);
        #500;
        assert(drum_code == CODE_MID_TOM) else $error("Test 13 failed");
        $display("PASS: Mid tom zone, low pitch -> mid tom");
        
        // Test 14: Mid tom zone, pitch > 50 -> ride
        $display("\nTest 14: Left hand mid tom, pitch=55 -> ride");
        zone_id = ZONE_MID_TOM;
        pitch = deg_to_q1615(55);
        #500;
        assert(drum_code == CODE_RIDE) else $error("Test 14 failed");
        $display("PASS: Mid tom zone, high pitch -> ride");
        
        // Test 15: Floor tom zone, pitch <= 30 -> floor tom
        $display("\nTest 15: Left hand floor tom, pitch=20 -> floor tom");
        zone_id = ZONE_FLOOR_TOM;
        pitch = deg_to_q1615(20);
        #500;
        assert(drum_code == CODE_FLOOR_TOM) else $error("Test 15 failed");
        $display("PASS: Floor tom zone, low pitch -> floor tom");
        
        // Test 16: Floor tom zone, pitch > 30 -> ride
        $display("\nTest 16: Left hand floor tom, pitch=35 -> ride");
        zone_id = ZONE_FLOOR_TOM;
        pitch = deg_to_q1615(35);
        #500;
        assert(drum_code == CODE_RIDE) else $error("Test 16 failed");
        $display("PASS: Floor tom zone, high pitch -> ride");
        
        $display("\n=== All Drum Selector Tests PASSED ===");
        #1000;
        $finish;
    end

endmodule

