`timescale 1ns / 1ps

// Minimal Clock Test Module
// This module only tests if HSOSC is working
// Connect led_test to an LED pin to verify clock

module clock_test (
    input  logic        rst_n,
    output logic        led_test,    // Should blink if clock works
    output logic        clk_out      // Direct clock output for scope
);

    logic clk;
    
    // HSOSC Clock Generation
    HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
        .CLKHFPU(1'b1),   // Power up
        .CLKHFEN(1'b1),   // Enable
        .CLKHF(clk)       // Output clock (3MHz)
    );
    
    // Simple counter for LED blink
    logic [21:0] counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 22'd0;
        end else begin
            counter <= counter + 1;
        end
    end
    
    // LED blinks at ~1Hz (bit 21 toggles every ~0.7s at 3MHz)
    assign led_test = counter[21];
    
    // Direct clock output for oscilloscope
    assign clk_out = clk;
    
endmodule

