`timescale 1ns / 1ps

// Mock HSOSC module for simulation
// In synthesis, this will be replaced by the actual Lattice HSOSC primitive

module HSOSC #(
    parameter CLKHF_DIV = 2'b00
) (
    input wire CLKHFPU,
    input wire CLKHFEN,
    output reg CLKHF
);

    initial begin
        CLKHF = 0;
        $display("[HSOSC] Clock started");
    end
    
    // Generate ~3MHz clock (333ns period)
    always #166.67 CLKHF = ~CLKHF;

endmodule

