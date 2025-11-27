`timescale 1ns / 1ps

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
    
    always #166.67 CLKHF = ~CLKHF; // ~3MHz

endmodule

