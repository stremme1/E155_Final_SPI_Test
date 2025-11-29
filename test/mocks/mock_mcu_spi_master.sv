`timescale 1ns / 1ps

// Mock MCU SPI Master
// Simulates MCU SPI master behavior for FPGA tests
// SPI Mode 0 (CPOL=0, CPHA=0)

module mock_mcu_spi_master(
    input  logic        clk,
    output logic        sck,      // SPI clock (MCU generates)
    output logic        sdi,      // SPI data in (not used for reading)
    input  logic        sdo,      // SPI data out (from FPGA)
    output logic        load,     // Load signal
    input  logic        done      // Done signal from FPGA
);

    // SPI clock generation (1MHz for testing)
    parameter SCK_PERIOD = 1000;
    initial begin
        sck = 0;
    end
    
    // Task: Read 32-byte packet from FPGA
    task read_packet(output logic [7:0] packet [0:31]);
        integer i, j;
        begin
            // Wait for DONE signal
            while (!done) begin
                #100;
            end
            $display("[%0t] Mock MCU: DONE detected, reading packet", $time);
            
            // Read 32 bytes (256 bits total)
            for (i = 0; i < 32; i = i + 1) begin
                packet[i] = 8'h00;
                
                // Read 8 bits MSB first (SPI Mode 0)
                for (j = 7; j >= 0; j = j - 1) begin
                    #(SCK_PERIOD/4);
                    sck = 1;  // Rising edge - sample data
                    packet[i][j] = sdo;
                    #(SCK_PERIOD/4);
                    sck = 0;  // Falling edge
                    #(SCK_PERIOD/2);
                end
            end
            
            $display("[%0t] Mock MCU: Packet read complete", $time);
            
            // Acknowledge with LOAD
            load = 1;
            #1000;
            load = 0;
            #1000;
        end
    endtask
    
    // Initialize
    initial begin
        sck = 0;
        sdi = 0;
        load = 0;
    end
    
endmodule

