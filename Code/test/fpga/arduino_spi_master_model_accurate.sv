`timescale 1ns / 1ps

// Accurate Arduino SPI Master Model
// Simulates EXACT behavior of Arduino SPI.transfer() in SPI Mode 0, MSB First
// Based on actual Arduino SPI hardware behavior
// 
// SPI Mode 0 (CPOL=0, CPHA=0):
// - Clock idle LOW
// - Data is sampled on RISING edge of SCK
// - Data must be stable BEFORE rising edge
// - First bit is set up BEFORE first SCK rising edge
//
// MSB First:
// - Most significant bit (bit 7) is sent first
// - Least significant bit (bit 0) is sent last

module arduino_spi_master_model_accurate(
    output reg cs_n,      // Chip select (active low)
    output reg sck,       // SPI clock
    output reg sdi,        // SPI data (MOSI)
    
    // Sensor data inputs (can be set from testbench)
    input  logic signed [15:0] roll,      // Euler angle scaled by 100
    input  logic signed [15:0] pitch,     // Euler angle scaled by 100
    input  logic signed [15:0] yaw,       // Euler angle scaled by 100
    input  logic signed [15:0] gyro_x,   // Gyro scaled by 2000
    input  logic signed [15:0] gyro_y,   // Gyro scaled by 2000
    input  logic signed [15:0] gyro_z,   // Gyro scaled by 2000
    input  logic euler_valid,             // Euler data valid flag
    input  logic gyro_valid,              // Gyro data valid flag
    
    // Control
    input  logic send_packet,             // Trigger to send packet
    output reg packet_sent                // High when packet transmission complete
);

    // SPI parameters (matching Arduino code exactly)
    parameter SCK_PERIOD = 10000;  // 100kHz = 10us period
    parameter HEADER_BYTE = 8'hAA;
    
    // Function to get packet byte (exact Arduino format)
    function [7:0] get_packet_byte(input [3:0] idx);
        case (idx)
            4'd0:  get_packet_byte = HEADER_BYTE;
            4'd1:  get_packet_byte = roll[15:8];
            4'd2:  get_packet_byte = roll[7:0];
            4'd3:  get_packet_byte = pitch[15:8];
            4'd4:  get_packet_byte = pitch[7:0];
            4'd5:  get_packet_byte = yaw[15:8];
            4'd6:  get_packet_byte = yaw[7:0];
            4'd7:  get_packet_byte = gyro_x[15:8];
            4'd8:  get_packet_byte = gyro_x[7:0];
            4'd9:  get_packet_byte = gyro_y[15:8];
            4'd10: get_packet_byte = gyro_y[7:0];
            4'd11: get_packet_byte = gyro_z[15:8];
            4'd12: get_packet_byte = gyro_z[7:0];
            4'd13: get_packet_byte = {6'h0, gyro_valid, euler_valid};
            4'd14: get_packet_byte = 8'h00;
            4'd15: get_packet_byte = 8'h00;
            default: get_packet_byte = 8'h00;
        endcase
    endfunction
    
    // Initialize
    initial begin
        cs_n = 1'b1;
        sck = 1'b0;  // SPI Mode 0: idle low
        sdi = 1'b0;
        packet_sent = 1'b0;
    end
    
    // Accurate SPI.transfer() simulation
    // This matches Arduino SPI.transfer() behavior exactly:
    // 1. First bit is set up BEFORE first SCK rising edge
    // 2. Each subsequent bit is set up before its SCK rising edge
    // 3. Data is stable for setup time before rising edge
    task send_spi_byte_accurate(input [7:0] data);
        integer i;
        // Set up first bit (MSB, bit 7) BEFORE first SCK rising edge
        sdi = data[7];
        #(SCK_PERIOD / 2);  // Setup time - data stable before first edge
        
        // Send 8 bits: 7 rising edges (bits 7-1) + 1 final edge (bit 0)
        for (i = 7; i >= 0; i = i - 1) begin
            // Rising edge - slave samples sdi here
            sck = 1'b1;
            #(SCK_PERIOD / 2);
            
            // Falling edge
            sck = 1'b0;
            
            // Set up next bit (if not last bit) before next rising edge
            if (i > 0) begin
                sdi = data[i-1];  // Next bit (MSB first: 7,6,5,4,3,2,1,0)
                #(SCK_PERIOD / 2);  // Setup time for next bit
            end else begin
                // Last bit sent - keep data stable
                #(SCK_PERIOD / 2);
            end
        end
    endtask
    
    // SPI transmission - matches Arduino SPI.beginTransaction() and SPI.transfer() exactly
    task send_spi_packet_accurate();
        // SPI.beginTransaction() - CS is already high from initialization
        // digitalWrite(FPGA_SPI_CS, LOW) - Pull CS low
        cs_n = 1'b0;
        sck = 1'b0;  // Ensure clock is idle low
        sdi = 1'b0;
        #(SCK_PERIOD * 2);  // Wait for CS to stabilize low
        
        // Send all 16 bytes using SPI.transfer()
        for (int i = 0; i < 16; i = i + 1) begin
            logic [7:0] byte_to_send = get_packet_byte(i);
            send_spi_byte_accurate(byte_to_send);
        end
        
        // digitalWrite(FPGA_SPI_CS, HIGH) - Pull CS high
        // Wait a bit before pulling CS high (Arduino does this)
        #(SCK_PERIOD / 2);
        cs_n = 1'b1;
        sck = 1'b0;  // Ensure clock is idle low
        sdi = 1'b0;
        
        // SPI.endTransaction() - wait for transaction to complete
        #(SCK_PERIOD * 3);  // Wait for CS to stabilize high
        
        packet_sent = 1'b1;
        #(SCK_PERIOD);
        packet_sent = 1'b0;
    endtask
    
    // Trigger transmission on send_packet
    always @(posedge send_packet) begin
        send_spi_packet_accurate();
    end

endmodule

