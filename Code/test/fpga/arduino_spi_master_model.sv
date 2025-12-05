`timescale 1ns / 1ps

// Arduino SPI Master Model
// Simulates the exact behavior of ARDUINO_SENSOR_BRIDGE.ino
// Sends 16-byte packets via SPI Mode 0 at 100kHz
// Packet format: [Header][Roll][Pitch][Yaw][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]

module arduino_spi_master_model(
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

    // SPI parameters (matching Arduino code)
    parameter SCK_PERIOD = 10000;  // 100kHz = 10us period
    parameter HEADER_BYTE = 8'hAA;
    
    // SPI Mode 0: CPOL=0 (clock idle low), CPHA=0 (sample on rising edge)
    // MSB First: Most significant bit sent first
    
    // Function to get packet byte
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
        sck = 1'b0;
        sdi = 1'b0;
        packet_sent = 1'b0;
    end
    
    // SPI transmission task - called from testbench
    task send_spi_packet();
        // Pull CS low
        cs_n = 1'b0;
        sck = 1'b0;
        #(SCK_PERIOD * 2);
        
        // Send all 16 bytes
        for (int i = 0; i < 16; i = i + 1) begin
            logic [7:0] byte_to_send = get_packet_byte(i);
            
            // Send 8 bits (MSB first) - same as original testbench
            for (int j = 7; j >= 0; j = j - 1) begin
                // Set data before rising edge (SPI Mode 0)
                sdi = byte_to_send[j];
                #(SCK_PERIOD / 4);  // Setup time
                
                // Rising edge - slave samples here
                sck = 1'b1;
                #(SCK_PERIOD / 2);
                
                // Falling edge
                sck = 1'b0;
                #(SCK_PERIOD / 4);
            end
        end
        
        // CS rising edge
        #(SCK_PERIOD);
        cs_n = 1'b1;
        #(SCK_PERIOD * 3);  // Wait for CS to stabilize
        
        packet_sent = 1'b1;
        #(SCK_PERIOD);
        packet_sent = 1'b0;
    endtask
    
    // Trigger transmission on send_packet
    always @(posedge send_packet) begin
        send_spi_packet();
    end

endmodule
