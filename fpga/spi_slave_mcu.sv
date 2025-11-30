`timescale 1ns / 1ps

// MCU SPI Slave Module
// FPGA is SPI slave to MCU (MCU is master) - READ-ONLY MODE
// Sends RAW sensor data packets to MCU using CS-based protocol
// Based on working CS-based SPI slave pattern
// SPI Mode 0 (CPOL=0, CPHA=0): MCU samples on rising edge, FPGA changes on falling edge
// 
// Read-Only Protocol:
// - FPGA ignores MOSI (sdi) completely - only shifts out data on MISO (sdo)
// - MCU generates SCK by sending dummy bytes (0x00) via spiSendReceive(0)
// - FPGA shifts out 1 bit per SCK edge from its shift register
// - MCU reads data from MISO during the transaction
// - CS (chip select) controls when transaction is active (active low)

module spi_slave_mcu(
    input  logic        clk,           // FPGA system clock
    input  logic        cs_n,          // Chip select from MCU (active low, PA11)
    input  logic        sck,           // SPI clock from MCU (PB3)
    input  logic        sdi,            // SPI data in (MOSI from MCU - IGNORED in read-only mode)
    output logic        sdo,           // SPI data out (MISO to MCU - PB4)
    
    // Sensor data inputs (RAW data from BNO085 controller)
    // Sensor 1 (Right Hand) - Single sensor only
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z
);

    // Packet format: 16 bytes total (single sensor only)
    // Byte 0:    Header (0xAA)
    // Byte 1-8:  Sensor 1 Quaternion (w, x, y, z - MSB,LSB each)
    // Byte 9-14: Sensor 1 Gyroscope (x, y, z - MSB,LSB each)
    // Byte 15:   Sensor 1 Flags (bit 0=quat_valid, bit 1=gyro_valid)
    
    localparam PACKET_SIZE = 16;
    localparam HEADER_BYTE = 8'hAA;
    
    // Packet buffer - assembled from sensor data
    logic [7:0] packet_buffer [0:PACKET_SIZE-1];
    
    // Assemble packet from sensor data (using assign statements for iverilog compatibility)
    // Header
    assign packet_buffer[0] = HEADER_BYTE;
    
    // Sensor 1 Quaternion (MSB,LSB format)
    assign packet_buffer[1] = quat1_w[15:8];  // W MSB
    assign packet_buffer[2] = quat1_w[7:0];   // W LSB
    assign packet_buffer[3] = quat1_x[15:8];  // X MSB
    assign packet_buffer[4] = quat1_x[7:0];   // X LSB
    assign packet_buffer[5] = quat1_y[15:8];  // Y MSB
    assign packet_buffer[6] = quat1_y[7:0];   // Y LSB
    assign packet_buffer[7] = quat1_z[15:8];  // Z MSB
    assign packet_buffer[8] = quat1_z[7:0];   // Z LSB
    
    // Sensor 1 Gyroscope (MSB,LSB format)
    assign packet_buffer[9]  = gyro1_x[15:8];  // X MSB
    assign packet_buffer[10] = gyro1_x[7:0];   // X LSB
    assign packet_buffer[11] = gyro1_y[15:8];  // Y MSB
    assign packet_buffer[12] = gyro1_y[7:0];   // Y LSB
    assign packet_buffer[13] = gyro1_z[15:8];  // Z MSB
    assign packet_buffer[14] = gyro1_z[7:0];   // Z LSB
    
    // Sensor 1 Flags
    assign packet_buffer[15] = {6'h0, gyro1_valid, quat1_valid};
    
    // ----------------------------
    // Synchronize SCK to FPGA clk
    // ----------------------------
    logic sck_d, sck_dd;
    always_ff @(posedge clk) begin
        sck_d  <= sck;
        sck_dd <= sck_d;
    end
    
    wire sck_rising  = (sck_d & ~sck_dd);
    wire sck_falling = (~sck_d & sck_dd);
    
    // ----------------------------
    // Shift registers and counters
    // ----------------------------
    logic [7:0] shift_out;  // Current byte being shifted out
    
    logic [3:0] byte_count;  // 0-15 (4 bits for 16 bytes)
    logic [2:0] bit_count;   // 0-7 (3 bits for 8 bits per byte)
    
    // Create 128-bit packet from packet buffer (16 bytes * 8 bits)
    logic [127:0] tx_packet;
    assign tx_packet = {
                packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3],
                packet_buffer[4], packet_buffer[5], packet_buffer[6], packet_buffer[7],
                packet_buffer[8], packet_buffer[9], packet_buffer[10], packet_buffer[11],
        packet_buffer[12], packet_buffer[13], packet_buffer[14], packet_buffer[15]
    };
    
    // MISO output (tri-state when CS high)
    // Read-only mode: FPGA ignores MOSI, only shifts out on MISO
    assign sdo = cs_n ? 1'bz : shift_out[7];
    
    // ----------------------------
    // Main SPI slave logic
    // ----------------------------
    // Detect CS falling edge to load first byte
    logic cs_n_d, cs_n_dd;
    always_ff @(posedge clk) begin
        cs_n_d  <= cs_n;
        cs_n_dd <= cs_n_d;
    end
    wire cs_falling = (~cs_n_d & cs_n_dd);  // CS going from high to low
    
    always_ff @(posedge clk) begin
        
        if (cs_n) begin
            // Reset when CS goes high
            byte_count   <= 0;
            bit_count    <= 0;
            shift_out    <= tx_packet[127 -: 8];  // First byte is MSB side (packet_buffer[0])
            
        end else begin
            
            // Load first byte when CS goes low (if not already loaded)
            if (cs_falling) begin
                shift_out <= tx_packet[127 -: 8];  // Load first byte (packet_buffer[0])
                byte_count <= 0;
                bit_count <= 0;
            end
            
            // SPI Mode 0 (CPOL=0, CPHA=0):
            // - MCU samples MISO on RISING edge of SCK
            // - FPGA must SETUP data on FALLING edge of SCK (before next rising edge)
            // - First bit is already stable when CS goes low (loaded above)
            
            // Shift on FALLING edge to prepare next bit for MCU to sample on rising edge
            if (sck_falling) begin
                // Check if we've completed a full byte (8 bits: 0-7)
                // Check BEFORE incrementing: if bit_count is 7, we've already output 8 bits
                if (bit_count == 3'd7) begin
                    // Just finished 8th bit, move to next byte
                    byte_count <= byte_count + 1;
                    bit_count <= 0;  // Reset bit counter
                    
                    // Load next byte: when byte_count was 0 (just finished byte 0), load packet_buffer[1]
                    // Formula: 127 - (byte_count+1)*8 gives correct byte index
                    if (byte_count < 15) begin  // Prevent overflow when byte_count reaches 15
                        shift_out <= tx_packet[127 - (byte_count+1)*8 -: 8];
                    end
                end else begin
                    // Shift LEFT so next bit moves into MSB position [7] for output
                    // For MSB-first: we output shift_out[7] (bit 7), then shift left so bit[6] moves to position [7]
                    // Shift left, shift in 0 from right (LSB side)
                    shift_out <= {shift_out[6:0], 1'b0};  // Shift left, shift in 0 from right
                    bit_count <= bit_count + 1;  // Increment bit counter
                end
            end
    
        end
    end
    
endmodule
