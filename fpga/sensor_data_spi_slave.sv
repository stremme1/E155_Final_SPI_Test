/**
 * Sensor Data SPI Slave Module
 * 
 * Sends multi-byte sensor data packets to MCU via SPI
 * Based on Lab07 aes_spi pattern, adapted for 32-byte data packets
 * 
 * Protocol:
 * - MCU pulls LOAD low to request data
 * - FPGA asserts DONE when data is ready
 * - MCU reads 32 bytes via SPI (256 bits total)
 * - MCU pulls LOAD high to acknowledge
 * 
 * SPI Mode: CPOL=0, CPHA=0 (data sampled on rising edge, changed on falling edge)
 */

`timescale 1ns / 1ps

module sensor_data_spi_slave(
    input  logic        clk,           // FPGA system clock
    input  logic        sck,           // SPI clock from MCU
    input  logic        sdi,           // SPI data in (MOSI from MCU, not used)
    output logic        sdo,           // SPI data out (MISO to MCU)
    input  logic        load,          // Load signal from MCU (request/acknowledge)
    output logic        done,          // Done signal to MCU (data ready)
    
    // Sensor data interface
    input  logic [7:0]  data_bytes [0:31],  // 32-byte data packet
    input  logic        data_ready,          // New data available
    output logic        data_ack             // Data has been sent
);

    // Internal signals - following Lab07 pattern
    logic [7:0] tx_buffer [0:31];      // Transmit buffer (32 bytes) - clk domain
    logic [7:0] tx_shift_reg;          // Shift register for current byte - sck domain
    logic [5:0] byte_index;            // Current byte index (0-31) - sck domain
    logic [2:0] bit_index;             // Current bit index within byte (0-7) - sck domain
    logic       sdodelayed;            // Delayed SDO
    logic       wasdone;               // Previous done state
    logic       packet_ready;          // Packet ready to send - clk domain
    logic       packet_acknowledged;   // Packet has been acknowledged - clk domain
    logic       load_prev;             // Previous load state
    
    // Latch data packet when ready (clk domain)
    always_ff @(posedge clk) begin
        load_prev <= load;
        
        if (data_ready && !packet_ready) begin
            // Latch new data packet
            for (int i = 0; i < 32; i = i + 1) begin
                tx_buffer[i] <= data_bytes[i];
            end
            packet_ready <= 1'b1;
            packet_acknowledged <= 1'b0;
        end else if (packet_ready && load && !load_prev) begin
            // Reset when load goes high (MCU acknowledges)
            packet_ready <= 1'b0;
            packet_acknowledged <= 1'b1;
        end else if (packet_acknowledged) begin
            packet_acknowledged <= 1'b0;
        end
    end
    
    // Done signal: Assert when packet is ready and not yet acknowledged (clk domain)
    always_ff @(posedge clk) begin
        done <= packet_ready && !packet_acknowledged;
    end
    
    // SPI transmission: Shift on posedge sck (sck domain)
    // Following Lab07 pattern: shift out bytes MSB first
    always_ff @(posedge sck) begin
        if (!wasdone && done) begin
            // First posedge: load first byte
            tx_shift_reg <= tx_buffer[0];
            byte_index <= 6'd1;
            bit_index <= 3'd0;
        end else if (wasdone && done) begin
            // Continue transmission
            if (bit_index == 3'd7) begin
                // Byte complete, load next byte
                if (byte_index < 32) begin
                    tx_shift_reg <= tx_buffer[byte_index];
                    byte_index <= byte_index + 1;
                    bit_index <= 3'd0;
                end else begin
                    // All bytes sent, keep last byte
                    bit_index <= 3'd0;
                end
            end else begin
                // Shift bit
                tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
                bit_index <= bit_index + 1;
            end
        end
    end
    
    // Track previous done state (for edge detection - like Lab07)
    always_ff @(negedge sck) begin
        wasdone <= done;
        sdodelayed <= tx_shift_reg[7];  // Current MSB (will be output next)
    end
    
    // SDO output: Following Lab07 pattern
    // When done is first asserted (!wasdone), output MSB before first clock edge
    // After first negedge, wasdone becomes true, so use sdodelayed
    assign sdo = (done & !wasdone) ? tx_buffer[0][7] : sdodelayed;
    
    // Acknowledge when transmission complete
    assign data_ack = packet_acknowledged;
    
endmodule
