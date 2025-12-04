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
    
    // Raw packet buffer from arduino_spi_slave (16 bytes)
    input  logic [7:0]  packet_buffer [0:15],
    
    // Status inputs (from header check)
    input  logic        initialized,
    input  logic        error
);

    // Simplified: Pass raw packet buffer directly to MCU
    // Packet format from Arduino (16 bytes):
    // Byte 0:    Header (0xAA)
    // Bytes 1-2: Roll (int16_t, MSB first)
    // Bytes 3-4: Pitch (int16_t, MSB first)
    // Bytes 5-6: Yaw (int16_t, MSB first)
    // Bytes 7-8: Gyro X (int16_t, MSB first)
    // Bytes 9-10: Gyro Y (int16_t, MSB first)
    // Bytes 11-12: Gyro Z (int16_t, MSB first)
    // Byte 13:   Flags (bit 0 = Euler valid, bit 1 = Gyro valid)
    // Bytes 14-15: Reserved (0x00)
    //
    // Data Pipeline: Arduino → arduino_spi_slave → this module → MCU
    // This module receives raw packet buffer from arduino_spi_slave and sends it directly to MCU
    
    localparam PACKET_SIZE = 16;
    
    // Test mode: When enabled, output known test pattern instead of sensor data
    // This helps verify SPI shift logic works independently of data capture
    localparam TEST_MODE = 1'b1;  // Set to 1 to enable test mode (ENABLED for debugging)
    
    // Test pattern - known values for debugging
    logic [7:0] test_pattern [0:15];
    assign test_pattern[0] = 8'hAA;
    assign test_pattern[1] = 8'h11;
    assign test_pattern[2] = 8'h22;
    assign test_pattern[3] = 8'h33;
    assign test_pattern[4] = 8'h44;
    assign test_pattern[5] = 8'h55;
    assign test_pattern[6] = 8'h66;
    assign test_pattern[7] = 8'h77;
    assign test_pattern[8] = 8'h88;
    assign test_pattern[9] = 8'h99;
    assign test_pattern[10] = 8'hAA;
    assign test_pattern[11] = 8'hBB;
    assign test_pattern[12] = 8'hCC;
    assign test_pattern[13] = 8'hDD;
    assign test_pattern[14] = 8'hEE;
    assign test_pattern[15] = 8'hFF;
    
    // ========================================================================
    // Clock Domain Crossing: Synchronize packet buffer from clk domain to sck domain
    // ========================================================================
    // Packet buffer comes from arduino_spi_slave (clocked on FPGA clk)
    // SPI slave logic is clocked on MCU sck (asynchronous)
    // Strategy: Capture packet buffer on CS falling edge (safe due to setup time)
    
    // Synchronize CS to clk domain (2-stage synchronizer)
    logic cs_n_sync1, cs_n_sync2;
    always_ff @(posedge clk) begin
        cs_n_sync1 <= cs_n;
        cs_n_sync2 <= cs_n_sync1;
    end
    
    // Packet snapshot - captured when CS goes low (stable during transaction)
    logic [7:0] packet_snapshot [0:PACKET_SIZE-1];
    
    // Initialize snapshot
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_snapshot[i] = 8'h00;
        end
    end
    
    // Capture packet buffer on CS falling edge (start of transaction)
    // Update snapshot continuously when CS is high, freeze when CS is low (during transaction)
    // In test mode, use test pattern; otherwise use packet_buffer
    always_ff @(posedge clk) begin
        if (cs_n) begin
            // CS high: Update snapshot continuously with latest data
            // This ensures we always have the latest data when CS goes low
            if (TEST_MODE) begin
                // Test mode: Use known test pattern
                for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
                    packet_snapshot[i] <= test_pattern[i];
                end
            end else begin
                // Normal mode: Use packet buffer from arduino_spi_slave
                for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
                    packet_snapshot[i] <= packet_buffer[i];
                end
            end
        end
        // When CS is low (during transaction), snapshot does NOT update - it remains stable
        // This ensures data consistency during the entire SPI transaction
    end
    
    // ----------------------------
    // Create 128-bit packet from packet snapshot (16 bytes * 8 bits)
    // ----------------------------
    logic [127:0] tx_packet;
    assign tx_packet = {
                packet_snapshot[0], packet_snapshot[1], packet_snapshot[2], packet_snapshot[3],
                packet_snapshot[4], packet_snapshot[5], packet_snapshot[6], packet_snapshot[7],
                packet_snapshot[8], packet_snapshot[9], packet_snapshot[10], packet_snapshot[11],
                packet_snapshot[12], packet_snapshot[13], packet_snapshot[14], packet_snapshot[15]
    };
    
    // ----------------------------
    // Shift registers and counters - clocked on SCK for reliable timing
    // ----------------------------
    logic [7:0] shift_out;  // Current byte being shifted out
    logic [3:0] byte_count;  // 0-15 (4 bits for 16 bytes)
    logic [2:0] bit_count;   // 0-7 (3 bits for 8 bits per byte)
    
    // CS state tracking for edge detection
    logic cs_n_sync_sck = 1'b1;  // CS synchronized to SCK domain
    logic cs_n_prev_sck = 1'b1;  // Previous CS state in SCK domain
    logic seen_first_rising = 1'b0;  // Track if we've seen first SCK rising edge after CS low
    
    // Synchronize CS to SCK domain (2-stage synchronizer on SCK falling edge)
    always_ff @(negedge sck) begin
        cs_n_sync_sck <= cs_n;
        cs_n_prev_sck <= cs_n_sync_sck;
    end
    
    // Detect first SCK rising edge after CS goes low (with async reset on CS high)
    always_ff @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            // Async reset when CS goes high
            seen_first_rising <= 1'b0;
        end else if (!cs_n && !seen_first_rising) begin
            // First SCK rising edge after CS goes low - first bit is now sampled
            seen_first_rising <= 1'b1;
        end
    end
    
    // Detect CS falling edge in SCK domain (combinational)
    logic cs_falling_edge_sck;
    assign cs_falling_edge_sck = cs_n_prev_sck && !cs_n_sync_sck;
    
    // ----------------------------
    // Main SPI slave logic - single always_ff block clocked on SCK falling edge
    // ----------------------------
    // SPI Mode 0 (CPOL=0, CPHA=0):
    // - MCU samples MISO on RISING edge of SCK
    // - FPGA must SETUP data on FALLING edge of SCK (before next rising edge)
    // - First bit must be stable when CS goes low (before first SCK edge)
    // - We only shift AFTER the first bit has been sampled (after first rising edge)
    
    // Main shift logic - clocked on SCK falling edge, with async reset on CS high
    // CRITICAL: Ensure shift_out is initialized before first SCK rising edge
    // Use first byte from packet snapshot (header byte)
    logic [7:0] first_byte_value;
    assign first_byte_value = packet_snapshot[0];
    
    always_ff @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            // Async reset when CS goes high - prepare for next transaction
            byte_count <= 0;
            bit_count  <= 0;
            shift_out  <= first_byte_value;  // Pre-initialize so it's ready when CS goes low
        end else begin
            // CS is low - ensure first byte is loaded before first SCK rising edge
            // When CS goes low, shift_out is already first_byte_value (from when CS was high)
            // Reload it on first SCK falling edge to ensure it's stable
            if (cs_falling_edge_sck) begin
                // CS falling edge detected in SCK domain - Load first byte immediately
                shift_out  <= first_byte_value;
                byte_count <= 0;
                bit_count  <= 0;
            end else if (seen_first_rising) begin
                // SCK falling edge AND CS is low AND first bit already sampled
                // Only shift if we've seen the first rising edge (first bit has been sampled)
                if (bit_count == 3'd7) begin
                    // Byte complete: Load next byte
                    // byte_count is the byte we just finished sending (0-15)
                    // In non-blocking assignment, RHS uses OLD value, so:
                    // - byte_count+1 is the NEXT byte index we want to load
                    // - packet_buffer[i] is at tx_packet[127 - i*8 -: 8]
                    if (byte_count < 15) begin
                        // Increment to next byte index
                        byte_count <= byte_count + 1;
                        bit_count  <= 0;
                        // Use (byte_count+1) because RHS uses OLD byte_count value
                        // When byte_count=0 (just sent header), load packet_buffer[1] at tx_packet[119:112]
                        shift_out <= tx_packet[127 - (byte_count+1)*8 -: 8];
                    end else begin
                        // Last byte (15) done, send zeros
                        byte_count <= byte_count + 1;
                        bit_count  <= 0;
                        shift_out <= 8'h00;
                    end
                end else begin
                    // Shift LEFT (MSB first) - move next bit into MSB position
                    // After sending MSB, we want the next bit (bit 6) in MSB position
                    // Left shift: bit 6 -> bit 7, bit 5 -> bit 6, ..., bit 0 -> bit 1, insert 0 in bit 0
                    shift_out <= {shift_out[6:0], 1'b0};
                    bit_count <= bit_count + 1;
                end
            end
            // If seen_first_rising is false, don't shift - ensures first byte stays stable
        end
    end
    
    // MISO output (tri-state when CS high)
    // Read-only mode: FPGA ignores MOSI, only shifts out on MISO
    // Output the MSB of the shift register
    assign sdo = cs_n ? 1'bz : shift_out[7];
    
endmodule
