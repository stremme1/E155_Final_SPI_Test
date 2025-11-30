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
    
    // ========================================================================
    // Clock Domain Crossing: Synchronize sensor data from clk domain to sck domain
    // ========================================================================
    // Sensor data comes from BNO085 controller (clocked on FPGA clk)
    // SPI slave logic is clocked on MCU sck (asynchronous)
    // Strategy: 
    // 1. Register data in clk domain to prevent glitches
    // 2. Use 2-stage synchronizer for single-bit signals (valid flags)
    // 3. Capture multi-bit data on CS falling edge (safe due to setup time)
    
    // Stage 1: Capture sensor data in clk domain (registered to prevent glitches)
    logic quat1_valid_clk, gyro1_valid_clk;
    logic signed [15:0] quat1_w_clk, quat1_x_clk, quat1_y_clk, quat1_z_clk;
    logic signed [15:0] gyro1_x_clk, gyro1_y_clk, gyro1_z_clk;
    
    always_ff @(posedge clk) begin
        quat1_valid_clk <= quat1_valid;
        gyro1_valid_clk <= gyro1_valid;
        quat1_w_clk <= quat1_w;
        quat1_x_clk <= quat1_x;
        quat1_y_clk <= quat1_y;
        quat1_z_clk <= quat1_z;
        gyro1_x_clk <= gyro1_x;
        gyro1_y_clk <= gyro1_y;
        gyro1_z_clk <= gyro1_z;
    end
    
    // Stage 2: 2-stage synchronizer for valid flags (single-bit CDC)
    // Synchronize on clk to ensure stable capture
    logic quat1_valid_sync1, quat1_valid_sync2;
    logic gyro1_valid_sync1, gyro1_valid_sync2;
    
    always_ff @(posedge clk) begin
        quat1_valid_sync1 <= quat1_valid_clk;
        quat1_valid_sync2 <= quat1_valid_sync1;
        gyro1_valid_sync1 <= gyro1_valid_clk;
        gyro1_valid_sync2 <= gyro1_valid_sync1;
    end
    
    // ========================================================================
    // Data Snapshot: Capture stable data when CS goes low
    // ========================================================================
    // Snapshot ensures data doesn't change during the SPI transaction
    // Capture on CS falling edge from registered clk-domain data
    // Safe because: data is registered (stable), CS provides setup time before first SCK
    logic quat1_valid_snap, gyro1_valid_snap;
    logic signed [15:0] quat1_w_snap, quat1_x_snap, quat1_y_snap, quat1_z_snap;
    logic signed [15:0] gyro1_x_snap, gyro1_y_snap, gyro1_z_snap;
    
    // Capture snapshot on CS falling edge (start of transaction)
    // Also initialize on first posedge clk when CS is high (for testbench to read packet_buffer before transaction)
    logic cs_n_sync1, cs_n_sync2;  // Synchronize CS to clk domain for edge detection
    logic snapshot_initialized = 1'b0;  // Track if snapshot has been initialized
    
    // Synchronize CS to clk domain (2-stage synchronizer)
    always_ff @(posedge clk) begin
        cs_n_sync1 <= cs_n;
        cs_n_sync2 <= cs_n_sync1;
    end
    
    // Detect CS falling edge (combinational)
    logic cs_falling_edge;
    assign cs_falling_edge = cs_n_sync2 && !cs_n;
    
    // Detect CS falling edge and capture snapshot (all in clk domain for synthesis)
    always_ff @(posedge clk) begin
        if (cs_falling_edge) begin
            // CS falling edge: Capture registered data from clk domain (stable, no metastability)
            // This snapshot remains stable during the entire transaction (while CS is low)
            quat1_w_snap <= quat1_w_clk;
            quat1_x_snap <= quat1_x_clk;
            quat1_y_snap <= quat1_y_clk;
            quat1_z_snap <= quat1_z_clk;
            gyro1_x_snap <= gyro1_x_clk;
            gyro1_y_snap <= gyro1_y_clk;
            gyro1_z_snap <= gyro1_z_clk;
            // Use synchronized valid flags
            quat1_valid_snap <= quat1_valid_sync2;
            gyro1_valid_snap <= gyro1_valid_sync2;
            snapshot_initialized <= 1'b1;
        end else if (cs_n && !snapshot_initialized) begin
            // CS high and snapshot not yet initialized: Initialize from clk domain
            // This allows testbench to read packet_buffer before first transaction
            quat1_w_snap <= quat1_w_clk;
            quat1_x_snap <= quat1_x_clk;
            quat1_y_snap <= quat1_y_clk;
            quat1_z_snap <= quat1_z_clk;
            gyro1_x_snap <= gyro1_x_clk;
            gyro1_y_snap <= gyro1_y_clk;
            gyro1_z_snap <= gyro1_z_clk;
            quat1_valid_snap <= quat1_valid_sync2;
            gyro1_valid_snap <= gyro1_valid_sync2;
            snapshot_initialized <= 1'b1;
        end else if (cs_n) begin
            // CS high: Reset initialization flag so snapshot can be updated on next CS low
            snapshot_initialized <= 1'b0;
        end
        // When CS is low (during transaction), snapshot does NOT update - it remains stable
    end
    
    // Packet buffer - assembled from SNAPSHOT data (stable during transaction)
    logic [7:0] packet_buffer [0:PACKET_SIZE-1];
    
    // Assemble packet from snapshot sensor data (using assign statements for iverilog compatibility)
    // Header
    assign packet_buffer[0] = HEADER_BYTE;
    
    // Sensor 1 Quaternion (MSB,LSB format) - from snapshot
    assign packet_buffer[1] = quat1_w_snap[15:8];  // W MSB
    assign packet_buffer[2] = quat1_w_snap[7:0];   // W LSB
    assign packet_buffer[3] = quat1_x_snap[15:8];  // X MSB
    assign packet_buffer[4] = quat1_x_snap[7:0];   // X LSB
    assign packet_buffer[5] = quat1_y_snap[15:8];  // Y MSB
    assign packet_buffer[6] = quat1_y_snap[7:0];   // Y LSB
    assign packet_buffer[7] = quat1_z_snap[15:8];  // Z MSB
    assign packet_buffer[8] = quat1_z_snap[7:0];   // Z LSB
    
    // Sensor 1 Gyroscope (MSB,LSB format) - from snapshot
    assign packet_buffer[9]  = gyro1_x_snap[15:8];  // X MSB
    assign packet_buffer[10] = gyro1_x_snap[7:0];   // X LSB
    assign packet_buffer[11] = gyro1_y_snap[15:8];  // Y MSB
    assign packet_buffer[12] = gyro1_y_snap[7:0];   // Y LSB
    assign packet_buffer[13] = gyro1_z_snap[15:8];  // Z MSB
    assign packet_buffer[14] = gyro1_z_snap[7:0];   // Z LSB
    
    // Sensor 1 Flags - from snapshot
    assign packet_buffer[15] = {6'h0, gyro1_valid_snap, quat1_valid_snap};
    
    // ----------------------------
    // Create 128-bit packet from packet buffer (16 bytes * 8 bits)
    // ----------------------------
    logic [127:0] tx_packet;
    assign tx_packet = {
                packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3],
                packet_buffer[4], packet_buffer[5], packet_buffer[6], packet_buffer[7],
                packet_buffer[8], packet_buffer[9], packet_buffer[10], packet_buffer[11],
        packet_buffer[12], packet_buffer[13], packet_buffer[14], packet_buffer[15]
    };
    
    // Registered first byte - updated in clk domain so it's ready when CS goes low
    // This ensures the first bit is stable before the first SCK edge
    // Use HEADER_BYTE directly since it's a constant
    logic [7:0] first_byte_reg = HEADER_BYTE;
    always_ff @(posedge clk) begin
        first_byte_reg <= HEADER_BYTE;  // Always 0xAA (constant)
    end
    
    // ----------------------------
    // Shift registers and counters - clocked on SCK for reliable timing
    // ----------------------------
    logic [7:0] shift_out;  // Current byte being shifted out
    logic [3:0] byte_count;  // 0-15 (4 bits for 16 bytes)
    logic [2:0] bit_count;   // 0-7 (3 bits for 8 bits per byte)
    
    // CS state tracking for edge detection
    logic cs_n_prev = 1'b1;  // Previous CS state (initialize to high)
    logic seen_first_rising = 1'b0;  // Track if we've seen first SCK rising edge after CS low
    
    // Track CS state - update on CS edges and SCK falling edge
    always_ff @(posedge cs_n or negedge cs_n or negedge sck) begin
        cs_n_prev <= cs_n;
    end
    
    // Reset seen_first_rising when CS goes high
    always_ff @(posedge cs_n) begin
        seen_first_rising <= 1'b0;
    end
    
    // Detect first SCK rising edge after CS goes low
    always_ff @(posedge sck) begin
        if (!cs_n && !seen_first_rising) begin
            // First SCK rising edge after CS goes low - first bit is now sampled
            seen_first_rising <= 1'b1;
        end
    end
    
    // Detect CS falling edge for SPI logic (combinational, but only used in always_ff with proper timing)
    logic cs_falling_edge_spi;
    assign cs_falling_edge_spi = cs_n_prev && !cs_n;
    
    // ----------------------------
    // Main SPI slave logic - simplified single always_ff with clear priority
    // ----------------------------
    // SPI Mode 0 (CPOL=0, CPHA=0):
    // - MCU samples MISO on RISING edge of SCK
    // - FPGA must SETUP data on FALLING edge of SCK (before next rising edge)
    // - First bit must be stable when CS goes low (before first SCK edge)
    // - We only shift AFTER the first bit has been sampled (after first rising edge)
    // Priority: CS high > CS falling > SCK falling
    always_ff @(negedge sck or posedge cs_n or negedge cs_n) begin
        if (cs_n) begin
            // Priority 1: CS high - Reset everything, prepare for next transaction
            byte_count <= 0;
            bit_count  <= 0;
            shift_out  <= HEADER_BYTE;  // Pre-load first byte
        end else if (cs_falling_edge_spi) begin
            // Priority 2: CS falling edge detected - Load first byte immediately
            shift_out  <= HEADER_BYTE;
            byte_count <= 0;
            bit_count  <= 0;
        end else if (!cs_n && seen_first_rising) begin
            // Priority 3: SCK falling edge AND CS is low AND first bit already sampled
            // Only shift if we've seen the first rising edge (first bit has been sampled)
            if (bit_count == 3'd7) begin
                // Byte complete: Load next byte
                byte_count <= byte_count + 1;
                bit_count  <= 0;
                if (byte_count < 15) begin
                    shift_out <= tx_packet[127 - (byte_count+1)*8 -: 8];
                end else begin
                    shift_out <= 8'h00;  // Last byte done
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
    
    // MISO output (tri-state when CS high)
    // Read-only mode: FPGA ignores MOSI, only shifts out on MISO
    // Output the MSB of the shift register
    assign sdo = cs_n ? 1'bz : shift_out[7];
    
endmodule
