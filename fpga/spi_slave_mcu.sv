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
    input  logic        initialized,
    input  logic        error,
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z
);

    // Packet format: 16 bytes total (single sensor only)
    // Byte 0:    Header (0xAA)
    // Byte 1-8:  Sensor 1 Quaternion (w, x, y, z - MSB,LSB each)
    // Byte 9-14: Sensor 1 Gyroscope (x, y, z - MSB,LSB each)
    // Byte 15:   Sensor 1 Flags (bit 0=quat_valid, bit 1=gyro_valid, bit 2=initialized, bit 3=error)
    
    localparam PACKET_SIZE = 16;
    localparam HEADER_BYTE = 8'hAA;
    
    // Test mode: When enabled, output known test pattern instead of sensor data
    // This helps verify SPI shift logic works independently of data capture
    localparam TEST_MODE = 1'b0;  // Set to 1 to enable test mode
    
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
    logic initialized_clk, error_clk;
    logic signed [15:0] quat1_w_clk, quat1_x_clk, quat1_y_clk, quat1_z_clk;
    logic signed [15:0] gyro1_x_clk, gyro1_y_clk, gyro1_z_clk;
    
    always_ff @(posedge clk) begin
        quat1_valid_clk <= quat1_valid;
        gyro1_valid_clk <= gyro1_valid;
        initialized_clk <= initialized;
        error_clk <= error;
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
    logic initialized_snap, error_snap;
    logic signed [15:0] quat1_w_snap, quat1_x_snap, quat1_y_snap, quat1_z_snap;
    logic signed [15:0] gyro1_x_snap, gyro1_y_snap, gyro1_z_snap;
    
    // Capture snapshot on CS falling edge (start of transaction)
    // Also initialize on first posedge clk when CS is high (for testbench to read packet_buffer before transaction)
    logic cs_n_sync1, cs_n_sync2;  // Synchronize CS to clk domain for edge detection
    
    // Synchronize CS to clk domain (2-stage synchronizer)
    always_ff @(posedge clk) begin
        cs_n_sync1 <= cs_n;
        cs_n_sync2 <= cs_n_sync1;
    end
    
    // Detect CS falling edge (combinational) - use current sync value, not delayed
    logic cs_falling_edge;
    assign cs_falling_edge = cs_n_sync1 && !cs_n;  // Use sync1 for faster detection (1 clock delay instead of 2)
    
    // Detect CS falling edge and capture snapshot (all in clk domain for synthesis)
    // Update snapshot when CS is high, freeze when CS is low (during transaction)
    // This ensures data consistency during the entire SPI transaction
    always_ff @(posedge clk) begin
        if (cs_n) begin
            // CS high: Update snapshot continuously with latest data
            // This ensures we always have the latest data when CS goes low
            quat1_w_snap <= quat1_w_clk;
            quat1_x_snap <= quat1_x_clk;
            quat1_y_snap <= quat1_y_clk;
            quat1_z_snap <= quat1_z_clk;
            gyro1_x_snap <= gyro1_x_clk;
            gyro1_y_snap <= gyro1_y_clk;
            gyro1_z_snap <= gyro1_z_clk;
            quat1_valid_snap <= quat1_valid_sync2;
            gyro1_valid_snap <= gyro1_valid_sync2;
            initialized_snap <= initialized_clk;
            error_snap <= error_clk;
        end
        // When CS is low (during transaction), snapshot does NOT update - it remains stable
        // This ensures data consistency during the entire SPI transaction
    end
    
    // Packet buffer - assembled from SNAPSHOT data (stable during transaction)
    logic [7:0] packet_buffer [0:PACKET_SIZE-1];
    
    // Assemble packet from snapshot sensor data (using assign statements for iverilog compatibility)
    // In test mode, use known test pattern; otherwise use sensor data
    generate
        if (TEST_MODE) begin : gen_test_mode
            // Test mode: Use known pattern
            assign packet_buffer[0] = test_pattern[0];
            assign packet_buffer[1] = test_pattern[1];
            assign packet_buffer[2] = test_pattern[2];
            assign packet_buffer[3] = test_pattern[3];
            assign packet_buffer[4] = test_pattern[4];
            assign packet_buffer[5] = test_pattern[5];
            assign packet_buffer[6] = test_pattern[6];
            assign packet_buffer[7] = test_pattern[7];
            assign packet_buffer[8] = test_pattern[8];
            assign packet_buffer[9] = test_pattern[9];
            assign packet_buffer[10] = test_pattern[10];
            assign packet_buffer[11] = test_pattern[11];
            assign packet_buffer[12] = test_pattern[12];
            assign packet_buffer[13] = test_pattern[13];
            assign packet_buffer[14] = test_pattern[14];
            assign packet_buffer[15] = test_pattern[15];
        end else begin : gen_normal_mode
            // Normal mode: Use sensor data
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
            
            // Sensor 1 Flags - from snapshot (bit 0=quat_valid, bit 1=gyro_valid, bit 2=initialized, bit 3=error)
            assign packet_buffer[15] = {4'h0, error_snap, initialized_snap, gyro1_valid_snap, quat1_valid_snap};
        end
    endgenerate
    
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
    always_ff @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            // Async reset when CS goes high
            byte_count <= 0;
            bit_count  <= 0;
            shift_out  <= HEADER_BYTE;
        end else begin
            // CS is low - check if we need to load first byte
            // This handles the case where CS goes low before any SCK edges
            if (cs_falling_edge_sck || (!seen_first_rising && byte_count == 0 && bit_count == 0)) begin
                // CS falling edge detected OR first SCK edge and shift_out not loaded yet
                // Load first byte immediately
                shift_out  <= HEADER_BYTE;
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
