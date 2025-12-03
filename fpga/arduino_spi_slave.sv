`timescale 1ns / 1ps

// Arduino SPI Slave Module
// FPGA is SPI slave to Arduino (Arduino is master) - READ-ONLY MODE
// Receives sensor data packets from Arduino using CS-based protocol
// SPI Mode 0 (CPOL=0, CPHA=0): Arduino samples on rising edge, FPGA changes on falling edge
// 
// Protocol:
// - Arduino sends 16-byte packets via SPI.transfer()
// - FPGA receives data on MOSI (sdi) and shifts it in on SCK rising edge
// - CS (chip select) controls when transaction is active (active low)
// - Packet format: [Header(0xAA)][quat_w][quat_x][quat_y][quat_z][gyro_x][gyro_y][gyro_z][Flags]
//   All 16-bit values are MSB-first (MSB byte, then LSB byte)

module arduino_spi_slave(
    input  logic        clk,           // FPGA system clock
    input  logic        cs_n,          // Chip select from Arduino (active low)
    input  logic        sck,           // SPI clock from Arduino
    input  logic        sdi,           // SPI data in (MOSI from Arduino)
    
    // Outputs mapped to spi_slave_mcu interface
    output logic        initialized,
    output logic        error,
    output logic        quat1_valid,
    output logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    output logic        gyro1_valid,
    output logic signed [15:0] gyro1_x, gyro1_y, gyro1_z
);

    localparam PACKET_SIZE = 16;
    localparam HEADER_BYTE = 8'hAA;
    
    // ========================================================================
    // SPI Slave Receive Logic - Clocked on Arduino SCK
    // ========================================================================
    // SPI Mode 0 (CPOL=0, CPHA=0):
    // - Arduino samples MISO on RISING edge of SCK
    // - FPGA must sample MOSI on RISING edge of SCK
    // - Data is stable on rising edge
    
    // Shift register for receiving data
    logic [7:0] rx_shift;
    logic [3:0] byte_count;  // 0-15 (4 bits for 16 bytes)
    logic [2:0] bit_count;   // 0-7 (3 bits for 8 bits per byte)
    
    // Packet buffer - stores received 16-byte packet
    logic [7:0] packet_buffer [0:PACKET_SIZE-1];
    
    // CS state tracking for SCK domain
    logic cs_n_sync_sck = 1'b1;  // CS synchronized to SCK domain
    logic cs_n_prev_sck = 1'b1;  // Previous CS state in SCK domain
    
    // Synchronize CS to SCK domain (2-stage synchronizer on SCK falling edge)
    always_ff @(negedge sck) begin
        cs_n_sync_sck <= cs_n;
        cs_n_prev_sck <= cs_n_sync_sck;
    end
    
    // Detect CS falling edge in SCK domain (combinational)
    logic cs_falling_edge_sck;
    assign cs_falling_edge_sck = cs_n_prev_sck && !cs_n_sync_sck;
    
    // Main SPI receive logic - clocked on SCK rising edge with async reset on CS
    // SPI Mode 0: Sample data on rising edge of SCK
    // No need for separate receiving signal - CS low is sufficient to start receiving
    always_ff @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            // Async reset when CS goes high
            byte_count <= 0;
            bit_count  <= 0;
            rx_shift   <= 8'd0;
        end else begin
            // CS is low - receive data on rising edge of SCK (MSB first)
            // SPI Mode 0 MSB-first: First bit received is MSB (bit 7), last is LSB (bit 0)
            // Shift right: new bit (sdi) goes into MSB, previous bits shift right
            if (bit_count == 3'd7) begin
                // Byte complete - shift in 8th bit and store
                packet_buffer[byte_count] <= {sdi, rx_shift[7:1]};
                byte_count <= byte_count + 1;
                bit_count  <= 0;
                rx_shift   <= 8'd0;  // Clear for next byte
            end else begin
                // Shift in current bit (MSB first)
                rx_shift <= {sdi, rx_shift[7:1]};
                bit_count <= bit_count + 1;
            end
        end
    end
    
    // ========================================================================
    // Clock Domain Crossing: Synchronize packet data from SCK domain to clk domain
    // ========================================================================
    // Packet data is captured in SCK domain (asynchronous to FPGA clk)
    // Need to synchronize to clk domain for stable output
    
    // Snapshot: Capture packet on CS rising edge (transaction complete)
    logic cs_n_sync_clk1, cs_n_sync_clk2;
    logic cs_n_prev_clk;
    logic [7:0] packet_snapshot [0:PACKET_SIZE-1];
    logic packet_valid;
    
    // Synchronize CS to clk domain (2-stage synchronizer)
    always_ff @(posedge clk) begin
        cs_n_sync_clk1 <= cs_n;
        cs_n_sync_clk2 <= cs_n_sync_clk1;
        cs_n_prev_clk <= cs_n_sync_clk2;
    end
    
    // Detect CS rising edge (transaction complete)
    logic cs_rising_edge_clk;
    assign cs_rising_edge_clk = !cs_n_prev_clk && cs_n_sync_clk2;
    
    // Capture packet snapshot on CS rising edge (transaction complete)
    // Sample packet_buffer from SCK domain when CS goes high (safe because CS high means transaction is done)
    always_ff @(posedge clk) begin
        if (cs_rising_edge_clk) begin
            // Transaction complete - capture packet
            packet_snapshot[0] <= packet_buffer[0];
            packet_snapshot[1] <= packet_buffer[1];
            packet_snapshot[2] <= packet_buffer[2];
            packet_snapshot[3] <= packet_buffer[3];
            packet_snapshot[4] <= packet_buffer[4];
            packet_snapshot[5] <= packet_buffer[5];
            packet_snapshot[6] <= packet_buffer[6];
            packet_snapshot[7] <= packet_buffer[7];
            packet_snapshot[8] <= packet_buffer[8];
            packet_snapshot[9] <= packet_buffer[9];
            packet_snapshot[10] <= packet_buffer[10];
            packet_snapshot[11] <= packet_buffer[11];
            packet_snapshot[12] <= packet_buffer[12];
            packet_snapshot[13] <= packet_buffer[13];
            packet_snapshot[14] <= packet_buffer[14];
            packet_snapshot[15] <= packet_buffer[15];
            packet_valid <= 1'b1;
        end else begin
            packet_valid <= 1'b0;
        end
    end
    
    // ========================================================================
    // Parse Packet and Map to spi_slave_mcu Interface
    // ========================================================================
    // Arduino packet format:
    // Byte 0: Header (0xAA)
    // Bytes 1-2: quat_w (int16_t, MSB first)
    // Bytes 3-4: quat_x (int16_t, MSB first)
    // Bytes 5-6: quat_y (int16_t, MSB first)
    // Bytes 7-8: quat_z (int16_t, MSB first)
    // Bytes 9-10: gyro_x (int16_t, MSB first)
    // Bytes 11-12: gyro_y (int16_t, MSB first)
    // Bytes 13-14: gyro_z (int16_t, MSB first)
    // Byte 15: Flags (bit 0 = quat_valid, bit 1 = gyro_valid)
    
    // Parse packet fields
    logic [7:0] header;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic [7:0] flags;
    
    assign header = packet_snapshot[0];
    assign quat_w = {packet_snapshot[1], packet_snapshot[2]};
    assign quat_x = {packet_snapshot[3], packet_snapshot[4]};
    assign quat_y = {packet_snapshot[5], packet_snapshot[6]};
    assign quat_z = {packet_snapshot[7], packet_snapshot[8]};
    assign gyro_x = {packet_snapshot[9], packet_snapshot[10]};
    assign gyro_y = {packet_snapshot[11], packet_snapshot[12]};
    assign gyro_z = {packet_snapshot[13], packet_snapshot[14]};
    assign flags = packet_snapshot[15];
    
    // Validate header and set status
    // Initialize to 0 (not initialized, no error) on startup
    always_ff @(posedge clk) begin
        if (packet_valid) begin
            if (header == HEADER_BYTE) begin
                initialized <= 1'b1;
                error <= 1'b0;
            end else begin
                initialized <= 1'b0;
                error <= 1'b1;
            end
        end
        // Note: If packet_valid is false, keep previous state
        // This allows status to persist between packets
    end
    
    // Initialize outputs (SystemVerilog allows initialization)
    initial begin
        initialized = 1'b0;
        error = 1'b0;
        quat1_valid = 1'b0;
        gyro1_valid = 1'b0;
        quat1_w = 16'd0;
        quat1_x = 16'd0;
        quat1_y = 16'd0;
        quat1_z = 16'd0;
        gyro1_x = 16'd0;
        gyro1_y = 16'd0;
        gyro1_z = 16'd0;
    end
    
    // Map Arduino packet fields to spi_slave_mcu interface
    // Direct pass-through: Arduino sends quaternion format, map directly to outputs
    always_ff @(posedge clk) begin
        if (packet_valid && (header == HEADER_BYTE)) begin
            // Pass through quaternion data directly
            quat1_w <= quat_w;
            quat1_x <= quat_x;
            quat1_y <= quat_y;
            quat1_z <= quat_z;
            
            // Pass through gyroscope data
            gyro1_x <= gyro_x;
            gyro1_y <= gyro_y;
            gyro1_z <= gyro_z;
            
            // Map flags: bit 0 = quat_valid, bit 1 = gyro_valid
            quat1_valid <= flags[0];  // Quat valid bit
            gyro1_valid <= flags[1];  // Gyro valid bit
        end else begin
            // Keep previous values if no new packet
            quat1_w <= quat1_w;
            quat1_x <= quat1_x;
            quat1_y <= quat1_y;
            quat1_z <= quat1_z;
            gyro1_x <= gyro1_x;
            gyro1_y <= gyro1_y;
            gyro1_z <= gyro1_z;
            quat1_valid <= quat1_valid;
            gyro1_valid <= gyro1_valid;
        end
    end
    
endmodule
