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
// - Packet format: [Header(0xAA)][Roll][Pitch][Yaw][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]
//   All 16-bit values are MSB-first (MSB byte, then LSB byte)
//   Roll/Pitch/Yaw are Euler angles (int16_t scaled by 100, 0.01 degree resolution)
//   Gyro values are int16_t scaled by 2000
//
// Simplified: Just receive and buffer raw packet, pass to MCU for parsing

module arduino_spi_slave(
    input  logic        clk,           // FPGA system clock
    input  logic        cs_n,          // Chip select from Arduino (active low)
    input  logic        sck,           // SPI clock from Arduino
    input  logic        sdi,           // SPI data in (MOSI from Arduino)
    
    // Output: Raw packet buffer (16 bytes) - passed directly to MCU
    output logic [7:0]  packet_buffer [0:15],
    
    // Status outputs (from header check only)
    output logic        initialized,
    output logic        error
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
    // Initialize to zeros to avoid 'x' values
    logic [7:0] packet_buffer [0:PACKET_SIZE-1];
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_buffer[i] = 8'h00;
        end
    end
    
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
    // In SPI Mode 0, the first bit is set up by master before first SCK rising edge
    // We sample on each rising edge, starting with the first one
    // CRITICAL: packet_buffer is NOT reset on CS high - it retains data until overwritten
    // This is correct because we want to capture the complete packet before CS goes high
    always_ff @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            // Async reset when CS goes high - reset counters but NOT packet_buffer
            // packet_buffer retains the complete packet for CDC capture
            byte_count <= 0;
            bit_count  <= 0;
            rx_shift   <= 8'd0;
        end else begin
            // CS is low - receive data on rising edge of SCK (MSB first)
            // SPI Mode 0 MSB-first: First bit received is MSB (bit 7), last is LSB (bit 0)
            // For MSB-first: we receive bits in order bit7, bit6, ..., bit0
            // 
            // CORRECT LOGIC: Use left shift with new bit in LSB position
            // This works because: first bit goes to LSB, then shifts left on each new bit
            // After 7 shifts, first bit is in MSB position (bit 7)
            // 
            // Example: Receiving 0xAA (10101010)
            // Bit 7 (1): rx_shift = 00000001 (bit 7 in LSB)
            // Bit 6 (0): rx_shift = 00000010 (shifted left, bit 7 now in bit 1)
            // Bit 5 (1): rx_shift = 00000101
            // ...
            // Bit 0 (0): rx_shift = 10101010 (complete byte, first bit now in MSB)
            if (bit_count == 3'd7) begin
                // 8th bit (LSB) - shift it in and store complete byte
                packet_buffer[byte_count] <= {rx_shift[6:0], sdi};
                byte_count <= byte_count + 1;
                bit_count  <= 0;
                rx_shift   <= 8'd0;  // Clear for next byte
            end else begin
                // Shift in current bit (bits 1-7): left shift, new bit in LSB
                rx_shift <= {rx_shift[6:0], sdi};
                bit_count <= bit_count + 1;
            end
        end
    end
    
    // ========================================================================
    // Clock Domain Crossing: Synchronize packet data from SCK domain to clk domain
    // ========================================================================
    // Packet data is captured in SCK domain (asynchronous to FPGA clk)
    // Need to synchronize to clk domain for stable output
    
    // Synchronized packet buffer (output) - captured from SCK domain on CS rising edge
    logic [7:0] packet_buffer_sync [0:PACKET_SIZE-1];
    
    // Initialize synchronized buffer to avoid 'x' values
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_buffer_sync[i] = 8'h00;
        end
    end
    
    // Synchronize CS to clk domain (2-stage synchronizer)
    logic cs_n_sync_clk1, cs_n_sync_clk2;
    logic cs_n_prev_clk;
    always_ff @(posedge clk) begin
        cs_n_sync_clk1 <= cs_n;
        cs_n_sync_clk2 <= cs_n_sync_clk1;
        cs_n_prev_clk <= cs_n_sync_clk2;
    end
    
    // Detect CS rising edge (transaction complete)
    logic cs_rising_edge_clk;
    assign cs_rising_edge_clk = !cs_n_prev_clk && cs_n_sync_clk2;
    
    // Capture packet buffer on CS rising edge (transaction complete)
    // When CS goes high, SCK is idle (SPI Mode 0: CPOL=0, idle low)
    // packet_buffer is stable, safe to read after CS rising edge
    always_ff @(posedge clk) begin
        if (cs_rising_edge_clk) begin
            // CS rising edge - transaction complete, capture packet
            // Atomic read of all 16 bytes in one clock cycle
            packet_buffer_sync[0] <= packet_buffer[0];
            packet_buffer_sync[1] <= packet_buffer[1];
            packet_buffer_sync[2] <= packet_buffer[2];
            packet_buffer_sync[3] <= packet_buffer[3];
            packet_buffer_sync[4] <= packet_buffer[4];
            packet_buffer_sync[5] <= packet_buffer[5];
            packet_buffer_sync[6] <= packet_buffer[6];
            packet_buffer_sync[7] <= packet_buffer[7];
            packet_buffer_sync[8] <= packet_buffer[8];
            packet_buffer_sync[9] <= packet_buffer[9];
            packet_buffer_sync[10] <= packet_buffer[10];
            packet_buffer_sync[11] <= packet_buffer[11];
            packet_buffer_sync[12] <= packet_buffer[12];
            packet_buffer_sync[13] <= packet_buffer[13];
            packet_buffer_sync[14] <= packet_buffer[14];
            packet_buffer_sync[15] <= packet_buffer[15];
        end
        // Data persists until next CS rising edge
    end
    
    // Output synchronized packet buffer
    assign packet_buffer = packet_buffer_sync;
    
    // ========================================================================
    // Header Validation for Status Outputs
    // ========================================================================
    // Simple header check for initialized/error outputs
    // Check header on CS rising edge when packet is captured
    logic packet_received;  // Track if we've ever received a packet
    
    always_ff @(posedge clk) begin
        if (cs_rising_edge_clk) begin
            // New packet captured - check header
            packet_received <= 1'b1;
            
            if (packet_buffer_sync[0] == HEADER_BYTE) begin
                initialized <= 1'b1;
                error <= 1'b0;
            end else begin
                // Invalid header - only set error if we've actually received data
                if (packet_received || (packet_buffer_sync[0] != 8'h00)) begin
                    initialized <= 1'b0;
                    error <= 1'b1;
                end
            end
        end
        // Status persists until next packet
    end
    
    // Initialize
    initial begin
        initialized = 1'b0;
        error = 1'b0;
        packet_received = 1'b0;
    end
    
endmodule
