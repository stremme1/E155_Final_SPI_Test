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
    // SPI Slave Receive Logic - Clocked on Arduino SCK (Lab7 style - simple shift)
    // ========================================================================
    // SPI Mode 0 (CPOL=0, CPHA=0): Data sampled on first edge (rising edge)
    // Simple shift register approach - shift in on posedge sck, MSB first
    // Similar to lab7 aes_spi.sv pattern - proven to work
    
    // 128-bit shift register (16 bytes * 8 bits) - MSB first
    // packet_buffer_rx[0] is first byte (header), packet_buffer_rx[15] is last byte
    logic [127:0] packet_shift;
    
    // Initialize shift register
    initial begin
        packet_shift = 128'd0;
    end
    
    // Main SPI receive logic - simple shift on SCK rising edge (Lab7 style)
    // Shift in data on posedge sck: {packet_shift[126:0], sdi}
    // CRITICAL: Do NOT reset packet_shift on CS high - we need to read it for CDC!
    // Reset only on CS falling edge (new transaction starting)
    logic cs_n_prev_sck = 1'b1;
    
    always_ff @(posedge sck) begin
        cs_n_prev_sck <= cs_n;
        
        if (cs_n_prev_sck && !cs_n) begin
            // CS falling edge - new transaction starting, reset shift register
            packet_shift <= 128'd0;
        end else if (!cs_n) begin
            // CS low - shift in data (MSB first)
            // First bit (MSB of first byte) goes to bit 127, shifts right
            packet_shift <= {packet_shift[126:0], sdi};
        end
        // When CS is high, packet_shift retains its value for CDC capture
    end
    
    // Extract 16 bytes from 128-bit shift register
    // packet_shift[127:120] is first byte (header), packet_shift[7:0] is last byte
    logic [7:0] packet_buffer_rx [0:PACKET_SIZE-1];
    assign packet_buffer_rx[0]  = packet_shift[127:120];
    assign packet_buffer_rx[1]  = packet_shift[119:112];
    assign packet_buffer_rx[2]  = packet_shift[111:104];
    assign packet_buffer_rx[3]  = packet_shift[103:96];
    assign packet_buffer_rx[4]  = packet_shift[95:88];
    assign packet_buffer_rx[5]  = packet_shift[87:80];
    assign packet_buffer_rx[6]  = packet_shift[79:72];
    assign packet_buffer_rx[7]  = packet_shift[71:64];
    assign packet_buffer_rx[8]  = packet_shift[63:56];
    assign packet_buffer_rx[9]  = packet_shift[55:48];
    assign packet_buffer_rx[10] = packet_shift[47:40];
    assign packet_buffer_rx[11] = packet_shift[39:32];
    assign packet_buffer_rx[12] = packet_shift[31:24];
    assign packet_buffer_rx[13] = packet_shift[23:16];
    assign packet_buffer_rx[14] = packet_shift[15:8];
    assign packet_buffer_rx[15] = packet_shift[7:0];
    
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
    
    // Capture packet buffer when CS is high (transaction complete)
    // When CS goes high, SCK is idle (SPI Mode 0: CPOL=0, idle low)
    // packet_buffer_rx is stable, safe to read when CS is high
    // Update continuously when CS is high to ensure latest data is available
    always_ff @(posedge clk) begin
        if (cs_n_sync_clk2) begin
            // CS is high - transaction complete, update packet buffer continuously
            // This ensures we always have the latest data available
            // Atomic read of all 16 bytes in one clock cycle
            packet_buffer_sync[0] <= packet_buffer_rx[0];
            packet_buffer_sync[1] <= packet_buffer_rx[1];
            packet_buffer_sync[2] <= packet_buffer_rx[2];
            packet_buffer_sync[3] <= packet_buffer_rx[3];
            packet_buffer_sync[4] <= packet_buffer_rx[4];
            packet_buffer_sync[5] <= packet_buffer_rx[5];
            packet_buffer_sync[6] <= packet_buffer_rx[6];
            packet_buffer_sync[7] <= packet_buffer_rx[7];
            packet_buffer_sync[8] <= packet_buffer_rx[8];
            packet_buffer_sync[9] <= packet_buffer_rx[9];
            packet_buffer_sync[10] <= packet_buffer_rx[10];
            packet_buffer_sync[11] <= packet_buffer_rx[11];
            packet_buffer_sync[12] <= packet_buffer_rx[12];
            packet_buffer_sync[13] <= packet_buffer_rx[13];
            packet_buffer_sync[14] <= packet_buffer_rx[14];
            packet_buffer_sync[15] <= packet_buffer_rx[15];
        end
        // When CS is low (during transaction), packet_buffer_sync does NOT update
        // This ensures data consistency during the entire SPI transaction
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
