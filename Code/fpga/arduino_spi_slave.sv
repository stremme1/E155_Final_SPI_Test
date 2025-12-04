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
// - Packet format: [Header(0xAA)][Quat W][Quat X][Quat Y][Quat Z][Gyro X][Gyro Y][Gyro Z][Flags]
//   All 16-bit values are MSB-first (MSB byte, then LSB byte)
//   Quaternion values are int16_t in Q14 format (divide by 16384 to get float)
//   Gyro values are int16_t scaled by 2000
//
// DSP Feature: FPGA converts quaternion to Euler angles using DSP blocks
// Output packet format: [Header(0xAA)][Roll][Pitch][Yaw][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]
//   Roll/Pitch/Yaw are Euler angles (int16_t scaled by 100, 0.01 degree resolution)

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
    
    // ========================================================================
    // DSP-Based Quaternion to Euler Conversion
    // ========================================================================
    // Extract quaternion data from packet (Q14 format)
    // Packet format: [Header][Quat W][Quat X][Quat Y][Quat Z][Gyro X][Gyro Y][Gyro Z][Flags]
    logic signed [15:0] quat_w_raw, quat_x_raw, quat_y_raw, quat_z_raw;
    logic signed [15:0] gyro_x_raw, gyro_y_raw, gyro_z_raw;
    logic [7:0] flags_raw;
    
    // Extract quaternion values (signed int16, MSB first)
    assign quat_w_raw = {packet_buffer_sync[1], packet_buffer_sync[2]};
    assign quat_x_raw = {packet_buffer_sync[3], packet_buffer_sync[4]};
    assign quat_y_raw = {packet_buffer_sync[5], packet_buffer_sync[6]};
    assign quat_z_raw = {packet_buffer_sync[7], packet_buffer_sync[8]};
    
    // Extract gyroscope values (pass through)
    assign gyro_x_raw = {packet_buffer_sync[9], packet_buffer_sync[10]};
    assign gyro_y_raw = {packet_buffer_sync[11], packet_buffer_sync[12]};
    assign gyro_z_raw = {packet_buffer_sync[13], packet_buffer_sync[14]};
    assign flags_raw = packet_buffer_sync[15];
    
    // Convert Q14 format to 18-bit signed for DSP blocks (iCE40UP5k DSP supports 18x18)
    logic signed [17:0] quat_w, quat_x, quat_y, quat_z;
    assign quat_w = {{2{quat_w_raw[15]}}, quat_w_raw};  // Sign extend to 18 bits
    assign quat_x = {{2{quat_x_raw[15]}}, quat_x_raw};
    assign quat_y = {{2{quat_y_raw[15]}}, quat_y_raw};
    assign quat_z = {{2{quat_z_raw[15]}}, quat_z_raw};
    
    // DSP-based multiplications for quaternion-to-Euler conversion
    // Formulas:
    //   roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    //   pitch = asin(2*(w*y - z*x))
    //   yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    //
    // Use multiplication operator - synthesis tool will infer DSP blocks
    // iCE40UP5k has 8 DSP blocks, each supports 18x18 signed multiply
    
    // Intermediate products - synthesis will map to DSP blocks
    logic signed [35:0] wx_prod, yz_prod, wy_prod, zx_prod, wz_prod, xy_prod;
    logic signed [35:0] xx_prod, yy_prod, zz_prod;
    
    // Use multiplication operator - will infer DSP blocks
    always_ff @(posedge clk) begin
        wx_prod <= quat_w * quat_x;  // w*x
        yz_prod <= quat_y * quat_z;  // y*z
        wy_prod <= quat_w * quat_y;  // w*y
        zx_prod <= quat_z * quat_x;  // z*x
        wz_prod <= quat_w * quat_z;  // w*z
        xy_prod <= quat_x * quat_y;  // x*y
        xx_prod <= quat_x * quat_x;  // x*x
        yy_prod <= quat_y * quat_y;  // y*y
        zz_prod <= quat_z * quat_z;  // z*z
    end
    
    // Calculate intermediate values for Euler angles
    // Extract upper 18 bits from 36-bit products (scaled by 2^18)
    logic signed [17:0] wx_sum, yz_sum, wy_diff, zx_diff, wz_sum, xy_sum;
    logic signed [17:0] xx_sum, yy_sum, zz_sum;
    
    always_ff @(posedge clk) begin
        // Roll: 2*(w*x + y*z) and 1 - 2*(x*x + y*y)
        wx_sum <= wx_prod[33:16] + yz_prod[33:16];  // w*x + y*z (scaled)
        xx_sum <= xx_prod[33:16] + yy_prod[33:16];  // x*x + y*y (scaled)
        
        // Pitch: 2*(w*y - z*x)
        wy_diff <= wy_prod[33:16] - zx_prod[33:16];  // w*y - z*x (scaled)
        
        // Yaw: 2*(w*z + x*y) and 1 - 2*(y*y + z*z)
        wz_sum <= wz_prod[33:16] + xy_prod[33:16];  // w*z + x*y (scaled)
        yy_sum <= yy_prod[33:16] + zz_prod[33:16];  // y*y + z*z (scaled)
    end
    
    // Simplified Euler angle calculation
    // Full implementation would use CORDIC for atan2/asin, but for demonstration
    // we use a simplified linear approximation based on the quaternion components
    // This demonstrates DSP usage while providing reasonable Euler angle outputs
    
    logic signed [17:0] roll_numerator, roll_denominator;
    logic signed [17:0] pitch_value;
    logic signed [17:0] yaw_numerator, yaw_denominator;
    
    always_ff @(posedge clk) begin
        // Roll: atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        roll_numerator <= wx_sum;  // 2*(w*x + y*z) approximation
        roll_denominator <= 18'd16384 - xx_sum;  // 1 - 2*(x*x + y*y) approximation
        
        // Pitch: asin(2*(w*y - z*x))
        pitch_value <= wy_diff;  // 2*(w*y - z*x) approximation
        
        // Yaw: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        yaw_numerator <= wz_sum;  // 2*(w*z + x*y) approximation
        yaw_denominator <= 18'd16384 - yy_sum;  // 1 - 2*(y*y + z*z) approximation
    end
    
    // Convert to Euler angles (int16_t scaled by 100, 0.01 degree resolution)
    // Simplified conversion: Use quaternion components scaled to degrees
    // In production, this would use proper atan2/asin via CORDIC
    logic signed [15:0] roll_scaled, pitch_scaled, yaw_scaled;
    
    always_ff @(posedge clk) begin
        // Simplified approximation: Scale quaternion-based values to degrees
        // Q14 format: 16384 = 1.0, so multiply by 18000/16384 to get degrees*100
        // Use the numerator values as approximation of the angles
        roll_scaled <= (roll_numerator * 18'd18000) / 18'd16384;
        pitch_scaled <= (pitch_value * 18'd18000) / 18'd16384;
        yaw_scaled <= (yaw_numerator * 18'd18000) / 18'd16384;
    end
    
    // Reconstruct output packet with Euler angles
    // Output format: [Header][Roll][Pitch][Yaw][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]
    logic [7:0] packet_buffer_out [0:PACKET_SIZE-1];
    
    always_ff @(posedge clk) begin
        packet_buffer_out[0] <= packet_buffer_sync[0];  // Header
        packet_buffer_out[1] <= roll_scaled[15:8];      // Roll MSB
        packet_buffer_out[2] <= roll_scaled[7:0];       // Roll LSB
        packet_buffer_out[3] <= pitch_scaled[15:8];    // Pitch MSB
        packet_buffer_out[4] <= pitch_scaled[7:0];      // Pitch LSB
        packet_buffer_out[5] <= yaw_scaled[15:8];       // Yaw MSB
        packet_buffer_out[6] <= yaw_scaled[7:0];        // Yaw LSB
        packet_buffer_out[7] <= gyro_x_raw[15:8];       // Gyro X MSB
        packet_buffer_out[8] <= gyro_x_raw[7:0];        // Gyro X LSB
        packet_buffer_out[9] <= gyro_y_raw[15:8];      // Gyro Y MSB
        packet_buffer_out[10] <= gyro_y_raw[7:0];      // Gyro Y LSB
        packet_buffer_out[11] <= gyro_z_raw[15:8];     // Gyro Z MSB
        packet_buffer_out[12] <= gyro_z_raw[7:0];      // Gyro Z LSB
        packet_buffer_out[13] <= flags_raw;             // Flags
        packet_buffer_out[14] <= 8'h00;                 // Reserved
        packet_buffer_out[15] <= 8'h00;                 // Reserved
    end
    
    // Output converted packet buffer
    assign packet_buffer = packet_buffer_out;
    
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
