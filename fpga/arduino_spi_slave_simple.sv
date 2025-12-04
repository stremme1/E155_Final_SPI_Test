`timescale 1ns / 1ps

// Arduino SPI Slave Module - SIMPLIFIED VERSION (based on Lab 7 pattern)
// FPGA is SPI slave to Arduino (Arduino is master) - READ-ONLY MODE
// Receives sensor data packets from Arduino using simple shift register approach
// SPI Mode 0 (CPOL=0, CPHA=0): Sample on SCK rising edge, change on falling edge
// 
// Protocol:
// - Arduino sends 16-byte packets via SPI.transfer()
// - FPGA receives data on MOSI (sdi) and shifts it in on SCK rising edge
// - CS (chip select) controls when transaction is active (active low)
// - Packet format: [Header(0xAA)][Roll][Pitch][Yaw][Gyro X][Gyro Y][Gyro Z][Flags][Reserved]
//   All 16-bit values are MSB-first (MSB byte, then LSB byte)

module arduino_spi_slave_simple(
    input  logic        clk,           // FPGA system clock (for CDC and output updates)
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
    // Simple SPI Receive Logic - Based on Lab 7 pattern
    // ========================================================================
    // SPI Mode 0 (CPOL=0, CPHA=0):
    // - Arduino samples MISO on RISING edge of SCK
    // - FPGA must sample MOSI on RISING edge of SCK
    // - Data is stable on rising edge
    //
    // Simple approach: Use a 128-bit shift register (16 bytes = 128 bits)
    // Shift in on posedge sck, no complex CDC needed during shift
    
    logic [127:0] packet_shift = 128'd0;  // 16 bytes = 128 bits
    logic [7:0]   bit_count = 8'd0;
    logic [7:0]   packet_buffer [0:PACKET_SIZE-1];
    
    // Initialize packet_buffer
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_buffer[i] = 8'h00;
        end
    end
    
    // Receive data on SCK rising edge (SPI Mode 0)
    // Simple: Shift in all 128 bits, then store to packet_buffer when complete
    always_ff @(posedge sck) begin
        if (!cs_n) begin
            // CS low - shift in data MSB-first
            logic [127:0] packet_shift_new = {packet_shift[126:0], sdi};
            packet_shift <= packet_shift_new;
            
            if (bit_count == 8'd127) begin
                // All 128 bits received - store into packet_buffer
                packet_buffer[0]  <= packet_shift_new[127:120];
                packet_buffer[1]  <= packet_shift_new[119:112];
                packet_buffer[2]  <= packet_shift_new[111:104];
                packet_buffer[3]  <= packet_shift_new[103:96];
                packet_buffer[4]  <= packet_shift_new[95:88];
                packet_buffer[5]  <= packet_shift_new[87:80];
                packet_buffer[6]  <= packet_shift_new[79:72];
                packet_buffer[7]  <= packet_shift_new[71:64];
                packet_buffer[8]  <= packet_shift_new[63:56];
                packet_buffer[9]  <= packet_shift_new[55:48];
                packet_buffer[10] <= packet_shift_new[47:40];
                packet_buffer[11] <= packet_shift_new[39:32];
                packet_buffer[12] <= packet_shift_new[31:24];
                packet_buffer[13] <= packet_shift_new[23:16];
                packet_buffer[14] <= packet_shift_new[15:8];
                packet_buffer[15] <= packet_shift_new[7:0];
                bit_count <= 8'd0;
            end else begin
                bit_count <= bit_count + 1;
            end
        end
    end
    
    // Reset bit counter when CS goes high
    always_ff @(posedge sck) begin
        if (cs_n) begin
            bit_count <= 8'd0;
        end
    end
    
    // ========================================================================
    // Simple Clock Domain Crossing - Read packet when CS goes high
    // ========================================================================
    logic cs_n_sync1, cs_n_sync2, cs_n_prev;
    logic [7:0] packet_snapshot [0:PACKET_SIZE-1];
    logic packet_ready = 1'b0;
    
    // Initialize packet_snapshot
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_snapshot[i] = 8'h00;
        end
    end
    
    // Synchronize CS to clk domain
    always_ff @(posedge clk) begin
        cs_n_sync1 <= cs_n;
        cs_n_sync2 <= cs_n_sync1;
        cs_n_prev <= cs_n_sync2;
    end
    
    // Capture packet when CS goes high (transaction complete)
    // Simple: Wait for CS to be high for 2 cycles, then read packet_buffer
    logic [1:0] cs_high_count = 2'd0;
    always_ff @(posedge clk) begin
        if (cs_n_sync2) begin
            // CS is high
            if (cs_high_count < 2'd2) begin
                cs_high_count <= cs_high_count + 1;
                packet_ready <= 1'b0;
            end else begin
                // CS has been high for 2+ cycles - safe to read
                packet_snapshot[0]  <= packet_buffer[0];
                packet_snapshot[1]  <= packet_buffer[1];
                packet_snapshot[2]  <= packet_buffer[2];
                packet_snapshot[3]  <= packet_buffer[3];
                packet_snapshot[4]  <= packet_buffer[4];
                packet_snapshot[5]  <= packet_buffer[5];
                packet_snapshot[6]  <= packet_buffer[6];
                packet_snapshot[7]  <= packet_buffer[7];
                packet_snapshot[8]  <= packet_buffer[8];
                packet_snapshot[9]  <= packet_buffer[9];
                packet_snapshot[10] <= packet_buffer[10];
                packet_snapshot[11] <= packet_buffer[11];
                packet_snapshot[12] <= packet_buffer[12];
                packet_snapshot[13] <= packet_buffer[13];
                packet_snapshot[14] <= packet_buffer[14];
                packet_snapshot[15] <= packet_buffer[15];
                packet_ready <= 1'b1;
            end
        end else begin
            // CS is low - reset counter and clear ready flag
            cs_high_count <= 2'd0;
            packet_ready <= 1'b0;
        end
    end
    
    // ========================================================================
    // Parse Packet and Update Outputs - SIMPLIFIED
    // ========================================================================
    logic [7:0] header = 8'h00;
    logic signed [15:0] roll = 16'd0;
    logic signed [15:0] pitch = 16'd0;
    logic signed [15:0] yaw = 16'd0;
    logic signed [15:0] gyro_x = 16'd0;
    logic signed [15:0] gyro_y = 16'd0;
    logic signed [15:0] gyro_z = 16'd0;
    logic [7:0] flags = 8'h00;
    
    // Parse packet when ready
    always_ff @(posedge clk) begin
        if (packet_ready) begin
            header <= packet_snapshot[0];
            roll <= {packet_snapshot[1], packet_snapshot[2]};
            pitch <= {packet_snapshot[3], packet_snapshot[4]};
            yaw <= {packet_snapshot[5], packet_snapshot[6]};
            gyro_x <= {packet_snapshot[7], packet_snapshot[8]};
            gyro_y <= {packet_snapshot[9], packet_snapshot[10]};
            gyro_z <= {packet_snapshot[11], packet_snapshot[12]};
            flags <= packet_snapshot[13];
        end
    end
    
    // Update outputs and status - SIMPLIFIED
    // Parse first, then update outputs on next cycle (parsed values need one cycle to register)
    logic packet_ready_delayed = 1'b0;
    always_ff @(posedge clk) begin
        packet_ready_delayed <= packet_ready;
        
        // Use delayed packet_ready so parsed values are ready
        if (packet_ready_delayed && (header == HEADER_BYTE)) begin
            // Valid packet - update everything (parsed values are ready from previous cycle)
            initialized <= 1'b1;
            error <= 1'b0;
            
            quat1_w <= 16'd16384;  // Q14 format representation of 1.0
            quat1_x <= roll;        // Roll → quat_x
            quat1_y <= pitch;       // Pitch → quat_y
            quat1_z <= yaw;         // Yaw → quat_z
            
            gyro1_x <= gyro_x;
            gyro1_y <= gyro_y;
            gyro1_z <= gyro_z;
            
            quat1_valid <= flags[0];
            gyro1_valid <= flags[1];
        end else if (packet_ready_delayed && (header != HEADER_BYTE)) begin
            // Invalid header
            error <= 1'b1;
            initialized <= 1'b0;
        end
        // Values persist when packet_ready_delayed is false
    end
    
    // Initialize outputs
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

endmodule

