`timescale 1ns / 1ps

// MCU SPI Slave Module
// FPGA is SPI slave to MCU (MCU is master)
// Sends RAW sensor data packets to MCU
// Based on e155-lab7-main 2/fpga/src/aes_spi.sv pattern
// SPI Mode 0 (CPOL=0, CPHA=0): data sampled on rising edge, changed on falling edge

module mcu_spi_slave(
    input  logic        clk,           // FPGA system clock
    input  logic        sck,           // SPI clock from MCU
    input  logic        sdi,           // SPI data in (MOSI from MCU, not used for commands)
    output logic        sdo,           // SPI data out (MISO to MCU)
    input  logic        load,          // Load signal from MCU (acknowledge)
    output logic        done,          // Done signal to MCU (data ready)
    
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
    logic [7:0] tx_shift_reg;
    logic       sdodelayed, wasdone;
    logic       data_ready;
    logic       load_prev;
    logic [4:0] bit_cnt;  // 5 bits for 16 bytes * 8 bits = 128 bits
    
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
    
    // Synchronize LOAD signal from MCU (cross-clock domain: MCU 80MHz -> FPGA 3MHz)
    // 2-stage synchronizer to prevent metastability
    logic load_sync1, load_sync2, load_sync_prev;
    
    always_ff @(posedge clk) begin
        load_sync1 <= load;      // First stage
        load_sync2 <= load_sync1; // Second stage (synchronized)
        load_sync_prev <= load_sync2; // Previous value for edge detection
    end
    
    // Detect rising edge of synchronized LOAD signal
    logic load_edge;
    assign load_edge = load_sync2 && !load_sync_prev;
    
    // Data ready when either sensor has valid data
    // Simplified logic: set data_ready when valid data arrives, clear when LOAD is acknowledged
    logic data_ready_reg = 1'b0;
    logic has_valid_prev = 1'b0;
    logic load_acknowledged = 1'b0;  // Track if we've just acknowledged
    logic has_valid;  // Combinational signal
    
    assign has_valid = (quat1_valid || gyro1_valid);
    
    always_ff @(posedge clk) begin
        // Check for LOAD edge (acknowledgment) - highest priority
        // LOAD rising edge means MCU has read the data and acknowledged
        if (load_edge) begin
            // MCU acknowledged - clear data ready
            data_ready_reg <= 1'b0;
            has_valid_prev <= 1'b0;
            load_acknowledged <= 1'b1;  // Mark that we just acknowledged
        end else begin
            // Clear acknowledgment flag after one cycle
            load_acknowledged <= 1'b0;
            
            // If valid data is present and we haven't seen it before, set data ready
            // This captures the one-cycle pulse from the BNO085 controller
            // Don't set if we just acknowledged (prevent immediate re-assertion)
            if (has_valid && !has_valid_prev && !load_acknowledged) begin
                // New valid data detected - set data ready
                data_ready_reg <= 1'b1;
                has_valid_prev <= 1'b1;
            end else if (!has_valid) begin
                // No valid data - update tracking but keep data_ready_reg set
                // (it will be cleared by LOAD acknowledgment)
                has_valid_prev <= 1'b0;
            end
            // If has_valid && has_valid_prev, keep data_ready_reg set (already asserted)
        end
    end
    
    // Done signal: Assert when data is ready
    // Assert DONE when data_ready_reg is set (new sensor data available)
    // DONE will be deasserted when MCU acknowledges with LOAD
    assign done = data_ready_reg;
    
    // Create a 128-bit shift register from packet buffer (16 bytes * 8 bits)
    logic [127:0] packet_shift_reg;
    
    // SPI transmission: Shift on posedge sck (like aes_spi.sv)
    // Following aes_spi pattern: on first posedge (!wasdone), load AND shift
    // On subsequent posedges (wasdone), shift left
    always_ff @(posedge sck) begin
        if (!wasdone) begin
            // First posedge: load buffer and shift in one operation (like aes_spi)
            // Pack all bytes into shift register: MSB first (packet_buffer[0] is MSB)
            packet_shift_reg <= {
                packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3],
                packet_buffer[4], packet_buffer[5], packet_buffer[6], packet_buffer[7],
                packet_buffer[8], packet_buffer[9], packet_buffer[10], packet_buffer[11],
                packet_buffer[12], packet_buffer[13], packet_buffer[14], packet_buffer[15],
                sdi  // Shift in SDI on first edge (like aes_spi)
            };
        end else begin
            // Subsequent posedges: shift left (MSB first)
            packet_shift_reg <= {packet_shift_reg[126:0], sdi};
        end
    end
    
    // Track previous done state (for edge detection - like aes_spi)
    // aes_spi uses blocking assignment for immediate update
    always @(negedge sck) begin
        wasdone = done;
        sdodelayed = packet_shift_reg[126];  // Next bit to output (MSB of remaining data)
    end
    
    // SDO output: Following aes_spi pattern exactly
    // When done is first asserted (!wasdone), output MSB before first clock edge
    // After first negedge, wasdone becomes true, so use sdodelayed
    assign sdo = (done & !wasdone) ? packet_buffer[0][7] : sdodelayed;
    
endmodule


