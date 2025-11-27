/**
 * Sensor Data Packer Module
 * 
 * Packs raw BNO085 sensor data into bytes for SPI transmission to MCU
 * 
 * Data Format (per sensor):
 * - Quaternion: w, x, y, z (4x 16-bit signed = 8 bytes)
 * - Gyroscope: x, y, z (3x 16-bit signed = 6 bytes)
 * - Valid flags: 1 byte (bit 0=quat_valid, bit 1=gyro_valid)
 * 
 * Total: 2 sensors * (8 + 6 + 1) = 30 bytes + 2 button bytes = 32 bytes
 */

`timescale 1ns / 1ps

module sensor_data_packer (
    input  logic        clk,
    input  logic        rst_n,
    
    // Sensor 1 (Right Hand) inputs
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // Sensor 2 (Left Hand) inputs
    input  logic        quat2_valid,
    input  logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z,
    input  logic        gyro2_valid,
    input  logic signed [15:0] gyro2_x, gyro2_y, gyro2_z,
    
    // Button inputs (raw button states, not debounced - MCU will handle debouncing)
    input  logic        calibrate_btn_n,  // Raw button state (active low)
    input  logic        kick_btn_n,        // Raw button state (active low)
    
    // Packed data output (32 bytes total)
    output logic [7:0] data_bytes [0:31],
    output logic        data_ready,  // New data available
    input  logic        data_ack     // Data has been sent
);

    // Internal registers to latch sensor data
    logic signed [15:0] quat1_w_reg, quat1_x_reg, quat1_y_reg, quat1_z_reg;
    logic signed [15:0] gyro1_x_reg, gyro1_y_reg, gyro1_z_reg;
    logic               quat1_valid_reg, gyro1_valid_reg;
    
    logic signed [15:0] quat2_w_reg, quat2_x_reg, quat2_y_reg, quat2_z_reg;
    logic signed [15:0] gyro2_x_reg, gyro2_y_reg, gyro2_z_reg;
    logic               quat2_valid_reg, gyro2_valid_reg;
    
    // No need to register buttons - just pass raw state to MCU
    
    // Latch sensor data when valid
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quat1_w_reg <= 16'sd0;
            quat1_x_reg <= 16'sd0;
            quat1_y_reg <= 16'sd0;
            quat1_z_reg <= 16'sd0;
            gyro1_x_reg <= 16'sd0;
            gyro1_y_reg <= 16'sd0;
            gyro1_z_reg <= 16'sd0;
            quat1_valid_reg <= 1'b0;
            gyro1_valid_reg <= 1'b0;
            
            quat2_w_reg <= 16'sd0;
            quat2_x_reg <= 16'sd0;
            quat2_y_reg <= 16'sd0;
            quat2_z_reg <= 16'sd0;
            gyro2_x_reg <= 16'sd0;
            gyro2_y_reg <= 16'sd0;
            gyro2_z_reg <= 16'sd0;
            quat2_valid_reg <= 1'b0;
            gyro2_valid_reg <= 1'b0;
            
            calibrate_btn_reg <= 1'b0;
            kick_btn_reg <= 1'b0;
            data_ready <= 1'b0;
        end else begin
            // Latch quaternion data when valid
            if (quat1_valid) begin
                quat1_w_reg <= quat1_w;
                quat1_x_reg <= quat1_x;
                quat1_y_reg <= quat1_y;
                quat1_z_reg <= quat1_z;
                quat1_valid_reg <= 1'b1;
            end
            
            if (quat2_valid) begin
                quat2_w_reg <= quat2_w;
                quat2_x_reg <= quat2_x;
                quat2_y_reg <= quat2_y;
                quat2_z_reg <= quat2_z;
                quat2_valid_reg <= 1'b1;
            end
            
            // Latch gyro data when valid
            if (gyro1_valid) begin
                gyro1_x_reg <= gyro1_x;
                gyro1_y_reg <= gyro1_y;
                gyro1_z_reg <= gyro1_z;
                gyro1_valid_reg <= 1'b1;
            end
            
            if (gyro2_valid) begin
                gyro2_x_reg <= gyro2_x;
                gyro2_y_reg <= gyro2_y;
                gyro2_z_reg <= gyro2_z;
                gyro2_valid_reg <= 1'b1;
            end
            
            // Update data_ready when new sensor data arrives
            // Buttons are always included in packet, no need to trigger on button change
            if (quat1_valid || quat2_valid || gyro1_valid || gyro2_valid) begin
                data_ready <= 1'b1;
            end else if (data_ack) begin
                data_ready <= 1'b0;
            end
        end
    end
    
    // Pack data into bytes (combinational)
    // Byte order: Sensor 1 quat (8 bytes), Sensor 1 gyro (6 bytes), Sensor 1 flags (1 byte),
    //             Sensor 2 quat (8 bytes), Sensor 2 gyro (6 bytes), Sensor 2 flags (1 byte),
    //             Buttons (2 bytes)
    // Using generate to avoid iverilog constant select issues
    genvar i;
    generate
        // Sensor 1 Quaternion (8 bytes: w, x, y, z - MSB first)
        assign data_bytes[0] = quat1_w_reg[15:8];  // W MSB
        assign data_bytes[1] = quat1_w_reg[7:0];   // W LSB
        assign data_bytes[2] = quat1_x_reg[15:8];  // X MSB
        assign data_bytes[3] = quat1_x_reg[7:0];   // X LSB
        assign data_bytes[4] = quat1_y_reg[15:8];  // Y MSB
        assign data_bytes[5] = quat1_y_reg[7:0];   // Y LSB
        assign data_bytes[6] = quat1_z_reg[15:8];  // Z MSB
        assign data_bytes[7] = quat1_z_reg[7:0];   // Z LSB
        
        // Sensor 1 Gyroscope (6 bytes: x, y, z - MSB first)
        assign data_bytes[8]  = gyro1_x_reg[15:8];  // X MSB
        assign data_bytes[9]  = gyro1_x_reg[7:0];   // X LSB
        assign data_bytes[10] = gyro1_y_reg[15:8];  // Y MSB
        assign data_bytes[11] = gyro1_y_reg[7:0];   // Y LSB
        assign data_bytes[12] = gyro1_z_reg[15:8];  // Z MSB
        assign data_bytes[13] = gyro1_z_reg[7:0];   // Z LSB
        
        // Sensor 1 Flags (1 byte)
        assign data_bytes[14] = {6'b0, gyro1_valid_reg, quat1_valid_reg};
        
        // Sensor 2 Quaternion (8 bytes: w, x, y, z - MSB first)
        assign data_bytes[15] = quat2_w_reg[15:8];  // W MSB
        assign data_bytes[16] = quat2_w_reg[7:0];   // W LSB
        assign data_bytes[17] = quat2_x_reg[15:8];  // X MSB
        assign data_bytes[18] = quat2_x_reg[7:0];   // X LSB
        assign data_bytes[19] = quat2_y_reg[15:8];  // Y MSB
        assign data_bytes[20] = quat2_y_reg[7:0];   // Y LSB
        assign data_bytes[21] = quat2_z_reg[15:8];  // Z MSB
        assign data_bytes[22] = quat2_z_reg[7:0];   // Z LSB
        
        // Sensor 2 Gyroscope (6 bytes: x, y, z - MSB first)
        assign data_bytes[23] = gyro2_x_reg[15:8];  // X MSB
        assign data_bytes[24] = gyro2_x_reg[7:0];   // X LSB
        assign data_bytes[25] = gyro2_y_reg[15:8];  // Y MSB
        assign data_bytes[26] = gyro2_y_reg[7:0];   // Y LSB
        assign data_bytes[27] = gyro2_z_reg[15:8];  // Z MSB
        assign data_bytes[28] = gyro2_z_reg[7:0];   // Z LSB
        
        // Sensor 2 Flags (1 byte)
        assign data_bytes[29] = {6'b0, gyro2_valid_reg, quat2_valid_reg};
        
        // Buttons (2 bytes) - send raw button states (active low, so invert)
        // MCU will handle debouncing and processing
        assign data_bytes[30] = {7'b0, !kick_btn_n};        // 1 when button pressed
        assign data_bytes[31] = {7'b0, !calibrate_btn_n};   // 1 when button pressed
    endgenerate
    
endmodule

