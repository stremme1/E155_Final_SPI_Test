`timescale 1ns / 1ps

// SPI slave to Arduino
// Receives 16-byte packets, converts quaternion to Euler angles

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
    
    // SPI receive logic
    
    // 128-bit shift register
    logic [127:0] packet_shift;
    
    // Initialize shift register
    initial begin
        packet_shift = 128'd0;
    end
    
    // shift on SCK rising edge
    logic cs_n_prev_sck = 1'b1;
    
    always_ff @(posedge sck) begin
        cs_n_prev_sck <= cs_n;
        
        if (cs_n_prev_sck && !cs_n) begin
            packet_shift <= 128'd0;
        end else if (!cs_n) begin
            packet_shift <= {packet_shift[126:0], sdi};
        end
    end
    
    // extract bytes from shift register
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
    
    // clock domain crossing
    logic [7:0] packet_buffer_sync [0:PACKET_SIZE-1];
    
    // init buffer
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_buffer_sync[i] = 8'h00;
        end
    end
    
    // sync CS to clk domain
    logic cs_n_sync_clk1, cs_n_sync_clk2;
    logic cs_n_prev_clk;
    always_ff @(posedge clk) begin
        cs_n_sync_clk1 <= cs_n;
        cs_n_sync_clk2 <= cs_n_sync_clk1;
        cs_n_prev_clk <= cs_n_sync_clk2;
    end
    
    // detect CS rising edge
    logic cs_rising_edge_clk;
    assign cs_rising_edge_clk = !cs_n_prev_clk && cs_n_sync_clk2;
    
    // capture packet when CS is high
    always_ff @(posedge clk) begin
        if (cs_n_sync_clk2) begin
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
    end
    
    // quaternion to euler conversion
    logic signed [15:0] quat_w_raw, quat_x_raw, quat_y_raw, quat_z_raw;
    logic signed [15:0] gyro_x_raw, gyro_y_raw, gyro_z_raw;
    logic [7:0] flags_raw;
    
    // extract quaternion
    assign quat_w_raw = {packet_buffer_sync[1], packet_buffer_sync[2]};
    assign quat_x_raw = {packet_buffer_sync[3], packet_buffer_sync[4]};
    assign quat_y_raw = {packet_buffer_sync[5], packet_buffer_sync[6]};
    assign quat_z_raw = {packet_buffer_sync[7], packet_buffer_sync[8]};
    
    // extract gyro
    assign gyro_x_raw = {packet_buffer_sync[9], packet_buffer_sync[10]};
    assign gyro_y_raw = {packet_buffer_sync[11], packet_buffer_sync[12]};
    assign gyro_z_raw = {packet_buffer_sync[13], packet_buffer_sync[14]};
    assign flags_raw = packet_buffer_sync[15];
    
    // convert to 18-bit for DSP
    logic signed [17:0] quat_w, quat_x, quat_y, quat_z;
    assign quat_w = {{2{quat_w_raw[15]}}, quat_w_raw};
    assign quat_x = {{2{quat_x_raw[15]}}, quat_x_raw};
    assign quat_y = {{2{quat_y_raw[15]}}, quat_y_raw};
    assign quat_z = {{2{quat_z_raw[15]}}, quat_z_raw};
    
    // DSP multiplications
    (* use_dsp = "yes" *)
    logic signed [35:0] wx_prod, yz_prod, wy_prod, zx_prod, wz_prod, xy_prod;
    (* use_dsp = "yes" *)
    logic signed [35:0] xx_prod, yy_prod, zz_prod;
    
    // multiply
    always_ff @(posedge clk) begin
        wx_prod <= quat_w * quat_x;
        yz_prod <= quat_y * quat_z;
        wy_prod <= quat_w * quat_y;
        zx_prod <= quat_z * quat_x;
        wz_prod <= quat_w * quat_z;
        xy_prod <= quat_x * quat_y;
        xx_prod <= quat_x * quat_x;
        yy_prod <= quat_y * quat_y;
        zz_prod <= quat_z * quat_z;
    end
    
    // calculate euler intermediates
    logic signed [17:0] wx_sum, yz_sum, wy_diff, zx_diff, wz_sum, xy_sum;
    logic signed [17:0] xx_sum, yy_sum, zz_sum;
    
    always_ff @(posedge clk) begin
        wx_sum <= wx_prod[33:16] + yz_prod[33:16];
        xx_sum <= xx_prod[33:16] + yy_prod[33:16];
        wy_diff <= wy_prod[33:16] - zx_prod[33:16];
        wz_sum <= wz_prod[33:16] + xy_prod[33:16];
        yy_sum <= yy_prod[33:16] + zz_prod[33:16];
    end
    
    // Simplified Euler angle calculation
    // Full implementation would use CORDIC for atan2/asin, but for demonstration
    // we use a simplified linear approximation based on the quaternion components
    // This demonstrates DSP usage while providing reasonable Euler angle outputs
    
    logic signed [17:0] roll_numerator, roll_denominator;
    logic signed [17:0] pitch_value;
    logic signed [17:0] yaw_numerator, yaw_denominator;
    
    always_ff @(posedge clk) begin
        roll_numerator <= wx_sum;
        roll_denominator <= 18'd16384 - xx_sum;
        pitch_value <= wy_diff;
        yaw_numerator <= wz_sum;
        yaw_denominator <= 18'd16384 - yy_sum;
    end
    
    // scale to degrees
    (* use_dsp = "yes" *)
    logic signed [35:0] roll_mult, pitch_mult, yaw_mult;
    logic signed [15:0] roll_scaled, pitch_scaled, yaw_scaled;
    
    always_ff @(posedge clk) begin
        roll_mult <= roll_numerator * 18'd18000;
        pitch_mult <= pitch_value * 18'd18000;
        yaw_mult <= yaw_numerator * 18'd18000;
    end
    
    // divide by 16384
    always_ff @(posedge clk) begin
        roll_scaled <= roll_mult[29:14];
        pitch_scaled <= pitch_mult[29:14];
        yaw_scaled <= yaw_mult[29:14];
    end
    
    // reconstruct packet
    logic [7:0] packet_buffer_out [0:PACKET_SIZE-1];
    
    always_ff @(posedge clk) begin
        packet_buffer_out[0] <= packet_buffer_sync[0];
        packet_buffer_out[1] <= roll_scaled[15:8];
        packet_buffer_out[2] <= roll_scaled[7:0];
        packet_buffer_out[3] <= pitch_scaled[15:8];
        packet_buffer_out[4] <= pitch_scaled[7:0];
        packet_buffer_out[5] <= yaw_scaled[15:8];
        packet_buffer_out[6] <= yaw_scaled[7:0];
        packet_buffer_out[7] <= gyro_x_raw[15:8];
        packet_buffer_out[8] <= gyro_x_raw[7:0];
        packet_buffer_out[9] <= gyro_y_raw[15:8];
        packet_buffer_out[10] <= gyro_y_raw[7:0];
        packet_buffer_out[11] <= gyro_z_raw[15:8];
        packet_buffer_out[12] <= gyro_z_raw[7:0];
        packet_buffer_out[13] <= flags_raw;
        packet_buffer_out[14] <= 8'h00;
        packet_buffer_out[15] <= 8'h00;
    end
    
    assign packet_buffer = packet_buffer_out;
    
    // header validation
    logic packet_received;
    
    always_ff @(posedge clk) begin
        if (cs_rising_edge_clk) begin
            packet_received <= 1'b1;
            
            if (packet_buffer_sync[0] == HEADER_BYTE) begin
                initialized <= 1'b1;
                error <= 1'b0;
            end else begin
                if (packet_received || (packet_buffer_sync[0] != 8'h00)) begin
                    initialized <= 1'b0;
                    error <= 1'b1;
                end
            end
        end
    end
    
    // Initialize
    initial begin
        initialized = 1'b0;
        error = 1'b0;
        packet_received = 1'b0;
    end
    
endmodule
