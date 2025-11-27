`timescale 1ns / 1ps

// BNO085 Controller Module
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Reads Rotation Vector (quaternion) and Gyroscope reports
// Optimized for low resource usage (ROM-based initialization, no large buffers)

module bno085_controller (
    input  logic        clk,
    input  logic        rst_n,

    // SPI interface
    output logic        spi_start,
    output logic        spi_tx_valid,
    output logic [7:0]  spi_tx_data,
    input  logic        spi_tx_ready,
    input  logic        spi_rx_valid,
    input  logic [7:0]  spi_rx_data,
    input  logic        spi_busy,
    output logic        cs_n,     // Chip Select (active low)
    output logic        ps0_wake, // PS0/WAKE (active low)

    // INT pin (REQUIRED for stable SPI operation per Adafruit documentation)
    input  logic        int_n,  // Active LOW interrupt - goes LOW when data ready

    // Sensor data outputs
    output logic        quat_valid,
    output logic signed [15:0] quat_w,
    output logic signed [15:0] quat_x,
    output logic signed [15:0] quat_y,
    output logic signed [15:0] quat_z,

    output logic        gyro_valid,
    output logic signed [15:0] gyro_x,
    output logic signed [15:0] gyro_y,
    output logic signed [15:0] gyro_z,

    // Status
    output logic        initialized,
    output logic        error
);

    // SHTP Protocol constants (per datasheet Section 1.3.1)
    localparam [7:0] CHANNEL_COMMAND = 8'h00;      // SHTP command channel
    localparam [7:0] CHANNEL_EXECUTABLE = 8'h01;    // Executable channel
    localparam [7:0] CHANNEL_CONTROL = 8'h02;       // Sensor hub control channel
    localparam [7:0] CHANNEL_REPORTS = 8'h03;       // Input sensor reports (non-wake, not gyroRV)
    localparam [7:0] CHANNEL_WAKE_REPORTS = 8'h04; // Wake input sensor reports
    localparam [7:0] CHANNEL_GYRO_RV = 8'h05;       // Gyro rotation vector
    
    // Report IDs (per datasheet Section 1.3.2, Figure 1-34)
    localparam [7:0] REPORT_ID_ROTATION_VECTOR = 8'h05;
    localparam [7:0] REPORT_ID_GYROSCOPE = 8'h02;  // Fixed: Calibrated gyroscope per Fig 1-34
    
    typedef enum logic [3:0] {
        IDLE,
        INIT_WAIT_RESET,
        INIT_WAKE,
        INIT_WAIT_INT,
        INIT_CS_SETUP,      // Added: CS setup before SPI transaction
        INIT_SEND_BODY,
        INIT_DONE_CHECK,
        WAIT_DATA,
        READ_HEADER_START,
        READ_HEADER,
        READ_PAYLOAD,
        ERROR_STATE
    } state_t;
    
    state_t state;
    logic [7:0] byte_cnt;
    logic [15:0] packet_length;
    logic [7:0] channel;
    logic [7:0] current_report_id;
    
    // Reduced counter width (19 bits is enough for >300,000 counts)
    logic [18:0] delay_counter;
    
    // Command selection
    logic [1:0] cmd_select; // 0=ProdID, 1=Rot, 2=Gyro
    
    // Temporary storage for parsing (little-endian: LSB first)
    logic [7:0] temp_byte_lsb;
    
    // INT pin handling
    logic int_n_sync, int_n_prev;
    
    // Sequence number tracking (per datasheet 1.3.5.2)
    logic [7:0] last_seq_num;
    
    // Status and delay fields
    logic [7:0] report_status;
    logic [7:0] report_delay;
    
    // ========================================================================
    // Initialization Command ROM
    // ========================================================================
    function [7:0] get_init_byte(input [1:0] cmd, input [7:0] idx);
        case (cmd)
            // Product ID Request (5 bytes): 04 00 02 00 F9
            // SHTP Header: Length=5, Channel=2 (SH-2 control), Seq=0
            // Payload: Report ID 0xF9
            2'd0: begin
                case (idx)
                    0: get_init_byte = 8'h05; // Length LSB (5 bytes total)
                    1: get_init_byte = 8'h00; // Length MSB
                    2: get_init_byte = 8'h02; // Channel 2 (SH-2 control per datasheet Fig 1-30)
                    3: get_init_byte = 8'h00; // Seq 0
                    4: get_init_byte = 8'hF9; // Report ID (Product ID Request)
                    default: get_init_byte = 8'h00;
                endcase
            end
            // Enable Rotation Vector (17 bytes)
            // Report interval: 0x00004E20 = 20,000 µs = 50 Hz (within 400 Hz max per datasheet 6.9)
            2'd1: begin
                case (idx)
                    0: get_init_byte = 8'd17;
                    1: get_init_byte = 8'h00;
                    2: get_init_byte = 8'h02; // Channel Control
                    3: get_init_byte = 8'd1;  // Seq 1
                    4: get_init_byte = 8'hFD; // Set Feature
                    5: get_init_byte = 8'h05; // Report ID (Rot Vec)
                    6: get_init_byte = 8'h00; // Flags
                    7: get_init_byte = 8'h00; // Sensitivity LSB
                    8: get_init_byte = 8'h00; // Sensitivity MSB
                    9: get_init_byte = 8'h20; // Report Interval LSB (20,000 µs = 50 Hz)
                    10: get_init_byte = 8'h4E;
                    11: get_init_byte = 8'h00;
                    12: get_init_byte = 8'h00; // Report Interval MSB
                    default: get_init_byte = 8'h00;
                endcase
            end
            // Enable Gyroscope (17 bytes)
            // Report interval: 0x00004E20 = 20,000 µs = 50 Hz (within 400 Hz max per datasheet 6.9)
            2'd2: begin
                case (idx)
                    0: get_init_byte = 8'd17;
                    1: get_init_byte = 8'h00;
                    2: get_init_byte = 8'h02; // Channel Control
                    3: get_init_byte = 8'd2;  // Seq 2
                    4: get_init_byte = 8'hFD; // Set Feature
                    5: get_init_byte = 8'h02; // Report ID (Calibrated Gyro per Fig 1-34)
                    6: get_init_byte = 8'h00; // Flags
                    7: get_init_byte = 8'h00; // Sensitivity LSB
                    8: get_init_byte = 8'h00; // Sensitivity MSB
                    9: get_init_byte = 8'h20; // Report Interval LSB (20,000 µs = 50 Hz)
                    10: get_init_byte = 8'h4E;
                    11: get_init_byte = 8'h00;
                    12: get_init_byte = 8'h00; // Report Interval MSB
                    default: get_init_byte = 8'h00;
                endcase
            end
            default: get_init_byte = 8'h00;
        endcase
    endfunction
    
    // Command lengths (including SHTP header)
    function [7:0] get_cmd_len(input [1:0] cmd);
        case (cmd)
            2'd0: get_cmd_len = 8'd5;  // Product ID: 4 header + 1 payload
            2'd1: get_cmd_len = 8'd17; // Set Feature: 4 header + 13 payload
            2'd2: get_cmd_len = 8'd17; // Set Feature: 4 header + 13 payload
            default: get_cmd_len = 8'd0;
        endcase
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_n_sync <= 1'b1;
            int_n_prev <= 1'b1;
        end else begin
            int_n_sync <= int_n;
            int_n_prev <= int_n_sync;
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= INIT_WAIT_RESET;
            initialized <= 1'b0;
            error <= 1'b0;
            delay_counter <= 19'd0;
            spi_start <= 1'b0;
            spi_tx_valid <= 1'b0;
            byte_cnt <= 8'd0;
            cs_n <= 1'b1;
            ps0_wake <= 1'b1; // Must be high at reset for SPI mode
            quat_valid <= 1'b0;
            gyro_valid <= 1'b0;
            current_report_id <= 8'd0;
            temp_byte_lsb <= 8'd0;
            cmd_select <= 2'd0;
            last_seq_num <= 8'd0;
            report_status <= 8'd0;
            report_delay <= 8'd0;
            
            quat_w <= 16'd0;
            quat_x <= 16'd0;
            quat_y <= 16'd0;
            quat_z <= 16'd0;
            gyro_x <= 16'd0;
            gyro_y <= 16'd0;
            gyro_z <= 16'd0;
            
        end else begin
            // Default assignments
            spi_start <= 1'b0;
            spi_tx_valid <= 1'b0;
            quat_valid <= 1'b0;
            gyro_valid <= 1'b0;
            
            case (state)
                // 1. Wait after reset to ensure sensor is ready (~100ms)
                INIT_WAIT_RESET: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    if (delay_counter < 19'd300_000) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        state <= INIT_WAKE;
                        delay_counter <= 19'd0;
                        cmd_select <= 2'd0; // Start with ProdID
                    end
                end

                // 2. Wake the sensor (PS0 Low) - per datasheet 1.2.4.3
                INIT_WAKE: begin
                    ps0_wake <= 1'b0; // Drive Low to wake
                    delay_counter <= 19'd0;
                    state <= INIT_WAIT_INT;
                end

                // 3. Wait for INT low (Sensor Ready) with timeout - per datasheet 6.5.4 (twk = 150 µs max)
                INIT_WAIT_INT: begin
                    if (!int_n_sync) begin
                        // INT asserted, sensor is ready
                        delay_counter <= 19'd0;
                        state <= INIT_CS_SETUP; // Setup CS before starting SPI
                    end else if (delay_counter >= 19'd600) begin
                        // Timeout: 600 cycles @ 3MHz = 200 µs (exceeds 150 µs max)
                        state <= ERROR_STATE;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                // CS setup before SPI transaction - per datasheet 6.5.2 (tcssu = 0.1 µs min)
                INIT_CS_SETUP: begin
                    cs_n <= 1'b0; // Assert CS
                    ps0_wake <= 1'b1; // Release Wake once CS is asserted
                    delay_counter <= 19'd0;
                    byte_cnt <= 8'd0;
                    state <= INIT_SEND_BODY;
                end
                
                // 4. Send Command Body (simplified SPI handshake)
                INIT_SEND_BODY: begin
                    cs_n <= 1'b0;
                    
                    if (byte_cnt < get_cmd_len(cmd_select)) begin
                        if (!spi_busy && spi_tx_ready) begin
                            // Start new byte transfer
                            spi_tx_data <= get_init_byte(cmd_select, byte_cnt);
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                        end else if (spi_busy) begin
                            // Transfer in progress, release start after first cycle
                            spi_start <= 1'b0;
                        end else if (!spi_busy && spi_rx_valid) begin
                            // Transfer complete, advance to next byte
                            byte_cnt <= byte_cnt + 1;
                            spi_tx_valid <= 1'b0;
                        end
                    end else begin
                        // Done sending all bytes
                        cs_n <= 1'b1;
                        byte_cnt <= 8'd0;
                        state <= INIT_DONE_CHECK;
                        delay_counter <= 19'd0;
                    end
                end
                
                // 5. Check if more commands or done
                INIT_DONE_CHECK: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Release PS0 before next command
                    // Delay 10ms between commands
                    if (delay_counter < 19'd30_000) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        if (cmd_select < 2'd2) begin
                            cmd_select <= cmd_select + 1;
                            state <= INIT_WAKE; // Go back to Wake for next command
                        end else begin
                            initialized <= 1'b1;
                            state <= WAIT_DATA;
                        end
                    end
                end
                
                // 6. Normal Operation: Wait for Data Ready
                WAIT_DATA: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure wake is released
                    if (!int_n_sync) begin
                        state <= READ_HEADER_START;
                        byte_cnt <= 8'd0;
                    end
                end

                READ_HEADER_START: begin
                    // CS setup before SPI - per datasheet 6.5.2
                    cs_n <= 1'b0;
                    if (!int_n_sync) begin
                        // Start first SPI transaction to read header
                        spi_tx_data <= 8'h00; 
                        spi_tx_valid <= 1'b1; 
                        spi_start <= 1'b1;
                        byte_cnt <= 8'd0;
                        state <= READ_HEADER;
                    end else begin
                        // INT deasserted, no data
                        cs_n <= 1'b1;
                        state <= WAIT_DATA;
                    end
                end
                
                READ_HEADER: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid && !spi_busy) begin
                        case (byte_cnt)
                            0: begin
                                packet_length[7:0] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            1: begin
                                // Mask continuation bit (bit 15) per datasheet 1.3.1
                                packet_length[15:8] <= spi_rx_data;
                                packet_length[15] <= 1'b0; // Clear continuation bit
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            2: begin
                                channel <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            3: begin
                                // Validate packet length (max 32766 per datasheet 1.3.1)
                                if (packet_length > 16'd4 && packet_length < 16'd32767) begin
                                    byte_cnt <= 8'd0;
                                    state <= READ_PAYLOAD;
                                    spi_tx_data <= 8'h00; 
                                    spi_tx_valid <= 1'b1; 
                                    spi_start <= 1'b1;
                                end else begin
                                    // Invalid length
                                    cs_n <= 1'b1; 
                                    state <= WAIT_DATA;
                                end
                            end
                        endcase
                    end else if (!spi_busy && byte_cnt < 4) begin
                        // Continue reading header
                        spi_tx_data <= 8'h00; 
                        spi_tx_valid <= 1'b1; 
                        spi_start <= 1'b1;
                    end
                end
                
                READ_PAYLOAD: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid && !spi_busy) begin
                        // Parse on the fly - per datasheet 1.3.5.2
                        // Accept reports from Channel 3 (standard reports) or Channel 5 (gyro rotation vector)
                        if (channel == CHANNEL_REPORTS || channel == CHANNEL_GYRO_RV) begin
                            case (byte_cnt)
                                0: current_report_id <= spi_rx_data;
                                1: last_seq_num <= spi_rx_data; // Sequence number (for drop detection)
                                2: report_status <= spi_rx_data; // Status (accuracy in bits 1:0)
                                3: report_delay <= spi_rx_data;  // Delay (lower 8 bits)
                                4: begin
                                    // Rotation Vector: X-axis LSB (little-endian per datasheet 1.2.2.2)
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data; // Gyro X LSB
                                    end
                                end
                                5: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_x <= {spi_rx_data, temp_byte_lsb}; // X-axis MSB,LSB
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_x <= {spi_rx_data, temp_byte_lsb}; // X-axis MSB,LSB
                                    end
                                end
                                6: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data; // Y-axis LSB
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data; // Gyro Y LSB
                                    end
                                end
                                7: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_y <= {spi_rx_data, temp_byte_lsb}; // Y-axis MSB,LSB
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_y <= {spi_rx_data, temp_byte_lsb}; // Y-axis MSB,LSB
                                    end
                                end
                                8: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data; // Z-axis LSB
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data; // Gyro Z LSB
                                    end
                                end
                                9: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_z <= {spi_rx_data, temp_byte_lsb}; // Z-axis MSB,LSB
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_z <= {spi_rx_data, temp_byte_lsb}; // Z-axis MSB,LSB
                                        gyro_valid <= 1'b1; // Gyro complete
                                    end
                                end
                                10: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data; // W-axis LSB
                                    end
                                end
                                11: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_w <= {spi_rx_data, temp_byte_lsb}; // W-axis MSB,LSB
                                        quat_valid <= 1'b1; // Quaternion complete
                                    end
                                end
                            endcase
                        end

                        byte_cnt <= byte_cnt + 1;
                        
                        // Continue reading if more data
                        if (byte_cnt < (packet_length - 5)) begin
                            spi_tx_data <= 8'h00; 
                            spi_tx_valid <= 1'b1; 
                            spi_start <= 1'b1;
                        end else begin
                            // Packet complete
                            cs_n <= 1'b1;
                            byte_cnt <= 8'd0;
                            state <= WAIT_DATA;
                        end
                    end else if (!spi_busy && byte_cnt < (packet_length - 4)) begin
                        // Continue reading payload
                        spi_tx_data <= 8'h00; 
                        spi_tx_valid <= 1'b1; 
                        spi_start <= 1'b1;
                    end
                end
                
                ERROR_STATE: begin
                    error <= 1'b1;
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    // Could add recovery logic here (e.g., reset and retry)
                    state <= ERROR_STATE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
endmodule