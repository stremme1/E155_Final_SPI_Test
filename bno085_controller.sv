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

    // SHTP Protocol constants
    localparam [7:0] CHANNEL_REPORTS = 8'h05;
    localparam [7:0] REPORT_ID_ROTATION_VECTOR = 8'h05;
    localparam [7:0] REPORT_ID_GYROSCOPE = 8'h01;
    
    typedef enum logic [3:0] {
        IDLE,
        INIT_WAIT_RESET,
        INIT_WAKE,
        INIT_WAIT_INT,
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
    
    // Handshake state for SPI
    logic [1:0] spi_handshake;
    
    // Temporary storage for parsing
    logic [7:0] temp_byte_lsb;
    
    // INT pin handling
    logic int_n_sync, int_n_prev;
    
    // ========================================================================
    // Initialization Command ROM
    // ========================================================================
    function [7:0] get_init_byte(input [1:0] cmd, input [7:0] idx);
        case (cmd)
            // Product ID Request (5 bytes): 04 00 00 00 F9
            2'd0: begin
                case (idx)
                    0: get_init_byte = 8'h04;
                    1: get_init_byte = 8'h00;
                    2: get_init_byte = 8'h00;
                    3: get_init_byte = 8'h00; // Seq 0
                    4: get_init_byte = 8'hF9;
                    default: get_init_byte = 8'h00;
                endcase
            end
            // Enable Rotation Vector (17 bytes)
            // 17 00 02 01 FD 05 00 00 00 32 00 00 00 00 00 00 00
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
                    9: get_init_byte = 8'h32; // Report Interval LSB (50)
                    10: get_init_byte = 8'h00;
                    11: get_init_byte = 8'h00;
                    12: get_init_byte = 8'h00; // Report Interval MSB
                    default: get_init_byte = 8'h00;
                endcase
            end
            // Enable Gyroscope (17 bytes)
            // 17 00 02 02 FD 01 00 00 00 32 00 00 00 00 00 00 00
            2'd2: begin
                case (idx)
                    0: get_init_byte = 8'd17;
                    1: get_init_byte = 8'h00;
                    2: get_init_byte = 8'h02; // Channel Control
                    3: get_init_byte = 8'd2;  // Seq 2
                    4: get_init_byte = 8'hFD; // Set Feature
                    5: get_init_byte = 8'h01; // Report ID (Gyro)
                    6: get_init_byte = 8'h00; // Flags
                    7: get_init_byte = 8'h00; // Sensitivity LSB
                    8: get_init_byte = 8'h00; // Sensitivity MSB
                    9: get_init_byte = 8'h32; // Report Interval LSB (50)
                    10: get_init_byte = 8'h00;
                    11: get_init_byte = 8'h00;
                    12: get_init_byte = 8'h00; // Report Interval MSB
                    default: get_init_byte = 8'h00;
                endcase
            end
            default: get_init_byte = 8'h00;
        endcase
    endfunction
    
    // Command lengths
    function [7:0] get_cmd_len(input [1:0] cmd);
        case (cmd)
            2'd0: get_cmd_len = 8'd5;
            2'd1: get_cmd_len = 8'd17;
            2'd2: get_cmd_len = 8'd17;
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
            spi_handshake <= 2'd0;
            current_report_id <= 8'd0;
            temp_byte_lsb <= 8'd0;
            cmd_select <= 2'd0;
            
            quat_w <= 16'd0;
            quat_x <= 16'd0;
            quat_y <= 16'd0;
            quat_z <= 16'd0;
            gyro_x <= 16'd0;
            gyro_y <= 16'd0;
            gyro_z <= 16'd0;
            
        end else begin
            // Default assignments
            if (spi_handshake == 0 || spi_handshake == 2) begin
                spi_start <= 1'b0;
                spi_tx_valid <= 1'b0;
            end
            
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

                // 2. Wake the sensor (PS0 Low)
                INIT_WAKE: begin
                    ps0_wake <= 1'b0; // Drive Low to wake
                    delay_counter <= 19'd0;
                    // If INT is already low, sensor is awake, proceed
                    if (!int_n_sync) begin
                        state <= INIT_WAIT_INT;
                    end else begin
                        // Otherwise wait for INT to go low
                        // Add timeout? For now just wait.
                        state <= INIT_WAIT_INT;
                    end
                end

                // 3. Wait for INT low (Sensor Ready)
                INIT_WAIT_INT: begin
                    if (!int_n_sync) begin
                        cs_n <= 1'b0; // Select chip
                        ps0_wake <= 1'b1; // Release Wake (optional, but good practice once CS is asserted)
                        byte_cnt <= 8'd0;
                        spi_handshake <= 0;
                        state <= INIT_SEND_BODY;
                    end
                end
                
                // 4. Send Command Body
                INIT_SEND_BODY: begin
                    cs_n <= 1'b0;
                    
                    if (spi_handshake == 0) begin
                        if (!spi_busy && spi_tx_ready) begin
                            if (byte_cnt < get_cmd_len(cmd_select)) begin
                                spi_tx_data <= get_init_byte(cmd_select, byte_cnt);
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                                spi_handshake <= 1;
                            end else begin
                                // Done sending
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                state <= INIT_DONE_CHECK;
                                delay_counter <= 19'd0;
                            end
                        end
                    end else if (spi_handshake == 1) begin
                        spi_start <= 1'b1; // Hold start
                        spi_tx_valid <= 1'b1;
                        if (spi_busy) begin
                            spi_start <= 1'b0;
                            spi_handshake <= 2;
                        end
                    end else if (spi_handshake == 2) begin
                        if (!spi_busy) begin
                            byte_cnt <= byte_cnt + 1;
                            spi_handshake <= 0;
                        end
                    end
                end
                
                // 5. Check if more commands or done
                INIT_DONE_CHECK: begin
                    cs_n <= 1'b1;
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
                    cs_n <= 1'b0;
                    if (!int_n_sync) begin
                        state <= READ_HEADER;
                        spi_tx_data <= 8'h00; 
                        spi_tx_valid <= 1'b1; 
                        spi_start <= 1'b1;
                        byte_cnt <= 8'd0;
                        spi_handshake <= 0; // Re-use handshake logic if needed, or rely on master auto-seq
                    end
                end
                
                READ_HEADER: begin
                    if (spi_rx_valid && !spi_busy) begin
                        case (byte_cnt)
                            0: begin
                                packet_length[7:0] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; spi_tx_valid <= 1; spi_start <= 1;
                            end
                            1: begin
                                packet_length[15:8] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; spi_tx_valid <= 1; spi_start <= 1;
                            end
                            2: begin
                                channel <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; spi_tx_valid <= 1; spi_start <= 1;
                            end
                            3: begin
                                byte_cnt <= 8'd0;
                                if (packet_length > 4) begin
                                    state <= READ_PAYLOAD;
                                    spi_tx_data <= 8'h00; spi_tx_valid <= 1; spi_start <= 1;
                                end else begin
                                    cs_n <= 1'b1; 
                                    state <= WAIT_DATA;
                                end
                            end
                        endcase
                    end
                end
                
                READ_PAYLOAD: begin
                    if (spi_rx_valid && !spi_busy) begin
                        if (byte_cnt < packet_length - 4 && byte_cnt < 64) begin
                            // Parse on the fly
                            if (channel == CHANNEL_REPORTS) begin
                                if (byte_cnt == 0) begin
                                    current_report_id <= spi_rx_data;
                                end else if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                    case (byte_cnt)
                                        4: temp_byte_lsb <= spi_rx_data;
                                        5: quat_x <= {spi_rx_data, temp_byte_lsb};
                                        6: temp_byte_lsb <= spi_rx_data;
                                        7: quat_y <= {spi_rx_data, temp_byte_lsb};
                                        8: temp_byte_lsb <= spi_rx_data;
                                        9: quat_z <= {spi_rx_data, temp_byte_lsb};
                                        10: temp_byte_lsb <= spi_rx_data;
                                        11: begin
                                            quat_w <= {spi_rx_data, temp_byte_lsb};
                                            quat_valid <= 1'b1; 
                                        end
                                    endcase
                                end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                    case (byte_cnt)
                                        4: temp_byte_lsb <= spi_rx_data;
                                        5: gyro_x <= {spi_rx_data, temp_byte_lsb};
                                        6: temp_byte_lsb <= spi_rx_data;
                                        7: gyro_y <= {spi_rx_data, temp_byte_lsb};
                                        8: temp_byte_lsb <= spi_rx_data;
                                        9: begin
                                            gyro_z <= {spi_rx_data, temp_byte_lsb};
                                            gyro_valid <= 1'b1;
                                        end
                                    endcase
                                end
                            end

                            byte_cnt <= byte_cnt + 1;
                            
                            if (byte_cnt < packet_length - 5 && byte_cnt < 63) begin
                                spi_tx_data <= 8'h00; spi_tx_valid <= 1; spi_start <= 1;
                            end else begin
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                state <= WAIT_DATA;
                            end
                        end else begin
                            cs_n <= 1'b1;
                            byte_cnt <= 8'd0;
                            state <= WAIT_DATA;
                        end
                    end
                end
                
                ERROR_STATE: begin
                    error <= 1'b1;
                    cs_n <= 1'b1;
                    state <= ERROR_STATE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
endmodule
