`timescale 1ns / 1ps

// BNO085 Controller Module - SIMPLIFIED VERSION
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Reads Rotation Vector (quaternion) and Gyroscope reports
// Simplified state machine based on BNO085 datasheet requirements

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

    // INT pin (REQUIRED for stable SPI operation)
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
    localparam [7:0] CHANNEL_CONTROL = 8'h02;       // Sensor hub control channel
    localparam [7:0] CHANNEL_REPORTS = 8'h03;       // Input sensor reports
    localparam [7:0] CHANNEL_GYRO_RV = 8'h05;       // Gyro rotation vector
    
    // Report IDs
    localparam [7:0] REPORT_ID_ROTATION_VECTOR = 8'h05;
    localparam [7:0] REPORT_ID_GYROSCOPE = 8'h02;
    
    // Simplified state machine
    typedef enum logic [3:0] {
        ST_RESET,
        ST_WAKE,
        ST_WAIT_INT,
        ST_SEND_CMD,
        ST_READ_RESPONSE,
        ST_WAIT_DATA,
        ST_READ_HEADER,
        ST_READ_PAYLOAD,
        ST_ERROR
    } state_t;
    
    state_t state;
    
    // Counters and registers
    logic [7:0] byte_cnt;
    logic [15:0] packet_length;
    logic [7:0] channel;
    logic [7:0] current_report_id;
    logic [18:0] delay_counter;
    logic [1:0] cmd_select; // 0=ProdID, 1=RotVec, 2=Gyro
    logic [7:0] temp_byte_lsb;
    
    // INT synchronization
    logic int_n_sync;
    
    // Command ROM
    function [7:0] get_cmd_byte(input [1:0] cmd, input [7:0] idx);
        case (cmd)
            2'd0: begin // Product ID Request
                case (idx)
                    0: get_cmd_byte = 8'h05;
                    1: get_cmd_byte = 8'h00;
                    2: get_cmd_byte = 8'h02;
                    3: get_cmd_byte = 8'h00;
                    4: get_cmd_byte = 8'hF9;
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            2'd1: begin // Set Feature: Rotation Vector
                case (idx)
                    0: get_cmd_byte = 8'd17;
                    1: get_cmd_byte = 8'h00;
                    2: get_cmd_byte = 8'h02;
                    3: get_cmd_byte = 8'd1;
                    4: get_cmd_byte = 8'hFD;
                    5: get_cmd_byte = 8'h05; // Report ID for Rotation Vector
                    6: get_cmd_byte = 8'h00;
                    7: get_cmd_byte = 8'h00;
                    8: get_cmd_byte = 8'h00;
                    9: get_cmd_byte = 8'h20; // 20,000 µs = 50 Hz
                    10: get_cmd_byte = 8'h4E;
                    11: get_cmd_byte = 8'h00;
                    12: get_cmd_byte = 8'h00;
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            2'd2: begin // Set Feature: Gyroscope
                case (idx)
                    0: get_cmd_byte = 8'd17;
                    1: get_cmd_byte = 8'h00;
                    2: get_cmd_byte = 8'h02;
                    3: get_cmd_byte = 8'd2;
                    4: get_cmd_byte = 8'hFD;
                    5: get_cmd_byte = 8'h02; // Report ID for Gyroscope
                    6: get_cmd_byte = 8'h00;
                    7: get_cmd_byte = 8'h00;
                    8: get_cmd_byte = 8'h00;
                    9: get_cmd_byte = 8'h20; // 20,000 µs = 50 Hz
                    10: get_cmd_byte = 8'h4E;
                    11: get_cmd_byte = 8'h00;
                    12: get_cmd_byte = 8'h00;
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            default: get_cmd_byte = 8'h00;
        endcase
    endfunction
    
    function [7:0] get_cmd_len(input [1:0] cmd);
        case (cmd)
            2'd0: get_cmd_len = 8'd5;
            2'd1: get_cmd_len = 8'd17;
            2'd2: get_cmd_len = 8'd17;
            default: get_cmd_len = 8'd0;
        endcase
    endfunction

    // INT synchronization
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_n_sync <= 1'b1;
        end else begin
            int_n_sync <= int_n;
        end
    end
    
    // Main state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_RESET;
            initialized <= 1'b0;
            error <= 1'b0;
            delay_counter <= 19'd0;
            spi_start <= 1'b0;
            spi_tx_valid <= 1'b0;
            byte_cnt <= 8'd0;
            cs_n <= 1'b1;
            ps0_wake <= 1'b1;
            quat_valid <= 1'b0;
            gyro_valid <= 1'b0;
            current_report_id <= 8'd0;
            temp_byte_lsb <= 8'd0;
            cmd_select <= 2'd0;
            packet_length <= 16'd0;
            channel <= 8'd0;
            
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
                ST_RESET: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    if (delay_counter < 19'd300_000) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        cmd_select <= 2'd0;
                        state <= ST_WAKE;
                    end
                end
                
                ST_WAKE: begin
                    ps0_wake <= 1'b0; // Drive WAKE low
                    if (delay_counter < 19'd450) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        state <= ST_WAIT_INT;
                    end
                end
                
                ST_WAIT_INT: begin
                    ps0_wake <= 1'b1; // Release WAKE
                    if (!int_n_sync) begin
                        state <= ST_SEND_CMD;
                        byte_cnt <= 8'd0;
                        cs_n <= 1'b0;
                    end else if (delay_counter >= 19'd150_000) begin
                        state <= ST_ERROR;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                ST_SEND_CMD: begin
                    cs_n <= 1'b0;
                    if (byte_cnt < get_cmd_len(cmd_select)) begin
                        if (!spi_busy && spi_tx_ready) begin
                            spi_tx_data <= get_cmd_byte(cmd_select, byte_cnt);
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                        end else if (spi_rx_valid && !spi_busy) begin
                            // Byte sent and received, advance
                            byte_cnt <= byte_cnt + 1;
                            if (byte_cnt + 1 < get_cmd_len(cmd_select)) begin
                                // More bytes to send
                                spi_tx_data <= get_cmd_byte(cmd_select, byte_cnt + 1);
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end
                    end else begin
                        // Command sent, read response
                        cs_n <= 1'b1;
                        delay_counter <= 19'd0;
                        byte_cnt <= 8'd0;
                        packet_length <= 16'd0;
                        state <= ST_READ_RESPONSE;
                    end
                end
                
                ST_READ_RESPONSE: begin
                    cs_n <= 1'b1;
                    // Wait for INT (response ready)
                    if (!int_n_sync) begin
                        cs_n <= 1'b0;
                        byte_cnt <= 8'd0;
                        packet_length <= 16'd0;
                        state <= ST_READ_HEADER;
                    end else if (delay_counter >= 19'd30_000) begin
                        // Timeout - move to next command or done
                        delay_counter <= 19'd0;
                        if (cmd_select < 2'd2) begin
                            cmd_select <= cmd_select + 1;
                            state <= ST_WAKE;
                        end else begin
                            initialized <= 1'b1;
                            state <= ST_WAIT_DATA;
                        end
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                ST_WAIT_DATA: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    if (!int_n_sync) begin
                        state <= ST_READ_HEADER;
                        byte_cnt <= 8'd0;
                        packet_length <= 16'd0;
                    end
                end
                
                ST_READ_HEADER: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid) begin
                        case (byte_cnt)
                            0: begin
                                packet_length[7:0] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                            1: begin
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
                                if (packet_length > 16'd4 && packet_length < 16'd32767) begin
                                    byte_cnt <= 8'd0;
                                    state <= ST_READ_PAYLOAD;
                                    spi_tx_data <= 8'h00;
                                    spi_tx_valid <= 1'b1;
                                    spi_start <= 1'b1;
                                end else begin
                                    cs_n <= 1'b1;
                                    state <= ST_WAIT_DATA;
                                end
                            end
                        endcase
                    end else if (!spi_busy && spi_tx_ready && byte_cnt < 4) begin
                        spi_tx_data <= 8'h00;
                        spi_tx_valid <= 1'b1;
                        spi_start <= 1'b1;
                    end
                end
                
                ST_READ_PAYLOAD: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid) begin
                        // Only process sensor reports (Channel 3 or 5)
                        if (channel == CHANNEL_REPORTS || channel == CHANNEL_GYRO_RV) begin
                            case (byte_cnt)
                                0: current_report_id <= spi_rx_data;
                                1: ; // Sequence number
                                2: ; // Status
                                3: ; // Delay
                                4: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end
                                end
                                5: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_x <= {spi_rx_data, temp_byte_lsb};
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_x <= {spi_rx_data, temp_byte_lsb};
                                    end
                                end
                                6: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end
                                end
                                7: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_y <= {spi_rx_data, temp_byte_lsb};
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_y <= {spi_rx_data, temp_byte_lsb};
                                    end
                                end
                                8: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end
                                end
                                9: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_z <= {spi_rx_data, temp_byte_lsb};
                                    end else if (current_report_id == REPORT_ID_GYROSCOPE) begin
                                        gyro_z <= {spi_rx_data, temp_byte_lsb};
                                        gyro_valid <= 1'b1;
                                    end
                                end
                                10: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        temp_byte_lsb <= spi_rx_data;
                                    end
                                end
                                11: begin
                                    if (current_report_id == REPORT_ID_ROTATION_VECTOR) begin
                                        quat_w <= {spi_rx_data, temp_byte_lsb};
                                        quat_valid <= 1'b1;
                                    end
                                end
                            endcase
                        end
                        
                        byte_cnt <= byte_cnt + 1;
                        
                        // Continue reading if more data
                        if ((byte_cnt + 1) < (packet_length - 4)) begin
                            spi_tx_data <= 8'h00;
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                        end else begin
                            // Packet complete
                            cs_n <= 1'b1;
                            byte_cnt <= 8'd0;
                            state <= ST_WAIT_DATA;
                        end
                    end else if (!spi_busy && spi_tx_ready && byte_cnt < (packet_length - 4)) begin
                        spi_tx_data <= 8'h00;
                        spi_tx_valid <= 1'b1;
                        spi_start <= 1'b1;
                    end
                end
                
                ST_ERROR: begin
                    error <= 1'b1;
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    state <= ST_ERROR;
                end
                
                default: state <= ST_RESET;
            endcase
        end
    end
    
endmodule
