`timescale 1ns / 1ps

// BNO085 Controller Module - SIMPLIFIED VERSION
// Based on BNO08X datasheet and simplified FSM approach
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Reads Rotation Vector (quaternion) and Gyroscope reports

module bno085_controller_simple (
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
        ST_WAKE_ASSERT,
        ST_WAIT_INT,
        ST_CS_SETUP,
        ST_SEND_COMMAND,
        ST_DONE_CHECK,
        ST_IDLE,
        ST_READ_HEADER_START,
        ST_SPI_READ_HEADER,
        ST_SPI_READ_PAYLOAD
    } bno_state_t;
    
    bno_state_t state;
    
    // Counters and registers
    logic [7:0] byte_cnt;
    logic [15:0] packet_length;
    logic [7:0] channel;
    logic [7:0] current_report_id;
    logic [18:0] delay_counter;
    logic [1:0] init_step; // 0=ProdID, 1=RotVec, 2=Gyro, 3=Done
    
    // Temporary storage for parsing (little-endian: LSB first)
    logic [7:0] temp_byte_lsb;
    
    // INT synchronization
    logic int_n_sync;
    
    // Command ROM - simplified
    function [7:0] get_cmd_byte(input [1:0] step, input [7:0] idx);
        case (step)
            2'd0: begin // Product ID Request
                case (idx)
                    0: get_cmd_byte = 8'h05; // Length LSB
                    1: get_cmd_byte = 8'h00; // Length MSB
                    2: get_cmd_byte = 8'h02; // Channel Control
                    3: get_cmd_byte = 8'h00; // Seq
                    4: get_cmd_byte = 8'hF9; // Report ID
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            2'd1: begin // Set Feature: Rotation Vector
                case (idx)
                    0: get_cmd_byte = 8'd17; // Length LSB
                    1: get_cmd_byte = 8'h00; // Length MSB
                    2: get_cmd_byte = 8'h02; // Channel Control
                    3: get_cmd_byte = 8'd1;  // Seq
                    4: get_cmd_byte = 8'hFD; // Set Feature
                    5: get_cmd_byte = 8'h05; // Report ID (Rot Vec)
                    6: get_cmd_byte = 8'h00; // Flags
                    7: get_cmd_byte = 8'h00; // Sensitivity LSB
                    8: get_cmd_byte = 8'h00; // Sensitivity MSB
                    9: get_cmd_byte = 8'h20; // Report Interval LSB (20,000 µs = 50 Hz)
                    10: get_cmd_byte = 8'h4E;
                    11: get_cmd_byte = 8'h00;
                    12: get_cmd_byte = 8'h00; // Report Interval MSB
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            2'd2: begin // Set Feature: Gyroscope
                case (idx)
                    0: get_cmd_byte = 8'd17; // Length LSB
                    1: get_cmd_byte = 8'h00; // Length MSB
                    2: get_cmd_byte = 8'h02; // Channel Control
                    3: get_cmd_byte = 8'd2;  // Seq
                    4: get_cmd_byte = 8'hFD; // Set Feature
                    5: get_cmd_byte = 8'h02; // Report ID (Gyro)
                    6: get_cmd_byte = 8'h00; // Flags
                    7: get_cmd_byte = 8'h00; // Sensitivity LSB
                    8: get_cmd_byte = 8'h00; // Sensitivity MSB
                    9: get_cmd_byte = 8'h20; // Report Interval LSB (20,000 µs = 50 Hz)
                    10: get_cmd_byte = 8'h4E;
                    11: get_cmd_byte = 8'h00;
                    12: get_cmd_byte = 8'h00; // Report Interval MSB
                    default: get_cmd_byte = 8'h00;
                endcase
            end
            default: get_cmd_byte = 8'h00;
        endcase
    endfunction
    
    function [7:0] get_cmd_len(input [1:0] step);
        case (step)
            2'd0: get_cmd_len = 8'd5;  // Product ID
            2'd1: get_cmd_len = 8'd17; // Set Feature RotVec
            2'd2: get_cmd_len = 8'd17; // Set Feature Gyro
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
    
    // Main state machine - SIMPLIFIED
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
            init_step <= 2'd0;
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
                        init_step <= 2'd0;
                        state <= ST_WAKE_ASSERT;
                    end
                end
                
                ST_WAKE_ASSERT: begin
                    ps0_wake <= 1'b0; // WAKE active low
                    if (delay_counter < 19'd450) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        state <= ST_WAIT_INT;
                    end
                end
                
                ST_WAIT_INT: begin
                    // Don't release WAKE here - wait until CS is asserted
                    if (!int_n_sync) begin
                        // INT asserted, sensor is ready
                        if (init_step < 2'd3) begin
                            // Still initializing - setup CS first
                            delay_counter <= 19'd0;
                            byte_cnt <= 8'd0;
                            state <= ST_CS_SETUP;
                        end else begin
                            // Initialized - read data directly
                            cs_n <= 1'b0;
                            byte_cnt <= 8'd0;
                            spi_tx_data <= 8'h00;
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                            state <= ST_SPI_READ_HEADER;
                        end
                    end else if (delay_counter >= 19'd150_000) begin
                        // Timeout
                        error <= 1'b1;
                        state <= ST_RESET;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                // CS setup before SPI transaction - per datasheet 6.5.2 (tcssu = 0.1 µs min)
                ST_CS_SETUP: begin
                    cs_n <= 1'b0; // Assert CS
                    ps0_wake <= 1'b1; // Release Wake once CS is asserted (per original controller)
                    delay_counter <= 19'd0;
                    byte_cnt <= 8'd0;
                    state <= ST_SEND_COMMAND;
                end
                
                ST_SEND_COMMAND: begin
                    cs_n <= 1'b0;
                    if (byte_cnt < get_cmd_len(init_step)) begin
                        if (!spi_busy && spi_tx_ready) begin
                            // Start new byte transfer
                            spi_tx_data <= get_cmd_byte(init_step, byte_cnt);
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
                        // Done sending all bytes - don't wait for response during init
                        cs_n <= 1'b1;
                        byte_cnt <= 8'd0;
                        state <= ST_DONE_CHECK;
                        delay_counter <= 19'd0;
                    end
                end
                
                // Check if more commands or done (matches original controller)
                ST_DONE_CHECK: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure PS0 is high before next wake cycle
                    // Delay 10ms between commands
                    if (delay_counter < 19'd30_000) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        if (init_step < 2'd2) begin
                            init_step <= init_step + 1;
                            state <= ST_WAKE_ASSERT; // Go back to Wake for next command
                        end else begin
                            initialized <= 1'b1;
                            init_step <= 2'd3; // Mark as done
                            state <= ST_IDLE;
                        end
                    end
                end
                
                ST_SPI_READ_HEADER: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid && !spi_busy) begin
                        case (byte_cnt)
                            0: begin
                                packet_length[7:0] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                spi_tx_data <= 8'h00; // Always send 0x00 while reading
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
                                // Sequence number - ignore for now
                                byte_cnt <= 8'd0;
                                if (packet_length > 16'd4 && packet_length < 16'd32767) begin
                                    state <= ST_SPI_READ_PAYLOAD;
                                    spi_tx_data <= 8'h00;
                                    spi_tx_valid <= 1'b1;
                                    spi_start <= 1'b1;
                                end else begin
                                    // Invalid length
                                    cs_n <= 1'b1;
                                    if (init_step < 2'd3) begin
                                        init_step <= init_step + 1;
                                        delay_counter <= 19'd0;
                                        state <= ST_WAKE_ASSERT;
                                    end else begin
                                        state <= ST_IDLE;
                                    end
                                end
                            end
                        endcase
                    end else if (!spi_busy && byte_cnt < 4) begin
                        // Continue reading header
                        spi_tx_data <= 8'h00; // Always send 0x00 while reading
                        spi_tx_valid <= 1'b1;
                        spi_start <= 1'b1;
                    end
                end
                
                ST_SPI_READ_PAYLOAD: begin
                    cs_n <= 1'b0;
                    if (spi_rx_valid && !spi_busy) begin
                        // Parse on the fly - accept reports from Channel 3 (standard reports) or Channel 5 (gyro rotation vector)
                        // During initialization, responses come on Channel 2 - we read but don't process them
                        if (channel == CHANNEL_REPORTS || channel == CHANNEL_GYRO_RV) begin
                            // Process sensor reports (matches original controller - no init_step check)
                            case (byte_cnt)
                                0: current_report_id <= spi_rx_data;
                                1: ; // Sequence number - ignore
                                2: ; // Status - ignore
                                3: ; // Delay - ignore
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
                        // Always read the entire packet (even if we don't process it)
                        
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
                            if (init_step < 2'd3) begin
                                // Still initializing - read response but don't process, move to next step
                                init_step <= init_step + 1;
                                delay_counter <= 19'd0;
                                state <= ST_WAKE_ASSERT;
                            end else begin
                                // Initialized - wait for next interrupt
                                if (init_step == 2'd3) begin
                                    initialized <= 1'b1;
                                    init_step <= 2'd3; // Stay at 3
                                end
                                state <= ST_IDLE;
                            end
                        end
                    end else if (!spi_busy && byte_cnt < (packet_length - 4)) begin
                        // Continue reading payload
                        spi_tx_data <= 8'h00;
                        spi_tx_valid <= 1'b1;
                        spi_start <= 1'b1;
                    end
                end
                
                ST_IDLE: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure wake is released
                    if (!int_n_sync) begin
                        state <= ST_READ_HEADER_START;
                        byte_cnt <= 8'd0;
                    end
                end
                
                // CS setup before SPI - per datasheet 6.5.2
                // Per datasheet 6.5.4: When CS goes low, INT deasserts
                // So we should start reading immediately after CS goes low
                ST_READ_HEADER_START: begin
                    cs_n <= 1'b0;
                    // Start first SPI transaction to read header
                    // Don't check INT here - CS low causes INT to deassert per datasheet
                    spi_tx_data <= 8'h00; 
                    spi_tx_valid <= 1'b1; 
                    spi_start <= 1'b1;
                    byte_cnt <= 8'd0;
                    state <= ST_SPI_READ_HEADER;
                end
                
                default: state <= ST_RESET;
            endcase
        end
    end
    
endmodule

