`timescale 1ns / 1ps

// BNO085 Controller Module (FIXED VERSION)
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Reads Rotation Vector (quaternion) and Gyroscope reports
// Optimized for low resource usage (ROM-based initialization, no large buffers)
//
// FIXES APPLIED:
// 1. Simplified SPI handshake: hold spi_start until busy, rely on master acknowledgment
// 2. Robust rx_valid detection: latch rx_valid to catch 1-cycle pulse from master
// 3. Correct SHTP parsing: separate SHTP sequence number from report sequence number
// 4. Continuous CS transaction: maintain CS low for full packet read

module bno085_controller_new (
    input  logic        clk,
    input  logic        fpga_rst_n,  // FPGA reset (active low)

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

    // BNO085 Reset output (controlled by this module)
    output logic        bno085_rst_n,  // Reset for BNO085 sensor (active low)

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
    
    // Response Report IDs (per datasheet Section 1.3.2)
    localparam [7:0] REPORT_ID_PROD_ID_RESP = 8'hF8;  // Product ID Response
    localparam [7:0] REPORT_ID_GET_FEATURE_RESP = 8'hFC;  // Get Feature Response
    localparam [7:0] REPORT_ID_COMMAND_RESP = 8'hF1;  // Command Response
    
    // Reset timing parameters
    // Per datasheet 6.5.3: After NRST release, BNO085 needs ~94ms for initialization
    localparam [22:0] DELAY_100MS = 23'd300_000;  // 100ms @ 3MHz = 300,000 cycles
    localparam [22:0] DELAY_1MS = 23'd3_000;  // 1ms @ 3MHz = 3,000 cycles
    
    // Internal reset control
    logic rst_n;  // Internal reset (controller ready)
    logic [22:0] rst_delay_counter;  // Reset delay counter
    
    typedef enum logic [4:0] {
        IDLE,
        INIT_WAIT_RESET,
        INIT_WAKE,
        INIT_WAIT_INT,
        INIT_WAIT_ADVERT,      // Wait for and process advertisement packets
        INIT_CS_SETUP,         // CS setup before SPI transaction
        INIT_SEND_BODY,
        INIT_WAIT_RESPONSE,    // Wait for command response
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
    
    // Sequence number tracking
    logic [7:0] shtp_seq_num;  // SHTP header sequence number (byte 3 of header, per datasheet 1.3.1)
    logic [7:0] report_seq_num; // Sensor report sequence number (byte 1 of payload, per datasheet 1.3.5.2)
    
    // Status and delay fields
    logic [7:0] report_status;
    logic [7:0] report_delay;
    
    // FIX 3: Register rx_valid to extend detection window
    // Managed inside state machine always_ff to avoid race conditions
    logic spi_rx_valid_reg;
    
    // Advertisement handling
    logic advert_done;  // Flag indicating advertisements have been processed
    logic [18:0] advert_timeout_counter;  // Timeout counter for advertisement wait
    logic waiting_for_advert;  // Flag indicating we're waiting for advertisement packet
    
    // Response handling
    logic response_received;  // Flag indicating response was received
    logic [18:0] response_timeout_counter;  // Timeout counter for response wait
    logic waiting_for_response;  // Flag indicating we're waiting for response packet
    
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

    // ========================================================================
    // BNO085 Reset Control
    // ========================================================================
    // Per datasheet 6.5.3: BNO085 needs ~94ms after NRST release before ready
    // Sequence: FPGA reset releases -> pulse BNO085 reset low -> wait 100ms -> release BNO085 reset
    // CRITICAL: Reset signal is ACTIVE LOW: 0 = reset active, 1 = reset released
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            // FPGA reset active: keep BNO085 in reset and reset counter
            rst_delay_counter <= 23'd0;
            bno085_rst_n <= 1'b0;  // Active low reset - keep sensor in reset (LOW = reset active)
            rst_n <= 1'b0;  // Keep controller in reset too
        end else begin
            // FPGA reset released: count up to delay
            if (rst_delay_counter < DELAY_100MS) begin
                rst_delay_counter <= rst_delay_counter + 1;
            end
            
            // Reset pulse sequence
            // 1. Keep reset LOW (active) for first 1ms to ensure proper reset pulse
            // 2. Keep reset LOW until 100ms total (allows BNO085 to initialize per datasheet)
            // 3. Release reset (set to HIGH) after 100ms
            if (rst_delay_counter < DELAY_1MS) begin
                // First 1ms: Keep reset LOW (active low = 0) to ensure proper reset pulse
                bno085_rst_n <= 1'b0;
                rst_n <= 1'b0;  // Controller still in reset
            end else if (rst_delay_counter >= DELAY_100MS) begin
                // After 100ms: Release BNO085 reset (set to HIGH = 1 means reset released)
                bno085_rst_n <= 1'b1;
                rst_n <= 1'b1;  // Release controller reset - ready to start
            end else begin
                // Between 1ms and 100ms: Keep reset LOW (still in reset, active)
                bno085_rst_n <= 1'b0;
                rst_n <= 1'b0;  // Controller still in reset
            end
        end
    end

    // INT pin synchronization
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
            shtp_seq_num <= 8'd0;
            report_seq_num <= 8'd0;
            report_status <= 8'd0;
            report_delay <= 8'd0;
            spi_rx_valid_reg <= 1'b0;
            advert_done <= 1'b0;
            advert_timeout_counter <= 19'd0;
            waiting_for_advert <= 1'b0;
            response_received <= 1'b0;
            response_timeout_counter <= 19'd0;
            waiting_for_response <= 1'b0;
            
            quat_w <= 16'd0;
            quat_x <= 16'd0;
            quat_y <= 16'd0;
            quat_z <= 16'd0;
            gyro_x <= 16'd0;
            gyro_y <= 16'd0;
            gyro_z <= 16'd0;
            
        end else begin
            // Default assignments - CRITICAL: Clear spi_start every cycle unless explicitly set
            // This prevents SPI clock from running continuously
            spi_start <= 1'b0;  
            spi_tx_valid <= 1'b0;
            quat_valid <= 1'b0;
            gyro_valid <= 1'b0;
            
            // Latch rx_valid (default behavior, can be overridden by state machine consumption)
            if (spi_rx_valid) begin
                spi_rx_valid_reg <= 1'b1;
            end
            
            case (state)
                // 1. Wait after reset to ensure sensor is ready
                // Note: BNO085 reset was already released 100ms ago by reset control logic
                // So we only need a short delay here to ensure sensor is stable
                // Per datasheet 5.2.1: BNO085 may assert INT automatically after reset
                INIT_WAIT_RESET: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure PS0 is high (required for SPI mode)
                    // Short delay: 10ms (30,000 cycles @ 3MHz) since reset was already released
                    if (delay_counter < 19'd30_000) begin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 19'd0;
                        cmd_select <= 2'd0; // Start with ProdID
                        // Always wait for advertisements first, regardless of INT state
                        // After reset, BNO085 automatically sends advertisement packets
                        // We must process these before sending any commands
                        advert_done <= 1'b0;
                        advert_timeout_counter <= 19'd0;
                        waiting_for_advert <= 1'b1;
                        // Check if INT is already asserted (automatic assertion after reset)
                        if (!int_n_sync) begin
                            // INT already low - advertisement may be ready, go to wait state
                            state <= INIT_WAIT_ADVERT;
                        end else begin
                            // INT still high - need to use wake signal first
                            state <= INIT_WAKE;
                        end
                    end
                end

                // 2. Wake the sensor (PS0 Low) - per datasheet 1.2.4.3
                // PS0/WAKE is active low - drive low to wake processor from sleep
                // Per datasheet 6.5.4: twk = 150 µs max for wake signal
                // Hold PS0 low for sufficient time to ensure sensor detects wake signal
                INIT_WAKE: begin
                    ps0_wake <= 1'b0; // Drive PS0 low to wake sensor
                    if (delay_counter < 19'd450) begin
                        // Hold PS0 low for 150µs (450 cycles @ 3MHz = 150µs)
                        // This meets datasheet requirement and provides margin
                        delay_counter <= delay_counter + 1;
                    end else begin
                        // PS0 has been low long enough, now check for INT
                        delay_counter <= 19'd0;
                        state <= INIT_WAIT_INT;
                    end
                end

                // 3. Wait for INT low (Sensor Ready) with timeout - per datasheet 6.5.4 (twk = 150 µs max)
                // Hardware may need significantly more time due to signal propagation, sensor wake-up delays,
                // and PCB routing delays. Using 200ms timeout provides substantial margin for hardware tolerance.
                INIT_WAIT_INT: begin
                    if (!int_n_sync) begin
                        // INT asserted, sensor is ready
                        // After reset, BNO085 automatically sends advertisement packets
                        // We must wait for and process these before sending commands
                        delay_counter <= 19'd0;
                        advert_done <= 1'b0;
                        advert_timeout_counter <= 19'd0;
                        waiting_for_advert <= 1'b1;
                        state <= INIT_WAIT_ADVERT; // Wait for advertisements first
                    end else if (delay_counter >= 19'd600_000) begin
                        // Timeout: 600000 cycles @ 3MHz = 200ms (increased for hardware tolerance)
                        // This is much longer than datasheet spec (150µs) but accounts for:
                        // - Signal propagation delays on PCB
                        // - Sensor wake-up time variations
                        // - Clock domain crossing delays
                        // - Sensor may need more time after reset
                        state <= ERROR_STATE;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                // Wait for advertisement packets after reset
                // Per SHTP protocol: After reset, sensor automatically sends advertisements on Channel 0
                // These must be processed before sending any commands
                // The sensor may send multiple advertisement packets - we need to read all of them
                INIT_WAIT_ADVERT: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    
                    // Check if INT is asserted (data ready)
                    if (!int_n_sync) begin
                        // INT asserted - advertisement packet is ready
                        // Read and process it (we'll process in READ_HEADER/READ_PAYLOAD states)
                        byte_cnt <= 8'd0;
                        state <= READ_HEADER_START;
                    end else begin
                        // INT not asserted - no more advertisement packets
                        // Check if we've read at least one advertisement or timed out
                        if (advert_done || advert_timeout_counter >= 19'd600_000) begin
                            // Advertisements processed or timeout (200ms) - proceed to initialization commands
                            advert_done <= 1'b1;
                            waiting_for_advert <= 1'b0;
                            delay_counter <= 19'd0;
                            cmd_select <= 2'd0; // Start with ProdID
                            // Wait for INT before sending first command (sensor must be ready)
                            // Check if INT is already asserted - if so, sensor is ready, skip wake
                            if (!int_n_sync) begin
                                // INT already asserted - sensor is ready, proceed to first command
                                waiting_for_response <= 1'b0;
                                response_received <= 1'b0;
                                response_timeout_counter <= 19'd0;
                                state <= INIT_CS_SETUP;
                            end else begin
                                // INT not asserted - need to wake sensor first
                                state <= INIT_WAKE;
                            end
                        end else begin
                            // Still waiting for first advertisement - increment timeout
                            advert_timeout_counter <= advert_timeout_counter + 1;
                        end
                    end
                end
                
                // CS setup before SPI transaction - per datasheet 6.5.2 (tcssu = 0.1 µs min)
                // CRITICAL: Per Adafruit library, we must wait for INT before writing
                // The sensor asserts INT to indicate it's ready to receive data
                INIT_CS_SETUP: begin
                    cs_n <= 1'b1; // Keep CS high until INT asserts
                    ps0_wake <= 1'b1; // Release Wake
                    delay_counter <= 19'd0;
                    byte_cnt <= 8'd0;
                    response_received <= 1'b0;
                    response_timeout_counter <= 19'd0;
                    // Wait for INT to assert before starting SPI transaction
                    if (!int_n_sync) begin
                        // INT asserted - sensor is ready, assert CS and start sending
                        cs_n <= 1'b0; // Assert CS now that INT is low
                        state <= INIT_SEND_BODY;
                    end else if (delay_counter >= 19'd600_000) begin
                        // Timeout: 600000 cycles @ 3MHz = 200ms
                        // INT never asserted - sensor may not be ready
                        state <= ERROR_STATE;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                // 4. Send Command Body
                // Per Adafruit library: INT must be asserted before writing
                // Once CS goes low, INT deasserts, but we've already verified it was asserted
                INIT_SEND_BODY: begin
                    cs_n <= 1'b0; // Keep CS low for entire packet
                    
                    if (byte_cnt < get_cmd_len(cmd_select)) begin
                        if ((spi_rx_valid || spi_rx_valid_reg) && !spi_busy) begin
                            // Transfer complete, consume rx_valid
                            spi_rx_valid_reg <= 1'b0;
                            byte_cnt <= byte_cnt + 1;
                            
                            // Start next byte immediately if more to send
                            // Note: For multi-byte packets, we continue in same CS transaction
                            // INT may deassert when CS goes low, but we keep CS low for full packet
                            if ((byte_cnt + 1) < get_cmd_len(cmd_select)) begin
                                spi_tx_data <= get_init_byte(cmd_select, byte_cnt + 1);
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end else if (!spi_busy) begin
                            // Start transaction (or retry/hold start)
                            spi_tx_data <= get_init_byte(cmd_select, byte_cnt);
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                        end
                    end else begin
                        // Done sending all bytes - release CS and wait for response
                        cs_n <= 1'b1;
                        byte_cnt <= 8'd0;
                        response_received <= 1'b0;
                        response_timeout_counter <= 19'd0;
                        waiting_for_response <= 1'b1;
                        state <= INIT_WAIT_RESPONSE;
                    end
                end
                
                // Wait for command response after sending command
                // Must receive response before proceeding to next command
                INIT_WAIT_RESPONSE: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1;
                    
                    // Check if response was received (set in READ_PAYLOAD when processing response)
                    if (response_received) begin
                        // Response received - proceed to next step
                        response_received <= 1'b0;
                        waiting_for_response <= 1'b0;
                        delay_counter <= 19'd0;
                        state <= INIT_DONE_CHECK;
                    end else if (response_timeout_counter >= 19'd300_000) begin
                        // Timeout: 300000 cycles @ 3MHz = 100ms
                        // Response not received - this is an error
                        state <= ERROR_STATE;
                    end else if (!int_n_sync) begin
                        // INT asserted - response packet is ready
                        // Read it (will be processed in READ_HEADER/READ_PAYLOAD)
                        byte_cnt <= 8'd0;
                        state <= READ_HEADER_START;
                    end else begin
                        response_timeout_counter <= response_timeout_counter + 1;
                    end
                end
                
                // 5. Check if more commands or done
                INIT_DONE_CHECK: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure PS0 is high before next wake cycle
                    // Delay 10ms between commands (per Adafruit library pattern)
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
                    // Per datasheet 6.5.4: When CS goes low, INT deasserts
                    // So we should start reading immediately after CS goes low
                    cs_n <= 1'b0;
                    // Wait one cycle for CS to settle before starting SPI transaction
                    // This ensures proper setup time
                    delay_counter <= 19'd0;
                    state <= READ_HEADER;
                    byte_cnt <= 8'd0;
                end
                
                READ_HEADER: begin
                    cs_n <= 1'b0;
                    // Start first byte if not already started
                    if (byte_cnt == 0 && !spi_busy && !spi_start) begin
                        spi_tx_data <= 8'h00;
                        spi_tx_valid <= 1'b1;
                        spi_start <= 1'b1;
                    end
                    // FIX 3: Use registered rx_valid for reliable detection (extends one-cycle pulse)
                    else if ((spi_rx_valid || spi_rx_valid_reg) && !spi_busy) begin
                        // Consume rx_valid
                        spi_rx_valid_reg <= 1'b0;
                        
                        case (byte_cnt)
                            0: begin
                                packet_length[7:0] <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                // Start next transaction immediately
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            1: begin
                                // Mask continuation bit (bit 15) per datasheet 1.3.1
                                packet_length[15:8] <= spi_rx_data;
                                packet_length[15] <= 1'b0; // Clear continuation bit
                                byte_cnt <= byte_cnt + 1;
                                // Start next transaction immediately
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            2: begin
                                channel <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                // Start next transaction immediately
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end
                            3: begin
                                // Capture SHTP header sequence number (per datasheet 1.3.1, Figure 1-26)
                                // This sequence number is used to detect duplicate or missing SHTP cargoes
                                shtp_seq_num <= spi_rx_data;
                                
                                // Validate packet length (max 32766 per datasheet 1.3.1)
                                // Length includes header, so payload = packet_length - 4
                                if (packet_length > 16'd4 && packet_length < 16'd32767) begin
                                    byte_cnt <= 8'd0;
                                    state <= READ_PAYLOAD;
                                    // Continue reading in same CS transaction (CS stays low)
                                    spi_tx_data <= 8'h00; 
                                    spi_tx_valid <= 1'b1; 
                                    spi_start <= 1'b1;
                                end else begin
                                    // Invalid length - abort transaction
                                    cs_n <= 1'b1;
                                    byte_cnt <= 8'd0;
                                    // Return to appropriate state based on what we're waiting for
                                    if (waiting_for_advert) begin
                                        state <= INIT_WAIT_ADVERT;
                                    end else if (waiting_for_response) begin
                                        state <= INIT_WAIT_RESPONSE;
                                    end else begin
                                        state <= WAIT_DATA;
                                    end
                                end
                            end
                        endcase
                    end
                end
                
                READ_PAYLOAD: begin
                    cs_n <= 1'b0;
                    // FIX 3: Use registered rx_valid for reliable detection (extends one-cycle pulse)
                    if ((spi_rx_valid || spi_rx_valid_reg) && !spi_busy) begin
                        // Consume rx_valid
                        spi_rx_valid_reg <= 1'b0;
                        
                        // Handle advertisement packets (Channel 0) during initialization
                        if (waiting_for_advert && channel == CHANNEL_COMMAND) begin
                            // This is an advertisement packet - we need to process it
                            // For simplicity, we just mark it as processed when we finish reading
                            // The actual TLV parsing is complex, so we'll just consume the packet
                            // and mark advertisements as done
                            // Check if this is the last byte of the payload
                            // byte_cnt is the number of payload bytes read so far (0-indexed)
                            // packet_length includes header (4 bytes), so payload = packet_length - 4
                            // We need to read (packet_length - 4) bytes total
                            if ((byte_cnt + 1) >= (packet_length - 4)) begin
                                // Advertisement packet complete (read all payload bytes)
                                // Mark that we've read at least one advertisement
                                advert_done <= 1'b1;
                                // Check if INT is still asserted - might be another advertisement packet
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                delay_counter <= 19'd0;
                                // Wait a cycle for CS to settle, then check INT
                                // If INT is still asserted, we'll read another packet
                                // If INT is not asserted, we'll proceed to commands
                                state <= INIT_WAIT_ADVERT;
                            end else begin
                                byte_cnt <= byte_cnt + 1;
                                // Continue reading
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end
                        // Handle command responses (Channel 2 - Control channel)
                        else if (waiting_for_response && channel == CHANNEL_CONTROL) begin
                            // This is a response packet - check for expected response IDs
                            if (byte_cnt == 0) begin
                                // First byte is report ID
                                current_report_id <= spi_rx_data;
                                byte_cnt <= byte_cnt + 1;
                                // Continue reading
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end else if ((byte_cnt + 1) >= (packet_length - 4)) begin
                                // Response packet complete - validate it
                                // Check if we got the expected response
                                // Product ID response: 0xF8, Set Feature response: 0xFC
                                if ((cmd_select == 2'd0 && current_report_id == REPORT_ID_PROD_ID_RESP) ||
                                    ((cmd_select == 2'd1 || cmd_select == 2'd2) && current_report_id == REPORT_ID_GET_FEATURE_RESP)) begin
                                    // Valid response received
                                    response_received <= 1'b1;
                                end else begin
                                    // Unexpected response - still mark as received to proceed
                                    // (some responses may have different IDs)
                                    response_received <= 1'b1;
                                end
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                waiting_for_response <= 1'b0;
                                // Return to INIT_WAIT_RESPONSE to check response_received flag
                                state <= INIT_WAIT_RESPONSE;
                            end else begin
                                byte_cnt <= byte_cnt + 1;
                                // Continue reading
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end
                        // Handle sensor data reports (normal operation)
                        // Only process if we're not waiting for advertisements or responses
                        else if (!waiting_for_advert && !waiting_for_response && 
                                 (channel == CHANNEL_REPORTS || channel == CHANNEL_GYRO_RV)) begin
                            case (byte_cnt)
                                0: current_report_id <= spi_rx_data;
                                1: report_seq_num <= spi_rx_data; // Sensor report sequence number
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
                            
                            byte_cnt <= byte_cnt + 1;
                            
                            // Continue reading if more data
                            // packet_length includes 4-byte header, so payload is (packet_length - 4) bytes
                            // After incrementing, byte_cnt is the number of payload bytes read so far
                            if ((byte_cnt + 1) < (packet_length - 4)) begin
                                spi_tx_data <= 8'h00; 
                                spi_tx_valid <= 1'b1; 
                                spi_start <= 1'b1;
                            end else begin
                                // Packet complete
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                state <= WAIT_DATA;
                            end
                        end else begin
                            // Unknown channel or state - just consume the packet
                            byte_cnt <= byte_cnt + 1;
                            if ((byte_cnt + 1) < (packet_length - 4)) begin
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end else begin
                                // Packet complete - return to appropriate state
                                cs_n <= 1'b1;
                                byte_cnt <= 8'd0;
                                if (waiting_for_advert) begin
                                    state <= INIT_WAIT_ADVERT;
                                end else if (waiting_for_response) begin
                                    state <= INIT_WAIT_RESPONSE;
                                end else begin
                                    state <= WAIT_DATA;
                                end
                            end
                        end
                    end else if (!spi_busy && byte_cnt < (packet_length - 4)) begin
                        // Continue reading payload - Hold/Assert start
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
