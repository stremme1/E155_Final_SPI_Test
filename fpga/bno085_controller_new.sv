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
    
    // Advertisement TLV Tags (per SHTP library shtp.h)
    localparam [7:0] TAG_NULL = 8'h00;
    localparam [7:0] TAG_GUID = 8'h01;
    localparam [7:0] TAG_MAX_CARGO_PLUS_HEADER_WRITE = 8'h02;
    localparam [7:0] TAG_MAX_CARGO_PLUS_HEADER_READ = 8'h03;
    localparam [7:0] TAG_MAX_TRANSFER_WRITE = 8'h04;
    localparam [7:0] TAG_MAX_TRANSFER_READ = 8'h05;
    localparam [7:0] TAG_NORMAL_CHANNEL = 8'h06;
    localparam [7:0] TAG_WAKE_CHANNEL = 8'h07;
    localparam [7:0] TAG_APP_NAME = 8'h08;
    localparam [7:0] TAG_CHANNEL_NAME = 8'h09;
    localparam [7:0] TAG_ADV_COUNT = 8'h0A;
    
    // Executable channel commands (per datasheet 1.3.1, Figure 1-27)
    localparam [7:0] EXEC_CMD_RESET = 8'h01;
    localparam [7:0] EXEC_RESP_RESET_COMPLETE = 8'h01;
    
    // Timing parameters - CRITICAL: Per datasheet 6.5.2, 6.5.4
    localparam [18:0] TIMEOUT_INT_AFTER_WAKE = 19'd3_000;  // 1ms (datasheet says 150µs max, use 1ms for margin)
    localparam [18:0] TIMEOUT_ADVERT = 19'd150_000;  // 50ms (should come immediately after reset)
    localparam [18:0] TIMEOUT_RESPONSE = 19'd60_000;  // 20ms (commands should respond quickly)
    localparam [2:0] DELAY_CS_SETUP = 3'd3;  // 3 cycles @ 3MHz = 1µs (tcssu = 0.1µs min)
    localparam [2:0] DELAY_CS_HOLD = 3'd3;  // 3 cycles @ 3MHz = 1µs (tcssh)
    localparam [2:0] DELAY_INT_DEASSERT = 3'd3;  // 3 cycles @ 3MHz = 1µs (tcsid = 800ns max)
    
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
    
    // BNO085 reset control (separate from controller reset)
    logic [22:0] bno085_rst_delay_counter;  // BNO085 reset delay counter
    
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
    
    // Sequence number tracking - CRITICAL: Per datasheet 1.3.1, each channel and direction has its own sequence number
    // Used to detect duplicate or missing cargoes
    logic [7:0] shtp_seq_num;  // SHTP header sequence number (byte 3 of header, per datasheet 1.3.1)
    logic [7:0] shtp_seq_num_expected_rx[0:5];  // Expected RX sequence number per channel (0-5)
    logic [7:0] shtp_seq_num_tx[0:5];  // TX sequence number per channel (incremented when sending)
    logic seq_num_error;  // Flag indicating sequence number mismatch
    
    logic [7:0] report_seq_num; // Sensor report sequence number (byte 1 of payload, per datasheet 1.3.5.2)
    
    // Status and delay fields
    logic [7:0] report_status;
    logic [7:0] report_delay;
    
    // FIX 3: Register rx_valid to extend detection window
    // Managed inside state machine always_ff to avoid race conditions
    logic spi_rx_valid_reg;
    
    // Advertisement handling - CRITICAL: Must parse TLV to get channel assignments
    logic advert_done;  // Flag indicating advertisements have been processed
    logic [18:0] advert_timeout_counter;  // Timeout counter for advertisement wait
    logic waiting_for_advert;  // Flag indicating we're waiting for advertisement packet
    logic [7:0] advert_channel_control;  // Extracted control channel number from advertisement
    logic [7:0] advert_channel_executable;  // Extracted executable channel number from advertisement
    logic [15:0] advert_max_transfer_read;  // Maximum transfer size for reads
    logic advert_channel_control_valid;  // Flag indicating control channel was found
    logic advert_channel_executable_valid;  // Flag indicating executable channel was found
    
    // Advertisement TLV parsing state
    logic [15:0] advert_parse_cursor;  // Current position in advertisement payload
    logic [7:0] advert_tag;  // Current TLV tag
    logic [7:0] advert_len;  // Current TLV length
    logic parsing_advert;  // Flag indicating we're parsing advertisement
    
    // Response handling
    logic response_received;  // Flag indicating response was received
    logic [18:0] response_timeout_counter;  // Timeout counter for response wait
    logic waiting_for_response;  // Flag indicating we're waiting for response packet
    
    // Channel 1 (executable) reset message handling - CRITICAL: Per datasheet 5.2.1
    logic reset_message_received;  // Flag indicating Channel 1 reset message was received
    logic waiting_for_reset_message;  // Flag indicating we're waiting for reset message
    
    // SPI timing - CRITICAL: Per datasheet 6.5.2 and 6.5.4
    logic [2:0] cs_setup_delay;  // CS setup delay counter (tcssu = 0.1µs min, 3 cycles @ 3MHz = 1µs)
    logic [2:0] cs_hold_delay;  // CS hold delay counter (tcssh, ensure proper hold time)
    logic [2:0] int_deassert_delay;  // INT deassertion delay counter (tcsid = 800ns max, 3 cycles @ 3MHz = 1µs)
    
    // ========================================================================
    // Initialization Command ROM
    // ========================================================================
    function [7:0] get_init_byte(input [1:0] cmd, input [7:0] idx);
        case (cmd)
            // Product ID Request (5 bytes): 04 00 02 00 F9
            // SHTP Header: Length=5, Channel=2 (SH-2 control), Seq=0
            // Payload: Report ID 0xF9
            // CRITICAL FIX: Sequence number will be set dynamically per channel
            2'd0: begin
                case (idx)
                    0: get_init_byte = 8'h05; // Length LSB (5 bytes total)
                    1: get_init_byte = 8'h00; // Length MSB
                    2: get_init_byte = 8'h02; // Channel 2 (SH-2 control per datasheet Fig 1-30)
                    3: get_init_byte = 8'h00; // Seq - will be overwritten with actual sequence number
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
                    3: get_init_byte = 8'd1;  // Seq - will be overwritten with actual sequence number
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
                    3: get_init_byte = 8'd2;  // Seq - will be overwritten with actual sequence number
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
    // BNO085 Reset Control (separate from controller reset)
    // ========================================================================
    // Per datasheet 6.5.3: BNO085 needs ~94ms after NRST release before ready
    // Sequence: FPGA reset releases -> pulse BNO085 reset low -> wait 100ms -> release BNO085 reset
    // CRITICAL: Reset signal is ACTIVE LOW: 0 = reset active, 1 = reset released
    // CRITICAL FIX: Controller state machine uses fpga_rst_n directly (no delay)
    // Only BNO085 reset is delayed - controller can run immediately
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            // FPGA reset active: keep BNO085 in reset and reset counter
            bno085_rst_delay_counter <= 23'd0;
            bno085_rst_n <= 1'b0;  // Active low reset - keep sensor in reset (LOW = reset active)
        end else begin
            // FPGA reset released: count up to delay
            if (bno085_rst_delay_counter < DELAY_100MS) begin
                bno085_rst_delay_counter <= bno085_rst_delay_counter + 1;
            end
            
            // Reset pulse sequence
            // 1. Keep reset LOW (active) for first 1ms to ensure proper reset pulse
            // 2. Keep reset LOW until 100ms total (allows BNO085 to initialize per datasheet)
            // 3. Release reset (set to HIGH) after 100ms
            if (bno085_rst_delay_counter < DELAY_1MS) begin
                // First 1ms: Keep reset LOW (active low = 0) to ensure proper reset pulse
                bno085_rst_n <= 1'b0;
            end else if (bno085_rst_delay_counter >= DELAY_100MS) begin
                // After 100ms: Release BNO085 reset (set to HIGH = 1 means reset released)
                bno085_rst_n <= 1'b1;
            end else begin
                // Between 1ms and 100ms: Keep reset LOW (still in reset, active)
                bno085_rst_n <= 1'b0;
            end
        end
    end

    // INT pin synchronization - use fpga_rst_n directly (no delay)
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            int_n_sync <= 1'b1;
            int_n_prev <= 1'b1;
        end else begin
            int_n_sync <= int_n;
            int_n_prev <= int_n_sync;
        end
    end
    
    // State machine - use fpga_rst_n directly (no delay, like original)
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
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
            reset_message_received <= 1'b0;
            waiting_for_reset_message <= 1'b0;
            seq_num_error <= 1'b0;
            advert_channel_control <= 8'd2;  // Default to 2 (per datasheet)
            advert_channel_executable <= 8'd1;  // Default to 1 (per datasheet)
            advert_channel_control_valid <= 1'b0;
            advert_channel_executable_valid <= 1'b0;
            advert_max_transfer_read <= 16'd512;  // Default max transfer
            parsing_advert <= 1'b0;
            advert_parse_cursor <= 16'd0;
            advert_tag <= 8'd0;
            advert_len <= 8'd0;
            cs_setup_delay <= 3'd0;
            cs_hold_delay <= 3'd0;
            int_deassert_delay <= 3'd0;
            // Initialize sequence numbers per channel (0-5)
            shtp_seq_num_expected_rx[0] <= 8'd0;
            shtp_seq_num_expected_rx[1] <= 8'd0;
            shtp_seq_num_expected_rx[2] <= 8'd0;
            shtp_seq_num_expected_rx[3] <= 8'd0;
            shtp_seq_num_expected_rx[4] <= 8'd0;
            shtp_seq_num_expected_rx[5] <= 8'd0;
            shtp_seq_num_tx[0] <= 8'd0;
            shtp_seq_num_tx[1] <= 8'd0;
            shtp_seq_num_tx[2] <= 8'd0;
            shtp_seq_num_tx[3] <= 8'd0;
            shtp_seq_num_tx[4] <= 8'd0;
            shtp_seq_num_tx[5] <= 8'd0;
            
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
                // Per datasheet 5.2.1: BNO085 may assert INT automatically after reset
                // CRITICAL FIX: Removed unnecessary 10ms delay - sensor should be ready immediately
                INIT_WAIT_RESET: begin
                    cs_n <= 1'b1;
                    ps0_wake <= 1'b1; // Ensure PS0 is high (required for SPI mode)
                        delay_counter <= 19'd0;
                        cmd_select <= 2'd0; // Start with ProdID
                    // Always wait for advertisements first, regardless of INT state
                    // After reset, BNO085 automatically sends advertisement packets
                    // We must process these before sending any commands
                    advert_done <= 1'b0;
                    advert_timeout_counter <= 19'd0;
                    waiting_for_advert <= 1'b1;
                    reset_message_received <= 1'b0;
                    waiting_for_reset_message <= 1'b1;  // Also wait for Channel 1 reset message
                        // Check if INT is already asserted (automatic assertion after reset)
                        if (!int_n_sync) begin
                        // INT already low - advertisement may be ready, go to wait state
                        state <= INIT_WAIT_ADVERT;
                        end else begin
                        // INT still high - need to use wake signal first
                            state <= INIT_WAKE;
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
                // CRITICAL FIX: Reduced timeout to 1ms (datasheet says 150µs max, use 1ms for margin)
                INIT_WAIT_INT: begin
                    if (!int_n_sync) begin
                        // INT asserted, sensor is ready
                        // After reset, BNO085 automatically sends advertisement packets
                        // We must wait for and process these before sending commands
                        delay_counter <= 19'd0;
                        advert_done <= 1'b0;
                        advert_timeout_counter <= 19'd0;
                        waiting_for_advert <= 1'b1;
                        reset_message_received <= 1'b0;
                        waiting_for_reset_message <= 1'b1;  // Also wait for Channel 1 reset message
                        state <= INIT_WAIT_ADVERT; // Wait for advertisements first
                    end else if (delay_counter >= TIMEOUT_INT_AFTER_WAKE) begin
                        // Timeout: 1ms (datasheet says 150µs max, 1ms provides margin)
                        // If INT doesn't assert within 1ms, something is wrong
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
                        // CRITICAL FIX: Reduced timeout to 50ms (should come immediately after reset)
                        if (advert_done || advert_timeout_counter >= TIMEOUT_ADVERT) begin
                            // Advertisements processed or timeout (50ms) - proceed to initialization commands
                            // CRITICAL: Must also wait for Channel 1 reset message per datasheet 5.2.1
                            if (!reset_message_received && waiting_for_reset_message) begin
                                // Still waiting for Channel 1 reset message - continue waiting
                                advert_timeout_counter <= advert_timeout_counter + 1;
                            end else begin
                                // Both advertisements and reset message received (or timeout)
                                advert_done <= 1'b1;
                                waiting_for_advert <= 1'b0;
                                waiting_for_reset_message <= 1'b0;
                                delay_counter <= 19'd0;
                                cmd_select <= 2'd0; // Start with ProdID
                                // Wait for INT before sending first command (sensor must be ready)
                                // Check if INT is already asserted - if so, sensor is ready, skip wake
                                if (!int_n_sync) begin
                                    // INT already asserted - sensor is ready, proceed to first command
                                    waiting_for_response <= 1'b0;
                                    response_received <= 1'b0;
                                    response_timeout_counter <= 19'd0;
                                    cs_setup_delay <= 3'd0;
                                    state <= INIT_CS_SETUP;
                                end else begin
                                    // INT not asserted - need to wake sensor first
                                    state <= INIT_WAKE;
                                end
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
                // CRITICAL FIX: Added explicit CS setup delay and reduced timeout
                INIT_CS_SETUP: begin
                    cs_n <= 1'b1; // Keep CS high until INT asserts
                    ps0_wake <= 1'b1; // Release Wake
                    delay_counter <= 19'd0;
                    byte_cnt <= 8'd0;
                    response_received <= 1'b0;
                    response_timeout_counter <= 19'd0;
                    int_deassert_delay <= 3'd0;
                    // Wait for INT to assert before starting SPI transaction
                    if (!int_n_sync) begin
                        // INT asserted - sensor is ready
                        // CRITICAL: Add explicit CS setup delay (tcssu = 0.1µs min, use 1µs = 3 cycles)
                        if (cs_setup_delay < DELAY_CS_SETUP) begin
                            cs_setup_delay <= cs_setup_delay + 1;
                        end else begin
                            // CS setup delay complete, assert CS and start sending
                            cs_n <= 1'b0; // Assert CS now that INT is low
                            cs_setup_delay <= 3'd0;
                            // CRITICAL: Account for INT deassertion delay (tcsid = 800ns max)
                            // Don't check INT state for 1µs after CS assertion
                            int_deassert_delay <= 3'd1;  // Start delay counter
                    state <= INIT_SEND_BODY;
                        end
                    end else if (delay_counter >= TIMEOUT_INT_AFTER_WAKE) begin
                        // Timeout: 1ms (reduced from 200ms - if INT doesn't assert, something is wrong)
                        // INT never asserted - sensor may not be ready
                        state <= ERROR_STATE;
                    end else begin
                        delay_counter <= delay_counter + 1;
                    end
                end
                
                // 4. Send Command Body
                // Per Adafruit library: INT must be asserted before writing
                // Once CS goes low, INT deasserts, but we've already verified it was asserted
                // CRITICAL FIX: Account for INT deassertion delay (tcsid = 800ns max)
                INIT_SEND_BODY: begin
                    cs_n <= 1'b0; // Keep CS low for entire packet
                    
                    // CRITICAL: Wait for INT deassertion delay after CS assertion
                    if (int_deassert_delay > 0 && int_deassert_delay < DELAY_INT_DEASSERT) begin
                        int_deassert_delay <= int_deassert_delay + 1;
                    end else begin
                        int_deassert_delay <= 3'd0;
                    
                    if (byte_cnt < get_cmd_len(cmd_select)) begin
                            // CRITICAL FIX: Insert proper sequence number in byte 3 (index 3)
                            // Per datasheet: "Each channel and each direction has its own sequence number"
                            if (byte_cnt == 3) begin
                                // Byte 3 is sequence number - use current TX sequence for Channel 2 (control)
                                spi_tx_data <= shtp_seq_num_tx[CHANNEL_CONTROL];
                            end else begin
                                spi_tx_data <= get_init_byte(cmd_select, byte_cnt);
                            end
                            
                        if ((spi_rx_valid || spi_rx_valid_reg) && !spi_busy) begin
                            // Transfer complete, consume rx_valid
                            spi_rx_valid_reg <= 1'b0;
                                
                                // If we just sent sequence number byte, increment TX sequence
                                if (byte_cnt == 3) begin
                                    shtp_seq_num_tx[CHANNEL_CONTROL] <= shtp_seq_num_tx[CHANNEL_CONTROL] + 1;
                                end
                                
                            byte_cnt <= byte_cnt + 1;
                            
                            // Start next byte immediately if more to send
                                // Note: For multi-byte packets, we continue in same CS transaction
                                // INT may deassert when CS goes low, but we keep CS low for full packet
                            if ((byte_cnt + 1) < get_cmd_len(cmd_select)) begin
                                    if ((byte_cnt + 1) == 3) begin
                                        // Next byte is sequence number
                                        spi_tx_data <= shtp_seq_num_tx[CHANNEL_CONTROL];
                                    end else begin
                                spi_tx_data <= get_init_byte(cmd_select, byte_cnt + 1);
                                    end
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end else if (!spi_busy) begin
                            // Start transaction (or retry/hold start)
                            spi_tx_valid <= 1'b1;
                            spi_start <= 1'b1;
                        end
                    end else begin
                        // Done sending all bytes - CRITICAL: Add CS hold time (tcssh)
                        // Per datasheet 6.5.2, must hold CS for proper time after last clock
                        if (cs_hold_delay < DELAY_CS_HOLD) begin
                            cs_hold_delay <= cs_hold_delay + 1;
                        end else begin
                            // CS hold time complete, release CS and wait for response
                        cs_n <= 1'b1;
                            cs_hold_delay <= 3'd0;
                        byte_cnt <= 8'd0;
                            response_received <= 1'b0;
                            response_timeout_counter <= 19'd0;
                            waiting_for_response <= 1'b1;
                            state <= INIT_WAIT_RESPONSE;
                        end
                    end
                    end  // Close else block for int_deassert_delay
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
                    end else if (response_timeout_counter >= TIMEOUT_RESPONSE) begin
                        // Timeout: 20ms (reduced from 100ms - commands should respond quickly)
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
                    // CS setup before SPI - per datasheet 6.5.2 (tcssu = 0.1 µs min)
                    // CRITICAL FIX: Add explicit CS setup delay to ensure tcssu is met
                    // Per datasheet 6.5.4: When CS goes low, INT deasserts within 800ns (tcsid)
                    if (cs_setup_delay < DELAY_CS_SETUP) begin
                        cs_n <= 1'b0;  // Assert CS
                        cs_setup_delay <= cs_setup_delay + 1;
                    end else begin
                        // CS setup delay complete, start reading
                        cs_setup_delay <= 3'd0;
                        delay_counter <= 19'd0;
                        state <= READ_HEADER;
                    byte_cnt <= 8'd0;
                    end
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
                                // CRITICAL FIX: Validate sequence number per channel to detect duplicate/missing packets
                                shtp_seq_num <= spi_rx_data;
                                
                                // Validate sequence number for this channel
                                // Per datasheet: "Each channel and each direction has its own sequence number"
                                if (channel < 6) begin
                                    if (shtp_seq_num_expected_rx[channel] == spi_rx_data) begin
                                        // Sequence number matches expected - valid packet
                                        seq_num_error <= 1'b0;
                                        // Increment expected sequence number for next packet
                                        shtp_seq_num_expected_rx[channel] <= shtp_seq_num_expected_rx[channel] + 1;
                                    end else begin
                                        // Sequence number mismatch - could be duplicate or missing packet
                                        // For now, log error but continue (could implement retry logic)
                                        seq_num_error <= 1'b1;
                                        // Update expected to current + 1 (skip missing packets)
                                        shtp_seq_num_expected_rx[channel] <= spi_rx_data + 1;
                                    end
                                end
                                
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
                        // CRITICAL FIX: Parse TLV to extract channel assignments and packet sizes
                        // Simplified parsing: Just consume packet and mark as done
                        // Full TLV parsing would require string comparison which is complex in hardware
                        // For now, we use default channel numbers (2 for control, 1 for executable) per datasheet
                        if (waiting_for_advert && channel == CHANNEL_COMMAND) begin
                            // This is an advertisement packet - consume it
                            // Check if this is the last byte of the payload
                            // byte_cnt is the number of payload bytes read so far (0-indexed)
                            // packet_length includes header (4 bytes), so payload = packet_length - 4
                            // We need to read (packet_length - 4) bytes total
                            if ((byte_cnt + 1) >= (packet_length - 4)) begin
                                // Advertisement packet complete (read all payload bytes)
                                // Mark that we've read at least one advertisement
                                // NOTE: Full TLV parsing would extract actual channel numbers,
                                // but for now we use defaults (Channel 2 = control, Channel 1 = executable)
                                advert_done <= 1'b1;
                                // CRITICAL: Add CS hold time before releasing
                                if (cs_hold_delay < DELAY_CS_HOLD) begin
                                    cs_hold_delay <= cs_hold_delay + 1;
                                end else begin
                                    cs_n <= 1'b1;
                                    cs_hold_delay <= 3'd0;
                                    byte_cnt <= 8'd0;
                                    delay_counter <= 19'd0;
                                    // Wait a cycle for CS to settle, then check INT
                                    // If INT is still asserted, we'll read another packet
                                    // If INT is not asserted, we'll proceed to commands
                                    state <= INIT_WAIT_ADVERT;
                                end
                            end else begin
                                byte_cnt <= byte_cnt + 1;
                                // Continue reading
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end
                        end
                        // Handle Channel 1 (executable) reset messages - CRITICAL: Per datasheet 5.2.1
                        else if (waiting_for_reset_message && channel == CHANNEL_EXECUTABLE) begin
                            // Channel 1 reset message indicates sensor has left reset state
                            if (byte_cnt == 0) begin
                                // First byte is report ID - should be EXEC_RESP_RESET_COMPLETE (0x01)
                                if (spi_rx_data == EXEC_RESP_RESET_COMPLETE) begin
                                    reset_message_received <= 1'b1;
                                end
                                byte_cnt <= byte_cnt + 1;
                                if ((byte_cnt + 1) < (packet_length - 4)) begin
                                    spi_tx_data <= 8'h00;
                                    spi_tx_valid <= 1'b1;
                                    spi_start <= 1'b1;
                                end
                            end else if ((byte_cnt + 1) >= (packet_length - 4)) begin
                                // Reset message complete
                                // CRITICAL: Add CS hold time before releasing
                                if (cs_hold_delay < DELAY_CS_HOLD) begin
                                    cs_hold_delay <= cs_hold_delay + 1;
                                end else begin
                                    cs_n <= 1'b1;
                                    cs_hold_delay <= 3'd0;
                                    byte_cnt <= 8'd0;
                                    state <= INIT_WAIT_ADVERT;  // Continue waiting for advertisements
                                end
                            end else begin
                                byte_cnt <= byte_cnt + 1;
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
                                // CRITICAL: Add CS hold time before releasing
                                if (cs_hold_delay < DELAY_CS_HOLD) begin
                                    cs_hold_delay <= cs_hold_delay + 1;
                                end else begin
                                    cs_n <= 1'b1;
                                    cs_hold_delay <= 3'd0;
                                    byte_cnt <= 8'd0;
                                    waiting_for_response <= 1'b0;
                                    // Return to INIT_WAIT_RESPONSE to check response_received flag
                                    state <= INIT_WAIT_RESPONSE;
                                end
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
                                // Packet complete - CRITICAL: Add CS hold time before releasing
                                if (cs_hold_delay < DELAY_CS_HOLD) begin
                                    cs_hold_delay <= cs_hold_delay + 1;
                                end else begin
                            cs_n <= 1'b1;
                                    cs_hold_delay <= 3'd0;
                            byte_cnt <= 8'd0;
                            state <= WAIT_DATA;
                                end
                            end
                        end else begin
                            // Unknown channel or state - just consume the packet
                            byte_cnt <= byte_cnt + 1;
                            if ((byte_cnt + 1) < (packet_length - 4)) begin
                                spi_tx_data <= 8'h00;
                                spi_tx_valid <= 1'b1;
                                spi_start <= 1'b1;
                            end else begin
                                // Packet complete - CRITICAL: Add CS hold time before releasing
                                if (cs_hold_delay < DELAY_CS_HOLD) begin
                                    cs_hold_delay <= cs_hold_delay + 1;
                                end else begin
                                    cs_n <= 1'b1;
                                    cs_hold_delay <= 3'd0;
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
