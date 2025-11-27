`timescale 1ns / 1ps

// Mock BNO085 Sensor Model
// Simulates BNO085 behavior according to datasheet:
// - Responds to PS0/WAKE pin
// - Handles SHTP protocol over SPI Mode 3
// - Responds to initialization commands
// - Sends data reports when requested

module mock_bno085 (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ps0_wake,  // PS0/WAKE pin (active low)
    input  logic        cs_n,
    input  logic        sclk,
    input  logic        mosi,
    output logic        miso,
    output logic        int_n
);

    // Internal state
    logic [7:0] rx_byte;
    logic [3:0] rx_bit_cnt;
    logic [3:0] tx_bit_cnt;
    logic [7:0] tx_byte;
    logic [7:0] rx_buffer [0:31];
    integer rx_ptr = 0;
    integer rx_byte_count = 0;
    
    // Response buffers
    logic [7:0] response_queue [0:255];
    integer response_ptr = 0;
    integer response_len = 0;
    
    // Sensor state
    typedef enum {
        IDLE,
        ASLEEP,
        AWAKE,
        RECEIVING,
        RESPONDING
    } sensor_state_t;
    
    sensor_state_t state;
    
    // Wake detection
    logic ps0_sync, ps0_prev;
    logic wake_triggered;
    
    // Initialize state
    initial begin
        state = ASLEEP;
        wake_triggered = 0;
        rx_bit_cnt = 0;
        tx_bit_cnt = 0;
        rx_byte = 0;
        tx_byte = 0;
    end
    
    // Command handling
    logic [15:0] received_length = 0;
    logic [7:0] received_channel = 0;
    logic [7:0] received_seq = 0;
    integer cmd_byte_count = 0;
    
    // Product ID response - use function instead of array for iverilog compatibility
    function [7:0] get_prod_id_byte(input [3:0] idx);
        case (idx)
            0: get_prod_id_byte = 8'h09;  // Length LSB (9 bytes)
            1: get_prod_id_byte = 8'h00;  // Length MSB
            2: get_prod_id_byte = 8'h01;  // Channel (Command)
            3: get_prod_id_byte = 8'h00;  // Sequence
            4: get_prod_id_byte = 8'hF1;  // Get Product ID Response
            5: get_prod_id_byte = 8'h00;  // Product ID LSB
            6: get_prod_id_byte = 8'h00;  // Product ID
            7: get_prod_id_byte = 8'hA0;  // Product ID
            8: get_prod_id_byte = 8'h02;  // Product ID MSB (BNO085 = 0x0002A0)
            default: get_prod_id_byte = 8'h00;
        endcase
    endfunction
    
    // Set Feature Response (ACK)
    function [7:0] get_set_feature_ack(input [7:0] seq);
        case (seq)
            1: return 8'h05; // Length LSB (5 bytes)
            2: return 8'h00; // Length MSB
            3: return 8'h02; // Channel (Control)
            4: return seq;    // Sequence
            5: return 8'hFE; // Set Feature Response
            default: return 8'h00;
        endcase
    endfunction

    // Synchronize PS0
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ps0_sync <= 1'b1;
            ps0_prev <= 1'b1;
        end else begin
            ps0_sync <= ps0_wake;
            ps0_prev <= ps0_sync;
        end
    end
    
    // Wake detection: PS0 falling edge wakes sensor
    logic cs_n_sync, cs_n_prev;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n_sync <= 1'b1;
            cs_n_prev <= 1'b1;
        end else begin
            cs_n_sync <= cs_n;
            cs_n_prev <= cs_n_sync;
        end
    end
    
    // Wake detection and INT control
    logic ps0_falling_edge;
    assign ps0_falling_edge = ps0_prev && !ps0_sync;
    
    // Counter for INT deassertion delay
    logic [7:0] int_deassert_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wake_triggered <= 0;
            state <= ASLEEP;
            int_n <= 1'b1;
            int_deassert_cnt <= 8'd0;
        end else begin
            // Detect PS0 falling edge
            if (ps0_falling_edge) begin
                // PS0 went low - wake up
                wake_triggered <= 1;
                state <= AWAKE;
            end
            
            // Deassert INT when CS goes low (per datasheet: "deassert as soon as CS is detected")
            if (cs_n_prev && !cs_n_sync && int_n == 0) begin
                int_n <= 1'b1;
                int_deassert_cnt <= 8'd0;
            end
            
            // Also deassert INT after a delay if CS goes high (transaction complete)
            if (cs_n_sync && !cs_n_prev && int_n == 0) begin
                // Start counter to deassert INT after delay
                int_deassert_cnt <= 8'd10; // ~3.3us at 3MHz
            end else if (int_deassert_cnt > 0) begin
                int_deassert_cnt <= int_deassert_cnt - 1;
                if (int_deassert_cnt == 1) begin
                    int_n <= 1'b1;
                end
            end
        end
    end
    
    // Handle INT assertion with delay when PS0 goes low
    // Use edge detection on the actual ps0_wake signal
    logic ps0_wake_prev;
    always_ff @(posedge clk) begin
        ps0_wake_prev <= ps0_wake;
    end
    
    logic ps0_falling;
    assign ps0_falling = ps0_wake_prev && !ps0_wake;
    
    logic [7:0] int_assert_delay;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_assert_delay <= 8'd0;
            int_n <= 1'b1;
        end else begin
            if (ps0_falling) begin
                // PS0 went low, start delay counter to assert INT
                int_assert_delay <= 8'd3; // ~1us at 3MHz
                // Also deassert any pending deassert counter
                int_deassert_cnt <= 8'd0;
            end else if (int_assert_delay > 0) begin
                int_assert_delay <= int_assert_delay - 1;
                if (int_assert_delay == 1) begin
                    int_n <= 1'b0; // Assert INT after delay
                end
            end
        end
    end

    // SPI Receive Logic (Mode 3: CPOL=1, CPHA=1)
    // Sample MOSI on rising edge of SCLK
    always @(posedge sclk) begin
        if (!cs_n) begin
            rx_byte = {rx_byte[6:0], mosi};
            rx_bit_cnt = rx_bit_cnt + 1;
            
            if (rx_bit_cnt == 8) begin
                rx_bit_cnt = 0;
                rx_buffer[rx_ptr] = rx_byte;
                rx_ptr = rx_ptr + 1;
                rx_byte_count = rx_byte_count + 1;
                
                // Parse SHTP header
                if (rx_byte_count == 1) begin
                    received_length[7:0] = rx_byte;
                end else if (rx_byte_count == 2) begin
                    received_length[15:8] = rx_byte;
                end else if (rx_byte_count == 3) begin
                    received_channel = rx_byte;
                end else if (rx_byte_count == 4) begin
                    received_seq = rx_byte;
                    // Now we know the command, prepare response
                    prepare_response();
                end
            end
        end
    end
    
    // SPI Transmit Logic
    // Shift MISO on falling edge of SCLK (Mode 3: setup before rising edge)
    always @(negedge sclk) begin
        if (!cs_n) begin
            if (response_len > 0 && response_ptr < response_len) begin
                miso = tx_byte[7];
                tx_bit_cnt = tx_bit_cnt + 1;
                
                if (tx_bit_cnt == 8) begin
                    // Byte complete, load next byte
                    tx_bit_cnt = 0;
                    response_ptr = response_ptr + 1;
                    if (response_ptr < response_len) begin
                        tx_byte = response_queue[response_ptr];
                    end else begin
                        tx_byte = 8'h00; // Padding
                    end
                end else begin
                    tx_byte = {tx_byte[6:0], 1'b0};
                end
            end else begin
                miso = 1'b0;
            end
        end
    end
    
    // Reset on CS high
    always @(posedge cs_n) begin
        rx_ptr = 0;
        rx_byte_count = 0;
        response_ptr = 0;
        rx_bit_cnt = 0;
        tx_bit_cnt = 0;
        rx_byte = 0;
        received_length = 0;
        // Reload first byte for next transaction
        if (response_len > 0) begin
            tx_byte = response_queue[0];
        end
    end
    
    // Prepare response based on received command
    task prepare_response;
        integer i;
        begin
            // Check channel and command
            if (received_channel == 8'h01) begin // Command channel
                // Check for Product ID Request (0xF9)
                if (rx_buffer[4] == 8'hF9) begin
                    // Send Product ID response
                    for (i = 0; i < 9; i = i + 1) begin
                        response_queue[i] = get_prod_id_byte(i);
                    end
                    response_len = 9;
                    response_ptr = 0;
                    tx_byte = response_queue[0];
                    // Assert INT to indicate response ready
                    #500;
                    int_n = 1'b0;
                end
            end else if (received_channel == 8'h02) begin // Control channel
                // Check for Set Feature (0xFD)
                if (rx_buffer[4] == 8'hFD) begin
                    // Send Set Feature ACK
                    response_queue[0] = 8'h05; // Length LSB
                    response_queue[1] = 8'h00; // Length MSB
                    response_queue[2] = 8'h02; // Channel
                    response_queue[3] = received_seq; // Sequence
                    response_queue[4] = 8'hFE; // Set Feature Response
                    response_len = 5;
                    response_ptr = 0;
                    tx_byte = response_queue[0];
                    // Don't assert INT for ACK - controller doesn't read it during init
                    // The controller will wait for data reports later
                end
            end
        end
    endtask
    
    // Task to queue a report packet (Rotation Vector)
    // SHTP Packet Structure:
    // Header (4 bytes): LenLSB, LenMSB, Channel, Seq
    // Payload: Report ID, Sequence, Status, Delay, Data...
    task send_rotation_vector;
        input [15:0] x, y, z, w; // Quaternions (little-endian 16-bit)
        integer i;
        begin
            // Header (18 bytes total)
            response_queue[0] = 8'h12; // Length LSB (18)
            response_queue[1] = 8'h00; // Length MSB
            response_queue[2] = 8'h05; // Channel (Reports)
            response_queue[3] = 8'h00; // Sequence
            
            // Report Body
            response_queue[4] = 8'h05; // Report ID (Rotation Vector)
            response_queue[5] = 8'h00; // Sequence
            response_queue[6] = 8'h00; // Status
            response_queue[7] = 8'h00; // Delay
            
            // Quaternion data (little-endian)
            response_queue[8] = x[7:0];   // X LSB
            response_queue[9] = x[15:8];  // X MSB
            response_queue[10] = y[7:0];  // Y LSB
            response_queue[11] = y[15:8]; // Y MSB
            response_queue[12] = z[7:0];   // Z LSB
            response_queue[13] = z[15:8]; // Z MSB
            response_queue[14] = w[7:0];  // W LSB
            response_queue[15] = w[15:8]; // W MSB
            
            response_queue[16] = 8'h00; // Accuracy LSB
            response_queue[17] = 8'h00; // Accuracy MSB
            
            response_len = 18;
            response_ptr = 0;
            tx_byte = response_queue[0];
            
            // Indicate data ready by pulling INT low
            #1000;
            int_n = 1'b0;
        end
    endtask
    
    // Task to queue a gyroscope report
    task send_gyroscope;
        input [15:0] x, y, z; // Angular velocity (little-endian 16-bit)
        begin
            // Header (14 bytes total)
            response_queue[0] = 8'h0E; // Length LSB (14)
            response_queue[1] = 8'h00; // Length MSB
            response_queue[2] = 8'h05; // Channel (Reports)
            response_queue[3] = 8'h00; // Sequence
            
            // Report Body
            response_queue[4] = 8'h02; // Report ID (Calibrated Gyroscope per Fig 1-34)
            response_queue[5] = 8'h00; // Sequence
            response_queue[6] = 8'h00; // Status
            response_queue[7] = 8'h00; // Delay
            
            // Gyro data (little-endian)
            response_queue[8] = x[7:0];   // X LSB
            response_queue[9] = x[15:8];  // X MSB
            response_queue[10] = y[7:0];   // Y LSB
            response_queue[11] = y[15:8]; // Y MSB
            response_queue[12] = z[7:0];  // Z LSB
            response_queue[13] = z[15:8]; // Z MSB
            
            response_len = 14;
            response_ptr = 0;
            tx_byte = response_queue[0];
            
            // Indicate data ready
            #1000;
            int_n = 1'b0;
        end
    endtask

endmodule
