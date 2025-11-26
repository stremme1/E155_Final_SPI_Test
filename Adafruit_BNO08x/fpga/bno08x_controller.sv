/**
 * BNO08X Controller Module
 * 
 * Implements initialization, sensor configuration, and data reading
 * for the BNO08X sensor using SPI interface
 * 
 * Based on Adafruit_BNO08x library and BNO08X datasheet
 */

module bno08x_controller (
    input  logic        clk,              // 3MHz clock
    input  logic        rst_n,            // Active low reset
    
    // SPI Interface
    output logic        spi_cs_n,
    output logic        spi_sck,
    output logic        spi_mosi,
    input  logic        spi_miso,
    
    // Interrupt and Wake signals
    input  logic        h_intn,           // BNO08X interrupt (active low)
    output logic        wake,             // Wake signal (active low)
    
    // Control Interface
    input  logic        enable_sensor,    // Enable sensor reading
    input  logic [7:0]  sensor_id,       // Sensor ID to enable (SH2_SensorId_t)
    input  logic [31:0] report_interval, // Report interval in microseconds
    
    // Sensor Data Output
    output logic        data_ready,       // New sensor data available
    output logic [7:0]  sensor_report_id, // Report ID
    output logic [7:0]  sensor_data [0:15], // Sensor data buffer
    output logic [4:0]  sensor_data_len,  // Length of sensor data
    
    // Status
    output logic        initialized,     // Device initialized
    output logic        error            // Error occurred
);

    // SHTP Channel definitions
    localparam CHANNEL_COMMAND = 8'h00;
    localparam CHANNEL_EXECUTABLE = 8'h01;
    localparam CHANNEL_CONTROL = 8'h02;
    localparam CHANNEL_INPUT_REPORTS = 8'h03;
    localparam CHANNEL_WAKE_REPORTS = 8'h04;
    localparam CHANNEL_GYRO_RV = 8'h05;
    
    // Report IDs
    localparam REPORT_ID_PRODUCT_ID_REQUEST = 8'hF9;
    localparam REPORT_ID_PRODUCT_ID_RESPONSE = 8'hF8;
    localparam REPORT_ID_SET_FEATURE = 8'hFD;
    localparam REPORT_ID_GET_FEATURE_RESPONSE = 8'hFC;
    
    // Sensor IDs (from sh2.h)
    localparam SH2_GYROSCOPE_CALIBRATED = 8'h02;
    localparam SH2_GAME_ROTATION_VECTOR = 8'h08;
    localparam SH2_ROTATION_VECTOR = 8'h05;
    
    // Controller State Machine
    typedef enum logic [4:0] {
        INIT_RESET,
        INIT_WAIT_INT,
        INIT_SEND_RESET,
        INIT_WAIT_RESPONSE,
        INIT_GET_PRODUCT_ID,
        INIT_WAIT_PRODUCT_ID,
        IDLE,
        ENABLE_SENSOR,
        WAIT_SENSOR_RESPONSE,
        READ_SENSOR_DATA,
        PROCESS_SENSOR_DATA
    } controller_state_t;
    
    controller_state_t state, next_state;
    
    // SPI Master Interface
    logic spi_start;
    logic spi_write_en;
    logic [15:0] spi_data_length;
    logic [7:0] spi_tx_data;
    logic [7:0] spi_rx_data;
    logic spi_data_valid;
    logic spi_done;
    logic spi_busy;
    
    // Internal signals
    logic [15:0] timeout_counter;
    logic [7:0] sequence_num;
    logic [7:0] rx_buffer [0:255];
    logic [15:0] rx_buffer_idx;
    logic [15:0] packet_length;
    logic [7:0] packet_channel;
    logic [7:0] packet_sequence;
    logic packet_ready;
    logic [31:0] report_interval_counter;
    
    // Instantiate SPI Master
    bno08x_spi_master spi_master (
        .clk(clk),
        .rst_n(rst_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .start_transfer(spi_start),
        .write_en(spi_write_en),
        .data_length(spi_data_length),
        .tx_data(spi_tx_data),
        .rx_data(spi_rx_data),
        .data_valid(spi_data_valid),
        .transfer_done(spi_done),
        .transfer_busy(spi_busy)
    );
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= INIT_RESET;
            timeout_counter <= 16'd0;
            sequence_num <= 8'd0;
            rx_buffer_idx <= 16'd0;
            packet_length <= 16'd0;
            initialized <= 1'b0;
            error <= 1'b0;
            wake <= 1'b1; // Wake inactive (high)
            data_ready <= 1'b0;
            report_interval_counter <= 32'd0;
        end else begin
            state <= next_state;
            
            // Timeout counter
            if (state != next_state) begin
                timeout_counter <= 16'd0;
            end else if (timeout_counter < 16'hFFFF) begin
                timeout_counter <= timeout_counter + 1;
            end
            
            // Report interval counter
            if (initialized && enable_sensor) begin
                if (report_interval_counter < report_interval) begin
                    report_interval_counter <= report_interval_counter + 1;
                end else begin
                    report_interval_counter <= 32'd0;
                end
            end
            
            // SPI interface handling
            case (state)
                INIT_RESET: begin
                    wake <= 1'b0; // Assert wake
                    initialized <= 1'b0;
                end
                
                INIT_WAIT_INT: begin
                    wake <= 1'b1; // Deassert wake
                    if (h_intn == 1'b0) begin
                        timeout_counter <= 16'd0;
                    end
                end
                
                INIT_GET_PRODUCT_ID: begin
                    sequence_num <= sequence_num + 1;
                end
                
                READ_SENSOR_DATA: begin
                    if (spi_data_valid) begin
                        rx_buffer[rx_buffer_idx] <= spi_rx_data;
                        rx_buffer_idx <= rx_buffer_idx + 1;
                    end
                    if (spi_done) begin
                        rx_buffer_idx <= 16'd0;
                    end
                end
                
                PROCESS_SENSOR_DATA: begin
                    // Parse SHTP header
                    packet_length <= {rx_buffer[1], rx_buffer[0]};
                    packet_channel <= rx_buffer[2];
                    packet_sequence <= rx_buffer[3];
                    
                    // Extract sensor data (skip SHTP header and report ID)
                    if (packet_channel == CHANNEL_INPUT_REPORTS || 
                        packet_channel == CHANNEL_GYRO_RV) begin
                        sensor_report_id <= rx_buffer[4];
                        for (int i = 0; i < 16; i++) begin
                            if (i < packet_length - 5) begin
                                sensor_data[i] <= rx_buffer[5 + i];
                            end
                        end
                        sensor_data_len <= (packet_length > 21) ? 5'd16 : (packet_length - 5);
                        data_ready <= 1'b1;
                    end
                end
            endcase
            
            if (data_ready && !enable_sensor) begin
                data_ready <= 1'b0;
            end
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        spi_start = 1'b0;
        spi_write_en = 1'b0;
        spi_data_length = 16'd0;
        spi_tx_data = 8'd0;
        
        case (state)
            INIT_RESET: begin
                if (timeout_counter > 16'd100) begin // ~33us at 3MHz
                    next_state = INIT_WAIT_INT;
                end
            end
            
            INIT_WAIT_INT: begin
                if (h_intn == 1'b0) begin
                    next_state = INIT_GET_PRODUCT_ID;
                end else if (timeout_counter > 16'd3000) begin // ~1ms timeout
                    next_state = INIT_RESET;
                end
            end
            
            INIT_GET_PRODUCT_ID: begin
                // Send Product ID Request
                // SHTP Header: length=5, channel=2, sequence=sequence_num
                // Report ID: 0xF9
                spi_start = 1'b1;
                spi_write_en = 1'b1;
                spi_data_length = 16'd5;
                spi_tx_data = (rx_buffer_idx == 0) ? 8'd5 : // Length LSB
                              (rx_buffer_idx == 1) ? 8'd0 : // Length MSB
                              (rx_buffer_idx == 2) ? CHANNEL_CONTROL : // Channel
                              (rx_buffer_idx == 3) ? sequence_num : // Sequence
                              REPORT_ID_PRODUCT_ID_REQUEST; // Report ID
                if (spi_done) begin
                    next_state = INIT_WAIT_PRODUCT_ID;
                end
            end
            
            INIT_WAIT_PRODUCT_ID: begin
                if (h_intn == 1'b0) begin
                    next_state = READ_SENSOR_DATA;
                end else if (timeout_counter > 16'd30000) begin // ~10ms timeout
                    next_state = INIT_RESET;
                end
            end
            
            READ_SENSOR_DATA: begin
                if (h_intn == 1'b0) begin
                    // Read SHTP header first (4 bytes)
                    spi_start = 1'b1;
                    spi_write_en = 1'b0;
                    if (rx_buffer_idx == 0) begin
                        spi_data_length = 16'd4;
                    end else begin
                        // Read full packet
                        spi_data_length = packet_length;
                    end
                end
                if (spi_done && rx_buffer_idx > 0) begin
                    next_state = PROCESS_SENSOR_DATA;
                end
            end
            
            PROCESS_SENSOR_DATA: begin
                // Process will be handled in next_state logic
            end
            
            IDLE: begin
                // Wait for enable or interrupt
            end
            
            ENABLE_SENSOR: begin
                // Send Set Feature Command
                // Format: SHTP Header (4 bytes) + Set Feature Report (17 bytes)
                // Report ID: 0xFD
                // This would need to be properly formatted with all fields
            end
            
            WAIT_SENSOR_RESPONSE: begin
                // Wait for interrupt or timeout
            end
        endcase
    end
    
endmodule

