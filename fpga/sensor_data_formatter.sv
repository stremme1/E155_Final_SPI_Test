`timescale 1ns / 1ps

// Sensor Data Formatter
// Packages single BNO085 sensor data into bytes for MCU transmission
// Format: [Header][Sensor_Quat][Sensor_Gyro]
// Header: 1 byte (0xAA = data packet)
// Sensor: 14 bytes (4 quat + 3 gyro = 7 values * 2 bytes)

module sensor_data_formatter (
    input  logic        clk,
    input  logic        rst_n,
    
    // Sensor data inputs
    input  logic        data_ready,  // New data available
    input  logic        sensor_valid,
    
    // Sensor data
    input  logic signed [15:0] quat_w, quat_x, quat_y, quat_z,
    input  logic signed [15:0] gyro_x, gyro_y, gyro_z,
    
    // SPI slave interface
    output logic        tx_data_ready,  // Data ready for SPI slave
    output logic [7:0]  tx_data,        // Data byte to send
    input  logic        tx_ack,         // Acknowledge from SPI slave
    
    // Status
    output logic        busy            // Formatter is busy
);

    // Packet format: 15 bytes total
    // Byte 0: Header (0xAA)
    // Bytes 1-14: Sensor (W,X,Y,Z quat, X,Y,Z gyro) - 14 bytes
    
    localparam PACKET_SIZE = 6'd15;
    localparam HEADER_BYTE = 8'hAA;
    
    typedef enum logic [2:0] {
        IDLE,
        WAIT_DATA,
        SENDING_HEADER,
        SENDING_SENSOR,
        WAIT_ACK
    } state_t;
    
    state_t state;
    logic [5:0] byte_index;
    
    // Data buffer - store sensor data when ready
    logic signed [15:0] quat_w_buf, quat_x_buf, quat_y_buf, quat_z_buf;
    logic signed [15:0] gyro_x_buf, gyro_y_buf, gyro_z_buf;
    
    logic data_captured;
    
    // Helper function to extract LSB or MSB from 16-bit value
    // Using shift and mask to avoid bit select issues in Icarus Verilog
    function automatic [7:0] get_byte_lsb(input [15:0] value);
        get_byte_lsb = value & 8'hFF;
    endfunction
    
    function automatic [7:0] get_byte_msb(input [15:0] value);
        get_byte_msb = (value >> 8) & 8'hFF;
    endfunction
    
    // Combinational logic for tx_data - updates immediately when byte_index changes
    // This solves the timing issue where case statement uses old byte_index value
    always_comb begin
        case (state)
            SENDING_HEADER: begin
                tx_data = HEADER_BYTE;
            end
            
            SENDING_SENSOR: begin
                case (byte_index)
                    1: tx_data = get_byte_lsb(quat_w_buf);      // LSB
                    2: tx_data = get_byte_msb(quat_w_buf);     // MSB
                    3: tx_data = get_byte_lsb(quat_x_buf);
                    4: tx_data = get_byte_msb(quat_x_buf);
                    5: tx_data = get_byte_lsb(quat_y_buf);
                    6: tx_data = get_byte_msb(quat_y_buf);
                    7: tx_data = get_byte_lsb(quat_z_buf);
                    8: tx_data = get_byte_msb(quat_z_buf);
                    9: tx_data = get_byte_lsb(gyro_x_buf);
                    10: tx_data = get_byte_msb(gyro_x_buf);
                    11: tx_data = get_byte_lsb(gyro_y_buf);
                    12: tx_data = get_byte_msb(gyro_y_buf);
                    13: tx_data = get_byte_lsb(gyro_z_buf);
                    14: tx_data = get_byte_msb(gyro_z_buf);
                    default: tx_data = 8'd0;
                endcase
            end
            
            default: begin
                tx_data = 8'd0;
            end
        endcase
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            byte_index <= 6'd0;
            tx_data_ready <= 1'b0;
            // tx_data is now combinational, no need to reset
            data_captured <= 1'b0;
            busy <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    byte_index <= 6'd0;
                    tx_data_ready <= 1'b0;
                    data_captured <= 1'b0;
                    busy <= 1'b0;
                    // When data_ready is asserted, capture data and start sending
                    if (data_ready && !data_captured) begin
                        // Capture sensor data - these are registers that hold their values
                        // until new data arrives, so they should be stable
                        quat_w_buf <= quat_w;
                        quat_x_buf <= quat_x;
                        quat_y_buf <= quat_y;
                        quat_z_buf <= quat_z;
                        gyro_x_buf <= gyro_x;
                        gyro_y_buf <= gyro_y;
                        gyro_z_buf <= gyro_z;
                        data_captured <= 1'b1;
                        state <= SENDING_HEADER;
                        busy <= 1'b1;
                    end
                end
                
                WAIT_DATA: begin
                    // This state is no longer needed - we capture in IDLE
                    state <= SENDING_HEADER;
                end
                
                SENDING_HEADER: begin
                    // tx_data is set by combinational logic above
                    if (!tx_data_ready) begin
                        tx_data_ready <= 1'b1;
                    end else if (tx_ack) begin
                        tx_data_ready <= 1'b0;
                        // Transition to SENDING_SENSOR and set up first byte
                        state <= SENDING_SENSOR;
                        byte_index <= 6'd1;  // Start sensor data
                        // tx_data will be set by combinational logic based on byte_index
                    end
                end
                
                SENDING_SENSOR: begin
                    // tx_data is set by combinational logic above based on byte_index
                    // Manage tx_data_ready flag and byte_index
                    if (!tx_data_ready) begin
                        // First time entering this state - tx_data is set by combinational logic
                        tx_data_ready <= 1'b1;
                    end else if (tx_ack) begin
                        // Byte was sent - prepare for next byte
                        tx_data_ready <= 1'b0;
                        if (byte_index < 14) begin
                            // Increment byte_index - tx_data will update immediately via combinational logic
                            byte_index <= byte_index + 1;
                        end else begin
                            state <= WAIT_ACK;
                        end
                    end
                end
                
                WAIT_ACK: begin
                    // Wait for final ack, then return to idle
                    if (tx_ack) begin
                        state <= IDLE;
                        busy <= 1'b0;
                        byte_index <= 6'd0;
                        // Note: data_ready will be cleared by dual_bno085_controller
                        // when it sees that data has been sent
                    end
                end
            endcase
        end
    end
    
endmodule

