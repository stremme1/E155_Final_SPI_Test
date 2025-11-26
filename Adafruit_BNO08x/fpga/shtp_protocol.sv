/**
 * SHTP (Sensor Hub Transport Protocol) Handler
 * 
 * Handles SHTP packet formatting and parsing according to BNO08X datasheet
 * 
 * SHTP Header Format (4 bytes):
 * Byte 0: Length LSB
 * Byte 1: Length MSB (bit 15 = continuation bit)
 * Byte 2: Channel
 * Byte 3: Sequence Number
 */

module shtp_protocol (
    input  logic        clk,
    input  logic        rst_n,
    
    // Packet assembly interface
    input  logic        start_packet,
    input  logic [7:0] channel,
    input  logic [7:0] sequence_num,
    input  logic [15:0] payload_length,
    
    // Data interface
    input  logic        data_valid,
    input  logic [7:0] data_in,
    output logic [7:0] data_out,
    output logic        data_out_valid,
    
    // Packet interface
    output logic        packet_ready,
    output logic [15:0] packet_length,
    output logic [7:0] packet_channel,
    output logic [7:0] packet_sequence,
    input  logic        packet_ack
);

    // SHTP Header structure
    typedef struct packed {
        logic [15:0] length;      // Bits 14:0 = length, bit 15 = continuation
        logic [7:0]  channel;
        logic [7:0]  sequence;
    } shtp_header_t;
    
    shtp_header_t header;
    logic [15:0] byte_counter;
    logic [1:0] header_byte_idx;
    logic assembling_header;
    
    // Packet assembly state machine
    typedef enum logic [1:0] {
        IDLE,
        HEADER,
        PAYLOAD
    } packet_state_t;
    
    packet_state_t state, next_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            header.length <= 16'd0;
            header.channel <= 8'd0;
            header.sequence <= 8'd0;
            byte_counter <= 16'd0;
            header_byte_idx <= 2'd0;
            assembling_header <= 1'b0;
            packet_ready <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    packet_ready <= 1'b0;
                    if (start_packet) begin
                        header.length <= payload_length + 4; // Include header
                        header.channel <= channel;
                        header.sequence <= sequence_num;
                        byte_counter <= 16'd0;
                        header_byte_idx <= 2'd0;
                        assembling_header <= 1'b1;
                    end
                end
                
                HEADER: begin
                    if (data_valid) begin
                        case (header_byte_idx)
                            2'd0: header.length[7:0] <= data_in;
                            2'd1: header.length[15:8] <= data_in;
                            2'd2: header.channel <= data_in;
                            2'd3: begin
                                header.sequence <= data_in;
                                assembling_header <= 1'b0;
                            end
                        endcase
                        header_byte_idx <= header_byte_idx + 1;
                        if (header_byte_idx == 2'd3) begin
                            byte_counter <= 16'd4; // Header complete
                        end
                    end
                end
                
                PAYLOAD: begin
                    if (data_valid) begin
                        byte_counter <= byte_counter + 1;
                    end
                    if (byte_counter >= header.length) begin
                        packet_ready <= 1'b1;
                    end
                    if (packet_ack) begin
                        packet_ready <= 1'b0;
                    end
                end
            endcase
        end
    end
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start_packet) begin
                    next_state = HEADER;
                end
            end
            
            HEADER: begin
                if (header_byte_idx == 2'd3 && data_valid) begin
                    next_state = PAYLOAD;
                end
            end
            
            PAYLOAD: begin
                if (packet_ack) begin
                    next_state = IDLE;
                end
            end
        endcase
    end
    
    // Output assignments
    assign packet_length = header.length;
    assign packet_channel = header.channel;
    assign packet_sequence = header.sequence;
    assign data_out = data_in;
    assign data_out_valid = data_valid && (state == PAYLOAD);
    
endmodule

