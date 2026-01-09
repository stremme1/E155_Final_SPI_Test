`timescale 1ns / 1ps

// SPI slave to MCU
// Sends sensor data packets to MCU

module spi_slave_mcu(
    input  logic        clk,           // FPGA system clock
    input  logic        cs_n,          // Chip select from MCU (active low, PA11)
    input  logic        sck,           // SPI clock from MCU (PB3)
    input  logic        sdi,            // SPI data in (MOSI from MCU - IGNORED in read-only mode)
    output logic        sdo,           // SPI data out (MISO to MCU - PB4)
    
    // Raw packet buffer from arduino_spi_slave (16 bytes)
    input  logic [7:0]  packet_buffer [0:15],
    
    // Status inputs (from header check)
    input  logic        initialized,
    input  logic        error
);

    // pass packet buffer to MCU
    
    localparam PACKET_SIZE = 16;
    
    localparam TEST_MODE = 1'b1;
    
    // test pattern
    logic [7:0] test_pattern [0:15];
    assign test_pattern[0] = 8'hAA;
    assign test_pattern[1] = 8'h11;
    assign test_pattern[2] = 8'h22;
    assign test_pattern[3] = 8'h33;
    assign test_pattern[4] = 8'h44;
    assign test_pattern[5] = 8'h55;
    assign test_pattern[6] = 8'h66;
    assign test_pattern[7] = 8'h77;
    assign test_pattern[8] = 8'h88;
    assign test_pattern[9] = 8'h99;
    assign test_pattern[10] = 8'hAA;
    assign test_pattern[11] = 8'hBB;
    assign test_pattern[12] = 8'hCC;
    assign test_pattern[13] = 8'hDD;
    assign test_pattern[14] = 8'hEE;
    assign test_pattern[15] = 8'hFF;
    
    // clock domain crossing
    logic cs_n_sync1, cs_n_sync2;
    always_ff @(posedge clk) begin
        cs_n_sync1 <= cs_n;
        cs_n_sync2 <= cs_n_sync1;
    end
    
    // packet snapshot
    logic [7:0] packet_snapshot [0:PACKET_SIZE-1];
    
    // init snapshot
    initial begin
        for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
            packet_snapshot[i] = 8'h00;
        end
    end
    
    // capture packet on CS falling edge
    always_ff @(posedge clk) begin
        if (cs_n) begin
            if (TEST_MODE) begin
                for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
                    packet_snapshot[i] <= test_pattern[i];
                end
            end else begin
                for (int i = 0; i < PACKET_SIZE; i = i + 1) begin
                    packet_snapshot[i] <= packet_buffer[i];
                end
            end
        end
    end
    
    // create 128-bit packet
    logic [127:0] tx_packet;
    assign tx_packet = {
                packet_snapshot[0], packet_snapshot[1], packet_snapshot[2], packet_snapshot[3],
                packet_snapshot[4], packet_snapshot[5], packet_snapshot[6], packet_snapshot[7],
                packet_snapshot[8], packet_snapshot[9], packet_snapshot[10], packet_snapshot[11],
                packet_snapshot[12], packet_snapshot[13], packet_snapshot[14], packet_snapshot[15]
    };
    
    // shift registers
    logic [7:0] shift_out;
    logic [3:0] byte_count;
    logic [2:0] bit_count;
    
    logic cs_n_sync_sck = 1'b1;
    logic cs_n_prev_sck = 1'b1;
    logic seen_first_rising = 1'b0;
    
    // sync CS to SCK domain
    always_ff @(negedge sck) begin
        cs_n_sync_sck <= cs_n;
        cs_n_prev_sck <= cs_n_sync_sck;
    end
    
    // detect first SCK rising edge
    always_ff @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            seen_first_rising <= 1'b0;
        end else if (!cs_n && !seen_first_rising) begin
            seen_first_rising <= 1'b1;
        end
    end
    
    // detect CS falling edge
    logic cs_falling_edge_sck;
    assign cs_falling_edge_sck = cs_n_prev_sck && !cs_n_sync_sck;
    
    // SPI slave logic
    logic [7:0] first_byte_value;
    assign first_byte_value = packet_snapshot[0];
    
    always_ff @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            byte_count <= 0;
            bit_count  <= 0;
            shift_out  <= first_byte_value;
        end else begin
            if (cs_falling_edge_sck) begin
                shift_out  <= first_byte_value;
                byte_count <= 0;
                bit_count  <= 0;
            end else if (seen_first_rising) begin
                if (bit_count == 3'd7) begin
                    if (byte_count < 15) begin
                        byte_count <= byte_count + 1;
                        bit_count  <= 0;
                        shift_out <= tx_packet[127 - (byte_count+1)*8 -: 8];
                    end else begin
                        byte_count <= byte_count + 1;
                        bit_count  <= 0;
                        shift_out <= 8'h00;
                    end
                end else begin
                    shift_out <= {shift_out[6:0], 1'b0};
                    bit_count <= bit_count + 1;
                end
            end
        end
    end
    
    // MISO output
    assign sdo = cs_n ? 1'bz : shift_out[7];
    
endmodule
