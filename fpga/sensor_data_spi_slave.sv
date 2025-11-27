/**
 * Sensor Data SPI Slave Module
 * 
 * Sends multi-byte sensor data packets to MCU via SPI
 * EXACT COPY of Lab07 aes_spi pattern, adapted for 32-byte (256-bit) data packets
 * 
 * Key insight from Lab07: 
 * - cyphertext is an INPUT to aes_spi, always available
 * - On first posedge sck when !wasdone, captures cyphertext into cyphertextcaptured
 * - First bit cyphertext[127] is output before first clock edge
 * - Subsequent bits come from cyphertextcaptured[126] after shifting
 * 
 * Protocol:
 * - MCU waits for DONE signal
 * - MCU reads 32 bytes via SPI (256 bits total)
 * - MCU pulls LOAD high to acknowledge
 * 
 * SPI Mode: CPOL=0, CPHA=0 (data sampled on rising edge, changed on falling edge)
 */

`timescale 1ns / 1ps

module sensor_data_spi_slave(
    input  logic        clk,           // FPGA system clock (for done/ack logic)
    input  logic        sck,           // SPI clock from MCU
    input  logic        sdi,           // SPI data in (MOSI from MCU, not used)
    output logic        sdo,           // SPI data out (MISO to MCU)
    input  logic        load,          // Load signal from MCU (acknowledge)
    output logic        done,          // Done signal to MCU (data ready)
    
    // Sensor data interface
    input  logic [7:0]  data_bytes [0:31],  // 32-byte data packet (like cyphertext input in Lab07)
    input  logic        data_ready,          // New data available
    output logic        data_ack             // Data has been sent
);

    // Following Lab07 aes_spi pattern EXACTLY
    // Lab07: logic [127:0] cyphertextcaptured;
    // We use: logic [255:0] datacaptured; (32 bytes = 256 bits)
    logic         sdodelayed, wasdone;
    logic [255:0] datacaptured;  // Captured data to shift out (like cyphertextcaptured in Lab07)
    
    // Initialize wasdone (critical for first transmission)
    initial begin
        wasdone = 1'b0;
        datacaptured = 256'd0;
        sdodelayed = 1'b0;
    end
    
    // Register data_to_send when data_ready goes high (like cyphertext is ready when done is high in Lab07)
    // Pack 32 bytes into 256-bit word (MSB first)
    // Byte 0 is MSB [255:248], Byte 31 is LSB [7:0]
    logic [255:0] data_to_send;
    logic packet_ready;
    logic packet_acknowledged;
    logic load_prev;
    
    // Initialize data_to_send to avoid 'x' values
    initial begin
        data_to_send = 256'd0;
        packet_ready = 1'b0;
        packet_acknowledged = 1'b0;
        load_prev = 1'b0;
    end
    
    always_ff @(posedge clk) begin
        load_prev <= load;
        
        if (data_ready && !packet_ready) begin
            // Pack and register data when ready (like cyphertext is ready in Lab07)
            for (int i = 0; i < 32; i = i + 1) begin
                data_to_send[255 - 8*i -: 8] <= data_bytes[i];
            end
            packet_ready <= 1'b1;
            packet_acknowledged <= 1'b0;
        end else if (packet_ready && load && !load_prev) begin
            // Reset when load goes high (MCU acknowledges)
            packet_ready <= 1'b0;
            packet_acknowledged <= 1'b1;
        end else if (packet_acknowledged) begin
            packet_acknowledged <= 1'b0;
        end
    end
    
    // Done signal: Assert when packet is ready and not yet acknowledged (clk domain)
    // Like Lab07: done comes from aes_core, we generate it from packet_ready
    always_ff @(posedge clk) begin
        done <= packet_ready && !packet_acknowledged;
    end
    
    // SPI transmission: EXACT Lab07 pattern
    // Lab07: always_ff @(posedge sck)
    //        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi};
    //        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[126:0], plaintext, key, sdi};
    // 
    // Lab07 captures cyphertext (full 128 bits) into cyphertextcaptured on first posedge
    // The first bit cyphertext[127] is already output, so cyphertextcaptured[126] is next
    // For us: Capture data_to_send[254:0] (255 bits) into datacaptured, shift in 0
    //        data_to_send[255] is already output, so datacaptured[254] is next
    always_ff @(posedge sck) begin
        if (!wasdone) begin
            // First posedge: capture data and shift in one operation (like Lab07)
            // Lab07: {cyphertextcaptured, ...} = {cyphertext, ...}
            // We capture data_to_send[254:0] and shift left, shifting in 0 (no input)
            datacaptured <= {data_to_send[254:0], 1'b0};
        end else begin
            // Subsequent posedges: shift left (like Lab07)
            datacaptured <= {datacaptured[254:0], 1'b0};
        end
    end
    
    // Track previous done state: EXACT Lab07 pattern
    // Lab07 uses blocking assignments for immediate update
    // For synthesis, we use non-blocking but the logic is the same
    always_ff @(negedge sck) begin
        wasdone <= done;  // Track done state
        sdodelayed <= datacaptured[254];  // Current MSB (will be output next, like Lab07's [126])
    end
    
    // SDO output: EXACT Lab07 pattern
    // Lab07: assign sdo = (done & !wasdone) ? cyphertext[127] : sdodelayed;
    // When done is first asserted (!wasdone), output MSB before first clock edge
    // After first negedge, wasdone becomes true, so use sdodelayed
    // Use data_to_send[255] directly (combinational, always available) like Lab07 uses cyphertext[127]
    assign sdo = (done & !wasdone) ? data_to_send[255] : sdodelayed;
    
    // Acknowledge when transmission complete
    assign data_ack = packet_acknowledged;
    
endmodule
