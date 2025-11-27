/**
 * Sensor Data SPI Slave Module
 * 
 * Sends multi-byte sensor data packets to MCU via SPI
 * EXACT COPY of Lab07 aes_spi pattern, adapted for 32-byte (256-bit) data packets
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
    input  logic        clk,           // FPGA system clock (for data latching)
    input  logic        sck,           // SPI clock from MCU
    input  logic        sdi,           // SPI data in (MOSI from MCU, not used)
    output logic        sdo,           // SPI data out (MISO to MCU)
    input  logic        load,          // Load signal from MCU (acknowledge)
    output logic        done,          // Done signal to MCU (data ready)
    
    // Sensor data interface
    input  logic [7:0]  data_bytes [0:31],  // 32-byte data packet
    input  logic        data_ready,          // New data available
    output logic        data_ack             // Data has been sent
);

    // Following Lab07 aes_spi pattern EXACTLY
    // Lab07 uses: logic [127:0] cyphertextcaptured;
    // We use: logic [255:0] datacaptured; (32 bytes = 256 bits)
    logic         sdodelayed, wasdone;
    logic [255:0] datacaptured;  // Captured data to shift out (like cyphertextcaptured in Lab07)
    logic [255:0] data_to_send;   // Current data packet to send (like cyphertext in Lab07)
    
    // Initialize signals
    initial begin
        wasdone = 1'b0;
        datacaptured = 256'd0;
        sdodelayed = 1'b0;
        data_to_send = 256'd0;
    end
    
    // Packet ready/acknowledged logic (clk domain)
    logic packet_ready;
    logic packet_acknowledged;
    logic load_prev;
    
    // Latch data packet when ready (clk domain)
    always_ff @(posedge clk) begin
        load_prev <= load;
        
        if (data_ready && !packet_ready) begin
            // Pack 32 bytes into 256-bit word (MSB first, like Lab07)
            // Byte 0 is MSB [255:248], Byte 31 is LSB [7:0]
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
    logic done_sync;  // Synchronized done signal
    always_ff @(posedge clk) begin
        done_sync <= packet_ready && !packet_acknowledged;
    end
    assign done = done_sync;
    
    // Capture data_to_send into datacaptured when done goes high (clk domain)
    // This ensures data is stable before first sck edge
    logic done_prev;
    always_ff @(posedge clk) begin
        done_prev <= done_sync;
        if (done_sync && !done_prev) begin
            // done just went high - capture data (like Lab07 captures cyphertext)
            datacaptured <= data_to_send;
        end
    end
    
    // SPI transmission: EXACT Lab07 pattern
    // Lab07: always_ff @(posedge sck)
    //        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi};
    //        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[126:0], plaintext, key, sdi};
    // 
    // For us: datacaptured is already captured when done goes high (in clk domain)
    //        if (!wasdone)  datacaptured = {datacaptured[254:0], 1'b0};  // Shift (already captured)
    //        else           datacaptured = {datacaptured[254:0], 1'b0};  // Continue shifting
    always_ff @(posedge sck) begin
        if (!wasdone) begin
            // First posedge: shift (datacaptured already contains data from clk domain)
            datacaptured <= {datacaptured[254:0], 1'b0};
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
    // Use datacaptured[255] since it's already captured when done goes high
    assign sdo = (done & !wasdone) ? datacaptured[255] : sdodelayed;
    
    // Acknowledge when transmission complete
    assign data_ack = packet_acknowledged;
    
endmodule
