/**
 * Drum SPI Slave Module
 * 
 * SPI slave interface for sending drum trigger commands to MCU
 * Based on Lab07 aes_spi module (proven working code), simplified for 1-byte drum commands
 * 
 * Protocol:
 * - FPGA sends 1 byte drum command (0-7) when drum is detected
 * - MCU reads command via SPI
 * - Uses LOAD/DONE handshaking for synchronization
 * 
 * SPI Mode: CPOL=0, CPHA=0 (data sampled on rising edge, changed on falling edge)
 */

module drum_spi_slave(
    input  logic        clk,           // FPGA system clock
    input  logic        sck,           // SPI clock from MCU
    input  logic        sdi,           // SPI data in (MOSI from MCU, not used for commands)
    output logic        sdo,           // SPI data out (MISO to MCU)
    input  logic        load,          // Load signal from MCU (acknowledge)
    output logic        done,          // Done signal to MCU (data ready)
    
    // Drum trigger interface (from drum_trigger_processor)
    input  logic        drum_trigger_valid,  // Pulse when drum detected
    input  logic [3:0]  drum_code,           // Drum code (0-7)
    output logic        command_sent          // Command has been sent (acknowledgment)
);

    // Internal signals - following Lab07 pattern exactly
    logic [7:0] tx_buffer;           // Transmit buffer (drum command)
    logic [7:0] tx_shift_reg;         // Shift register for transmission
    logic       sdodelayed;           // Delayed SDO (like Lab07)
    logic       wasdone;              // Previous done state (like Lab07)
    logic       command_ready;        // Command ready to send
    logic       command_acknowledged;  // Command has been acknowledged
    logic       load_prev;            // Previous load state
    
    // Command handling: Latch command when trigger detected
    // Handle rapid triggers by updating command even if previous one is still ready
    always_ff @(posedge clk) begin
        load_prev <= load;
        
        if (drum_trigger_valid) begin
            // Latch new command when trigger detected (even if previous command is still ready)
            // This allows rapid triggers to update the command
            tx_buffer <= {4'h0, drum_code};
            tx_shift_reg <= {4'h0, drum_code};  // Initialize shift register
            command_ready <= 1'b1;
            command_acknowledged <= 1'b0;
        end else if (command_ready && load && !load_prev) begin
            // Reset when load goes high (MCU acknowledges by pulling load high)
            command_ready <= 1'b0;
            command_acknowledged <= 1'b1;
        end else if (command_acknowledged) begin
            command_acknowledged <= 1'b0;
        end
    end
    
    // Done signal: Assert when command is ready and not yet acknowledged
    always_ff @(posedge clk) begin
        done <= command_ready && !command_acknowledged;
    end
    
    // SPI transmission: Shift on posedge sck (like Lab07)
    // Following Lab07 pattern exactly: on first posedge (!wasdone), load AND shift
    // On subsequent posedges (wasdone), shift left
    always_ff @(posedge sck) begin
        if (!wasdone) begin
            // First posedge: load buffer and shift in one operation (like Lab07)
            // Lab07: {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi}
            // For us: load tx_buffer and shift it
            tx_shift_reg <= {tx_buffer[6:0], 1'b0};  // Load and shift in one step
        end else begin
            // Subsequent posedges: shift left (MSB first)
            tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
        end
    end
    
    // Track previous done state (for edge detection - like Lab07)
    // Lab07 uses always_ff @(negedge sck) with blocking assignments for immediate update
    // This is synthesizable - blocking assignments in always_ff are allowed for immediate combinational update
    always_ff @(negedge sck) begin
        wasdone = done;  // Track done state (like Lab07)
        sdodelayed = tx_shift_reg[7];  // Current MSB (will be output next)
    end
    
    // SDO output: Following Lab07 pattern exactly
    // When done is first asserted (!wasdone), output MSB before first clock edge
    // After first negedge, wasdone becomes true, so use sdodelayed
    assign sdo = (done & !wasdone) ? tx_buffer[7] : sdodelayed;
    
    // Output assignment
    assign command_sent = command_acknowledged;
    
endmodule
