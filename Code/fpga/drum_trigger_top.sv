`timescale 1ns / 1ps

/**
 * Drum Trigger Top-Level Module with MCU SPI Interface
 * 
 * Integrates:
 * - Arduino SPI slave for receiving sensor data from Arduino/ESP32
 * - MCU SPI slave for sending raw sensor data to MCU
 * 
 * This module connects Arduino sensor data to the MCU via SPI.
 * FPGA acts as bridge: Arduino (SPI master) â†’ FPGA (SPI slave) â†’ MCU (SPI master reads from FPGA slave).
 */

module drum_trigger_top (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // MCU SPI Interface (FPGA is slave)
    input  logic        mcu_sck,     // SPI clock from MCU (PB3)
    input  logic        mcu_sdi,      // SPI data in from MCU (PB5 MOSI, not used in read-only mode)
    output logic        mcu_sdo,     // SPI data out to MCU (PB4 MISO)
    input  logic        mcu_cs_n,    // Chip select from MCU (PA11, active low)
    
    // Arduino SPI Interface (FPGA is slave to Arduino)
    input  logic        arduino_sck,  // SPI clock from Arduino
    input  logic        arduino_sdi,  // SPI data in from Arduino (MOSI)
    input  logic        arduino_cs_n, // Chip select from Arduino (active low)
    
    // Debug / Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // Internal signals
    logic clk;
    logic rst_n;
    
    // Arduino SPI Slave outputs (connected to MCU SPI slave)
    logic [7:0] packet_buffer [0:15];  // Raw packet buffer from Arduino
    logic initialized1, error1;
    
    // Simple reset: just use FPGA reset directly (no initialization delays needed)
    assign rst_n = fpga_rst_n;
    
    // ========================================================================
    // System Clock Generation - HSOSC (Hardware Oscillator)
    // ========================================================================
    // HSOSC is a built-in primitive for iCE40UP5k
    // CLKHF_DIV settings (divide ratio):
    //   2'b00 = divide by 2  → 48MHz/2  = 24MHz
    //   2'b01 = divide by 4  → 48MHz/4  = 12MHz
    //   2'b10 = divide by 8  → 48MHz/8  =  6MHz
    //   2'b11 = divide by 16 → 48MHz/16 =  3MHz
    //
    // Current setting: 2'b11 = 3MHz
    // Rationale:
    //   - 3MHz provides sufficient speed for SPI operations (Arduino: 100kHz, MCU: variable)
    //   - Lower frequency reduces power consumption
    //   - Adequate for CDC timing margins (3 cycles = 1us, well above SPI timing)
    //   - Matches all timing calculations in code comments
    //
    // Timing Margins:
    //   - Arduino SPI: 100kHz (10us period) → 30 FPGA clocks per SPI bit
    //   - MCU SPI: Variable, but 3MHz provides sufficient margin
    //   - CDC: 3-cycle delay = 1us, provides 10x margin over worst-case SPI timing
    HSOSC #(.CLKHF_DIV(2'b00)) hf_osc (
        .CLKHFPU(1'b1),   // Power up (must be 1 for oscillator to run)
        .CLKHFEN(1'b1),   // Enable (must be 1 for clock output)
        .CLKHF(clk)       // Output: 3MHz system clock (48MHz / 16)
    );
    
    // Heartbeat LED (1Hz approx)
    // 3MHz = 3,000,000 cycles/sec. 2^22 = ~4M. Bit 21 toggles every ~0.7s
    logic [21:0] heartbeat_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) heartbeat_cnt <= 0;
        else heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led_heartbeat = heartbeat_cnt[21];
    
    // ============================================
    // Arduino SPI Slave - Receive sensor data from Arduino
    // ============================================
    // Receives 16-byte packets from Arduino/ESP32 over SPI
    // Outputs raw packet buffer for MCU to parse
    
    arduino_spi_slave arduino_spi_slave_inst (
        .clk(clk),
        .cs_n(arduino_cs_n),
        .sck(arduino_sck),
        .sdi(arduino_sdi),
        .packet_buffer(packet_buffer),
        .initialized(initialized1),
        .error(error1)
    );
    
    // Status LEDs
    assign led_initialized = initialized1;
    assign led_error = error1;
    
    // ============================================
    // MCU SPI Slave for sending raw sensor data
    // ============================================
    // Sends raw 16-byte packet buffer directly to MCU
    // Uses CS-based protocol (chip select, active low)
    // MCU will parse the Arduino packet format (Roll/Pitch/Yaw)
    
    spi_slave_mcu spi_slave_mcu_inst (
        .clk(clk),
        .cs_n(mcu_cs_n),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .packet_buffer(packet_buffer),
        .initialized(initialized1),
        .error(error1)
    );
    
endmodule
