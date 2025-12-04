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
    output logic        led_heartbeat,
    
    // Diagnostic outputs (for debugging SPI reception)
    output logic        led_cs_detected,      // High when CS is low (Arduino is sending)
    output logic        led_packet_received   // High when packet_valid is true
);

    // Internal signals
    logic clk;
    logic rst_n;
    
    // Arduino SPI Slave outputs (connected to MCU SPI slave)
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
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
    // Current setting: 2'b00 = 24MHz
    // Rationale:
    //   - 24MHz provides sufficient speed for SPI operations (Arduino: 100kHz, MCU: variable)
    //   - Adequate for CDC timing margins (3 cycles = 125ns, well above SPI timing)
    //
    // Timing Margins:
    //   - Arduino SPI: 100kHz (10us period) → 240 FPGA clocks per SPI bit
    //   - MCU SPI: Variable, but 24MHz provides sufficient margin
    //   - CDC: 3-cycle delay = 125ns, provides 80x margin over worst-case SPI timing
    // HSOSC power and enable pins - use wire connections instead of constant drivers
    wire clkhfpu_wire = 1'b1;  // Power up (must be 1 for oscillator to run)
    wire clkhfen_wire = 1'b1;  // Enable (must be 1 for clock output)
    
    HSOSC #(.CLKHF_DIV(2'b00)) hf_osc (
        .CLKHFPU(clkhfpu_wire),   // Power up (must be 1 for oscillator to run)
        .CLKHFEN(clkhfen_wire),   // Enable (must be 1 for clock output)
        .CLKHF(clk)               // Output: 24MHz system clock (48MHz / 2)
    );
    
    // Heartbeat LED (1Hz approx)
    // 24MHz = 24,000,000 cycles/sec. 2^24 = ~16M. Bit 23 toggles every ~0.7s
    logic [23:0] heartbeat_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) heartbeat_cnt <= 0;
        else heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led_heartbeat = heartbeat_cnt[23];
    
    // ============================================
    // Arduino SPI Slave - Receive sensor data from Arduino
    // ============================================
    // Receives 16-byte packets from Arduino/ESP32 over SPI
    // Maps Euler angles (Roll, Pitch, Yaw) to quaternion fields for MCU interface
    
    arduino_spi_slave arduino_spi_slave_inst (
        .clk(clk),
        .cs_n(arduino_cs_n),
        .sck(arduino_sck),
        .sdi(arduino_sdi),
        .initialized(initialized1),
        .error(error1),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .cs_detected(cs_detected_internal),
        .packet_received(packet_received_internal)
    );
    
    // Internal diagnostic signals
    logic cs_detected_internal, packet_received_internal;
    
    // Status LEDs
    assign led_initialized = initialized1;
    assign led_error = error1;
    assign led_cs_detected = cs_detected_internal;
    assign led_packet_received = packet_received_internal;
    
    // ============================================
    // MCU SPI Slave for sending raw sensor data
    // ============================================
    // Single sensor only - sends 16-byte packet with sensor 1 data
    // Uses CS-based protocol (chip select, active low)
    // Receives sensor data directly from BNO085 controller (spi_slave_mcu handles snapshot internally)
    
    spi_slave_mcu spi_slave_mcu_inst (
        .clk(clk),
        .cs_n(mcu_cs_n),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        // Sensor 1 (Right Hand) - single sensor only
        .initialized(initialized1),
        .error(error1),
        // Sensor data directly from BNO085 controller (spi_slave_mcu handles snapshot internally)
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z)
    );
    
endmodule
