`timescale 1ns / 1ps

/**
 * Drum Trigger Top-Level Module with MCU SPI Interface
 * 
 * Integrates:
 * - Dual BNO085 sensors (right and left hand)
 * - MCU SPI slave for sending raw sensor data to MCU
 * - BNO085 controllers and SPI masters
 * 
 * This module connects the FPGA drum detection system to the MCU via SPI
 * FPGA is SPI slave, MCU is SPI master
 * 
 * Pin Sharing:
 * - Shared: sclk, mosi, bno085_rst_n (both sensors)
 * - Separate: cs_n1, cs_n2, miso1, miso2, int1, int2
 * - PS0/WAKE pins removed (hardwired to 3.3V)
 */

module drum_trigger_top (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // MCU SPI Interface (FPGA is slave)
    input  logic        mcu_sck,     // SPI clock from MCU (PB3)
    input  logic        mcu_sdi,      // SPI data in from MCU (PB5 MOSI, not used for commands)
    output logic        mcu_sdo,     // SPI data out to MCU (PB4 MISO)
    input  logic        mcu_load,    // Load signal from MCU (PA5)
    output logic        mcu_done,    // Done signal to MCU (PA6)
    
    // BNO085 Sensor 1 SPI Interface (Right Hand)
    output logic        sclk,        // Shared SPI clock
    output logic        mosi,        // Shared SPI MOSI
    input  logic        miso1,       // Sensor 1 MISO (separate)
    output logic        cs_n1,       // Sensor 1 chip select (separate)
    input  logic        int1,        // Sensor 1 interrupt (separate)
    
    // BNO085 Sensor 2 SPI Interface (Left Hand)
    input  logic        miso2,       // Sensor 2 MISO (separate)
    output logic        cs_n2,       // Sensor 2 chip select (separate)
    input  logic        int2,        // Sensor 2 interrupt (separate)
    
    // Shared BNO085 Control
    output logic        bno085_rst_n, // Shared reset (both sensors)
    
    // Debug / Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // Internal signals
    logic clk;
    logic rst_n;
    
    // BNO085 Sensor 1 signals (Right Hand)
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    logic initialized1, error1;
    
    // BNO085 Sensor 2 signals (Left Hand) - UNUSED (single-sensor configuration)
    // Kept for interface compatibility with mcu_spi_slave, but tied off below
    logic quat2_valid, gyro2_valid;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    logic initialized2, error2;
    
    // SPI master signals for Sensor 1
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    
    // SPI master signals for Sensor 2 - UNUSED in single-sensor configuration
    logic spi2_start, spi2_tx_valid, spi2_tx_ready, spi2_rx_valid, spi2_busy;
    logic [7:0] spi2_tx_data, spi2_rx_data;
    
    // Reset delay counter
    localparam [22:0] DELAY_100MS = 23'd300_000;
    localparam [22:0] DELAY_2SEC = 23'd6_000_000;
    logic [22:0] rst_delay_counter;
    logic bno085_rst_n_delayed;
    logic controller_rst_n;
    
    // BNO085 Reset with delay (shared for both sensors)
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            rst_delay_counter <= 23'd0;
            bno085_rst_n_delayed <= 1'b0;
            controller_rst_n <= 1'b0;
        end else begin
            if (rst_delay_counter < DELAY_2SEC) begin
                rst_delay_counter <= rst_delay_counter + 1;
            end
            
            if (rst_delay_counter >= DELAY_100MS) begin
                bno085_rst_n_delayed <= 1'b1;
            end else begin
                bno085_rst_n_delayed <= 1'b0;
            end
            
            if (rst_delay_counter >= DELAY_2SEC) begin
                controller_rst_n <= 1'b1;
            end else begin
                controller_rst_n <= 1'b0;
            end
        end
    end
    
    assign bno085_rst_n = bno085_rst_n_delayed;
    assign rst_n = controller_rst_n;
    
    // Clock generation (3MHz from HSOSC)
    HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)
    );
    
    // Heartbeat LED (blinks every ~1 second at 3MHz)
    localparam [23:0] HEARTBEAT_DIV = 24'd3_000_000;
    logic [23:0] heartbeat_counter;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            heartbeat_counter <= 24'd0;
            led_heartbeat <= 1'b0;
        end else begin
            heartbeat_counter <= heartbeat_counter + 1;
            if (heartbeat_counter >= HEARTBEAT_DIV) begin
                heartbeat_counter <= 24'd0;
                led_heartbeat <= ~led_heartbeat;
            end
        end
    end
    
    // ============================================
    // BNO085 Sensor 1 (Right Hand)
    // ============================================
    
    // SPI Master for BNO085 Sensor 1
    spi_master spi_master_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi1_start),
        .tx_valid(spi1_tx_valid),
        .tx_data(spi1_tx_data),
        .tx_ready(spi1_tx_ready),
        .rx_valid(spi1_rx_valid),
        .rx_data(spi1_rx_data),
        .busy(spi1_busy),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1)
    );
    
    // BNO085 Controller for Sensor 1
    bno085_controller bno085_ctrl_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi1_start),
        .spi_tx_valid(spi1_tx_valid),
        .spi_tx_data(spi1_tx_data),
        .spi_tx_ready(spi1_tx_ready),
        .spi_rx_valid(spi1_rx_valid),
        .spi_rx_data(spi1_rx_data),
        .spi_busy(spi1_busy),
        .cs_n(cs_n1),
        .int_n(int1),
        .quat_valid(quat1_valid),
        .quat_w(quat1_w),
        .quat_x(quat1_x),
        .quat_y(quat1_y),
        .quat_z(quat1_z),
        .gyro_valid(gyro1_valid),
        .gyro_x(gyro1_x),
        .gyro_y(gyro1_y),
        .gyro_z(gyro1_z),
        .initialized(initialized1),
        .error(error1)
    );
    
    // ============================================
    // BNO085 Sensor 2 (Left Hand) - TIED OFF
    // ============================================
    //
    // NOTE: For this build we only use a single BNO085 sensor.
    // Sensor 2 logic is removed to avoid multiple drivers on `sclk`/`mosi`.
    // We keep the signals for compatibility, but tie them to safe values.

    // Keep second sensor chip select inactive and ignore its interrupt
    assign cs_n2 = 1'b1;  // Never select sensor 2

    // Tie off all Sensor 2 data/flags so MCU knows this sensor is inactive
    assign quat2_valid = 1'b0;
    assign gyro2_valid = 1'b0;
    assign quat2_w = '0;
    assign quat2_x = '0;
    assign quat2_y = '0;
    assign quat2_z = '0;
    assign gyro2_x = '0;
    assign gyro2_y = '0;
    assign gyro2_z = '0;
    assign initialized2 = 1'b0;
    assign error2 = 1'b0;
    
    // Status LEDs (only sensor 1 is used)
    assign led_initialized = initialized1;
    assign led_error = error1;
    
    // ============================================
    // MCU SPI Slave for sending raw sensor data
    // ============================================
    
    mcu_spi_slave mcu_spi_slave_inst (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done),
        // Sensor 1 (Right Hand)
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        // Sensor 2 (Left Hand)
        .quat2_valid(quat2_valid),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_valid(gyro2_valid),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z)
    );
    
endmodule

