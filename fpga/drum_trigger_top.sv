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
    input  logic        fpga_rst_n,
    
    // MCU SPI
    input  logic        mcu_sck,
    input  logic        mcu_sdi,
    output logic        mcu_sdo,
    input  logic        mcu_cs_n,
    
    // Arduino SPI
    input  logic        arduino_sck,
    input  logic        arduino_sdi,
    input  logic        arduino_cs_n,
    
    // status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // internal signals
    logic clk;
    logic rst_n;
    
    // Arduino SPI slave outputs
    logic [7:0] packet_buffer [0:15];
    logic initialized1, error1;
    
    // Simple reset: just use FPGA reset directly (no initialization delays needed)
    assign rst_n = fpga_rst_n;
    
    // system clock
    HSOSC #(.CLKHF_DIV(2'b00)) hf_osc (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(clk)
    );
    
    // heartbeat LED
    logic [21:0] heartbeat_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) heartbeat_cnt <= 0;
        else heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led_heartbeat = heartbeat_cnt[21];
    
    // Arduino SPI slave
    
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
    
    // MCU SPI slave
    
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
