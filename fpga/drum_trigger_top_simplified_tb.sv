/**
 * Simplified Drum Trigger Top Module (Testbench Version)
 * 
 * Same as drum_trigger_top_simplified.sv but with HSOSC replaced by direct clock
 * for simulation compatibility
 */

`timescale 1ns / 1ps

module drum_trigger_top_simplified_tb (
    input  logic        clk,           // Direct clock input (for simulation)
    input  logic        fpga_rst_n,
    
    // MCU SPI Interface
    input  logic        mcu_sck,
    input  logic        mcu_sdi,
    output logic        mcu_sdo,
    input  logic        mcu_load,
    output logic        mcu_done,
    
    // BNO085 Sensor 1 SPI Interface
    output logic        sclk,
    output logic        mosi,
    input  logic        miso1,
    output logic        cs_n1,
    input  logic        int1,
    
    // BNO085 Sensor 2 SPI Interface
    input  logic        miso2,
    output logic        cs_n2,
    input  logic        int2,
    
    // Shared BNO085 Control
    output logic        bno085_rst_n,
    
    // Button Inputs
    input  logic        calibrate_btn_n,
    input  logic        kick_btn_n,
    
    // Debug / Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // Use direct clock (no HSOSC for simulation)
    // Rest of module is identical to drum_trigger_top_simplified.sv
    // ... (copy rest of the module from drum_trigger_top_simplified.sv)
    
    // For now, just include the simplified version and comment out HSOSC
    // In actual use, create a version without HSOSC instantiation
    
endmodule

