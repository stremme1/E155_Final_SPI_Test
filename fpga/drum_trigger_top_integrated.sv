/**
 * Drum Trigger Top-Level Module with MCU SPI Interface
 * 
 * Integrates:
 * - Dual BNO085 sensors (right and left hand)
 * - Existing drum_trigger_processor (BNO085 sensor processing)
 * - New drum_spi_slave (MCU communication)
 * - BNO085 controllers and SPI masters
 * 
 * This module connects the FPGA drum detection system to the MCU via SPI
 * 
 * Pin Sharing:
 * - Shared: sclk, mosi, bno085_rst_n (both sensors)
 * - Separate: cs_n1, cs_n2, miso1, miso2, int1, int2
 */

`timescale 1ns / 1ps

module drum_trigger_top_integrated (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // MCU SPI Interface (NEW)
    input  logic        mcu_sck,     // SPI clock from MCU (PB3)
    input  logic        mcu_sdi,     // SPI data in from MCU (PB5 MOSI, not used for commands)
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
    
    // Button Inputs (EXISTING)
    input  logic        calibrate_btn_n,
    input  logic        kick_btn_n,
    
    // Debug / Status LEDs (EXISTING)
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
    
    // BNO085 Sensor 2 signals (Left Hand)
    logic quat2_valid, gyro2_valid;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    logic initialized2, error2;
    
    // Drum trigger signals
    logic drum_trigger_valid;
    logic [3:0] drum_code;
    logic drum_hand;
    
    // Button debounced signals
    logic calibrate_btn_pulse, kick_btn_pulse;
    
    // SPI master signals for Sensor 1
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    
    // SPI master signals for Sensor 2
    logic spi2_start, spi2_tx_valid, spi2_tx_ready, spi2_rx_valid, spi2_busy;
    logic [7:0] spi2_tx_data, spi2_rx_data;
    
    // Internal ps0_wake signals (not exposed as pins)
    logic ps0_wake1, ps0_wake2;
    
    // Reset delay counter (same as spi_test_top)
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
    
    // Button debouncers
    button_debouncer #(.DEBOUNCE_CYCLES(150000)) calibrate_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(calibrate_btn_n),
        .btn_pressed(calibrate_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    button_debouncer #(.DEBOUNCE_CYCLES(150000)) kick_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(kick_btn_n),
        .btn_pressed(kick_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    // ============================================
    // BNO085 Sensor 1 (Right Hand)
    // ============================================
    
    // SPI Master for BNO085 Sensor 1
    spi_master spi_master_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1),
        .cs_n(cs_n1),
        .start_transfer(spi1_start),
        .write_en(spi1_tx_valid),
        .data_length(16'd1),
        .tx_data(spi1_tx_data),
        .rx_data(spi1_rx_data),
        .data_valid(spi1_rx_valid),
        .transfer_done(!spi1_busy),
        .transfer_busy(spi1_busy)
    );
    
    // BNO085 Controller for Sensor 1
    bno085_controller bno085_ctrl_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1),
        .cs_n(cs_n1),
        .ps0_wake(ps0_wake1),  // Internal signal, not exposed as pin
        .int_n(int1),
        .spi_start(spi1_start),
        .spi_tx_valid(spi1_tx_valid),
        .spi_tx_ready(spi1_tx_ready),
        .spi_rx_valid(spi1_rx_valid),
        .spi_rx_data(spi1_rx_data),
        .spi_busy(spi1_busy),
        .spi_tx_data(spi1_tx_data),
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
    
    assign spi1_tx_ready = !spi1_busy;
    
    // ============================================
    // BNO085 Sensor 2 (Left Hand)
    // ============================================
    
    // SPI Master for BNO085 Sensor 2
    spi_master spi_master_inst2 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso2),
        .cs_n(cs_n2),
        .start_transfer(spi2_start),
        .write_en(spi2_tx_valid),
        .data_length(16'd1),
        .tx_data(spi2_tx_data),
        .rx_data(spi2_rx_data),
        .data_valid(spi2_rx_valid),
        .transfer_done(!spi2_busy),
        .transfer_busy(spi2_busy)
    );
    
    // BNO085 Controller for Sensor 2
    bno085_controller bno085_ctrl_inst2 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso2),
        .cs_n(cs_n2),
        .ps0_wake(ps0_wake2),  // Internal signal, not exposed as pin
        .int_n(int2),
        .spi_start(spi2_start),
        .spi_tx_valid(spi2_tx_valid),
        .spi_tx_ready(spi2_tx_ready),
        .spi_rx_valid(spi2_rx_valid),
        .spi_rx_data(spi2_rx_data),
        .spi_busy(spi2_busy),
        .spi_tx_data(spi2_tx_data),
        .quat_valid(quat2_valid),
        .quat_w(quat2_w),
        .quat_x(quat2_x),
        .quat_y(quat2_y),
        .quat_z(quat2_z),
        .gyro_valid(gyro2_valid),
        .gyro_x(gyro2_x),
        .gyro_y(gyro2_y),
        .gyro_z(gyro2_z),
        .initialized(initialized2),
        .error(error2)
    );
    
    assign spi2_tx_ready = !spi2_busy;
    
    // Status LEDs (both sensors must be initialized)
    assign led_initialized = initialized1 && initialized2;
    assign led_error = error1 || error2;
    
    // ============================================
    // Drum Trigger Processor (Dual Sensor)
    // ============================================
    
    drum_trigger_processor drum_processor (
        .clk(clk),
        .rst_n(rst_n),
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
        .gyro2_z(gyro2_z),
        // Buttons
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        // Outputs
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .drum_hand(drum_hand)
    );
    
    // ============================================
    // Drum SPI Slave for MCU Communication
    // ============================================
    
    drum_spi_slave mcu_spi_slave (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done),
        .drum_trigger_valid(drum_trigger_valid),
        .drum_code(drum_code),
        .command_sent()  // Not used in top-level
    );
    
endmodule
