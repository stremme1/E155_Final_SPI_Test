/**
 * Simplified Drum Trigger Top-Level Module
 * 
 * NEW ARCHITECTURE: Send raw sensor data to MCU for processing
 * 
 * Removed (to save DSP blocks):
 * - quaternion_to_euler_dsp (18 DSP blocks)
 * - yaw_normalizer
 * - drum_zone_detector
 * - strike_detector
 * - drum_selector
 * - drum_trigger_processor
 * 
 * Kept:
 * - bno085_controller (2x) - Read sensors
 * - spi_master (2x) - BNO085 communication
 * - sensor_data_packer - Pack raw data
 * - sensor_data_spi_slave - Send to MCU
 * 
 * This reduces resource usage from 18 DSP blocks to 0 DSP blocks
 */

`timescale 1ns / 1ps

module drum_trigger_top_simplified (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // MCU SPI Interface
    input  logic        mcu_sck,     // SPI clock from MCU (PB3)
    input  logic        mcu_sdi,     // SPI data in from MCU (PB5 MOSI, not used)
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
    
    // Button Inputs
    input  logic        calibrate_btn_n,
    input  logic        kick_btn_n,
    
    // Debug / Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // Internal signals
    logic clk;
    logic rst_n; // Internal reset for controllers
    
    // BNO085 signals for Sensor 1
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    logic initialized1, error1;
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    logic ps0_wake1;
    logic spi1_tx_ready_internal;
    
    // BNO085 signals for Sensor 2
    logic quat2_valid, gyro2_valid;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    logic initialized2, error2;
    logic spi2_start, spi2_tx_valid, spi2_tx_ready, spi2_rx_valid, spi2_busy;
    logic [7:0] spi2_tx_data, spi2_rx_data;
    logic ps0_wake2;
    logic spi2_tx_ready_internal;
    
    // Button debounced signals
    logic calibrate_btn_pulse, kick_btn_pulse;
    
    // Sensor data packing
    logic [7:0] sensor_data_bytes [0:31];
    logic sensor_data_ready, sensor_data_ack;
    
    // Reset delay counter
    localparam [22:0] DELAY_100MS = 23'd300_000;  // 100ms for BNO085 initialization
    localparam [22:0] DELAY_2SEC = 23'd6_000_000;  // 2 seconds total delay
    logic [22:0] rst_delay_counter;
    logic bno085_rst_n_delayed;
    logic controller_rst_n;
    
    // BNO085 Reset with delay after FPGA reset release
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
    
    // HSOSC for 3MHz clock
    HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(clk)
    );
    
    // Heartbeat LED
    localparam [23:0] HEARTBEAT_PERIOD = 24'd1_500_000; // 0.5s blink at 3MHz
    logic [23:0] heartbeat_counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_counter <= 24'd0;
            led_heartbeat <= 1'b0;
        end else begin
            if (heartbeat_counter == HEARTBEAT_PERIOD) begin
                heartbeat_counter <= 24'd0;
                led_heartbeat <= ~led_heartbeat;
            end else begin
                heartbeat_counter <= heartbeat_counter + 1;
            end
        end
    end
    
    // Button Debouncers
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
    logic sclk1, mosi1;
    spi_master spi_master_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk1),
        .mosi(mosi1),
        .miso(miso1),
        .start(spi1_start),
        .tx_valid(spi1_tx_valid),
        .tx_data(spi1_tx_data),
        .tx_ready(spi1_tx_ready_internal),
        .rx_data(spi1_rx_data),
        .rx_valid(spi1_rx_valid),
        .busy(spi1_busy)
    );
    
    assign spi1_tx_ready = spi1_tx_ready_internal;
    
    // BNO085 Controller for Sensor 1
    bno085_controller bno085_ctrl_inst1 (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi1_start),
        .spi_tx_valid(spi1_tx_valid),
        .spi_tx_ready(spi1_tx_ready),
        .spi_rx_valid(spi1_rx_valid),
        .spi_rx_data(spi1_rx_data),
        .spi_busy(spi1_busy),
        .spi_tx_data(spi1_tx_data),
        .cs_n(cs_n1),
        .ps0_wake(ps0_wake1),
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
    // BNO085 Sensor 2 (Left Hand)
    // ============================================
    
    // SPI Master for BNO085 Sensor 2
    logic sclk2, mosi2;
    spi_master spi_master_inst2 (
        .clk(clk),
        .rst_n(rst_n),
        .sclk(sclk2),
        .mosi(mosi2),
        .miso(miso2),
        .start(spi2_start),
        .tx_valid(spi2_tx_valid),
        .tx_data(spi2_tx_data),
        .tx_ready(spi2_tx_ready_internal),
        .rx_data(spi2_rx_data),
        .rx_valid(spi2_rx_valid),
        .busy(spi2_busy)
    );
    
    assign spi2_tx_ready = spi2_tx_ready_internal;
    
    // Mux shared SPI signals (only one sensor active at a time via cs_n)
    assign sclk = cs_n1 ? sclk2 : sclk1;  // Sensor 1 active when cs_n1 is low
    assign mosi = cs_n1 ? mosi2 : mosi1;  // Sensor 1 active when cs_n1 is low
    
    // BNO085 Controller for Sensor 2
    bno085_controller bno085_ctrl_inst2 (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi2_start),
        .spi_tx_valid(spi2_tx_valid),
        .spi_tx_ready(spi2_tx_ready),
        .spi_rx_valid(spi2_rx_valid),
        .spi_rx_data(spi2_rx_data),
        .spi_busy(spi2_busy),
        .spi_tx_data(spi2_tx_data),
        .cs_n(cs_n2),
        .ps0_wake(ps0_wake2),
        .int_n(int2),
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
    
    // Status LEDs (both sensors must be initialized)
    assign led_initialized = initialized1 && initialized2;
    assign led_error = error1 || error2;
    
    // ============================================
    // Sensor Data Packer
    // ============================================
    sensor_data_packer data_packer (
        .clk(clk),
        .rst_n(rst_n),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .quat2_valid(quat2_valid),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_valid(gyro2_valid),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z),
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        .data_bytes(sensor_data_bytes),
        .data_ready(sensor_data_ready),
        .data_ack(sensor_data_ack)
    );
    
    // ============================================
    // Sensor Data SPI Slave for MCU Communication
    // ============================================
    sensor_data_spi_slave mcu_spi_slave (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done),
        .data_bytes(sensor_data_bytes),
        .data_ready(sensor_data_ready),
        .data_ack(sensor_data_ack)
    );
    
endmodule

