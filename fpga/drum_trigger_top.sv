`timescale 1ns / 1ps

/**
 * Drum Trigger Top-Level Module with MCU SPI Interface
 * 
 * Integrates:
 * - Single BNO085 sensor (right hand)
 * - MCU SPI slave for sending raw sensor data to MCU
 * - BNO085 controller and SPI master
 * 
 * This module connects the FPGA drum sensor interface to the MCU via SPI.
 * FPGA is SPI slave, MCU is SPI master.
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
    output logic        ps0_wake1,   // Sensor 1 PS0/WAKE pin (active low)
    input  logic        int1,        // Sensor 1 interrupt (separate)
    
    // BNO085 Control
    output logic        bno085_rst_n, // Reset for the BNO085 sensor
    
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
    
    // SPI master signals for Sensor 1
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    
    // BNO085 Reset Delay Counter
    // Per datasheet 6.5.3: After NRST release, BNO085 needs:
    //   - t1 = 90ms for internal initialization
    //   - t2 = 4ms for internal configuration  
    //   - Total: ~94ms minimum, we use 100ms for safety
    // Clock is 3MHz, so 100ms = 300,000 cycles
    // We add 2 seconds total delay as requested
    localparam [22:0] DELAY_100MS = 23'd300_000;  // 100ms for BNO085 initialization
    localparam [22:0] DELAY_2SEC = 23'd6_000_000;  // 2 seconds total delay
    logic [22:0] rst_delay_counter;
    logic bno085_rst_n_delayed;
    logic controller_rst_n;  // Controller reset synchronized with BNO085 reset
    
    // BNO085 Reset with delay after FPGA reset release
    // Per datasheet 6.5.3: BNO085 needs ~94ms after NRST release before ready
    // Sequence: FPGA reset releases -> wait 100ms -> release BNO085 reset -> wait 1.9s -> release controller reset
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            // FPGA reset is active: keep BNO085 in reset and reset counter
            rst_delay_counter <= 23'd0;
            bno085_rst_n_delayed <= 1'b0;
            controller_rst_n <= 1'b0;  // Keep controller in reset too
        end else begin
            // FPGA reset released: count up to delay
            if (rst_delay_counter < DELAY_2SEC) begin
                rst_delay_counter <= rst_delay_counter + 1;
            end
            
            // Release BNO085 reset after 100ms (allows BNO085 to initialize per datasheet)
            if (rst_delay_counter >= DELAY_100MS) begin
                bno085_rst_n_delayed <= 1'b1;  // Release BNO085 reset
            end else begin
                bno085_rst_n_delayed <= 1'b0;  // Keep BNO085 in reset
            end
            
            // Release controller reset after full 2-second delay
            // This ensures BNO085 has completed initialization before controller starts
            if (rst_delay_counter >= DELAY_2SEC) begin
                controller_rst_n <= 1'b1;  // Release controller reset
            end else begin
                controller_rst_n <= 1'b0;  // Keep controller in reset
            end
        end
    end
    
    assign bno085_rst_n = bno085_rst_n_delayed;
    assign rst_n = controller_rst_n;  // Controller reset synchronized with BNO085 reset release
    
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
    // CLK_DIV=2 matches old working code (750 kHz SPI clock @ 3MHz system clock)
    spi_master #(.CLK_DIV(2)) spi_master_inst1 (
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
    
    // Status LEDs (sensor 1 only)
    assign led_initialized = initialized1;
    assign led_error = error1;
    
    // ============================================
    // MCU SPI Slave for sending raw sensor data
    // ============================================
    // Single sensor only - sends 16-byte packet with sensor 1 data
    
    mcu_spi_slave mcu_spi_slave_inst (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done),
        // Sensor 1 (Right Hand) - single sensor only
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

