`timescale 1ns / 1ps

// Top-level module for Single BNO085 Sensor with MCU SPI Communication
// Integrates:
// - Single BNO085 sensor controller
// - SPI slave for MCU communication
// - Data formatter

module drum_trigger_top (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // Sensor SPI Interface
    output logic        sclk,
    output logic        mosi,
    input  logic        miso,
    output logic        cs_n,
    output logic        bno085_rst_n,
    input  logic        int_n,
    
    // MCU SPI Interface (FPGA as slave)
    input  logic        sclk_mcu,    // SPI clock from MCU
    input  logic        mosi_mcu,    // MCU -> FPGA (not used, but required)
    output logic        miso_mcu,    // FPGA -> MCU (data output)
    input  logic        cs_n_mcu,    // Chip select from MCU
    output logic        done,         // DONE signal to MCU (PA6)
    input  logic        load,         // LOAD signal from MCU (PA5)
    
    // Debug / Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);

    // Internal signals
    logic clk;
    logic rst_n;  // Internal reset signal
    
    // Sensor SPI master signals
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    
    // Sensor controller signals
    logic data_ready, sensor_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic initialized, error;
    
    // Data formatter signals
    logic tx_data_ready, tx_ack;
    logic [7:0] tx_data;
    logic formatter_busy;
    
    // BNO085 Reset Delay Counter
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
    
    // HARDWARE CLOCK - HSOSC
    HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(clk)
    );
    
    // Heartbeat LED
    logic [21:0] heartbeat_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) heartbeat_cnt <= 0;
        else heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led_heartbeat = heartbeat_cnt[21];
    
    // SPI Master for Sensor
    spi_master #(.CLK_DIV(2)) spi_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi_start),
        .tx_valid(spi_tx_valid),
        .tx_data(spi_tx_data),
        .tx_ready(spi_tx_ready),
        .rx_valid(spi_rx_valid),
        .rx_data(spi_rx_data),
        .busy(spi_busy),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso)
    );
    
    // Single BNO085 Controller
    bno085_controller bno085_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi_start),
        .spi_tx_valid(spi_tx_valid),
        .spi_tx_data(spi_tx_data),
        .spi_tx_ready(spi_tx_ready),
        .spi_rx_valid(spi_rx_valid),
        .spi_rx_data(spi_rx_data),
        .spi_busy(spi_busy),
        .cs_n(cs_n),
        .int_n(int_n),
        .quat_valid(sensor_valid),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_valid(),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .initialized(initialized),
        .error(error)
    );
    
    // Data ready when sensor has valid quaternion data
    assign data_ready = sensor_valid;
    
    // Sensor Data Formatter
    sensor_data_formatter data_formatter (
        .clk(clk),
        .rst_n(rst_n),
        .data_ready(data_ready),
        .sensor_valid(sensor_valid),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .tx_data_ready(tx_data_ready),
        .tx_data(tx_data),
        .tx_ack(tx_ack),
        .busy(formatter_busy)
    );
    
    // SPI Slave for MCU Communication
    spi_slave_mcu spi_slave (
        .clk(clk),
        .rst_n(rst_n),
        .sclk_mcu(sclk_mcu),
        .mosi_mcu(mosi_mcu),
        .miso_mcu(miso_mcu),
        .cs_n_mcu(cs_n_mcu),
        .done(done),
        .load(load),
        .data_ready(tx_data_ready),
        .tx_data(tx_data),
        .tx_ack(tx_ack)
    );
    
    // Status LEDs
    assign led_initialized = initialized;
    assign led_error = error;
    
endmodule

