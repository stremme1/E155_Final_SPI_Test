`timescale 1ns / 1ps

// Top-level module for Dual BNO085 Sensors with MCU SPI Communication
// Integrates:
// - Dual BNO085 sensor controllers
// - SPI slave for MCU communication
// - Data formatter

module drum_trigger_top (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // Sensor 1 SPI Interface
    output logic        sclk1,
    output logic        mosi1,
    input  logic        miso1,
    output logic        cs_n1,
    output logic        ps0_wake1,
    output logic        bno085_rst_n1,
    input  logic        int_n1,
    
    // Sensor 2 SPI Interface
    output logic        sclk2,
    output logic        mosi2,
    input  logic        miso2,
    output logic        cs_n2,
    output logic        ps0_wake2,
    output logic        bno085_rst_n2,
    input  logic        int_n2,
    
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
    
    // Sensor 1 SPI master signals
    logic spi1_start, spi1_tx_valid, spi1_tx_ready, spi1_rx_valid, spi1_busy;
    logic [7:0] spi1_tx_data, spi1_rx_data;
    
    // Sensor 2 SPI master signals
    logic spi2_start, spi2_tx_valid, spi2_tx_ready, spi2_rx_valid, spi2_busy;
    logic [7:0] spi2_tx_data, spi2_rx_data;
    
    // Dual sensor controller signals
    logic data_ready, sensor1_valid, sensor2_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    logic initialized, error;
    
    // Data formatter signals
    logic tx_data_ready, tx_ack;
    logic [7:0] tx_data;
    logic formatter_busy;
    
    // BNO085 Reset Delay Counter
    localparam [22:0] DELAY_100MS = 23'd300_000;  // 100ms for BNO085 initialization
    localparam [22:0] DELAY_2SEC = 23'd6_000_000;  // 2 seconds total delay
    logic [22:0] rst_delay_counter;
    logic bno085_rst_n1_delayed, bno085_rst_n2_delayed;
    logic controller_rst_n;
    
    // BNO085 Reset with delay after FPGA reset release
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            rst_delay_counter <= 23'd0;
            bno085_rst_n1_delayed <= 1'b0;
            bno085_rst_n2_delayed <= 1'b0;
            controller_rst_n <= 1'b0;
        end else begin
            if (rst_delay_counter < DELAY_2SEC) begin
                rst_delay_counter <= rst_delay_counter + 1;
            end
            
            if (rst_delay_counter >= DELAY_100MS) begin
                bno085_rst_n1_delayed <= 1'b1;
                bno085_rst_n2_delayed <= 1'b1;
            end else begin
                bno085_rst_n1_delayed <= 1'b0;
                bno085_rst_n2_delayed <= 1'b0;
            end
            
            if (rst_delay_counter >= DELAY_2SEC) begin
                controller_rst_n <= 1'b1;
            end else begin
                controller_rst_n <= 1'b0;
            end
        end
    end
    
    assign bno085_rst_n1 = bno085_rst_n1_delayed;
    assign bno085_rst_n2 = bno085_rst_n2_delayed;
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
    
    // SPI Master for Sensor 1
    spi_master #(.CLK_DIV(2)) spi_master1 (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi1_start),
        .tx_valid(spi1_tx_valid),
        .tx_data(spi1_tx_data),
        .tx_ready(spi1_tx_ready),
        .rx_valid(spi1_rx_valid),
        .rx_data(spi1_rx_data),
        .busy(spi1_busy),
        .sclk(sclk1),
        .mosi(mosi1),
        .miso(miso1)
    );
    
    // SPI Master for Sensor 2
    spi_master #(.CLK_DIV(2)) spi_master2 (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi2_start),
        .tx_valid(spi2_tx_valid),
        .tx_data(spi2_tx_data),
        .tx_ready(spi2_tx_ready),
        .rx_valid(spi2_rx_valid),
        .rx_data(spi2_rx_data),
        .busy(spi2_busy),
        .sclk(sclk2),
        .mosi(mosi2),
        .miso(miso2)
    );
    
    // Dual BNO085 Controller
    dual_bno085_controller dual_sensor_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .spi1_start(spi1_start),
        .spi1_tx_valid(spi1_tx_valid),
        .spi1_tx_data(spi1_tx_data),
        .spi1_tx_ready(spi1_tx_ready),
        .spi1_rx_valid(spi1_rx_valid),
        .spi1_rx_data(spi1_rx_data),
        .spi1_busy(spi1_busy),
        .cs_n1(cs_n1),
        .ps0_wake1(ps0_wake1),
        .int_n1(int_n1),
        .spi2_start(spi2_start),
        .spi2_tx_valid(spi2_tx_valid),
        .spi2_tx_data(spi2_tx_data),
        .spi2_tx_ready(spi2_tx_ready),
        .spi2_rx_valid(spi2_rx_valid),
        .spi2_rx_data(spi2_rx_data),
        .spi2_busy(spi2_busy),
        .cs_n2(cs_n2),
        .ps0_wake2(ps0_wake2),
        .int_n2(int_n2),
        .data_ready(data_ready),
        .sensor1_valid(sensor1_valid),
        .sensor2_valid(sensor2_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z),
        .initialized(initialized),
        .error(error)
    );
    
    // Sensor Data Formatter
    sensor_data_formatter data_formatter (
        .clk(clk),
        .rst_n(rst_n),
        .data_ready(data_ready),
        .sensor1_valid(sensor1_valid),
        .sensor2_valid(sensor2_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z),
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

