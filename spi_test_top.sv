`timescale 1ns / 1ps

module spi_test_top (
    input  logic        rst_n,
    
    // SPI Interface (matches Sensor 1 in drum_set_top)
    output logic        sclk1,
    output logic        mosi1,
    input  logic        miso1,
    output logic        cs_n1,
    output logic        ps0_1,  // Added PS0/WAKE pin
    input  logic        int1,
    
    // Debug / Status LEDs
    output logic        led_initialized, // ON when BNO085 init complete
    output logic        led_error,       // ON if error state
    output logic        led_heartbeat    // Blinks to show clock is running
);

    // Internal signals
    logic clk;
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic initialized, error;

    // HARDWARE CLOCK - HSOSC (ACTIVE FOR HARDWARE)
    // CLKHF_DIV(2'b11) = divide by 16 to get 3MHz from 48MHz
    // For 48MHz HSOSC: divide by 16 = 3MHz (suitable for SPI)
    // For different frequencies, adjust CLKHF_DIV:
    //   2'b00 = divide by 2
    //   2'b01 = divide by 4  
    //   2'b10 = divide by 8
    //   2'b11 = divide by 16
    
    // HARDWARE CLOCK - HSOSC (ACTIVE FOR HARDWARE)
    // Note: HSOSC is a built-in primitive for iCE40UP5k
    // Make sure your synthesis tool recognizes this primitive
    HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
        .CLKHFPU(1'b1),   // Power up (must be 1)
        .CLKHFEN(1'b1),   // Enable (must be 1)
        .CLKHF(clk)       // Output clock (3MHz from 48MHz / 16)
    );

    // Heartbeat LED (1Hz approx)
    // 3MHz = 3,000,000 cycles/sec. 2^22 = ~4M. Bit 21 toggles every ~0.7s
    logic [21:0] heartbeat_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) heartbeat_cnt <= 0;
        else heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led_heartbeat = heartbeat_cnt[21];

    // SPI Master
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
        .sclk(sclk1),
        .mosi(mosi1),
        .miso(miso1)
    );

    // BNO085 Controller
    bno085_controller bno085_ctrl_inst (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi_start),
        .spi_tx_valid(spi_tx_valid),
        .spi_tx_data(spi_tx_data),
        .spi_tx_ready(spi_tx_ready),
        .spi_rx_valid(spi_rx_valid),
        .spi_rx_data(spi_rx_data),
        .spi_busy(spi_busy),
        .cs_n(cs_n1),
        .ps0_wake(ps0_1),
        .int_n(int1),
        .quat_valid(quat_valid),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_valid(gyro_valid),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .initialized(initialized),
        .error(error)
    );

    // Status LEDs
    assign led_initialized = initialized;
    assign led_error = error;

endmodule