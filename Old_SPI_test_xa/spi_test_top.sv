`timescale 1ns / 1ps

module spi_test_top (
    input  logic        fpga_rst_n,  // Global FPGA reset pin (active low)
    
    // SPI Interface (matches Sensor 1 in drum_set_top)
    output logic        sclk1,
    output logic        mosi1,
    input  logic        miso1,
    output logic        cs_n1,
    output logic        ps0_1,  // Added PS0/WAKE pin
    output logic        bno085_rst_n1, // BNO085 Reset pin (active low)
    input  logic        int1,
    
    // Debug / Status LEDs
    output logic        led_initialized, // ON when BNO085 init complete
    output logic        led_error,       // ON if error state
    output logic        led_heartbeat    // Blinks to show clock is running
);

    // Internal signals
    logic clk;
    logic rst_n;  // Internal reset signal
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic initialized, error;
    
    // BNO085 Reset Two-Stage Sequence:
    // Stage 1: imu_rst=1 (HIGH), FPGA reset pulse -> INT oscillates
    // Stage 2: imu_rst=0 (LOW), FPGA reset pulse -> protocol starts
    localparam [22:0] DELAY_2SEC = 23'd6_000_000;
    logic [22:0] rst_delay_counter;
    logic bno085_rst_n_delayed;
    logic fpga_rst_n_sync, fpga_rst_n_prev;
    logic [1:0] reset_stage;  // 0=initial, 1=first pulse done, 2=second pulse active
    
    // Connect FPGA reset to internal reset
    assign rst_n = fpga_rst_n;
    
    // Synchronize FPGA reset for edge detection
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            fpga_rst_n_sync <= 1'b0;
            fpga_rst_n_prev <= 1'b0;
        end else begin
            fpga_rst_n_prev <= fpga_rst_n_sync;
            fpga_rst_n_sync <= fpga_rst_n;
        end
    end
    
    // Detect FPGA reset rising edge (reset pulse complete: was LOW, now HIGH)
    logic fpga_rst_rising_edge;
    assign fpga_rst_rising_edge = !fpga_rst_n_prev && fpga_rst_n_sync;
    
    // BNO085 Reset Two-Stage State Machine
    // Note: reset_stage persists across FPGA reset pulses to track which stage we're in
    always_ff @(posedge clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            // FPGA reset is active (LOW)
            rst_delay_counter <= 23'd0;
            // Don't reset reset_stage - it needs to persist to track which pulse we're on
            
            if (reset_stage == 2'd0) begin
                // Stage 0: First reset pulse - keep imu_rst HIGH
                bno085_rst_n_delayed <= 1'b1;  // Drive HIGH (imu_rst=1) -> INT oscillates
            end else if (reset_stage == 2'd1) begin
                // Stage 1: Second reset pulse - drive imu_rst LOW
                bno085_rst_n_delayed <= 1'b0;  // Drive LOW (imu_rst=0)
            end else begin
                // Stage 2: Protocol running - keep LOW during reset
                bno085_rst_n_delayed <= 1'b0;
            end
        end else begin
            // FPGA reset released (HIGH)
            if (reset_stage == 2'd0) begin
                // Stage 0: Waiting for first reset pulse to complete
                if (fpga_rst_rising_edge) begin
                    // First reset pulse complete -> move to stage 1, set imu_rst LOW
                    reset_stage <= 2'd1;
                    bno085_rst_n_delayed <= 1'b0;  // Drive LOW (imu_rst=0) for second stage
                end else begin
                    // Keep HIGH until first pulse completes
                    bno085_rst_n_delayed <= 1'b1;
                end
            end else if (reset_stage == 2'd1) begin
                // Stage 1: Waiting for second reset pulse
                if (fpga_rst_rising_edge) begin
                    // Second reset pulse detected -> start delay counter
                    reset_stage <= 2'd2;
                    rst_delay_counter <= 23'd0;
                    bno085_rst_n_delayed <= 1'b0;  // Keep LOW during delay
                end else begin
                    // Keep LOW while waiting for second pulse
                    bno085_rst_n_delayed <= 1'b0;
                end
            end else if (reset_stage == 2'd2) begin
                // Stage 2: Count delay, then release to start protocol
                if (rst_delay_counter < DELAY_2SEC) begin
                    rst_delay_counter <= rst_delay_counter + 1;
                    bno085_rst_n_delayed <= 1'b0;  // Keep LOW during delay
                end else begin
                    bno085_rst_n_delayed <= 1'b1;  // Drive HIGH - protocol starts
                end
            end
        end
    end
    
    assign bno085_rst_n1 = bno085_rst_n_delayed;

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