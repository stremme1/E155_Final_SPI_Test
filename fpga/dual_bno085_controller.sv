`timescale 1ns / 1ps

// Dual BNO085 Controller Module
// Manages two BNO085 sensors with separate SPI interfaces
// Provides unified data output when both sensors have new data

module dual_bno085_controller (
    input  logic        clk,
    input  logic        rst_n,
    
    // Sensor 1 SPI interface
    output logic        spi1_start,
    output logic        spi1_tx_valid,
    output logic [7:0]  spi1_tx_data,
    input  logic        spi1_tx_ready,
    input  logic        spi1_rx_valid,
    input  logic [7:0]  spi1_rx_data,
    input  logic        spi1_busy,
    output logic        cs_n1,
    output logic        ps0_wake1,
    input  logic        int_n1,
    
    // Sensor 2 SPI interface
    output logic        spi2_start,
    output logic        spi2_tx_valid,
    output logic [7:0]  spi2_tx_data,
    input  logic        spi2_tx_ready,
    input  logic        spi2_rx_valid,
    input  logic [7:0]  spi2_rx_data,
    input  logic        spi2_busy,
    output logic        cs_n2,
    output logic        ps0_wake2,
    input  logic        int_n2,
    
    // Combined sensor data outputs
    output logic        data_ready,  // Both sensors have new data
    output logic        sensor1_valid,
    output logic        sensor2_valid,
    
    // Sensor 1 data
    output logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    output logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // Sensor 2 data
    output logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z,
    output logic signed [15:0] gyro2_x, gyro2_y, gyro2_z,
    
    // Status
    output logic        initialized,  // Both sensors initialized
    output logic        error          // Error from either sensor
);

    // Internal signals for sensor controllers
    logic quat1_valid, gyro1_valid, quat2_valid, gyro2_valid;
    logic initialized1, initialized2, error1, error2;
    
    // BNO085 Controller for Sensor 1
    bno085_controller bno085_ctrl1 (
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
        .int_n(int_n1),
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
    
    // BNO085 Controller for Sensor 2
    bno085_controller bno085_ctrl2 (
        .clk(clk),
        .rst_n(rst_n),
        .spi_start(spi2_start),
        .spi_tx_valid(spi2_tx_valid),
        .spi_tx_data(spi2_tx_data),
        .spi_tx_ready(spi2_tx_ready),
        .spi_rx_valid(spi2_rx_valid),
        .spi_rx_data(spi2_rx_data),
        .spi_busy(spi2_busy),
        .cs_n(cs_n2),
        .ps0_wake(ps0_wake2),
        .int_n(int_n2),
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
    
    // Combined status
    assign initialized = initialized1 && initialized2;
    assign error = error1 || error2;
    
    // Data ready when both sensors have new quaternion AND gyro data
    // Track both quat and gyro separately to ensure complete data
    logic sensor1_quat_pending, sensor1_gyro_pending;
    logic sensor2_quat_pending, sensor2_gyro_pending;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor1_quat_pending <= 1'b0;
            sensor1_gyro_pending <= 1'b0;
            sensor2_quat_pending <= 1'b0;
            sensor2_gyro_pending <= 1'b0;
            sensor1_valid <= 1'b0;
            sensor2_valid <= 1'b0;
        end else begin
            // Track quaternion and gyroscope separately
            if (quat1_valid) begin
                sensor1_quat_pending <= 1'b1;
                sensor1_valid <= 1'b1;
            end
            if (gyro1_valid) begin
                sensor1_gyro_pending <= 1'b1;
                sensor1_valid <= 1'b1;
            end
            if (quat2_valid) begin
                sensor2_quat_pending <= 1'b1;
                sensor2_valid <= 1'b1;
            end
            if (gyro2_valid) begin
                sensor2_gyro_pending <= 1'b1;
                sensor2_valid <= 1'b1;
            end
            
            // Clear pending flags when formatter has processed the data
            // For now, keep them set - they'll be cleared when new data arrives
        end
    end
    
    // Data ready when both sensors have BOTH quat AND gyro data
    assign data_ready = (sensor1_quat_pending && sensor1_gyro_pending) &&
                        (sensor2_quat_pending && sensor2_gyro_pending);
    
endmodule

