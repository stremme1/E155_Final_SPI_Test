/**
 * BNO08X Drum System Top-Level Module
 * 
 * Integrates BNO08X SPI controller, sensor processing, and drum logic
 * Implements the drum trigger system from Code_for_C_imp/main.c
 * 
 * Clock: 3MHz (from 48MHz HSOSC / 16)
 */

module bno08x_drum_system (
    input  logic        clk,              // 3MHz clock from HSOSC
    input  logic        rst_n,            // Active low reset
    
    // BNO08X SPI Interface
    output logic        spi_cs_n,
    output logic        spi_sck,
    output logic        spi_mosi,
    input  logic        spi_miso,
    
    // BNO08X Control Signals
    input  logic        h_intn,           // BNO08X interrupt (active low)
    output logic        wake,             // Wake signal (active low)
    output logic        reset_bno,        // Reset signal for BNO08X
    
    // Sensor Selection (for dual sensor system)
    input  logic        sensor_select,    // 0 = sensor 1, 1 = sensor 2
    
    // Control Interface
    input  logic        enable_system,
    input  logic [7:0]  sensor_id,       // Sensor ID to enable
    input  logic [31:0] report_interval,  // Report interval in microseconds
    
    // Yaw Offset Configuration
    input  logic        set_yaw_offset,   // Set current yaw as offset
    input  logic [31:0] yaw_offset1,       // Yaw offset for sensor 1
    input  logic [31:0] yaw_offset2,       // Yaw offset for sensor 2
    
    // Drum Trigger Output
    output logic [3:0]  drum_trigger,     // 0-7: drum sounds, 8: no trigger
    output logic        trigger_valid,
    
    // Status Outputs
    output logic        initialized,
    output logic        error,
    
    // Debug Outputs
    output logic [31:0] quat_w, quat_x, quat_y, quat_z,
    output logic [31:0] gyro_x, gyro_y, gyro_z,
    output logic [31:0] roll, pitch, yaw
);

    // Internal signals
    logic controller_data_ready;
    logic [7:0] controller_report_id;
    logic [7:0] controller_sensor_data [0:15];
    logic [4:0] controller_data_len;
    
    logic processor_euler_valid;
    logic [31:0] processor_quat_w, processor_quat_x, processor_quat_y, processor_quat_z;
    logic [31:0] processor_gyro_x, processor_gyro_y, processor_gyro_z;
    logic [31:0] processor_roll, processor_pitch, processor_yaw;
    
    logic [31:0] yaw_offset_reg1, yaw_offset_reg2;
    logic [31:0] current_yaw_offset;
    
    // Yaw offset management
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_offset_reg1 <= 32'd0;
            yaw_offset_reg2 <= 32'd0;
        end else begin
            if (set_yaw_offset) begin
                if (!sensor_select) begin
                    yaw_offset_reg1 <= processor_yaw;
                end else begin
                    yaw_offset_reg2 <= processor_yaw;
                end
            end else begin
                if (yaw_offset1 != 32'd0) yaw_offset_reg1 <= yaw_offset1;
                if (yaw_offset2 != 32'd0) yaw_offset_reg2 <= yaw_offset2;
            end
        end
    end
    
    assign current_yaw_offset = sensor_select ? yaw_offset_reg2 : yaw_offset_reg1;
    
    // Instantiate BNO08X Controller
    bno08x_controller controller (
        .clk(clk),
        .rst_n(rst_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .h_intn(h_intn),
        .wake(wake),
        .enable_sensor(enable_system),
        .sensor_id(sensor_id),
        .report_interval(report_interval),
        .data_ready(controller_data_ready),
        .sensor_report_id(controller_report_id),
        .sensor_data(controller_sensor_data),
        .sensor_data_len(controller_data_len),
        .initialized(initialized),
        .error(error)
    );
    
    // Instantiate Sensor Processor
    sensor_processor processor (
        .clk(clk),
        .rst_n(rst_n),
        .data_ready(controller_data_ready),
        .sensor_report_id(controller_report_id),
        .sensor_data(controller_sensor_data),
        .sensor_data_len(controller_data_len),
        .quat_w(processor_quat_w),
        .quat_x(processor_quat_x),
        .quat_y(processor_quat_y),
        .quat_z(processor_quat_z),
        .gyro_x(processor_gyro_x),
        .gyro_y(processor_gyro_y),
        .gyro_z(processor_gyro_z),
        .roll(processor_roll),
        .pitch(processor_pitch),
        .yaw(processor_yaw),
        .euler_valid(processor_euler_valid),
        .drum_trigger(drum_trigger),
        .trigger_valid(trigger_valid),
        .yaw_offset1(yaw_offset_reg1),
        .yaw_offset2(yaw_offset_reg2),
        .sensor_select(sensor_select)
    );
    
    // Output assignments
    assign quat_w = processor_quat_w;
    assign quat_x = processor_quat_x;
    assign quat_y = processor_quat_y;
    assign quat_z = processor_quat_z;
    assign gyro_x = processor_gyro_x;
    assign gyro_y = processor_gyro_y;
    assign gyro_z = processor_gyro_z;
    assign roll = processor_roll;
    assign pitch = processor_pitch;
    assign yaw = processor_yaw;
    
    // Reset signal (can be used for hardware reset)
    assign reset_bno = rst_n;
    
endmodule

