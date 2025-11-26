/**
 * Testbench for BNO08X Drum System Top-Level Module
 * 
 * Integration test for the complete drum system
 */

`timescale 1ns / 1ps

module tb_bno08x_drum_system;

    // Parameters
    parameter CLK_PERIOD = 333; // 3MHz = 333ns period
    
    // Signals
    logic        clk;
    logic        rst_n;
    logic        spi_cs_n;
    logic        spi_sck;
    logic        spi_mosi;
    logic        spi_miso;
    logic        h_intn;
    logic        wake;
    logic        reset_bno;
    logic        sensor_select;
    logic        enable_system;
    logic [7:0]  sensor_id;
    logic [31:0] report_interval;
    logic        set_yaw_offset;
    logic [31:0] yaw_offset1;
    logic [31:0] yaw_offset2;
    logic [3:0]  drum_trigger;
    logic        trigger_valid;
    logic        initialized;
    logic        error;
    logic [31:0] quat_w, quat_x, quat_y, quat_z;
    logic [31:0] gyro_x, gyro_y, gyro_z;
    logic [31:0] roll, pitch, yaw;
    
    // Testbench signals - BNO08X simulation
    logic [7:0]  bno08x_tx_buffer [0:255];
    int          bno08x_tx_idx;
    logic        bno08x_has_data;
    logic [15:0] bno08x_packet_length;
    int          bno08x_bit_counter;
    
    // Instantiate DUT
    bno08x_drum_system dut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .h_intn(h_intn),
        .wake(wake),
        .reset_bno(reset_bno),
        .sensor_select(sensor_select),
        .enable_system(enable_system),
        .sensor_id(sensor_id),
        .report_interval(report_interval),
        .set_yaw_offset(set_yaw_offset),
        .yaw_offset1(yaw_offset1),
        .yaw_offset2(yaw_offset2),
        .drum_trigger(drum_trigger),
        .trigger_valid(trigger_valid),
        .initialized(initialized),
        .error(error),
        .quat_w(quat_w),
        .quat_x(quat_x),
        .quat_y(quat_y),
        .quat_z(quat_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .roll(roll),
        .pitch(pitch),
        .yaw(yaw)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // BNO08X Simulation - MISO driver
    always_ff @(negedge spi_sck or negedge rst_n) begin
        if (!rst_n) begin
            spi_miso <= 1'b0;
            bno08x_tx_idx <= 0;
            bno08x_bit_counter <= 0;
        end else if (!spi_cs_n && bno08x_has_data) begin
            if (bno08x_tx_idx < bno08x_packet_length) begin
                spi_miso <= bno08x_tx_buffer[bno08x_tx_idx][7 - bno08x_bit_counter];
                if (bno08x_bit_counter == 7) begin
                    bno08x_bit_counter <= 0;
                    bno08x_tx_idx <= bno08x_tx_idx + 1;
                end else begin
                    bno08x_bit_counter <= bno08x_bit_counter + 1;
                end
            end
        end
    end
    
    // BNO08X Simulation - Reset packet counter on CS deassert
    always_ff @(posedge spi_cs_n or negedge rst_n) begin
        if (!rst_n) begin
            bno08x_tx_idx <= 0;
            bno08x_bit_counter <= 0;
        end else begin
            bno08x_tx_idx <= 0;
            bno08x_bit_counter <= 0;
        end
    end
    
    // Task to prepare Product ID Response
    task prepare_product_id_response();
        bno08x_tx_buffer[0] = 8'h15; // Length LSB (21 bytes)
        bno08x_tx_buffer[1] = 8'h00; // Length MSB
        bno08x_tx_buffer[2] = 8'h02; // Channel (Control)
        bno08x_tx_buffer[3] = 8'h01; // Sequence
        bno08x_tx_buffer[4] = 8'hF8; // Report ID (Product ID Response)
        bno08x_tx_buffer[5] = 8'h01; // Reset Cause
        bno08x_tx_buffer[6] = 8'h03; // SW Version Major
        bno08x_tx_buffer[7] = 8'h02; // SW Version Minor
        bno08x_tx_buffer[8] = 8'h00; // SW Part Number LSB
        bno08x_tx_buffer[9] = 8'h00;
        bno08x_tx_buffer[10] = 8'h00;
        bno08x_tx_buffer[11] = 8'h00; // SW Part Number MSB
        bno08x_tx_buffer[12] = 8'h00; // SW Build Number LSB
        bno08x_tx_buffer[13] = 8'h00;
        bno08x_tx_buffer[14] = 8'h00;
        bno08x_tx_buffer[15] = 8'h00; // SW Build Number MSB
        bno08x_tx_buffer[16] = 8'h00; // SW Version Patch LSB
        bno08x_tx_buffer[17] = 8'h00; // SW Version Patch MSB
        bno08x_tx_buffer[18] = 8'h00; // Reserved
        bno08x_tx_buffer[19] = 8'h00; // Reserved
        bno08x_tx_buffer[20] = 8'h00; // Reserved
        
        bno08x_packet_length = 21;
        bno08x_has_data = 1'b1;
        bno08x_tx_idx = 0;
        bno08x_bit_counter = 0;
    endtask
    
    // Task to prepare Game Rotation Vector report
    task prepare_rotation_vector_report(
        input logic [31:0] q_i,
        input logic [31:0] q_j,
        input logic [31:0] q_k
    );
        bno08x_tx_buffer[0] = 8'h13; // Length LSB (19 bytes)
        bno08x_tx_buffer[1] = 8'h00; // Length MSB
        bno08x_tx_buffer[2] = 8'h03; // Channel (Input Reports)
        bno08x_tx_buffer[3] = 8'h01; // Sequence
        bno08x_tx_buffer[4] = 8'h08; // Report ID (Game Rotation Vector)
        bno08x_tx_buffer[5] = 8'h01; // Sequence number
        bno08x_tx_buffer[6] = 8'h03; // Status (Accuracy High)
        bno08x_tx_buffer[7] = 8'h00; // Delay LSB
        // Quaternion i (Q30) - little endian
        bno08x_tx_buffer[8] = q_i[7:0];
        bno08x_tx_buffer[9] = q_i[15:8];
        bno08x_tx_buffer[10] = q_i[23:16];
        bno08x_tx_buffer[11] = q_i[31:24];
        // Quaternion j (Q30)
        bno08x_tx_buffer[12] = q_j[7:0];
        bno08x_tx_buffer[13] = q_j[15:8];
        bno08x_tx_buffer[14] = q_j[23:16];
        bno08x_tx_buffer[15] = q_j[31:24];
        // Quaternion k (Q30)
        bno08x_tx_buffer[16] = q_k[7:0];
        bno08x_tx_buffer[17] = q_k[15:8];
        bno08x_tx_buffer[18] = q_k[23:16];
        
        bno08x_packet_length = 19;
        bno08x_has_data = 1'b1;
        bno08x_tx_idx = 0;
        bno08x_bit_counter = 0;
    endtask
    
    // Task to prepare Gyroscope report
    task prepare_gyroscope_report(
        input logic [15:0] g_x,
        input logic [15:0] g_y,
        input logic [15:0] g_z
    );
        bno08x_tx_buffer[0] = 8'h0A; // Length LSB (10 bytes)
        bno08x_tx_buffer[1] = 8'h00; // Length MSB
        bno08x_tx_buffer[2] = 8'h03; // Channel (Input Reports)
        bno08x_tx_buffer[3] = 8'h02; // Sequence
        bno08x_tx_buffer[4] = 8'h02; // Report ID (Gyroscope)
        bno08x_tx_buffer[5] = 8'h01; // Sequence number
        bno08x_tx_buffer[6] = 8'h03; // Status
        bno08x_tx_buffer[7] = 8'h00; // Delay LSB
        // Gyro X (Q16, little endian)
        bno08x_tx_buffer[8] = g_x[7:0];
        bno08x_tx_buffer[9] = g_x[15:8];
        
        bno08x_packet_length = 10;
        bno08x_has_data = 1'b1;
        bno08x_tx_idx = 0;
        bno08x_bit_counter = 0;
    endtask
    
    // Test tasks
    task reset_dut();
        rst_n = 0;
        enable_system = 0;
        sensor_id = 0;
        report_interval = 0;
        sensor_select = 0;
        set_yaw_offset = 0;
        yaw_offset1 = 0;
        yaw_offset2 = 0;
        h_intn = 1'b1;
        bno08x_has_data = 0;
        #(10 * CLK_PERIOD);
        rst_n = 1;
        #(5 * CLK_PERIOD);
    endtask
    
    // Main test sequence
    initial begin
        $display("=========================================");
        $display("BNO08X Drum System Integration Testbench");
        $display("=========================================");
        
        // Initialize
        reset_dut();
        
        // Test 1: System Initialization
        $display("\n--- Test 1: System Initialization ---");
        
        // Simulate BNO08X asserting interrupt after reset
        #(100 * CLK_PERIOD);
        prepare_product_id_response();
        h_intn <= 1'b0;
        $display("[%0t] H_INTN asserted, Product ID response ready", $time);
        
        // Wait for initialization
        #(5000 * CLK_PERIOD);
        
        wait(initialized || error || (($time / CLK_PERIOD) > 50000));
        if (initialized) begin
            $display("[%0t] ✓ System initialized successfully", $time);
        end else if (error) begin
            $display("[%0t] ✗ Error during initialization", $time);
        end else begin
            $display("[%0t] ✗ Initialization timeout", $time);
        end
        
        #(100 * CLK_PERIOD);
        
        // Test 2: Enable Sensor
        $display("\n--- Test 2: Enable Sensor ---");
        sensor_id = 8'h08; // Game Rotation Vector
        report_interval = 32'd10000; // 10ms = 100Hz
        enable_system = 1;
        
        #(1000 * CLK_PERIOD);
        
        // Test 3: Send Sensor Data and Check Drum Trigger
        $display("\n--- Test 3: Drum Trigger Test ---");
        sensor_select = 0; // Right hand
        
        // Send quaternion for yaw = 60 degrees (in snare zone 20-120)
        prepare_rotation_vector_report(
            32'h00000000,  // i
            32'h00000000,  // j
            32'h2AAAAAAA   // k (simplified for ~60 deg yaw)
        );
        h_intn <= 1'b0;
        #(200 * CLK_PERIOD);
        h_intn <= 1'b1;
        
        #(1000 * CLK_PERIOD);
        
        // Send gyroscope with negative Y (trigger condition)
        prepare_gyroscope_report(
            16'h0000,  // X = 0
            16'hF63C,  // Y = -2500 (below threshold)
            16'h0000   // Z = 0
        );
        h_intn <= 1'b0;
        #(200 * CLK_PERIOD);
        h_intn <= 1'b1;
        
        #(1000 * CLK_PERIOD);
        
        if (trigger_valid) begin
            $display("[%0t] ✓ Drum trigger detected: %0d", $time, drum_trigger);
            if (drum_trigger == 0) begin
                $display("    Expected: Snare drum (0)");
            end
        end else begin
            $display("[%0t] ✗ No trigger detected", $time);
        end
        
        // Test 4: Yaw Offset
        $display("\n--- Test 4: Yaw Offset Test ---");
        set_yaw_offset = 1;
        #(10 * CLK_PERIOD);
        set_yaw_offset = 0;
        $display("[%0t] Yaw offset set", $time);
        
        #(100 * CLK_PERIOD);
        
        $display("\n=========================================");
        $display("All Tests Complete");
        $display("=========================================");
        $display("Final Status:");
        $display("  Initialized: %b", initialized);
        $display("  Error: %b", error);
        $display("  Last Trigger: %0d (valid: %b)", drum_trigger, trigger_valid);
        $display("=========================================");
        
        #(1000 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor key signals
    initial begin
        $monitor("[%0t] INIT=%b ERR=%b TRIGGER=%0d TRIG_VALID=%b INT=%b", 
                 $time, initialized, error, drum_trigger, trigger_valid, h_intn);
    end

endmodule

