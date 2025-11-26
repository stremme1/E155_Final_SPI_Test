/**
 * Testbench for BNO08X Controller Module
 * 
 * Tests initialization sequence, sensor configuration, and data reading
 */

`timescale 1ns / 1ps

module tb_bno08x_controller;

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
    logic        enable_sensor;
    logic [7:0]  sensor_id;
    logic [31:0] report_interval;
    logic        data_ready;
    logic [7:0]  sensor_report_id;
    logic [7:0]  sensor_data [0:15];
    logic [4:0]  sensor_data_len;
    logic        initialized;
    logic        error;
    
    // Testbench signals - BNO08X simulation
    logic [7:0]  bno08x_rx_buffer [0:255];
    logic [7:0]  bno08x_tx_buffer [0:255];
    int          bno08x_rx_idx;
    int          bno08x_tx_idx;
    logic        bno08x_has_data;
    logic [15:0] bno08x_packet_length;
    
    // Instantiate DUT
    bno08x_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .h_intn(h_intn),
        .wake(wake),
        .enable_sensor(enable_sensor),
        .sensor_id(sensor_id),
        .report_interval(report_interval),
        .data_ready(data_ready),
        .sensor_report_id(sensor_report_id),
        .sensor_data(sensor_data),
        .sensor_data_len(sensor_data_len),
        .initialized(initialized),
        .error(error)
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
        end else if (!spi_cs_n && bno08x_has_data) begin
            // Send data from BNO08X
            if (bno08x_tx_idx < bno08x_packet_length) begin
                // Shift out MSB first
                spi_miso <= bno08x_tx_buffer[bno08x_tx_idx][7 - (bno08x_tx_idx % 8)];
                if ((bno08x_tx_idx % 8) == 7) begin
                    bno08x_tx_idx <= bno08x_tx_idx + 1;
                end
            end
        end
    end
    
    // BNO08X Simulation - MOSI receiver
    always_ff @(posedge spi_sck or negedge rst_n) begin
        if (!rst_n) begin
            bno08x_rx_idx <= 0;
        end else if (!spi_cs_n) begin
            // Receive data from host
            bno08x_rx_buffer[bno08x_rx_idx] <= {bno08x_rx_buffer[bno08x_rx_idx][6:0], spi_mosi};
            if ((bno08x_rx_idx % 8) == 7) begin
                bno08x_rx_idx <= bno08x_rx_idx + 1;
            end
        end
    end
    
    // BNO08X Simulation - Process received commands
    always_ff @(posedge spi_cs_n or negedge rst_n) begin
        if (!rst_n) begin
            bno08x_has_data <= 1'b0;
            h_intn <= 1'b1;
        end else begin
            // Process received command
            if (bno08x_rx_idx > 0) begin
                // Check if it's a Product ID Request
                if (bno08x_rx_buffer[4] == 8'hF9) begin
                    // Prepare Product ID Response
                    prepare_product_id_response();
                    h_intn <= 1'b0; // Assert interrupt
                    #(100 * CLK_PERIOD);
                    h_intn <= 1'b1; // Deassert interrupt
                end
            end
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
    endtask
    
    // Task to prepare Game Rotation Vector report
    task prepare_rotation_vector_report();
        bno08x_tx_buffer[0] = 8'h13; // Length LSB (19 bytes)
        bno08x_tx_buffer[1] = 8'h00; // Length MSB
        bno08x_tx_buffer[2] = 8'h03; // Channel (Input Reports)
        bno08x_tx_buffer[3] = 8'h01; // Sequence
        bno08x_tx_buffer[4] = 8'h08; // Report ID (Game Rotation Vector)
        bno08x_tx_buffer[5] = 8'h01; // Sequence number
        bno08x_tx_buffer[6] = 8'h03; // Status (Accuracy High)
        bno08x_tx_buffer[7] = 8'h00; // Delay LSB
        // Quaternion i (Q30) - example values
        bno08x_tx_buffer[8] = 8'h00;
        bno08x_tx_buffer[9] = 8'h00;
        bno08x_tx_buffer[10] = 8'h00;
        bno08x_tx_buffer[11] = 8'h00;
        // Quaternion j (Q30)
        bno08x_tx_buffer[12] = 8'h00;
        bno08x_tx_buffer[13] = 8'h00;
        bno08x_tx_buffer[14] = 8'h00;
        bno08x_tx_buffer[15] = 8'h00;
        // Quaternion k (Q30)
        bno08x_tx_buffer[16] = 8'h00;
        bno08x_tx_buffer[17] = 8'h00;
        bno08x_tx_buffer[18] = 8'h00;
        
        bno08x_packet_length = 19;
        bno08x_has_data = 1'b1;
        bno08x_tx_idx = 0;
    endtask
    
    // Test tasks
    task reset_dut();
        rst_n = 0;
        enable_sensor = 0;
        sensor_id = 0;
        report_interval = 0;
        h_intn = 1'b1;
        #(10 * CLK_PERIOD);
        rst_n = 1;
        #(5 * CLK_PERIOD);
    endtask
    
    // Main test sequence
    initial begin
        $display("=========================================");
        $display("BNO08X Controller Testbench");
        $display("=========================================");
        
        // Initialize
        reset_dut();
        
        // Test 1: Wait for initialization
        $display("\n--- Test 1: Initialization Sequence ---");
        $display("[%0t] Waiting for interrupt...", $time);
        
        // Simulate BNO08X asserting interrupt after reset
        #(100 * CLK_PERIOD);
        h_intn <= 1'b0;
        $display("[%0t] H_INTN asserted", $time);
        
        // Wait for Product ID request
        #(1000 * CLK_PERIOD);
        
        // Check if initialized
        wait(initialized || error);
        if (initialized) begin
            $display("[%0t] ✓ Device initialized successfully", $time);
        end else if (error) begin
            $display("[%0t] ✗ Error during initialization", $time);
        end
        
        #(100 * CLK_PERIOD);
        
        // Test 2: Enable sensor
        $display("\n--- Test 2: Enable Sensor ---");
        sensor_id = 8'h08; // Game Rotation Vector
        report_interval = 32'd10000; // 10ms = 100Hz
        enable_sensor = 1;
        
        #(1000 * CLK_PERIOD);
        
        // Test 3: Simulate sensor data
        $display("\n--- Test 3: Sensor Data Reading ---");
        prepare_rotation_vector_report();
        h_intn <= 1'b0;
        #(100 * CLK_PERIOD);
        h_intn <= 1'b1;
        
        // Wait for data ready
        wait(data_ready || (($time / CLK_PERIOD) > 10000));
        if (data_ready) begin
            $display("[%0t] ✓ Sensor data received", $time);
            $display("    Report ID: 0x%02X", sensor_report_id);
            $display("    Data Length: %0d", sensor_data_len);
        end
        
        #(100 * CLK_PERIOD);
        
        $display("\n=========================================");
        $display("All Tests Complete");
        $display("=========================================");
        #(1000 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor signals
    initial begin
        $monitor("[%0t] INIT=%b ERR=%b DATA_RDY=%b INT=%b WAKE=%b", 
                 $time, initialized, error, data_ready, h_intn, wake);
    end

endmodule

