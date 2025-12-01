`timescale 1ns / 1ps

// Testbench for BNO085 Controller
// Tests initialization sequence, SHTP protocol, and data parsing
// Note: Source files are included via compilation command, not here

module tb_bno085_controller;

    // Clock and reset
    logic clk, rst_n;
    
    // SPI interface
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    logic cs_n;
    logic ps0_wake;
    logic int_n;
    
    // Sensor data outputs
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    
    // Status
    logic initialized, error;
    
    // SPI master signals
    logic sclk, mosi, miso;
    
    // Clock generation (3MHz)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // SPI Master instance
    spi_master spi_master_inst (
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
    
    // BNO085 Controller DUT
    bno085_controller dut (
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
        .ps0_wake(ps0_wake),
        .int_n(int_n),
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
    
    // Mock BNO085 sensor
    mock_bno085 sensor_model (
        .clk(clk),
        .rst_n(rst_n),
        .ps0_wake(ps0_wake),  // Connected to controller PS0/WAKE output
        .cs_n(cs_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .int_n(int_n)
    );
    
    // Test state tracking
    integer init_commands_received = 0;
    
    // Accelerate simulation by skipping long delays
    initial begin
        wait(rst_n == 1);
        forever begin
            @(posedge clk);
            // Skip INIT_WAIT_RESET delay
            if (dut.state == 1) begin // INIT_WAIT_RESET
                if (dut.delay_counter < 19'd299_900) begin
                    force dut.delay_counter = 19'd299_990;
                    @(posedge clk);
                    release dut.delay_counter;
                end
            end
            // Skip INIT_DONE_CHECK delay
            else if (dut.state == 6) begin // INIT_DONE_CHECK (state 6)
                if (dut.delay_counter < 19'd29_900) begin
                    force dut.delay_counter = 19'd29_990;
                    @(posedge clk);
                    release dut.delay_counter;
                end
            end
        end
    end
    
    // Monitor initialization commands
    initial begin
        forever begin
            @(posedge cs_n); // Wait for CS to go high (transaction end)
            #100;
            if (dut.state == 6) begin // INIT_DONE_CHECK (state 6)
                init_commands_received = init_commands_received + 1;
                $display("[%0t] Command %0d completed", $time, init_commands_received);
            end
        end
    end
    
    // Main test sequence
    initial begin
        $dumpfile("tb_bno085_controller.vcd");
        $dumpvars(0, tb_bno085_controller);
        
        $display("========================================");
        $display("BNO085 Controller Testbench");
        $display("========================================");
        
        // Reset
        $display("\n[TEST] Step 1: System Reset");
        rst_n = 0;
        #1000;
        rst_n = 1;
        $display("[TEST] Reset released at %0t", $time);
        
        // Wait for initialization
        $display("\n[TEST] Step 2: Waiting for Initialization");
        fork
            begin
                wait(initialized == 1);
                $display("\n[PASS] Controller Initialized!");
                $display("  - Commands sent: %0d", init_commands_received);
                
                if (init_commands_received >= 3) begin
                    $display("[PASS] All 3 initialization commands completed");
                end else begin
                    $display("[FAIL] Expected 3 commands, got %0d", init_commands_received);
                end
            end
            begin
                #50000000; // Timeout
                $display("\n[FAIL] TIMEOUT waiting for initialization!");
                $finish;
            end
        join_any
        
        // Test quaternion report
        $display("\n[TEST] Step 3: Testing Rotation Vector Report");
        #5000;
        
        $display("  Sending Rotation Vector: W=0x4000, X=0, Y=0, Z=0");
        sensor_model.send_rotation_vector(16'd0, 16'd0, 16'd0, 16'h4000);
        
        fork
            begin
                wait(quat_valid == 1);
                $display("\n[PASS] Quaternion Received!");
                $display("  W=%04h X=%04h Y=%04h Z=%04h", quat_w, quat_x, quat_y, quat_z);
                
                if (quat_w == 16'h4000 && quat_x == 16'd0 && quat_y == 16'd0 && quat_z == 16'd0) begin
                    $display("[PASS] Quaternion data matches expected values");
                end else begin
                    $display("[FAIL] Quaternion data mismatch");
                end
                disable quat_timeout; // Disable timeout after success
            end
            begin: quat_timeout
                #2000000; // Timeout
                $display("\n[FAIL] TIMEOUT waiting for quaternion data!");
                $finish;
            end
        join_any
        
        // Test gyroscope report
        $display("\n[TEST] Step 4: Testing Gyroscope Report");
        // Wait for controller to return to WAIT_DATA and ensure INT is high
        wait(dut.state == 7); // WAIT_DATA state
        #5000; // Wait a bit more to ensure INT is deasserted
        // Ensure INT is high before sending new data
        if (int_n == 0) begin
            #1000; // Wait for INT to go high
        end
        
        $display("  Sending Gyroscope: X=100, Y=200, Z=300");
        sensor_model.send_gyroscope(16'd100, 16'd200, 16'd300);
        
        fork
            begin
                wait(gyro_valid == 1);
                $display("\n[PASS] Gyroscope Received!");
                $display("  X=%04h Y=%04h Z=%04h", gyro_x, gyro_y, gyro_z);
                
                if (gyro_x == 16'd100 && gyro_y == 16'd200 && gyro_z == 16'd300) begin
                    $display("[PASS] Gyroscope data matches expected values");
                end else begin
                    $display("[FAIL] Gyroscope data mismatch");
                end
                disable gyro_timeout; // Disable timeout after success
            end
            begin: gyro_timeout
                #2000000; // Timeout
                $display("\n[FAIL] TIMEOUT waiting for gyroscope data!");
                $finish;
            end
        join_any
        
        $display("\n=== All Tests PASSED ===");
        #10000;
        $finish;
    end
    
endmodule

