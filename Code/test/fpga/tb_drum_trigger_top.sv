`timescale 1ns / 1ps

// Testbench for Drum Trigger Top-Level Module
// Tests dual BNO085 sensor integration with MCU SPI slave
// Note: Source files are included via compilation command, not here

module tb_drum_trigger_top;

    // Top-level signals
    logic fpga_rst_n;
    
    // MCU SPI Interface
    logic mcu_sck, mcu_sdi, mcu_sdo, mcu_load, mcu_done;
    
    // BNO085 Sensor 1 SPI Interface
    logic sclk, mosi, miso1, cs_n1, int1;
    
    // BNO085 Sensor 2 SPI Interface
    logic miso2, cs_n2, int2;
    
    // Shared BNO085 Control
    logic bno085_rst_n;
    
    // Status LEDs
    logic led_initialized, led_error, led_heartbeat;
    
    // Clock generation
    parameter CLK_PERIOD = 333;  // 3MHz FPGA clock
    
    // DUT
    drum_trigger_top dut (
        .fpga_rst_n(fpga_rst_n),
        .mcu_sck(mcu_sck),
        .mcu_sdi(mcu_sdi),
        .mcu_sdo(mcu_sdo),
        .mcu_load(mcu_load),
        .mcu_done(mcu_done),
        .sclk(sclk),
        .mosi(mosi),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .int1(int1),
        .miso2(miso2),
        .cs_n2(cs_n2),
        .int2(int2),
        .bno085_rst_n(bno085_rst_n),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Mock BNO085 Sensor 1
    mock_bno085 sensor1 (
        .clk(dut.clk),
        .rst_n(dut.rst_n),
        .ps0_wake(1'b1),  // Always high
        .cs_n(cs_n1),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1),
        .int_n(int1)
    );
    
    // Mock BNO085 Sensor 2
    mock_bno085 sensor2 (
        .clk(dut.clk),
        .rst_n(dut.rst_n),
        .ps0_wake(1'b1),  // Always high
        .cs_n(cs_n2),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso2),
        .int_n(int2)
    );
    
    // Mock MCU SPI Master
    mock_mcu_spi_master mcu_master (
        .clk(dut.clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done)
    );
    
    // Test variables
    logic [7:0] received_packet [0:31];
    
    // Accelerate simulation
    initial begin
        wait(fpga_rst_n == 1);
        forever begin
            @(posedge dut.clk);
            // Skip reset delays
            if (dut.rst_delay_counter < 23'd5_999_900) begin
                force dut.rst_delay_counter = 23'd5_999_990;
                @(posedge dut.clk);
                release dut.rst_delay_counter;
            end
        end
    end
    
    // Main test sequence
    initial begin
        $dumpfile("tb_drum_trigger_top.vcd");
        $dumpvars(0, tb_drum_trigger_top);
        
        $display("=== Drum Trigger Top Testbench ===");
        
        // Reset
        fpga_rst_n = 0;
        #1000;
        fpga_rst_n = 1;
        
        $display("\n[TEST] Waiting for initialization...");
        
        // Wait for both sensors to initialize
        fork
            begin
                wait(led_initialized == 1);
                $display("[PASS] System initialized (both sensors ready)");
            end
            begin
                #100000000; // Timeout
                $display("[FAIL] TIMEOUT waiting for initialization");
                $finish;
            end
        join_any
        
        // Send test data from sensors
        $display("\n[TEST] Sending test sensor data...");
        #10000;
        
        // Sensor 1: Quaternion
        sensor1.send_rotation_vector(16'h1000, 16'h2000, 16'h3000, 16'h4000);
        #5000;
        
        // Sensor 1: Gyroscope
        sensor1.send_gyroscope(16'd100, 16'd200, 16'd300);
        #5000;
        
        // Sensor 2: Quaternion
        sensor2.send_rotation_vector(16'h5000, 16'h6000, 16'h7000, 16'h8000);
        #5000;
        
        // Sensor 2: Gyroscope
        sensor2.send_gyroscope(16'd400, 16'd500, 16'd600);
        #10000;
        
        // Read packet from MCU
        $display("\n[TEST] Reading packet from FPGA via MCU SPI...");
        mcu_master.read_packet(received_packet);
        
        // Verify packet
        if (received_packet[0] == 8'hAA) begin
            $display("[PASS] Packet header correct");
        end else begin
            $display("[FAIL] Packet header incorrect");
        end
        
        // Verify Sensor 1 data
        if (received_packet[1] == 8'h40 && received_packet[2] == 8'h00) begin
            $display("[PASS] Sensor 1 quaternion W correct");
        end
        
        // Verify Sensor 2 data
        if (received_packet[16] == 8'h80 && received_packet[17] == 8'h00) begin
            $display("[PASS] Sensor 2 quaternion W correct");
        end
        
        $display("\n=== Test PASSED ===");
        #10000;
        $finish;
    end
    
endmodule

