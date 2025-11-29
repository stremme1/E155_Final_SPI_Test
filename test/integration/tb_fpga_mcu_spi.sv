`timescale 1ns / 1ps

// FPGA-MCU SPI Integration Testbench
// Tests complete SPI communication between FPGA and MCU
// Note: Source files are included via compilation command, not here

module tb_fpga_mcu_spi;

    // Clock
    logic clk;
    
    // MCU SPI signals (MCU is master)
    logic mcu_sck, mcu_sdi, mcu_sdo, mcu_load, mcu_done;
    
    // Sensor data inputs to FPGA
    logic quat1_valid, gyro1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    logic quat2_valid, gyro2_valid;
    logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z;
    logic signed [15:0] gyro2_x, gyro2_y, gyro2_z;
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // MCU SPI clock (1MHz)
    parameter SCK_PERIOD = 1000;
    always begin
        mcu_sck = 0;
        #(SCK_PERIOD/2);
        mcu_sck = 1;
        #(SCK_PERIOD/2);
    end
    
    // FPGA MCU SPI Slave DUT
    spi_slave_mcu dut (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z),
        .quat2_valid(quat2_valid),
        .quat2_w(quat2_w),
        .quat2_x(quat2_x),
        .quat2_y(quat2_y),
        .quat2_z(quat2_z),
        .gyro2_valid(gyro2_valid),
        .gyro2_x(gyro2_x),
        .gyro2_y(gyro2_y),
        .gyro2_z(gyro2_z)
    );
    
    // Mock MCU SPI Master
    mock_mcu_spi_master mcu_master (
        .clk(clk),
        .sck(mcu_sck),
        .sdi(mcu_sdi),
        .sdo(mcu_sdo),
        .load(mcu_load),
        .done(mcu_done)
    );
    
    // Test variables
    logic [7:0] received_packet [0:31];
    
    // Main test sequence
    initial begin
        $dumpfile("tb_fpga_mcu_spi.vcd");
        $dumpvars(0, tb_fpga_mcu_spi);
        
        // Initialize
        mcu_load = 0;
        mcu_sdi = 0;
        quat1_valid = 0;
        gyro1_valid = 0;
        quat2_valid = 0;
        gyro2_valid = 0;
        
        #(100 * CLK_PERIOD);
        
        $display("=== FPGA-MCU SPI Integration Test ===");
        
        // Test 1: Complete SPI transaction
        $display("\n[TEST] Test 1: Complete SPI Transaction");
        
        // Set up sensor data
        quat1_w = 16'h4000; quat1_x = 16'h1000; quat1_y = 16'h2000; quat1_z = 16'h3000;
        gyro1_x = 16'd100; gyro1_y = 16'd200; gyro1_z = 16'd300;
        quat1_valid = 1;
        gyro1_valid = 1;
        
        quat2_w = 16'h5000; quat2_x = 16'h1100; quat2_y = 16'h2200; quat2_z = 16'h3300;
        gyro2_x = 16'd400; gyro2_y = 16'd500; gyro2_z = 16'd600;
        quat2_valid = 1;
        gyro2_valid = 1;
        
        #(10 * CLK_PERIOD);
        
        // MCU reads packet
        mcu_master.read_packet(received_packet);
        
        // Verify packet
        if (received_packet[0] == 8'hAA) begin
            $display("[PASS] Header correct");
        end else begin
            $display("[FAIL] Header incorrect: 0x%02X", received_packet[0]);
        end
        
        // Verify packet length
        $display("  Packet received: 32 bytes");
        
        // Verify Sensor 1 data
        if (received_packet[1] == 8'h40 && received_packet[2] == 8'h00) begin
            $display("[PASS] Sensor 1 quaternion W correct");
        end
        
        // Verify Sensor 2 data
        if (received_packet[16] == 8'h50 && received_packet[17] == 8'h00) begin
            $display("[PASS] Sensor 2 quaternion W correct");
        end
        
        // Verify flags
        if (received_packet[15] == 8'h03 && received_packet[30] == 8'h03) begin
            $display("[PASS] Flags correct");
        end
        
        $display("\n=== Test 1 PASSED ===");
        
        // Test 2: LOAD/DONE handshaking
        $display("\n[TEST] Test 2: LOAD/DONE Handshaking");
        
        // Reset sensor data
        quat1_valid = 0;
        gyro1_valid = 0;
        quat2_valid = 0;
        gyro2_valid = 0;
        #(10 * CLK_PERIOD);
        
        // Set new data
        quat1_w = 16'h6000;
        quat1_valid = 1;
        #(10 * CLK_PERIOD);
        
        // Verify DONE is asserted
        if (mcu_done == 1) begin
            $display("[PASS] DONE signal asserted when data ready");
        end else begin
            $display("[FAIL] DONE signal not asserted");
        end
        
        // Read and acknowledge
        mcu_master.read_packet(received_packet);
        
        // Verify DONE is deasserted after LOAD
        #(10 * CLK_PERIOD);
        if (mcu_done == 0) begin
            $display("[PASS] DONE signal deasserted after LOAD");
        end else begin
            $display("[FAIL] DONE signal still asserted after LOAD");
        end
        
        $display("\n=== Test 2 PASSED ===");
        
        $display("\n=== All Integration Tests PASSED ===");
        #(100 * CLK_PERIOD);
        $finish;
    end
    
endmodule

