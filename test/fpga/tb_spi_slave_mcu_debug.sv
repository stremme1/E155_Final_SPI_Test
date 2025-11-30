`timescale 1ns / 1ps

// Debug Testbench for SPI Slave MCU
// Simulates MCU SPI master in Mode 0 and tests FPGA SPI slave
// This will help identify the bit shifting issue

module tb_spi_slave_mcu_debug;

    // Clock
    logic clk;
    parameter CLK_PERIOD = 333;  // 3MHz FPGA clock
    
    // SPI interface
    logic cs_n = 1;      // Chip select (active low)
    logic sck = 0;       // SPI clock
    logic sdi = 0;       // SPI data in (MOSI - not used in read-only mode)
    logic sdo;           // SPI data out (MISO)
    
    // Sensor data inputs
    logic quat1_valid = 1;
    logic gyro1_valid = 1;
    logic signed [15:0] quat1_w = 16'h0000;
    logic signed [15:0] quat1_x = 16'h0000;
    logic signed [15:0] quat1_y = 16'h0000;
    logic signed [15:0] quat1_z = 16'h0000;
    logic signed [15:0] gyro1_x = 16'h0000;
    logic signed [15:0] gyro1_y = 16'h0000;
    logic signed [15:0] gyro1_z = 16'h0000;
    
    // Clock generation
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // DUT
    spi_slave_mcu dut (
        .clk(clk),
        .cs_n(cs_n),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .quat1_valid(quat1_valid),
        .quat1_w(quat1_w),
        .quat1_x(quat1_x),
        .quat1_y(quat1_y),
        .quat1_z(quat1_z),
        .gyro1_valid(gyro1_valid),
        .gyro1_x(gyro1_x),
        .gyro1_y(gyro1_y),
        .gyro1_z(gyro1_z)
    );
    
    // SPI Master simulation (MCU side)
    // SPI Mode 0: CPOL=0, CPHA=0
    // - Clock idle is LOW
    // - Data is sampled on RISING edge
    // - Data should be stable before rising edge (setup on falling edge)
    
    parameter SCK_PERIOD = 50;  // 20MHz SPI clock (faster than FPGA clock for testing)
    
    // Task to read one byte via SPI
    task read_byte_spi(output logic [7:0] data);
        integer i;
        data = 8'h00;
        
        // CS goes low to start transaction
        cs_n = 0;
        #(CLK_PERIOD * 2);  // Wait for FPGA to prepare
        
        // Read 8 bits, MSB first
        for (i = 7; i >= 0; i--) begin
            // Setup phase: clock is low, data should be stable
            sck = 0;
            #(SCK_PERIOD/2);
            
            // Sample on rising edge (Mode 0)
            sck = 1;
            data[i] = sdo;  // Sample MISO
            #(SCK_PERIOD/2);
            
            // Clock goes low for next bit (FPGA will shift on this falling edge)
            sck = 0;
            #(SCK_PERIOD/2);
        end
        
        // CS goes high to end transaction
        cs_n = 1;
        #(CLK_PERIOD * 2);
    endtask
    
    // Task to read 16-byte packet
    task read_packet(output logic [7:0] packet [0:15]);
        integer i;
        for (i = 0; i < 16; i++) begin
            read_byte_spi(packet[i]);
            $display("  Byte[%2d] = 0x%02X", i, packet[i]);
        end
    endtask
    
    // Main test
    initial begin
        $dumpfile("tb_spi_slave_mcu_debug.vcd");
        $dumpvars(0, tb_spi_slave_mcu_debug);
        
        $display("========================================");
        $display("SPI Slave MCU Debug Testbench");
        $display("========================================");
        $display("");
        
        // Initialize
        cs_n = 1;
        sck = 0;
        sdi = 0;
        
        // Set test data
        quat1_w = 16'h1234;
        quat1_x = 16'h5678;
        quat1_y = 16'h9ABC;
        quat1_z = 16'hDEF0;
        gyro1_x = 16'h1111;
        gyro1_y = 16'h2222;
        gyro1_z = 16'h3333;
        quat1_valid = 1;
        gyro1_valid = 1;
        
        #(CLK_PERIOD * 10);
        
        $display("Expected packet:");
        $display("  Byte[0]  = 0xAA (header)");
        $display("  Byte[1]  = 0x12 (quat1_w MSB)");
        $display("  Byte[2]  = 0x34 (quat1_w LSB)");
        $display("  Byte[3]  = 0x56 (quat1_x MSB)");
        $display("  Byte[4]  = 0x78 (quat1_x LSB)");
        $display("  Byte[5]  = 0x9A (quat1_y MSB)");
        $display("  Byte[6]  = 0xBC (quat1_y LSB)");
        $display("  Byte[7]  = 0xDE (quat1_z MSB)");
        $display("  Byte[8]  = 0xF0 (quat1_z LSB)");
        $display("  Byte[9]  = 0x11 (gyro1_x MSB)");
        $display("  Byte[10] = 0x11 (gyro1_x LSB)");
        $display("  Byte[11] = 0x22 (gyro1_y MSB)");
        $display("  Byte[12] = 0x22 (gyro1_y LSB)");
        $display("  Byte[13] = 0x33 (gyro1_z MSB)");
        $display("  Byte[14] = 0x33 (gyro1_z LSB)");
        $display("  Byte[15] = 0x03 (flags: both valid)");
        $display("");
        
        $display("Reading packet via SPI...");
        $display("");
        
        // Read packet
        logic [7:0] received_packet [0:15];
        read_packet(received_packet);
        
        $display("");
        $display("Received packet:");
        integer i;
        for (i = 0; i < 16; i++) begin
            $display("  Byte[%2d] = 0x%02X", i, received_packet[i]);
        end
        
        $display("");
        $display("Verification:");
        
        // Check header
        if (received_packet[0] == 8'hAA) begin
            $display("  [PASS] Header byte = 0xAA");
        end else begin
            $display("  [FAIL] Header byte = 0x%02X (expected 0xAA)", received_packet[0]);
        end
        
        // Check first few bytes
        if (received_packet[1] == 8'h12 && received_packet[2] == 8'h34) begin
            $display("  [PASS] Quat W = 0x1234");
        end else begin
            $display("  [FAIL] Quat W = 0x%02X%02X (expected 0x1234)", 
                     received_packet[1], received_packet[2]);
        end
        
        // Check flags
        if (received_packet[15] == 8'h03) begin
            $display("  [PASS] Flags = 0x03");
        end else begin
            $display("  [FAIL] Flags = 0x%02X (expected 0x03)", received_packet[15]);
        end
        
        $display("");
        $display("========================================");
        
        #(CLK_PERIOD * 10);
        $finish;
    end
    
endmodule

