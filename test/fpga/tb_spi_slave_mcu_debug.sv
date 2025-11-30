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
    
    // Task to read one byte via SPI (CS must already be low, doesn't toggle CS)
    task read_byte_spi;
        output [7:0] data;
        integer i;
        reg [7:0] temp_data;
        begin
            temp_data = 8'h00;
            
            // Read 8 bits, MSB first
            for (i = 7; i >= 0; i = i - 1) begin
                // SCK is already low from previous iteration (or initial state)
                // Wait a bit for setup time
                #(SCK_PERIOD/4);
                
                // Rising edge: MCU samples MISO (Mode 0)
                sck = 1;
                // Wait for signal to propagate and FPGA to see rising edge
                repeat(2) @(posedge clk);
                temp_data[i] = sdo;  // Sample MISO on rising edge
                #(SCK_PERIOD/4);
                
                // Falling edge: FPGA will shift on this edge (Mode 0)
                sck = 0;
                // Wait for FPGA to detect falling edge and shift
                repeat(2) @(posedge clk);
                #(SCK_PERIOD/4);
            end
            
            data = temp_data;
        end
    endtask
    
    // Task to read 16-byte packet (CS stays low for entire transaction)
    task read_packet;
        integer i;
        reg [7:0] temp_byte;
        begin
            // CS goes low to start transaction (only once for all 16 bytes)
            cs_n = 0;
            // Wait a few FPGA clock cycles for shift_out to be loaded
            repeat(3) @(posedge clk);
            
            // Ensure SCK starts low (SPI Mode 0: CPOL=0)
            sck = 0;
            repeat(2) @(posedge clk);  // Give FPGA time to see CS low and prepare first bit
            
            // Read all 16 bytes with CS staying low
            for (i = 0; i < 16; i = i + 1) begin
                read_byte_spi(temp_byte);
                received_packet[i] = temp_byte;
                $display("  Byte[%2d] = 0x%02X", i, received_packet[i]);
            end
            
            // CS goes high to end transaction (only after all bytes are read)
            cs_n = 1;
            repeat(2) @(posedge clk);
        end
    endtask
    
    // Packet storage
    reg [7:0] received_packet [0:15];
    integer i;
    
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
        
        // Debug: Check internal signals
        $display("Debug: Checking internal signals...");
        $display("  dut.packet_buffer[0] = 0x%02X", dut.packet_buffer[0]);
        $display("  dut.tx_packet[127:120] = 0x%02X", dut.tx_packet[127:120]);
        $display("  dut.shift_out = 0x%02X", dut.shift_out);
        $display("  dut.sdo = %b (cs_n=%b)", dut.sdo, cs_n);
        $display("");
        
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
        read_packet;
        
        $display("");
        $display("Received packet:");
        for (i = 0; i < 16; i = i + 1) begin
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

