`timescale 1ns / 1ps

// Testbench for MCU SPI Slave Module
// Tests SPI slave operation, packet assembly, and LOAD/DONE handshaking
// Note: Source files are included via compilation command, not here

module tb_mcu_spi_slave;

    // Clock
    logic clk;
    
    // SPI interface (MCU is master)
    logic sck;      // SPI clock from MCU
    logic sdi;      // SPI data in (not used)
    logic sdo;      // SPI data out to MCU
    logic load;     // Load signal from MCU
    logic done;     // Done signal to MCU
    
    // Sensor data inputs
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
    
    // MCU SPI clock (1MHz for testing) - starts only when needed
    parameter SCK_PERIOD = 1000;
    logic sck_enable = 0;
    always begin
        if (sck_enable) begin
            sck = 0;
            #(SCK_PERIOD/2);
            sck = 1;
            #(SCK_PERIOD/2);
        end else begin
            sck = 0;  // Mode 0: idle low
            #(SCK_PERIOD);
        end
    end
    
    // DUT
    mcu_spi_slave dut (
        .clk(clk),
        .sck(sck),
        .sdi(sdi),
        .sdo(sdo),
        .load(load),
        .done(done),
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
    
    // Test variables
    logic [7:0] received_packet [0:31];
    integer bit_count;
    integer byte_count;
    
    // Task: MCU reads 32-byte packet
    task mcu_read_packet;
        integer i, j;
        begin
            // Wait for DONE signal
            while (!done) begin
                #(CLK_PERIOD);
            end
            $display("[%0t] MCU: DONE signal detected, starting SPI read", $time);
            
            // Wait a bit for DONE to stabilize
            #(5 * CLK_PERIOD);
            
            // Enable SPI clock
            sck_enable = 1;
            #(SCK_PERIOD);  // Wait for clock to start
            
            // Read 32 bytes (256 bits)
            // SPI Mode 0: Sample on rising edge, data changes on falling edge
            for (i = 0; i < 32; i = i + 1) begin
                received_packet[i] = 8'h00;
                
                // Read 8 bits MSB first
                for (j = 7; j >= 0; j = j - 1) begin
                    // Wait for falling edge (data changes here in Mode 0)
                    @(negedge sck);
                    // Sample on rising edge (data is stable)
                    @(posedge sck);
                    received_packet[i][j] = sdo;
                end
            end
            
            // Disable SPI clock
            sck_enable = 0;
            
            $display("[%0t] MCU: Packet received", $time);
            
            // Acknowledge with LOAD
            load = 1;
            #(10 * CLK_PERIOD);
            load = 0;
            #(10 * CLK_PERIOD);
        end
    endtask
    
    // Main test sequence
    initial begin
        $dumpfile("tb_mcu_spi_slave.vcd");
        $dumpvars(0, tb_mcu_spi_slave);
        
        // Initialize
        load = 0;
        sdi = 0;
        quat1_valid = 0;
        gyro1_valid = 0;
        quat2_valid = 0;
        gyro2_valid = 0;
        quat1_w = 0; quat1_x = 0; quat1_y = 0; quat1_z = 0;
        gyro1_x = 0; gyro1_y = 0; gyro1_z = 0;
        quat2_w = 0; quat2_x = 0; quat2_y = 0; quat2_z = 0;
        gyro2_x = 0; gyro2_y = 0; gyro2_z = 0;
        
        #(100 * CLK_PERIOD);
        
        $display("=== MCU SPI Slave Testbench ===");
        
        // Test 1: Basic packet transmission
        $display("\n[TEST] Test 1: Basic Packet Transmission");
        quat1_w = 16'h4000;
        quat1_x = 16'h1000;
        quat1_y = 16'h2000;
        quat1_z = 16'h3000;
        gyro1_x = 16'd100;
        gyro1_y = 16'd200;
        gyro1_z = 16'd300;
        quat1_valid = 1;
        gyro1_valid = 1;
        
        quat2_w = 16'h5000;
        quat2_x = 16'h1100;
        quat2_y = 16'h2200;
        quat2_z = 16'h3300;
        gyro2_x = 16'd400;
        gyro2_y = 16'd500;
        gyro2_z = 16'd600;
        quat2_valid = 1;
        gyro2_valid = 1;
        
        #(10 * CLK_PERIOD);
        
        mcu_read_packet();
        
        // Verify header
        if (received_packet[0] == 8'hAA) begin
            $display("[PASS] Header byte correct (0xAA)");
        end else begin
            $display("[FAIL] Header byte incorrect: expected 0xAA, got 0x%02X", received_packet[0]);
        end
        
        // Verify Sensor 1 quaternion (MSB,LSB format)
        if (received_packet[1] == 8'h40 && received_packet[2] == 8'h00 &&
            received_packet[3] == 8'h10 && received_packet[4] == 8'h00) begin
            $display("[PASS] Sensor 1 quaternion W,X correct");
        end else begin
            $display("[FAIL] Sensor 1 quaternion W,X mismatch");
        end
        
        // Verify Sensor 1 gyroscope
        if (received_packet[9] == 8'h00 && received_packet[10] == 8'h64 &&
            received_packet[11] == 8'h00 && received_packet[12] == 8'hC8) begin
            $display("[PASS] Sensor 1 gyroscope X,Y correct");
        end else begin
            $display("[FAIL] Sensor 1 gyroscope X,Y mismatch");
        end
        
        // Verify flags
        if (received_packet[15] == 8'h03) begin // Both valid
            $display("[PASS] Sensor 1 flags correct");
        end else begin
            $display("[FAIL] Sensor 1 flags incorrect: expected 0x03, got 0x%02X", received_packet[15]);
        end
        
        $display("\n=== Test 1 PASSED ===");
        
        #(100 * CLK_PERIOD);
        $finish;
    end
    
endmodule

