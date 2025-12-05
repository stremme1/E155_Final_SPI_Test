`timescale 1ns / 1ps

// Testbench for Arduino SPI Slave Module with Arduino Model
// Uses exact Arduino SPI master model to test FPGA slave
// This simulates the real hardware behavior

module tb_arduino_spi_slave_with_arduino_model;

    // Clock
    logic clk;
    
    // Arduino SPI interface (Arduino is master, FPGA is slave)
    logic cs_n;      // Chip select from Arduino (active low)
    logic sck;       // SPI clock from Arduino
    logic sdi;       // SPI data in (MOSI from Arduino)
    
    // DUT outputs
    logic initialized;
    logic error;
    logic quat1_valid;
    logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z;
    logic gyro1_valid;
    logic signed [15:0] gyro1_x, gyro1_y, gyro1_z;
    
    // Arduino model - only used to build packet, not drive SPI
    logic signed [15:0] arduino_roll, arduino_pitch, arduino_yaw;
    logic signed [15:0] arduino_gyro_x, arduino_gyro_y, arduino_gyro_z;
    logic arduino_euler_valid, arduino_gyro_valid;
    wire [7:0] packet_byte;  // Get packet byte from model
    
    // Clock generation (3MHz FPGA clock)
    parameter CLK_PERIOD = 333;  // 3MHz = 333ns period
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // Arduino model not needed - we drive SPI directly using Arduino packet format
    
    // DUT
    arduino_spi_slave dut (
        .clk(clk),
        .cs_n(cs_n),
        .sck(sck),
        .sdi(sdi),
        .initialized(initialized),
        .error(error),
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
    
    // Access internal signals for debugging
    wire [7:0] packet_buffer_0 = dut.packet_buffer[0];
    wire [7:0] packet_buffer_1 = dut.packet_buffer[1];
    wire [7:0] packet_buffer_2 = dut.packet_buffer[2];
    wire [7:0] header = dut.header;
    wire signed [15:0] roll = dut.roll;
    wire signed [15:0] pitch = dut.pitch;
    wire signed [15:0] yaw = dut.yaw;
    wire cs_high_stable = dut.cs_high_stable;
    wire packet_valid_raw = dut.packet_valid_raw;
    wire packet_valid = dut.packet_valid;
    wire new_packet_available = dut.new_packet_available;
    wire [7:0] packet_snapshot_0 = dut.packet_snapshot[0];
    
    // Test tracking
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    
    // Helper task to check and report
    task check_test(input string test_name, input logic condition);
        test_count = test_count + 1;
        if (condition) begin
            $display("[PASS] %s", test_name);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] %s", test_name);
            fail_count = fail_count + 1;
        end
    endtask
    
    // Helper task to wait for CDC and packet processing
    task wait_cdc;
        wait(cs_n == 1'b1);  // Wait for CS to go high
        repeat(50) @(posedge clk);  // Wait for CDC and processing
    endtask
    
    // SPI parameters
    parameter SCK_PERIOD = 10000;  // 100kHz = 10us period
    
    // Task to send a byte via SPI (MSB first, Mode 0) - EXACT Arduino SPI.transfer() behavior
    // CRITICAL: First bit (MSB) must be set up BEFORE first SCK rising edge
    // This matches Arduino SPI.transfer() exactly:
    // - Arduino sets up first bit before first SCK edge
    // - Each bit is sampled on SCK rising edge
    // - Next bit is set up during SCK low time
    task send_spi_byte(input [7:0] data);
        integer i;
        // CRITICAL: Set up first bit (MSB, bit 7) BEFORE first SCK rising edge
        // Arduino SPI.transfer() does this - first bit is ready before first edge
        sdi = data[7];
        #(SCK_PERIOD / 2);  // Setup time - data stable before first edge
        
        // Send 8 bits: Each bit is sampled on rising edge, next bit set up during low time
        for (i = 7; i >= 0; i = i - 1) begin
            // Rising edge - FPGA slave samples sdi here (SPI Mode 0)
            sck = 1'b1;
            #(SCK_PERIOD / 2);
            
            // Falling edge - prepare for next bit
            sck = 1'b0;
            
            // Set up next bit (if not last bit) before next rising edge
            if (i > 0) begin
                sdi = data[i-1];  // Next bit (MSB first: 7,6,5,4,3,2,1,0)
                #(SCK_PERIOD / 2);  // Setup time for next bit
            end else begin
                // Last bit sent - keep data stable
                #(SCK_PERIOD / 2);
            end
        end
    endtask
    
    // Task to send packet with sensor data - builds packet from Arduino format and sends
    task send_sensor_packet(
        input signed [15:0] r, p, y,
        input signed [15:0] gx, gy, gz,
        input euler_v, gyro_v
    );
        // Set sensor data (for reference, but we build packet directly)
        arduino_roll = r;
        arduino_pitch = p;
        arduino_yaw = y;
        arduino_gyro_x = gx;
        arduino_gyro_y = gy;
        arduino_gyro_z = gz;
        arduino_euler_valid = euler_v;
        arduino_gyro_valid = gyro_v;
        
        // Build packet bytes (same format as Arduino code) - use send_packet_inline approach
        // CS falling edge - ensure CS is high first, then go low
        cs_n = 1;
        #(SCK_PERIOD * 3);
        cs_n = 0;
        #(SCK_PERIOD * 2);
        
        // Send all 16 bytes directly (same format as Arduino)
        send_spi_byte(8'hAA);  // Header
        send_spi_byte(r[15:8]);   // Roll MSB
        send_spi_byte(r[7:0]);    // Roll LSB
        send_spi_byte(p[15:8]);   // Pitch MSB
        send_spi_byte(p[7:0]);    // Pitch LSB
        send_spi_byte(y[15:8]);   // Yaw MSB
        send_spi_byte(y[7:0]);    // Yaw LSB
        send_spi_byte(gx[15:8]);  // Gyro X MSB
        send_spi_byte(gx[7:0]);   // Gyro X LSB
        send_spi_byte(gy[15:8]);  // Gyro Y MSB
        send_spi_byte(gy[7:0]);   // Gyro Y LSB
        send_spi_byte(gz[15:8]);  // Gyro Z MSB
        send_spi_byte(gz[7:0]);   // Gyro Z LSB
        send_spi_byte({6'h0, gyro_v, euler_v});  // Flags
        send_spi_byte(8'h00);    // Reserved
        send_spi_byte(8'h00);    // Reserved
        
        // CS rising edge (transaction complete)
        #(SCK_PERIOD);
        cs_n = 1;
        #(SCK_PERIOD * 3);
    endtask
    
    // Main test sequence
    initial begin
        $dumpfile("tb_arduino_spi_slave_with_arduino_model.vcd");
        $dumpvars(0, tb_arduino_spi_slave_with_arduino_model);
        
        $display("========================================");
        $display("Arduino SPI Slave Testbench with Arduino Model");
        $display("========================================");
        $display("");
        
        // Initialize
        arduino_roll = 0;
        arduino_pitch = 0;
        arduino_yaw = 0;
        arduino_gyro_x = 0;
        arduino_gyro_y = 0;
        arduino_gyro_z = 0;
        arduino_euler_valid = 1'b0;
        arduino_gyro_valid = 1'b0;
        
        // Wait for initial setup
        #(CLK_PERIOD * 10);
        
        // ========================================
        // TEST 1: Valid Packet with Realistic Data
        // ========================================
        $display("=== Test 1: Valid Packet with Realistic Sensor Data ===");
        
        // Test 1.1: Send packet with Roll=10.00°, Pitch=-5.00°, Yaw=20.00°
        // Scaled: Roll=1000, Pitch=-500, Yaw=2000
        // Gyro: X=0.05 rad/s, Y=-0.10 rad/s, Z=0.025 rad/s
        // Scaled: Gyro X=100, Y=-200, Z=50
        begin
            $display("Test 1.1: Sending packet with realistic sensor data");
            send_sensor_packet(
                16'd1000,   // Roll = 10.00° (scaled by 100)
                -16'd500,   // Pitch = -5.00°
                16'd2000,   // Yaw = 20.00°
                16'd100,    // Gyro X = 0.05 rad/s (scaled by 2000)
                -16'd200,   // Gyro Y = -0.10 rad/s
                16'd50,     // Gyro Z = 0.025 rad/s
                1'b1,       // Euler valid
                1'b1        // Gyro valid
            );
            
            wait_cdc();
            
            // Debug output
            $display("    Internal signals after packet reception:");
            $display("      packet_buffer[0] = 0x%02X (header, expected 0xAA)", packet_buffer_0);
            $display("      packet_buffer[1] = 0x%02X (Roll MSB, expected 0x03)", packet_buffer_1);
            $display("      packet_buffer[2] = 0x%02X (Roll LSB, expected 0xE8)", packet_buffer_2);
            $display("      CDC: cs_high_stable=%b, packet_valid=%b, new_packet_available=%b", 
                     cs_high_stable, packet_valid, new_packet_available);
            $display("      packet_snapshot[0] = 0x%02X", packet_snapshot_0);
            $display("      header = 0x%02X, roll = 0x%04X (%0d)", header, roll, roll);
            $display("      pitch = 0x%04X (%0d), yaw = 0x%04X (%0d)", pitch, pitch, yaw, yaw);
            
            // Check outputs
            $display("    Output signals after packet reception:");
            $display("      initialized = %b, error = %b", initialized, error);
            $display("      quat1_w = 0x%04X (%0d), quat1_x = 0x%04X (%0d)", quat1_w, quat1_w, quat1_x, quat1_x);
            $display("      quat1_y = 0x%04X (%0d), quat1_z = 0x%04X (%0d)", quat1_y, quat1_y, quat1_z, quat1_z);
            $display("      gyro1_x = 0x%04X (%0d), gyro1_y = 0x%04X (%0d), gyro1_z = 0x%04X (%0d)", 
                     gyro1_x, gyro1_x, gyro1_y, gyro1_y, gyro1_z, gyro1_z);
            $display("      quat1_valid = %b, gyro1_valid = %b", quat1_valid, gyro1_valid);
            
            // Verify results
            check_test("Test 1.1a: Header valid → initialized = 1", initialized == 1'b1);
            check_test("Test 1.1b: Header valid → error = 0", error == 1'b0);
            check_test("Test 1.1c: Quat W = 16384 (Q14 format)", quat1_w == 16'd16384);
            check_test("Test 1.1d: Quat X = Roll (1000)", quat1_x == 16'd1000);
            check_test("Test 1.1e: Quat Y = Pitch (-500)", quat1_y == -16'd500);
            check_test("Test 1.1f: Quat Z = Yaw (2000)", quat1_z == 16'd2000);
            check_test("Test 1.1g: Gyro X = 100", gyro1_x == 16'd100);
            check_test("Test 1.1h: Gyro Y = -200", gyro1_y == -16'd200);
            check_test("Test 1.1i: Gyro Z = 50", gyro1_z == 16'd50);
            check_test("Test 1.1j: Quat valid = 1 (from flags[0])", quat1_valid == 1'b1);
            check_test("Test 1.1k: Gyro valid = 1 (from flags[1])", gyro1_valid == 1'b1);
        end
        
        $display("");
        
        // ========================================
        // TEST 2: Multiple Packets
        // ========================================
        $display("=== Test 2: Multiple Packets ===");
        
        // Test 2.1: Send second packet with different data
        begin
            $display("Test 2.1: Sending second packet with different data");
            send_sensor_packet(
                16'd500,    // Roll = 5.00°
                -16'd250,   // Pitch = -2.50°
                16'd1000,   // Yaw = 10.00°
                16'd50,     // Gyro X = 0.025 rad/s
                -16'd100,   // Gyro Y = -0.05 rad/s
                16'd25,     // Gyro Z = 0.0125 rad/s
                1'b1,       // Euler valid
                1'b1        // Gyro valid
            );
            
            wait_cdc();
            
            check_test("Test 2.1a: Second packet Roll = 500", quat1_x == 16'd500);
            check_test("Test 2.1b: Second packet Pitch = -250", quat1_y == -16'd250);
            check_test("Test 2.1c: Second packet Yaw = 1000", quat1_z == 16'd1000);
        end
        
        $display("");
        
        // ========================================
        // TEST 3: Edge Cases
        // ========================================
        $display("=== Test 3: Edge Cases ===");
        
        // Test 3.1: Maximum values
        begin
            $display("Test 3.1: Maximum values");
            send_sensor_packet(
                16'd32767,  // Roll = 327.67° (max)
                16'd32767,  // Pitch = 327.67°
                16'd32767,  // Yaw = 327.67°
                16'd32767,  // Gyro X = 16.3835 rad/s (max)
                16'd32767,  // Gyro Y = 16.3835 rad/s
                16'd32767,  // Gyro Z = 16.3835 rad/s
                1'b1,       // Euler valid
                1'b1        // Gyro valid
            );
            
            wait_cdc();
            
            check_test("Test 3.1a: Max Roll", quat1_x == 16'd32767);
            check_test("Test 3.1b: Max Pitch", quat1_y == 16'd32767);
            check_test("Test 3.1c: Max Yaw", quat1_z == 16'd32767);
        end
        
        // Test 3.2: Minimum values
        begin
            $display("Test 3.2: Minimum values");
            send_sensor_packet(
                -16'd32768, // Roll = -327.68° (min)
                -16'd32768, // Pitch = -327.68°
                -16'd32768, // Yaw = -327.68°
                -16'd32768, // Gyro X = -16.384 rad/s (min)
                -16'd32768, // Gyro Y = -16.384 rad/s
                -16'd32768, // Gyro Z = -16.384 rad/s
                1'b1,       // Euler valid
                1'b1        // Gyro valid
            );
            
            wait_cdc();
            
            check_test("Test 3.2a: Min Roll", quat1_x == -16'd32768);
            check_test("Test 3.2b: Min Pitch", quat1_y == -16'd32768);
            check_test("Test 3.2c: Min Yaw", quat1_z == -16'd32768);
        end
        
        // Test 3.3: Invalid header (should set error)
        begin
            $display("Test 3.3: Invalid header - should set error");
            // Send packet with invalid header (0x55 instead of 0xAA)
            cs_n = 1;
            #(SCK_PERIOD * 3);
            cs_n = 0;
            #(SCK_PERIOD * 2);
            
            send_spi_byte(8'h55);  // Invalid header
            send_spi_byte(8'h00);  // Rest zeros
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            send_spi_byte(8'h00);
            
            #(SCK_PERIOD);
            cs_n = 1;
            #(SCK_PERIOD * 3);
            
            wait_cdc();
            
            check_test("Test 3.3a: Invalid header → error = 1", error == 1'b1);
            check_test("Test 3.3b: Invalid header → initialized = 0", initialized == 1'b0);
        end
        
        // Test 3.4: Valid header after invalid (should clear error)
        begin
            $display("Test 3.4: Valid header after invalid - should clear error");
            send_sensor_packet(
                16'd100,    // Roll = 1.00°
                16'd200,    // Pitch = 2.00°
                16'd300,    // Yaw = 3.00°
                16'd10,     // Gyro X
                16'd20,     // Gyro Y
                16'd30,     // Gyro Z
                1'b1,       // Euler valid
                1'b1        // Gyro valid
            );
            
            wait_cdc();
            
            check_test("Test 3.4a: Valid header → error = 0", error == 1'b0);
            check_test("Test 3.4b: Valid header → initialized = 1", initialized == 1'b1);
        end
        
        $display("");
        
        // ========================================
        // Test Summary
        // ========================================
        $display("========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Total Tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        $display("========================================");
        if (fail_count > 0) begin
            $display("SOME TESTS FAILED");
        end else begin
            $display("ALL TESTS PASSED");
        end
        $display("========================================");
        
        #(CLK_PERIOD * 100);
        $finish;
    end

endmodule

