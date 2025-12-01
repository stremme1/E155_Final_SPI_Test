`timescale 1ns / 1ps

// Testbench for simplified BNO085 controller
// Tests initialization and data reading

module tb_bno085_controller_simple;

    // Clock and reset
    logic clk = 0;
    logic rst_n = 0;
    
    // SPI interface
    logic spi_start, spi_tx_valid, spi_tx_ready, spi_rx_valid, spi_busy;
    logic [7:0] spi_tx_data, spi_rx_data;
    logic cs_n, ps0_wake, int_n;
    
    // Sensor outputs
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic initialized, error;
    
    // Mock SPI master
    logic sclk, mosi, miso;
    
    // Instantiate DUT
    bno085_controller_simple dut (
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
    
    // Mock SPI master - simplified
    logic [7:0] spi_tx_shift, spi_rx_shift;
    logic [3:0] spi_bit_cnt;
    logic spi_sclk;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_tx_ready <= 1'b1;
            spi_rx_valid <= 1'b0;
            spi_busy <= 1'b0;
            spi_rx_data <= 8'h00;
            spi_tx_shift <= 8'h00;
            spi_rx_shift <= 8'h00;
            spi_bit_cnt <= 4'd0;
            spi_sclk <= 1'b1;
        end else begin
            spi_rx_valid <= 1'b0;
            
            if (spi_start && spi_tx_valid && !spi_busy) begin
                spi_busy <= 1'b1;
                spi_tx_ready <= 1'b0;
                spi_tx_shift <= spi_tx_data;
                spi_rx_shift <= 8'h00;
                spi_bit_cnt <= 4'd0;
                spi_sclk <= 1'b1;
            end else if (spi_busy) begin
                // Simulate SPI clock
                if (spi_bit_cnt < 8) begin
                    spi_sclk <= ~spi_sclk;
                    if (spi_sclk && spi_bit_cnt < 8) begin
                        // Rising edge - sample MISO (simulate response)
                        spi_rx_shift <= {spi_rx_shift[6:0], miso};
                        spi_bit_cnt <= spi_bit_cnt + 1;
                    end
                end else begin
                    // Byte complete
                    spi_rx_data <= spi_rx_shift;
                    spi_rx_valid <= 1'b1;
                    spi_busy <= 1'b0;
                    spi_tx_ready <= 1'b1;
                end
            end
        end
    end
    
    assign sclk = (spi_busy) ? spi_sclk : 1'b1;
    assign mosi = (spi_busy) ? spi_tx_shift[7] : 1'b0;
    
    // Mock BNO085 sensor
    logic [7:0] mock_response [0:255];
    logic [7:0] mock_ptr;
    logic sending_response;
    
    // Initialize mock responses
    initial begin
        // Product ID Response (Channel 2)
        mock_response[0] = 8'h10; // Length LSB
        mock_response[1] = 8'h00; // Length MSB
        mock_response[2] = 8'h02; // Channel
        mock_response[3] = 8'h00; // Seq
        mock_response[4] = 8'hF8; // Report ID
        // ... rest of response
        
        // Set Feature Response (Channel 2)
        mock_response[20] = 8'h0D; // Length LSB
        mock_response[21] = 8'h00; // Length MSB
        mock_response[22] = 8'h02; // Channel
        mock_response[23] = 8'h01; // Seq
        mock_response[24] = 8'hFC; // Report ID (Get Feature Response)
        // ... rest of response
        
        // Rotation Vector Report (Channel 3)
        mock_response[40] = 8'h13; // Length LSB (19 bytes)
        mock_response[41] = 8'h00; // Length MSB
        mock_response[42] = 8'h03; // Channel
        mock_response[43] = 8'h00; // Seq
        mock_response[44] = 8'h05; // Report ID (Rotation Vector)
        mock_response[45] = 8'h01; // Sequence
        mock_response[46] = 8'h03; // Status (high accuracy)
        mock_response[47] = 8'h00; // Delay
        mock_response[48] = 8'hE8; // X LSB (1000 = 0x03E8)
        mock_response[49] = 8'h03; // X MSB
        mock_response[50] = 8'hD0; // Y LSB (2000 = 0x07D0)
        mock_response[51] = 8'h07; // Y MSB
        mock_response[52] = 8'hB8; // Z LSB (3000 = 0x0BB8)
        mock_response[53] = 8'h0B; // Z MSB
        mock_response[54] = 8'hA0; // W LSB (4000 = 0x0FA0)
        mock_response[55] = 8'h0F; // W MSB
        
        // Gyroscope Report (Channel 3)
        mock_response[60] = 8'h0F; // Length LSB (15 bytes)
        mock_response[61] = 8'h00; // Length MSB
        mock_response[62] = 8'h03; // Channel
        mock_response[63] = 8'h00; // Seq
        mock_response[64] = 8'h02; // Report ID (Gyroscope)
        mock_response[65] = 8'h01; // Sequence
        mock_response[66] = 8'h03; // Status
        mock_response[67] = 8'h00; // Delay
        mock_response[68] = 8'h88; // X LSB (500 = 0x01F4)
        mock_response[69] = 8'h01; // X MSB
        mock_response[70] = 8'h2C; // Y LSB (600 = 0x0258)
        mock_response[71] = 8'h02; // Y MSB
        mock_response[72] = 8'hD0; // Z LSB (700 = 0x02BC)
        mock_response[73] = 8'h02; // Z MSB
    end
    
    // Mock MISO - return response data
    always_ff @(posedge clk) begin
        if (cs_n) begin
            mock_ptr <= 8'd0;
            sending_response <= 1'b0;
        end else if (spi_rx_valid && !spi_busy) begin
            // Byte read - advance pointer
            mock_ptr <= mock_ptr + 1;
            sending_response <= 1'b1;
        end
    end
    
    assign miso = (cs_n) ? 1'b0 : mock_response[mock_ptr][7 - spi_bit_cnt];
    
    // Clock generation
    always #166.67 clk = ~clk; // 3MHz
    
    // Test sequence
    initial begin
        $display("=== BNO085 Controller Simple Testbench ===");
        
        // Reset
        rst_n = 0;
        int_n = 1'b1;
        #1000;
        rst_n = 1;
        #1000;
        
        $display("[%0t] Reset released", $time);
        
        // Wait for initialization
        wait(initialized);
        $display("[%0t] Controller initialized", $time);
        
        // Simulate sensor sending rotation vector
        #10000;
        int_n = 1'b0;
        $display("[%0t] INT asserted - rotation vector ready", $time);
        #5000;
        int_n = 1'b1;
        
        // Wait for data
        #50000;
        if (quat_valid) begin
            $display("[%0t] PASS: Quaternion received: w=%d x=%d y=%d z=%d", 
                     $time, quat_w, quat_x, quat_y, quat_z);
        end else begin
            $display("[%0t] FAIL: Quaternion not received", $time);
        end
        
        // Simulate sensor sending gyroscope
        #10000;
        int_n = 1'b0;
        $display("[%0t] INT asserted - gyroscope ready", $time);
        #5000;
        int_n = 1'b1;
        
        // Wait for data
        #50000;
        if (gyro_valid) begin
            $display("[%0t] PASS: Gyroscope received: x=%d y=%d z=%d", 
                     $time, gyro_x, gyro_y, gyro_z);
        end else begin
            $display("[%0t] FAIL: Gyroscope not received", $time);
        end
        
        #100000;
        $display("=== Test Complete ===");
        $finish;
    end

endmodule

