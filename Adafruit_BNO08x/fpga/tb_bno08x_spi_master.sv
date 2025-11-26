/**
 * Testbench for BNO08X SPI Master Module
 * 
 * Tests SPI Mode 3 communication with proper timing
 */

`timescale 1ns / 1ps

module tb_bno08x_spi_master;

    // Parameters
    parameter CLK_PERIOD = 333; // 3MHz = 333ns period
    
    // Signals
    logic        clk;
    logic        rst_n;
    logic        spi_cs_n;
    logic        spi_sck;
    logic        spi_mosi;
    logic        spi_miso;
    logic        start_transfer;
    logic        write_en;
    logic [15:0] data_length;
    logic [7:0]  tx_data;
    logic [7:0]  rx_data;
    logic        data_valid;
    logic        transfer_done;
    logic        transfer_busy;
    
    // Testbench signals
    logic [7:0]  tx_buffer [0:15];
    logic [7:0]  rx_buffer [0:15];
    int          tx_index;
    int          rx_index;
    
    // Instantiate DUT
    bno08x_spi_master dut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .start_transfer(start_transfer),
        .write_en(write_en),
        .data_length(data_length),
        .tx_data(tx_data),
        .rx_data(rx_data),
        .data_valid(data_valid),
        .transfer_done(transfer_done),
        .transfer_busy(transfer_busy)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // MISO simulation (simulating BNO08X response)
    always_ff @(negedge spi_sck or negedge rst_n) begin
        if (!rst_n) begin
            spi_miso <= 1'b0;
        end else if (!spi_cs_n) begin
            // Simulate BNO08X sending data
            // For test: send incrementing pattern
            if (rx_index < 16) begin
                spi_miso <= rx_buffer[rx_index][7 - (rx_index % 8)];
            end
        end
    end
    
    // Test tasks
    task reset_dut();
        rst_n = 0;
        start_transfer = 0;
        write_en = 0;
        data_length = 0;
        tx_data = 0;
        tx_index = 0;
        rx_index = 0;
        #(10 * CLK_PERIOD);
        rst_n = 1;
        #(5 * CLK_PERIOD);
    endtask
    
    task spi_write(input int length, input logic [7:0] data []);
        int i;
        $display("[%0t] Starting SPI Write: %0d bytes", $time, length);
        
        // Prepare TX buffer
        for (i = 0; i < length && i < 16; i++) begin
            tx_buffer[i] = data[i];
        end
        
        tx_index = 0;
        data_length = length[15:0]; // Truncate to 16 bits
        write_en = 1;
        tx_data = tx_buffer[0];
        
        @(posedge clk);
        start_transfer = 1;
        @(posedge clk);
        start_transfer = 0;
        
        // Wait for transfer to complete
        wait(transfer_done);
        #(5 * CLK_PERIOD);
        
        $display("[%0t] SPI Write Complete", $time);
    endtask
    
    task spi_read(input int length);
        int i;
        $display("[%0t] Starting SPI Read: %0d bytes", $time, length);
        
        // Prepare expected RX buffer (simulate BNO08X response)
        for (i = 0; i < length && i < 16; i++) begin
            rx_buffer[i] = 8'hA0 + i[7:0]; // Simulated response, cast to 8 bits
        end
        
        rx_index = 0;
        data_length = length[15:0]; // Truncate to 16 bits
        write_en = 0;
        tx_data = 0;
        
        @(posedge clk);
        start_transfer = 1;
        @(posedge clk);
        start_transfer = 0;
        
        // Wait for transfer to complete
        wait(transfer_done);
        #(5 * CLK_PERIOD);
        
        $display("[%0t] SPI Read Complete", $time);
    endtask
    
    // Main test sequence
    initial begin
        $display("=========================================");
        $display("BNO08X SPI Master Testbench");
        $display("=========================================");
        
        // Initialize
        reset_dut();
        
        // Test 1: Single byte write
        $display("\n--- Test 1: Single Byte Write ---");
        begin
            logic [7:0] test_data [1];
            test_data[0] = 8'hF9; // Product ID Request
            spi_write(1, test_data);
        end
        #(10 * CLK_PERIOD);
        
        // Test 2: Multi-byte write (SHTP header)
        $display("\n--- Test 2: Multi-Byte Write (SHTP Header) ---");
        begin
            logic [7:0] test_data [4];
            test_data[0] = 8'h05; // Length LSB
            test_data[1] = 8'h00; // Length MSB
            test_data[2] = 8'h02; // Channel (Control)
            test_data[3] = 8'h01; // Sequence
            spi_write(4, test_data);
        end
        #(10 * CLK_PERIOD);
        
        // Test 3: Single byte read
        $display("\n--- Test 3: Single Byte Read ---");
        spi_read(1);
        #(10 * CLK_PERIOD);
        
        // Test 4: Multi-byte read (SHTP response)
        $display("\n--- Test 4: Multi-Byte Read (SHTP Response) ---");
        spi_read(16);
        #(10 * CLK_PERIOD);
        
        // Test 5: Back-to-back transfers
        $display("\n--- Test 5: Back-to-Back Transfers ---");
        begin
            logic [7:0] test_data1 [4];
            test_data1[0] = 8'h05;
            test_data1[1] = 8'h00;
            test_data1[2] = 8'h02;
            test_data1[3] = 8'h02;
            spi_write(4, test_data1);
        end
        #(2 * CLK_PERIOD);
        spi_read(4);
        #(10 * CLK_PERIOD);
        
        $display("\n=========================================");
        $display("All Tests Complete");
        $display("=========================================");
        #(100 * CLK_PERIOD);
        $finish;
    end
    
    // Monitor SPI signals
    initial begin
        $monitor("[%0t] CS=%b SCK=%b MOSI=%b MISO=%b BUSY=%b DONE=%b", 
                 $time, spi_cs_n, spi_sck, spi_mosi, spi_miso, 
                 transfer_busy, transfer_done);
    end

endmodule

