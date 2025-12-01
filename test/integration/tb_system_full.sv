`timescale 1ns / 1ps

// Full System Testbench
// Tests complete flow: BNO085 -> FPGA Controller -> SPI Slave -> MCU
// Verifies end-to-end data path

module tb_system_full;

    // System clock (3MHz) - not used by drum_trigger_top (it has internal clock)
    // But needed for mock sensor
    logic clk;
    logic fpga_rst_n;
    parameter CLK_PERIOD = 333;
    always begin
        clk = 0;
        #(CLK_PERIOD/2);
        clk = 1;
        #(CLK_PERIOD/2);
    end
    
    // MCU SPI Interface
    logic mcu_sck, mcu_sdi, mcu_sdo, mcu_cs_n;
    
    // BNO085 Sensor Interface
    logic sclk, mosi, miso1, cs_n1, ps0_wake1, int1;
    logic bno085_rst_n;
    
    // Status LEDs
    logic led_initialized, led_error, led_heartbeat;
    
    // Top-level DUT
    drum_trigger_top dut (
        .fpga_rst_n(fpga_rst_n),
        .mcu_sck(mcu_sck),
        .mcu_sdi(mcu_sdi),
        .mcu_sdo(mcu_sdo),
        .mcu_cs_n(mcu_cs_n),
        .sclk(sclk),
        .mosi(mosi),
        .miso1(miso1),
        .cs_n1(cs_n1),
        .ps0_wake1(ps0_wake1),
        .int1(int1),
        .bno085_rst_n(bno085_rst_n),
        .led_initialized(led_initialized),
        .led_error(led_error),
        .led_heartbeat(led_heartbeat)
    );
    
    // Mock BNO085 Sensor
    mock_bno085 sensor (
        .clk(clk),
        .rst_n(bno085_rst_n),
        .ps0_wake(ps0_wake1),
        .cs_n(cs_n1),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso1),
        .int_n(int1)
    );
    
    // Mock MCU SPI Master
    logic [7:0] mcu_rx_byte;
    logic [7:0] mcu_tx_byte;
    logic [3:0] mcu_bit_cnt;
    logic [7:0] mcu_packet [0:15];
    integer mcu_byte_idx;
    logic mcu_transaction_active;
    
    // MCU SPI Mode 0: CPOL=0, CPHA=0
    // Clock idles low, sample on rising edge, change on falling edge
    logic mcu_sck_reg;
    logic [7:0] mcu_sck_div;
    
    // MCU clock generation (much slower than system clock for SPI)
    always_ff @(posedge clk) begin
        if (!mcu_cs_n && mcu_transaction_active) begin
            mcu_sck_div <= mcu_sck_div + 1;
            if (mcu_sck_div == 8'd10) begin
                mcu_sck_reg <= ~mcu_sck_reg;
                mcu_sck_div <= 8'd0;
            end
        end else begin
            mcu_sck_reg <= 1'b0;
            mcu_sck_div <= 8'd0;
        end
    end
    assign mcu_sck = mcu_sck_reg;
    
    // MCU transaction control
    always_ff @(posedge clk) begin
        if (!mcu_cs_n && !mcu_transaction_active) begin
            mcu_transaction_active <= 1'b1;
            mcu_bit_cnt <= 0;
            mcu_byte_idx <= 0;
            mcu_tx_byte <= 8'h00; // MCU sends dummy bytes
        end else if (mcu_cs_n) begin
            mcu_transaction_active <= 1'b0;
            mcu_bit_cnt <= 0;
            mcu_byte_idx <= 0;
        end
    end
    
    // MCU reads on rising edge (Mode 0)
    always_ff @(posedge mcu_sck) begin
        if (!mcu_cs_n && mcu_transaction_active) begin
            mcu_rx_byte <= {mcu_rx_byte[6:0], mcu_sdo};
            mcu_bit_cnt <= mcu_bit_cnt + 1;
            if (mcu_bit_cnt == 7) begin
                mcu_packet[mcu_byte_idx] <= mcu_rx_byte;
                mcu_byte_idx <= mcu_byte_idx + 1;
                mcu_bit_cnt <= 0;
            end
        end
    end
    
    // Test variables
    integer test_pass = 0;
    integer test_fail = 0;
    integer packet_count = 0;
    
    // Timeout parameters
    parameter TIMEOUT_INIT = 10_000_000; // 10M cycles = ~3.3 seconds at 3MHz
    parameter TIMEOUT_PACKET = 1_000_000; // 1M cycles = ~0.33 seconds per packet
    parameter TIMEOUT_TOTAL = 30_000_000; // 30M cycles = ~10 seconds total
    
    // Timeout monitoring
    integer cycle_count = 0;
    always @(posedge clk) begin
        if (fpga_rst_n) begin
            cycle_count <= cycle_count + 1;
            if (cycle_count > TIMEOUT_TOTAL) begin
                $display("\n[TIMEOUT] Test exceeded maximum time (%0d cycles)", TIMEOUT_TOTAL);
                $display("[FAIL] Test terminated due to timeout");
                test_fail = test_fail + 1;
                $finish;
            end
        end else begin
            cycle_count <= 0;
        end
    end
    
    // Test task: MCU reads packet
    task mcu_read_packet;
        integer i;
        begin
            mcu_transaction_active <= 1'b0;
            #(10 * CLK_PERIOD);
            mcu_cs_n <= 1'b0; // Start transaction
            #(200 * CLK_PERIOD); // Wait for 16 bytes
            mcu_cs_n <= 1'b1; // End transaction
            #(10 * CLK_PERIOD);
            
            packet_count = packet_count + 1;
            $display("\n[MCU] Packet %0d received:", packet_count);
            for (i = 0; i < 16; i = i + 1) begin
                $write("%02x ", mcu_packet[i]);
                if ((i + 1) % 8 == 0) $write("\n");
            end
        end
    endtask
    
    // Verify packet
    task verify_packet;
        input [7:0] expected_header;
        input integer test_num;
        begin
            if (mcu_packet[0] == expected_header) begin
                $display("[PASS] TC%0d: Header byte correct (0x%02x)", test_num, expected_header);
                test_pass = test_pass + 1;
            end else begin
                $display("[FAIL] TC%0d: Header byte incorrect (expected 0x%02x, got 0x%02x)", 
                         test_num, expected_header, mcu_packet[0]);
                test_fail = test_fail + 1;
            end
            
            // Check flags byte
            if ((mcu_packet[15] & 8'h04) == 8'h04) begin
                $display("[PASS] TC%0d: Initialized flag set", test_num);
                test_pass = test_pass + 1;
            end else begin
                $display("[FAIL] TC%0d: Initialized flag not set", test_num);
                test_fail = test_fail + 1;
            end
        end
    endtask
    
    // Main test sequence
    initial begin
        $display("========================================");
        $display("Full System Integration Test");
        $display("========================================");
        $display("Timeout: %0d cycles (~%0d seconds)", TIMEOUT_TOTAL, TIMEOUT_TOTAL * CLK_PERIOD / 1_000_000_000);
        
        // Initialize
        fpga_rst_n = 0;
        mcu_cs_n = 1'b1;
        mcu_transaction_active = 0;
        mcu_bit_cnt = 0;
        mcu_byte_idx = 0;
        mcu_sck_reg = 0;
        mcu_sck_div = 0;
        
        #(100 * CLK_PERIOD);
        fpga_rst_n = 1;
        
        $display("\n[TEST] Waiting for FPGA initialization... (timeout: %0d cycles)", TIMEOUT_INIT);
        
        // Wait for initialization with timeout
        fork
            begin
                wait(led_initialized == 1 || led_error == 1);
                $display("[INFO] FPGA initialization completed after %0d cycles", cycle_count);
            end
            begin
                #(TIMEOUT_INIT * CLK_PERIOD);
                if (!led_initialized && !led_error) begin
                    $display("\n[TIMEOUT] FPGA initialization did not complete within %0d cycles", TIMEOUT_INIT);
                    $display("[FAIL] FPGA stuck in initialization");
                    test_fail = test_fail + 1;
                end
            end
        join_any
        disable fork;
        
        #(100 * CLK_PERIOD);
        
        if (led_initialized) begin
            $display("[PASS] FPGA initialized (LED on)");
            test_pass = test_pass + 1;
        end else begin
            $display("[FAIL] FPGA not initialized (LED off)");
            test_fail = test_fail + 1;
        end
        
        if (led_error) begin
            $display("[FAIL] FPGA error LED on");
            test_fail = test_fail + 1;
        end else begin
            $display("[PASS] FPGA no error (error LED off)");
            test_pass = test_pass + 1;
        end
        
        $display("\n[TEST] MCU reading packets...");
        
        // Read several packets
        repeat (5) begin
            mcu_read_packet();
            verify_packet(8'hAA, packet_count);
            #(10000 * CLK_PERIOD); // Wait between reads
        end
        
        // Summary
        $display("\n========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Passed: %0d", test_pass);
        $display("Failed: %0d", test_fail);
        if (test_fail == 0) begin
            $display("*** ALL TESTS PASSED ***");
        end else begin
            $display("*** SOME TESTS FAILED ***");
        end
        $display("========================================");
        
        #(10000 * CLK_PERIOD);
        $finish;
    end
    
endmodule

