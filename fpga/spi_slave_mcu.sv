`timescale 1ns / 1ps

// SPI Slave Module for MCU Communication
// SPI Mode 0: CPOL=0, CPHA=0 (MCU is master)
// MCU expects: CPOL=0, CPHA=0 per lab07 pattern
// FPGA acts as slave, sends data when MCU reads

module spi_slave_mcu (
    input  logic        clk,
    input  logic        rst_n,
    
    // SPI interface (MCU is master)
    input  logic        sclk_mcu,    // SPI clock from MCU
    input  logic        mosi_mcu,    // Master out, slave in (MCU -> FPGA, not used for our case)
    output logic        miso_mcu,    // Master in, slave out (FPGA -> MCU)
    input  logic        cs_n_mcu,    // Chip select from MCU (active low)
    
    // Control signals (Lab07 pattern)
    output logic        done,        // DONE signal to MCU (PA6)
    input  logic        load,        // LOAD signal from MCU (PA5)
    
    // Data interface
    input  logic        data_ready,  // New data available to send
    input  logic [7:0]  tx_data,     // Data byte to send
    output logic        tx_ack       // Acknowledge that data was sent
);

    // SPI Mode 0: CPOL=0 (idle low), CPHA=0 
    // - Data is captured on rising edge of SCLK
    // - Data is changed on falling edge of SCLK
    // - MSB first
    
    logic [7:0] tx_shift;
    logic [2:0] bit_cnt;
    logic cs_prev;
    logic sclk_prev;
    logic sclk_rising;
    logic sclk_falling;
    logic load_prev;
    logic load_rising;
    logic load_falling;
    
    // State machine for DONE/LOAD handshake
            typedef enum logic [3:0] {
                IDLE,
                DATA_READY,
                SENDING,
                BYTE_COMPLETE,
                BYTE_COMPLETE_WAIT,
                BYTE_COMPLETE_WAIT2,
                BYTE_COMPLETE_WAIT3,
                BYTE_COMPLETE_WAIT4,
                ACK_WAIT
            } state_t;
    
    state_t state;
    
    // Synchronize external signals (double-flop for clock domain crossing)
    logic sclk_sync1, sclk_sync2;
    logic cs_sync1, cs_sync2;
    logic load_sync1, load_sync2;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync1 <= 1'b0;
            sclk_sync2 <= 1'b0;
            cs_sync1 <= 1'b1;
            cs_sync2 <= 1'b1;
            load_sync1 <= 1'b0;
            load_sync2 <= 1'b0;
            sclk_prev <= 1'b0;
            cs_prev <= 1'b1;
            load_prev <= 1'b0;
        end else begin
            // Double-flop synchronization
            sclk_sync1 <= sclk_mcu;
            sclk_sync2 <= sclk_sync1;
            cs_sync1 <= cs_n_mcu;
            cs_sync2 <= cs_sync1;
            load_sync1 <= load;
            load_sync2 <= load_sync1;
            
            sclk_prev <= sclk_sync2;
            cs_prev <= cs_sync2;
            load_prev <= load_sync2;
        end
    end
    
    assign sclk_rising = sclk_sync2 && !sclk_prev;
    assign sclk_falling = !sclk_sync2 && sclk_prev;
    assign load_rising = load_sync2 && !load_prev;
    assign load_falling = !load_sync2 && load_prev;
    
    // DONE/LOAD handshake state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            tx_ack <= 1'b0;
            tx_shift <= 8'd0;
            bit_cnt <= 3'd0;
        end else begin
            tx_ack <= 1'b0;
            
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    bit_cnt <= 3'd0;
                    if (data_ready) begin
                        state <= DATA_READY;
                        // Don't capture tx_data here - wait until we're actually sending
                        // tx_data will be set by formatter when we start the transaction
                        done <= 1'b1;  // Assert DONE to signal MCU
                    end
                end
                
                DATA_READY: begin
                    // Wait for MCU to start reading (CS goes low)
                    if (!cs_sync2 && cs_prev) begin
                        state <= SENDING;
                        bit_cnt <= 3'd0;
                        // Capture tx_data now, when CS goes low (transaction starting)
                        tx_shift <= tx_data;
                    end
                    // If load is toggled (acknowledge without reading), reset
                    if (load_rising) begin
                        state <= IDLE;
                        done <= 1'b0;
                        tx_ack <= 1'b1;
                    end
                end
                
                SENDING: begin
                    // In Mode 0: change data on falling edge, sample on rising edge
                    // MSB first: output bit 7, then bit 6, etc.
                    // We change MISO on falling edge of SCLK, MCU samples on rising edge
                    if (sclk_falling && !cs_sync2) begin
                        // Shift left after outputting current bit (MSB first)
                        if (bit_cnt < 7) begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_cnt <= bit_cnt + 1;
                        end
                    end
                    
                    // Detect when byte is complete (8 bits sent)
                    // After 8th bit is sampled (rising edge), byte is done
                    if (bit_cnt == 7 && sclk_rising && !cs_sync2) begin
                        state <= BYTE_COMPLETE;
                    end
                    
                    // CS rising edge - transaction aborted
                    if (cs_sync2 && !cs_prev) begin
                        state <= DATA_READY;  // Go back, keep DONE asserted
                        bit_cnt <= 3'd0;
                    end
                end
                
                BYTE_COMPLETE: begin
                    // Byte complete - acknowledge to formatter
                    tx_ack <= 1'b1;
                    bit_cnt <= 3'd0;
                    
                    // Check if CS is still low (continuous transaction)
                    if (!cs_sync2) begin
                        // CS still low - continuous transaction
                        // Wait for formatter to update tx_data, then capture next byte
                        state <= BYTE_COMPLETE_WAIT;
                    end else begin
                        // CS went high - transaction complete
                        tx_ack <= 1'b0;
                        state <= ACK_WAIT;
                        if (!data_ready) begin
                            done <= 1'b0;  // Deassert DONE if no more data
                        end
                    end
                end
                
                BYTE_COMPLETE_WAIT: begin
                    // Wait for formatter to update tx_data after receiving tx_ack
                    tx_ack <= 1'b0;
                    
                    // Check if CS is still low (continuous transaction)
                    if (!cs_sync2) begin
                        if (data_ready) begin
                            // Wait one cycle for formatter to update tx_data and byte_index
                            state <= BYTE_COMPLETE_WAIT2;
                        end else begin
                            // No more data, wait for CS to go high
                            state <= BYTE_COMPLETE_WAIT;
                        end
                    end else begin
                        // CS went high - transaction complete
                        state <= ACK_WAIT;
                        if (!data_ready) begin
                            done <= 1'b0;
                        end
                    end
                end
                
                BYTE_COMPLETE_WAIT2: begin
                    // Wait another cycle to ensure tx_data is fully stable after byte_index update
                    if (!cs_sync2) begin
                        state <= BYTE_COMPLETE_WAIT3;
                    end else begin
                        state <= ACK_WAIT;
                    end
                end
                
                BYTE_COMPLETE_WAIT3: begin
                    // Wait one more cycle to ensure formatter's combinational logic has fully updated
                    // This is needed because combinational logic uses OLD state/byte_index during transitions
                    if (!cs_sync2) begin
                        state <= BYTE_COMPLETE_WAIT4;
                    end else begin
                        // CS went high
                        state <= ACK_WAIT;
                    end
                end
                
                BYTE_COMPLETE_WAIT4: begin
                    // Capture tx_data now - formatter should have fully updated it for new byte_index
                    if (!cs_sync2) begin
                        tx_shift <= tx_data;
                        bit_cnt <= 3'd0;
                        state <= SENDING;
                    end else begin
                        // CS went high
                        state <= ACK_WAIT;
                    end
                end
                
                ACK_WAIT: begin
                    tx_ack <= 1'b0;  // Clear ack after one cycle
                    // Wait for LOAD toggle from MCU (acknowledge) or CS to go low again
                    if (!cs_sync2 && cs_prev) begin
                        // CS went low again - new transaction starting
                        if (data_ready) begin
                            state <= SENDING;
                            tx_shift <= tx_data;
                            bit_cnt <= 3'd0;
                        end else begin
                            state <= DATA_READY;
                        end
                    end else if (load_rising) begin
                        if (data_ready) begin
                            // More data available, go back to DATA_READY
                            state <= DATA_READY;
                            done <= 1'b1;
                        end else begin
                            // No more data, return to IDLE
                            state <= IDLE;
                            done <= 1'b0;
                        end
                    end
                end
            endcase
        end
    end
    
    // MISO output - MSB first, change on falling edge (Mode 0)
    assign miso_mcu = (!cs_sync2) ? tx_shift[7] : 1'b0;
    
endmodule

