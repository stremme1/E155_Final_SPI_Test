/**
 * BNO08X SPI Master Interface
 * 
 * Implements SPI Mode 3 (CPOL=1, CPHA=1) for BNO08X communication
 * Clock: 3MHz (from 48MHz / 16)
 * 
 * SPI Timing Requirements (from datasheet):
 * - Max SPI clock: 3MHz
 * - CS setup to CLK: 0.1us min
 * - CS hold: 16.83ns min
 * - CLK to MISO valid: 35ns max
 * - MOSI setup: 25ns min
 * - MOSI hold: 5.4ns min
 */

module bno08x_spi_master (
    input  logic        clk,           // 3MHz clock
    input  logic        rst_n,         // Active low reset
    
    // SPI Interface
    output logic        spi_cs_n,      // Chip select (active low)
    output logic        spi_sck,       // SPI clock
    output logic        spi_mosi,      // Master out, slave in
    input  logic        spi_miso,      // Master in, slave out
    
    // Control Interface
    input  logic        start_transfer,
    input  logic        write_en,     // 1 = write, 0 = read
    input  logic [15:0] data_length,   // Number of bytes to transfer
    input  logic [7:0]  tx_data,       // Data to transmit
    output logic [7:0]  rx_data,       // Data received
    output logic        data_valid,    // Valid data available
    output logic        transfer_done, // Transfer complete
    output logic        transfer_busy  // Transfer in progress
);

    // SPI State Machine
    typedef enum logic [2:0] {
        IDLE,
        CS_SETUP,
        TRANSFER,
        CS_HOLD,
        DONE
    } spi_state_t;
    
    spi_state_t state, next_state;
    
    // Internal signals
    logic [15:0] byte_counter;
    logic [3:0]  bit_counter;
    logic [7:0]  tx_shift_reg;
    logic [7:0]  rx_shift_reg;
    logic        sck_enable;
    logic        sck_phase;  // 0 = low phase, 1 = high phase
    
    // Clock divider for SPI clock (3MHz / 2 = 1.5MHz SPI clock)
    // This gives us 500ns per SPI clock period (within 3MHz max)
    logic spi_clk_div;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_clk_div <= 1'b0;
        end else begin
            spi_clk_div <= ~spi_clk_div;
        end
    end
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            byte_counter <= 16'd0;
            bit_counter <= 4'd0;
            tx_shift_reg <= 8'd0;
            rx_shift_reg <= 8'd0;
            sck_phase <= 1'b0;
            spi_cs_n <= 1'b1;
            spi_sck <= 1'b1;  // Idle high (CPOL=1)
            spi_mosi <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_sck <= 1'b1;
                    sck_phase <= 1'b0;
                    if (start_transfer) begin
                        byte_counter <= data_length;
                        bit_counter <= 4'd0;
                        tx_shift_reg <= tx_data;
                    end
                end
                
                CS_SETUP: begin
                    spi_cs_n <= 1'b0;
                    // CS setup time: wait at least 0.1us
                    // At 3MHz, that's ~0.3 cycles, so we wait 1 cycle
                end
                
                TRANSFER: begin
                    if (spi_clk_div) begin  // Only on rising edge of divided clock
                        if (sck_phase == 1'b0) begin
                            // Low phase: setup data
                            spi_sck <= 1'b0;
                            if (bit_counter < 8) begin
                                spi_mosi <= tx_shift_reg[7];
                                tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
                            end
                            sck_phase <= 1'b1;
                        end else begin
                            // High phase: capture data (CPHA=1, capture on rising edge)
                            spi_sck <= 1'b1;
                            if (bit_counter < 8) begin
                                rx_shift_reg <= {rx_shift_reg[6:0], spi_miso};
                                bit_counter <= bit_counter + 1;
                            end
                            sck_phase <= 1'b0;
                            
                            if (bit_counter == 7) begin
                                // Byte complete
                                bit_counter <= 4'd0;
                                byte_counter <= byte_counter - 1;
                            end
                        end
                    end
                end
                
                CS_HOLD: begin
                    spi_cs_n <= 1'b1;
                    // CS hold time: wait at least 16.83ns
                    // At 3MHz, that's ~0.05 cycles, so we wait 1 cycle
                end
                
            DONE: begin
                // Hold done state for one cycle
            end
            
            default: begin
                // Default case to satisfy Verilator
            end
        endcase
    end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start_transfer && data_length > 0) begin
                    next_state = CS_SETUP;
                end
            end
            
            CS_SETUP: begin
                next_state = TRANSFER;
            end
            
            TRANSFER: begin
                if (byte_counter == 0 && bit_counter == 0 && sck_phase == 1'b0) begin
                    next_state = CS_HOLD;
                end
            end
            
            CS_HOLD: begin
                next_state = DONE;
            end
            
            DONE: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Output assignments
    assign rx_data = rx_shift_reg;
    assign data_valid = (state == TRANSFER && bit_counter == 0 && sck_phase == 1'b0 && byte_counter < data_length);
    assign transfer_done = (state == DONE);
    assign transfer_busy = (state != IDLE && state != DONE);
    
endmodule

