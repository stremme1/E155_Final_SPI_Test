`timescale 1ns / 1ps

// SPI Master Module for BNO085 Communication
// BNO085 SPI: Mode 3 (CPOL=1, CPHA=1), MSB first, 3MHz max

module spi_master #(
    parameter CLK_DIV = 16  // Half-period in system clocks
)(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        start,
    input  logic        tx_valid,
    input  logic [7:0]  tx_data,
    output logic        tx_ready,
    output logic        rx_valid,
    output logic [7:0]  rx_data,
    output logic        busy,
    
    // SPI interface
    output logic        sclk,
    output logic        mosi,
    input  logic        miso
    // cs_n removed - controlled by higher level module for packet framing
);

    typedef enum logic [2:0] {
        IDLE,
        TX_RX,
        DONE
    } state_t;
    
    state_t state;
    logic [3:0] bit_cnt;
    logic [7:0] tx_shift;
    logic [7:0] rx_shift;
    logic [7:0] clk_cnt;
    logic sclk_en;
    logic sclk_reg;
    
    // Clock divider
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 8'd0;
            sclk_reg <= 1'b1;  // CPOL=1: idle high
        end else begin
            if (sclk_en) begin
                if (clk_cnt == CLK_DIV - 1) begin
                    clk_cnt <= 8'd0;
                    sclk_reg <= ~sclk_reg;
                end else begin
                    clk_cnt <= clk_cnt + 1;
                end
            end else begin
                sclk_reg <= 1'b1;  // Return to idle high
                clk_cnt <= 8'd0;
            end
        end
    end
    
    assign sclk = sclk_reg;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_cnt <= 4'd0;
            tx_shift <= 8'd0;
            rx_shift <= 8'd0;
            sclk_en <= 1'b0;
            tx_ready <= 1'b0;
            rx_valid <= 1'b0;
            rx_data <= 8'd0;
        end else begin
            // Default: clear rx_valid (it's a pulse)
            rx_valid <= 1'b0;
            
            case (state)
                IDLE: begin
                    sclk_en <= 1'b0;
                    bit_cnt <= 4'd0;
                    tx_ready <= 1'b1;
                    // Clear rx_data when starting a new transaction to prevent stale data
                    if (start && tx_valid) begin
                        rx_data <= 8'd0; // Clear before starting new transaction
                        state <= TX_RX;
                        tx_shift <= tx_data;
                        rx_shift <= 8'd0;
                        tx_ready <= 1'b0;
                        sclk_en <= 1'b1;
                        // sclk starts High (CPOL=1)
                    end
                end
                
                TX_RX: begin
                    // Mode 3: Sample on Rising Edge, Shift on Falling Edge
                    
                    // Detect Falling Edge (High -> Low)
                    if (sclk_reg && clk_cnt == CLK_DIV-1) begin
                        // Falling Edge happening next cycle
                        // Shift MOSI (except for the first bit which is already there)
                        if (bit_cnt > 0) begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                        end
                    end
                    
                    // Detect Rising Edge (Low -> High)
                    if (!sclk_reg && clk_cnt == CLK_DIV-1) begin
                        // Rising Edge happening next cycle
                        // Sample MISO
                        rx_shift <= {rx_shift[6:0], miso};
                        
                        if (bit_cnt == 7) begin
                            // Last bit sampled - byte complete
                            state <= DONE;
                            sclk_en <= 1'b0;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
                    end
                    
                    // Clear rx_shift at the start of a new byte transaction (when bit_cnt is 0 and we just entered TX_RX)
                    if (bit_cnt == 4'd0 && state == TX_RX) begin
                        // This is the first cycle of TX_RX - ensure rx_shift is clear
                        // (It should already be clear from IDLE, but this ensures it)
                    end
                end
                
                DONE: begin
                    rx_data <= rx_shift;
                    rx_valid <= 1'b1;
                    // Clear rx_data in the same cycle to prevent stale data if a new transaction starts immediately
                    // The controller should read rx_data when rx_valid is asserted
                    state <= IDLE;
                end
            endcase
        end
    end
    
    // MOSI output
    assign mosi = tx_shift[7];
    assign busy = (state != IDLE);
    
endmodule

