// SPI Slave Model for MCU Simulation
// Simulates an MCU receiving data from FPGA via SPI
// SPI Mode 0: CPOL=0, CPHA=0
// Simple implementation that samples on SCLK rising edge

module spi_slave_model (
    input  logic        clk,
    input  logic        rst_n,
    
    // SPI interface (from FPGA)
    input  logic        sclk,       // SPI clock from master
    input  logic        mosi,       // Master out, slave in
    input  logic        cs_n,       // Chip select (active low)
    
    // Received data
    output logic [7:0]  rx_data,    // Received byte
    output logic        rx_valid    // Data valid (pulse on complete byte)
);

    // Use SCLK directly for sampling (SPI Mode 0: sample on rising edge)
    logic [7:0] rx_shift;
    logic [2:0] bit_cnt;
    logic cs_prev;
    logic sclk_prev;
    logic sclk_rising;
    
    // Track CS and SCLK edges
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_prev <= 1'b1;
            sclk_prev <= 1'b0;
        end else begin
            cs_prev <= cs_n;
            sclk_prev <= sclk;
        end
    end
    
    assign sclk_rising = sclk && !sclk_prev;
    
    // SPI receive logic - sample on SCLK rising edge when CS is low
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_shift <= 8'd0;
            bit_cnt <= 3'd0;
            rx_data <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            
            // CS falling edge - start of transfer
            if (!cs_n && cs_prev) begin
                rx_shift <= 8'd0;
                bit_cnt <= 3'd0;
            end
            // CS rising edge - end of transfer
            else if (cs_n && !cs_prev) begin
                rx_data <= rx_shift;
                rx_valid <= 1'b1;
            end
            // During transfer: sample on rising edge of SCLK
            else if (!cs_n && sclk_rising) begin
                // Sample MSB first
                rx_shift <= {rx_shift[6:0], mosi};
                if (bit_cnt == 7) begin
                    bit_cnt <= 3'd0;
                end else begin
                    bit_cnt <= bit_cnt + 1;
                end
            end
        end
    end
    
endmodule
