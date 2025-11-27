// Quick debug: Check what tx_buffer and tx_shift_reg contain
module debug_spi;
    logic clk, sck;
    logic drum_trigger_valid = 0;
    logic [3:0] drum_code = 0;
    logic [7:0] tx_buffer, tx_shift_reg;
    
    always #166.67 clk = ~clk;
    always #500 sck = ~sck;
    
    always_ff @(posedge clk) begin
        if (drum_trigger_valid) begin
            tx_buffer <= {4'h0, drum_code};
            tx_shift_reg <= {4'h0, drum_code};
            $display("[%0t] LATCHED: drum_code=%0d, tx_buffer=0x%02X, tx_shift_reg=0x%02X", 
                     $time, drum_code, {4'h0, drum_code}, {4'h0, drum_code});
        end
    end
    
    initial begin
        clk = 0; sck = 0;
        #1000;
        drum_code = 2;
        drum_trigger_valid = 1;
        #1000;
        drum_trigger_valid = 0;
        #5000;
        $display("Final: tx_buffer=0x%02X, tx_shift_reg=0x%02X", tx_buffer, tx_shift_reg);
        $finish;
    end
endmodule
