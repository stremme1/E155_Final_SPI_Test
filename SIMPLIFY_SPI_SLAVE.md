# Simplifying SPI Slave Based on Lab07

## Key Insight from Lab07

Lab07's `aes_spi` module is **much simpler** than our current implementation:

1. **No complex state machine** - just shift registers and flags
2. **Shifts on clock edges** - `posedge sck` for shifting, `negedge sck` for output changes
3. **Simple `wasdone` flag** - tracks when data was ready
4. **Direct assignment** - `sdo = (done & !wasdone) ? cyphertext[127] : sdodelayed`

## Our Current Problem

Our `spi_slave_mcu.sv` has:
- 9 states (IDLE, DATA_READY, SENDING, BYTE_COMPLETE, BYTE_COMPLETE_WAIT, BYTE_COMPLETE_WAIT2, BYTE_COMPLETE_WAIT3, BYTE_COMPLETE_WAIT4, ACK_WAIT)
- Complex timing logic with multiple wait states
- Hard to debug and verify

## Lab07 Pattern for Our Use Case

We need to send 15 bytes sequentially. Lab07 sends 128 bits (16 bytes) but the pattern is the same:

1. **Capture data when ready** - set `done = 1`
2. **Shift out MSB first** on each clock edge
3. **Update shift register** on falling edge (Mode 0)
4. **Get next byte** when current byte completes

## Simplified Approach

```systemverilog
// Simple shift register approach
always_ff @(posedge sclk_mcu) begin
    if (!cs_n_mcu && packet_ready) begin
        // Shift left, MSB first
        tx_shift <= {tx_shift[6:0], 1'b0};
        bit_cnt <= bit_cnt + 1;
        
        if (bit_cnt == 7) begin
            // Byte complete - get next byte from formatter
            byte_index <= byte_index + 1;
            tx_shift <= tx_data;  // Formatter provides next byte
        end
    end
end

always_ff @(negedge sclk_mcu) begin
    if (!cs_n_mcu && packet_ready) begin
        // Update output (Mode 0: change on falling edge)
        // Already handled by combinational assignment
    end
end

assign miso_mcu = (!cs_n_mcu && packet_ready) ? tx_shift[7] : 1'b0;
```

This is **much simpler** than our current 9-state machine!

