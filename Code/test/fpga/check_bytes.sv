module check_bytes;
    initial begin
        logic [7:0] packet_buffer [0:15];
        packet_buffer[0] = 8'hAA;
        packet_buffer[1] = 8'h12;
        packet_buffer[2] = 8'h34;
        
        logic [127:0] tx_packet;
        tx_packet = {
            packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3],
            packet_buffer[4], packet_buffer[5], packet_buffer[6], packet_buffer[7],
            packet_buffer[8], packet_buffer[9], packet_buffer[10], packet_buffer[11],
            packet_buffer[12], packet_buffer[13], packet_buffer[14], packet_buffer[15]
        };
        
        $display("tx_packet[127:120] = 0x%02X (should be 0xAA)", tx_packet[127:120]);
        $display("tx_packet[119:112] = 0x%02X (should be 0x12)", tx_packet[119:112]);
        $display("tx_packet[111:104] = 0x%02X (should be 0x34)", tx_packet[111:104]);
        $display("");
        $display("Formula check:");
        $display("127 - (0+1)*8 = %d (should be 119)", 127 - (0+1)*8);
        $display("127 - (1+1)*8 = %d (should be 111)", 127 - (1+1)*8);
        $display("");
        $display("tx_packet[127 - (0+1)*8 -: 8] = tx_packet[119:112] = 0x%02X", tx_packet[127 - (0+1)*8 -: 8]);
        $display("tx_packet[127 - (1+1)*8 -: 8] = tx_packet[111:104] = 0x%02X", tx_packet[127 - (1+1)*8 -: 8]);
    end
endmodule
