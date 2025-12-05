// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// aes key expansion module to calculate the new key based on the previous one

module key_expansion(input logic clk,
                    input logic [3:0] round_num,
                    input  logic [127:0] key,
                    output logic [127:0] round_key);
 
   logic [31:0] rcon, col1, col2, col3, col4, out1, out2, out3, out4, byte_in, byte_out;
   logic [7:0] byte_out1, byte_out2, byte_out3, byte_out4;
  
   // picks the round constant based on which round we are in - this is fixed
   always_comb begin
       case(round_num)
           4'd1: rcon = 32'h01000000;
           4'd2: rcon = 32'h02000000;
           4'd3: rcon = 32'h04000000;
           4'd4: rcon = 32'h08000000;
           4'd5: rcon = 32'h10000000;
           4'd6: rcon = 32'h20000000;
           4'd7: rcon = 32'h40000000;
           4'd8: rcon = 32'h80000000;
           4'd9: rcon = 32'h1b000000;
           4'd10: rcon = 32'h36000000;
           default: rcon = 32'h01000000;
       endcase
   end
   
   // replaces each byte in the right most column of the original key
   sbox_sync sb1(key[23:16], clk, byte_out1);
   sbox_sync sb2(key[15:8], clk, byte_out2);
   sbox_sync sb3(key[7:0], clk, byte_out3);
   sbox_sync sb4(key[31:24], clk, byte_out4);
 
   // find the initial columns of the key based on the input
   assign col1 = key[31:0];
   assign col2 = key[63:32];
   assign col3 = key[95:64];
   assign col4 = key[127:96];
 
   // creates the output byte after shifting - bottom shifts the rightmost columns
   assign byte_out = {byte_out1, byte_out2, byte_out3, byte_out4};
  
   // computes the output columns of the new key based on the algorithm (used the tutorial)
   assign out1 = col4 ^ byte_out ^ rcon;
   assign out2 = col3 ^ out1;
   assign out3 = col2 ^ out2;
   assign out4 = col1 ^ out3;
   
   // puts the columns together for the final output
   assign round_key = {out1, out2, out3, out4};


endmodule