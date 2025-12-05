// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// top level aes module


/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////
module aes(input  logic clk,
         input  logic sck,
         input  logic sdi,
         output logic sdo,
         input  logic load,
         output logic done);
                
  logic [127:0] key, plaintext, cyphertext;
  logic int_osc;
  
   // Internal high-speed oscillator
   // HSOSC #(.CLKHF_DIV(2'b11))
       // hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));

        
  aes_spi spi(sck, sdi, sdo, done, key, plaintext, cyphertext); 
  aes_core core(clk, load, key, plaintext, done, cyphertext); // for tb this is clk
endmodule







/////////////////////////////////////////////
// sbox
//   Infamous AES byte substitutions with magic numbers
//   Combinational version which is mapped to LUTs (logic cells)
//   Section 5.1.1, Figure 7
/////////////////////////////////////////////
// module sbox(input  logic [7:0] a,
          // output logic [7:0] y);
        
	// sbox implemented as a ROM
	// This module is combinational and will be inferred using LUTs (logic cells)
	// logic [7:0] sbox[0:255];




	// initial   $readmemh("sbox.txt", sbox);
	// assign y = sbox[a];
// endmodule




















