// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// aes galoismult module to compute the galois field based on the input byte

/////////////////////////////////////////////
// galoismult
//   Multiply by x in GF(2^8) is a left shift
//   followed by an XOR if the result overflows
//   Uses irreducible polynomial x^8+x^4+x^3+x+1 = 00011011
/////////////////////////////////////////////

module galoismult(input  logic [7:0] a,
                output logic [7:0] y);




  logic [7:0] ashift;
   assign ashift = {a[6:0], 1'b0};
  assign y = a[7] ? (ashift ^ 8'b00011011) : ashift;
endmodule
