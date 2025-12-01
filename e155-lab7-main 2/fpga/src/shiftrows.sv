// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// aes module to shift the rows based on the row number

// module shifts the rows based on the row numbers
module shiftrows (input  logic [127:0] row_in,
                output logic [127:0] row_out);
   logic [31:0] r0_in, r1_in, r2_in, r3_in;
  logic [31:0] r0_out, r1_out, r2_out, r3_out;

  // breaks up the incoming data into row format through bit indexing
  assign r0_in = {row_in[127:120], row_in[95:88],  row_in[63:56],  row_in[31:24]};
  assign r1_in = {row_in[119:112], row_in[87:80],  row_in[55:48],  row_in[23:16]};
  assign r2_in = {row_in[111:104], row_in[79:72],  row_in[47:40],  row_in[15:8]};
  assign r3_in = {row_in[103:96],  row_in[71:64],  row_in[39:32],  row_in[7:0]};

  // call shiftrow where each row is shifted based on the row number
  shiftrow sr0(2'd0, r0_in, r0_out);
  shiftrow sr1(2'd1, r1_in, r1_out);
  shiftrow sr2(2'd2, r2_in, r2_out);
  shiftrow sr3(2'd3, r3_in, r3_out);

  // take the output shift rows and set it back to the output
  assign row_out = { r0_out[31:24], r1_out[31:24], r2_out[31:24], r3_out[31:24],
                     r0_out[23:16], r1_out[23:16], r2_out[23:16], r3_out[23:16],
                     r0_out[15:8],  r1_out[15:8],  r2_out[15:8],  r3_out[15:8],
                     r0_out[7:0],   r1_out[7:0],   r2_out[7:0],   r3_out[7:0] };
 endmodule



// shfits and individual row based on the shift amount
module shiftrow (input logic [1:0] shift_amt,
               input logic [31:0] row_in,
               output logic [31:0] row_out);
			   
  // left shfits the row by the number given
   always_comb begin
       case(shift_amt)
           2'd0: row_out = row_in;
           2'd1: row_out = {row_in[23:16], row_in[15:8], row_in[7:0], row_in[31:24]};
           2'd2: row_out = {row_in[15:8], row_in[7:0], row_in[31:24], row_in[23:16]};
           2'd3: row_out = {row_in[7:0], row_in[31:24], row_in[23:16], row_in[15:8]};
       endcase
   end




endmodule