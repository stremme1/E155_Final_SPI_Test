// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// aes core module which handles building the algorithmic nature of aes, and is the full fsm


/////////////////////////////////////////////
// aes_core
//   top level AES encryption module
//   when load is asserted, takes the current key and plaintext
//   generates cyphertext and asserts done when complete 11 cycles later
//
//   See FIPS-197 with Nk = 4, Nb = 4, Nr = 10
//
//   The key and message are 128-bit values packed into an array of 16 bytes as
//   shown below
//        [127:120] [95:88] [63:56] [31:24]     S0,0    S0,1    S0,2    S0,3
//        [119:112] [87:80] [55:48] [23:16]     S1,0    S1,1    S1,2    S1,3
//        [111:104] [79:72] [47:40] [15:8]      S2,0    S2,1    S2,2    S2,3
//        [103:96]  [71:64] [39:32] [7:0]       S3,0    S3,1    S3,2    S3,3
//
//   Equivalently, the values are packed into four words as given
//        [127:96]  [95:64] [63:32] [31:0]      w[0]    w[1]    w[2]    w[3]
/////////////////////////////////////////////
module aes_core(input  logic         clk,
              input  logic         load,
              input  logic [127:0] key,
              input  logic [127:0] plaintext,
              output logic         done,
              output logic [127:0] cyphertext);

  // initialize all signals requires
  
  // states used in the design
  typedef enum logic [3:0] {IDLE, TEMP, SUB_BYTE, SBOX_HOLD, ROUND_LOOP, FINAL_ROUND, END} statetype;
  statetype state, next_state;
  logic load_key, prev_load;
  logic [127:0] output_reg, next_output_reg;
  logic [127:0] after_rows, after_cols;
  logic [127:0] round_key, prev_key;
  logic [127:0] after_add_round_key, after_add_round_key_final;
  logic [3:0] round, next_round;
  logic [3:0] index, next_i;
  logic [7:0] byte_in, byte_out;
 
  // uses the round number and prev_key to create the next key
  key_expansion ke(clk, round, prev_key, round_key);
  
  // sbox module for byte substitution
  sbox_sync sb(byte_in, clk, byte_out);
  
  // shiftrows
  shiftrows sr(output_reg, after_rows);
  
  // mixcolumns
  mixcolumns mc(after_rows, after_cols);
  
  // computes the next round's starting point based on the output and the key
  add_round_key ark1(after_cols, round_key, after_add_round_key);
  add_round_key ark2(after_rows, round_key, after_add_round_key_final);
  
   always_ff @(posedge clk) begin
	  // if load is high, then stay in the initial/IDLE state
      if (load) begin
        round <= 4'd0;
        state <= IDLE;
        output_reg <= plaintext;
        index <= 4'd0;
        prev_key <= key;
		prev_load <= load;
	// otherwise begin execution of the FSM - transitioning through the different states
     end else begin
          round <= next_round;
          state <= next_state;
          output_reg <= next_output_reg;
          index <= next_i;
		  prev_load <= load;
          
		  // preserve reset logic for initial state so the key is correct on a loop back
		  if (state == IDLE) prev_key <= key;
		  else if (load_key) begin
			  // only if we want to load the key, load it
              prev_key <= round_key;
          end
     end
  end
 
  always_comb begin
      // default initialization here for next state logic
      next_state = state;
      next_output_reg = output_reg;
      next_round = round;
      next_i = index;
      done = 0;
      load_key = 0;
	  // computes each 8bit/1-byte output to go into the sbox substitution
	  byte_in = output_reg[127 - (index*8) -: 8];
    
      case(state)
		  // if the MCU has finisihed sending its data then create the first output and proceed to the next state
          IDLE: begin
              done = 0;
              if (!load && prev_load) begin
				  next_round = 1;
				  next_output_reg = plaintext ^ key;
				  next_i = 4'd0;
				  next_state = SUB_BYTE;
				  // load_key = 1;
              end
          end
        
		  // calcutes input byte for substitution
          SUB_BYTE: begin
			  // computes each 8bit/1-byte output to go into the sbox substitution
              byte_in = output_reg[127 - (index*8) -: 8]; // set the output to sbox in this state, so on the next clk cycle it ready to load in
              next_state = SBOX_HOLD;
          end
        
		  // stores substituted byte, and calculates whether to continue or end the round
          SBOX_HOLD: begin
              next_output_reg = output_reg;
			  // replace the byte after we have waited 1 clk cycle from the previous state
              next_output_reg[127 - (index*8) -: 8] = byte_out;
			  
			  // if we have completed 16 rounds of byte replacement, then move to the end round loop, and reset the counters
              if (index == 4'd15) begin
                  next_i = 4'd0;
                  next_state = ROUND_LOOP;
			 // otherwise, continue replacing the bytes and waiting 1 clk cycle
              end else begin
                  next_i = index + 4'd1;
                  next_state = SUB_BYTE;
              end
          end
        
		  // handles moving to the next round based on which round is completed - if 10 rounds are down, move to a different state
          ROUND_LOOP: begin
              if (next_round == 4'd10) begin
                  next_output_reg = after_add_round_key_final; // take the output before mixcolumns
                  next_state = END;
              end else begin
                  next_output_reg = after_add_round_key; // take the output after mixcolumns
                  next_round = round + 4'd1;
                  load_key = 1;
                  next_state = SUB_BYTE;
                  next_i = 0;
              end
            
          end
        
		 // end state where done is asserted, and we hold the output until we are ready to load again
          END: begin
              done = 1;
			  if (!load && prev_load) next_state = IDLE;
			  else next_state = END;
              next_round = 0;
              next_i = 0;
          end


          default: next_state = IDLE;
        
      endcase
  end

  // output - but this only means something when done is asserted
  assign cyphertext = output_reg;
            
                
endmodule