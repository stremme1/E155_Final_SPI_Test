// Sadhvi Narayanan, sanarayanan@g.hmc.edu, 10/26/2025
// aes module to computer the next rounds starting state between the current output and the key

// computes the xor to calculate the next state
module add_round_key (input logic [127:0] state,
                    input logic [127:0] key,
                    output logic [127:0] state_out);                  
    assign state_out = state ^ key;
endmodule