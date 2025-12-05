`timescale 10ns/1ns

/////////////////////////////////////////////
// testbench_aes_add_round_key
// Tests the round key module correctly computes the xor of the two inputs
// Added 10/26/2025 by Sadhvi Narayanan
// sanarayanan@g.hmc.edu
/////////////////////////////////////////////

module testbench_aes_add_round_key();
    logic [127:0] state, key, state_out, state_out_expected;
    
    // device under test
    add_round_key dut(state, key, state_out);
    
    // test case
    initial begin   
    // Test case from FIPS-197 Appendix A.1, B
    state    = 128'h3243F6A8885A308D313198A2E0370734;
    key      = 128'h2B7E151628AED2A6ABF7158809CF4F3C;
	state_out_expected = 128'h193DE3BEA0F4E22B9AC68D2AE9F84808;
    
	// add a delay and then test the output
    # 22 
	assert(state_out == state_out_expected) else $error("Incorrect ADD ROUND KEY after xor");
	$display("Add round key passed!");
		
    end
    
endmodule