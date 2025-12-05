`timescale 10ns/1ns

/////////////////////////////////////////////
// testbench_key_expansion
// Tests the keyexpansion module works correctly on a single round based on the algorithm
// Added 10/26/2025 by Sadhvi Narayanan
// sanarayanan@g.hmc.edu
/////////////////////////////////////////////

module testbench_key_expansion();
    logic [127:0] row_in, row_out_expected, row_out;
	logic [4:0] round_num;
	logic clk;
    
    // device under test
    key_expansion dut(clk, round_num, row_in, row_out);
	
	// generate a constant clk since this is sbox with RAM
	always begin
			clk = 1'b0; #5;
			clk = 1'b1; #5;
		end
    
    // test case
    initial begin  
		
	round_num = 1;

    row_in    = 128'h2B7E151628AED2A6ABF7158809CF4F3C;
    row_out_expected = 128'hA0FAFE1788542CB123A339392A6C7605;
	
	// add a delay and then test the output
	# 44
    assert(row_out == row_out_expected) else $error("Incorrect output from key expansion");
		
    $display("Key expansion passed!");
		
    end
    
endmodule