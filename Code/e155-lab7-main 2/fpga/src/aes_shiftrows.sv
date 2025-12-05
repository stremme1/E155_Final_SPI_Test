`timescale 10ns/1ns

/////////////////////////////////////////////
// testbench_shift_rows
// Tests the row shfits as per expected based on the round number
// Added 10/26/2025 by Sadhvi Narayanan
// sanarayanan@g.hmc.edu
/////////////////////////////////////////////

module testbench_shiftrows();
    logic [127:0] row_in, row_out_expected, row_out;
    
    // device under test
    shiftrows dut(row_in, row_out);
    
    // test case
    initial begin   
    // Test case from FIPS-197 Appendix A.1, B
    row_in    = 128'hD42711AEE0BF98F1B8B45DE51E415230;
    row_out_expected = 128'hD4BF5D30E0B452AEB84111F11E2798E5;
    
	// add a delay and then test the output
	# 22 
    assert(row_out == row_out_expected) else $error("Incorrect row output from shifting");
	$display("shift rows passed!");
    end
    
endmodule