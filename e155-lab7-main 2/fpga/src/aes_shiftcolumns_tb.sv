`timescale 10ns/1ns

/////////////////////////////////////////////
// testbench_mixcolumns
// Tests the columns are mixed around correctly based on the input
// Added 10/26/2025 by Sadhvi Narayanan
// sanarayanan@g.hmc.edu
/////////////////////////////////////////////

module testbench_mixcolumns();
    logic [127:0] row_in, row_out_expected, row_out;
    
    // device under test
    mixcolumns dut(row_in, row_out);
    
    // test case
    initial begin   

    row_in    = 128'hD4BF5D30E0B452AEB84111F11E2798E5;
    row_out_expected = 128'h046681E5E0CB199A48F8D37A2806264C;
    
	// add a delay and then test the output
	# 22 
    assert(row_out == row_out_expected) else $error("Incorrect output from mixing columns");
		
	$display("Shiftcolumns passed!");
	
    end
    
endmodule