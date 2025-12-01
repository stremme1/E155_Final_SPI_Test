`timescale 10ns/1ns

/////////////////////////////////////////////
// testbench_shift_rows
// Tests the galois module correctly applies the formula on the incoming byte
// Added 10/26/2025 by Sadhvi Narayanan
// sanarayanan@g.hmc.edu
/////////////////////////////////////////////

module testbench_galoismult();
    logic [7:0] a, y, y_expected;
    
    // device under test
    galoismult dut(a, y);
    
    // test case
    initial begin   

    a    = 8'b01101011;
    y_expected = 8'hd6;
	// add a delay and then test the output
	 # 44
    assert(y == y_expected) else $error("Incorrect same output from galoismult %h %h", y, y_expected);
		
	a    = 8'b00101011;
    y_expected = 8'h56;
	// add a delay and then test the output
	 # 44 
    assert(y == y_expected) else $error("Incorrect ashift output from galoismult %h %h", y, y_expected);
	$display("Galoismult passed!");
	
    end
    
endmodule
