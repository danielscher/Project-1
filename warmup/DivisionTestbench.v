module DivisionTestbench();

	// Generate input stimuli
	reg clk, s;
	wire [31:0] qres;
	wire [31:0] rres;

	initial
	begin
		s <= 1'b0; #1; s <= 1'b1; #4; s <= 1'b0; #160; $finish;
	end

	always
	begin
		clk <= 1'b1; #2; clk <= 1'b0; #2;
	end

	// Module under test
	Division divider(
		.clock(clk),
		.start(s),
		.a(32'd7), // TODO Feel free to try out other values!
		.b(32'd3),
		.q(qres),
		.r(rres)
	);

	// Test that 32 cycles (which take 4 time units each) after the start (at 
	// time 4) the correct result has been calculated.
	initial
	begin
		$dumpfile("divsim.vcd");
		$dumpvars;
		#133;
		if (qres == 32'd2 && rres == 32'd1)
			$display("Simulation succeeded");
		else
			$display("Simulation failed");
	end

endmodule

