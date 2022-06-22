module SanitizerWarmup();

	MealyPatternTestbench tb();

	wire clock, inp;
	wire [1:0] outp;
	MealyPattern mp(.clock(clock), .i(inp), .o(outp));

	wire        start;
	wire [31:0] a;
	wire [31:0] b;
	wire [31:0] q;
	wire [31:0] r;
	Division divs(
		.clock(clock),
		.start(start),
		.a(a),
		.b(b),
		.q(q),
		.r(r)
	);

	initial
		begin
			// Generate a waveform output
			$dumpfile("simres.vcd");
			$dumpvars(0, tb.machine);
			$dumpvars(0, tb.machine.i);
			$dumpvars(0, tb.machine.o);
			$dumpvars(0, tb.machine.clock);
		end

endmodule

