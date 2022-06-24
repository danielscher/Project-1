module ProcessorTestbench();

	reg clk;
	reg reset;

	integer i;
	reg [31:0] expectedRegContent [1:31];

	// Instantiate the Verilog module under test
	Processor proc(clk, reset);

	initial
		begin
			// Generate a waveform output with all (non-memory) variables
			$dumpfile("simres.vcd");
			$dumpvars(0, ProcessorTestbench);

			// initialize actual and expected registers to 0xcafebabe
			for(i=1; i<32; i=i+1) begin
				proc.mips.dp.gpr.registers[i] = 32'hcafebabe;
				expectedRegContent[i] = 32'hcafebabe;
			end

			// Read program to be executed
//			$readmemh("TestPrograms/Fibonacci.dat", proc.imem.INSTRROM, 0, 5);
//			$readmemh("TestPrograms/Fibonacci.expected", expectedRegContent);
//			$readmemh("TestPrograms/FunctionCall.dat", proc.imem.INSTRROM, 0, 4);
//			$readmemh("TestPrograms/FunctionCall.expected", expectedRegContent);
//			$readmemh("TestPrograms/Constants.dat", proc.imem.INSTRROM, 0, 2);
//			$readmemh("TestPrograms/Constants.expected", expectedRegContent);
//			$readmemh("TestPrograms/Multiplication.dat", proc.imem.INSTRROM, 0, 4);
//			$readmemh("TestPrograms/Multiplication.expected", expectedRegContent);
//			$readmemh("TestPrograms/Test_sltu_bne.dat", proc.imem.INSTRROM, 0, 6);
//			$readmemh("TestPrograms/Test_sltu_bne.expected", expectedRegContent);
			$readmemh("TestPrograms/OurOwnDivProgram.dat", proc.imem.INSTRROM, 0, 62);
			$readmemh("TestPrograms/OurOwnDivProgram.expected", expectedRegContent);
			

			// Generate reset input
			reset <= 1;
			#5; reset <= 0;
			// Number of simulated cycles
//			#117; // Fibonacci
//			#20; // FunctionCall
//			#16; // Constants
//			#24; // Multiplication
//			#24; // Test_sltu_bne
			#200; // OurOwnDivision

			for(i=1; i<32; i=i+1) begin
				$display("Register %d = %h", i, proc.mips.dp.gpr.registers[i]);
			end
			for(i=1; i<32; i=i+1) begin
				if(^proc.mips.dp.gpr.registers[i] === 1'bx || proc.mips.dp.gpr.registers[i] != expectedRegContent[i]) begin
					$write("FAILED");
					$display(": register %d = %h, expected %h",i, proc.mips.dp.gpr.registers[i], expectedRegContent[i]);
					$finish;
				end
			end
			$display("PASSED");
			$finish;
		end

	// Generate a periodic clock signal
	always
		begin
			clk <= 1; #2; clk <= 0; #2;
		end

endmodule

