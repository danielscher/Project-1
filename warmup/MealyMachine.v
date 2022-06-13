module MealyPattern(
	input        clock,
	input        i,
	output [1:0] o
);

/* q0 is the n-1 and q1 is n-2 input when i is the n-th input.
 o1 is the right bit of the output o and o0 is the left.*/
reg q0;
reg q1;
reg o0;
reg o1;
//different states of interest of the mealy machine
parameter 
	S11 = 2'b11,
	S00 = 2'b00,
	S10 = 2'b10;


always @(posedge clock) begin
	q0 <= i;
	q1 <= q0;
end

/*checks if q0q1 is 11 or 00 
  assigns o0 or o1 to 1 respectively*/
always @(i or q0)begin
	case ({q0,q1})
		S11: begin
			o0 = i ? 0 : 1;
			o1 = 0;
		end
		S00: begin
			o0 = 0;
			o1 = i ? 1 : 0;
		end
		S10: begin
			o1 = 0;
			o0 = 0;
		end
		default: begin
			o1 = 0;
			o0 = 0;
			//$display("here");
		end
	endcase	
end
assign o[1:0] = {o0,o1};
endmodule


module MealyPatternTestbench();

	reg[0:9]	str = 10'b1110011001;
	reg 		i;
	reg 		clk = 0;
	output[1:0] 	o;
	reg[0:31]	j;


	//instantiating 
	MealyPattern machine(.clock(clk), .i(i), .o(o));
	initial 
	begin
		$dumpfile("meal.vcd");
		$dumpvars;
	end

	//!! t2/t1 = 2 has to be true. doesn't work otherwise ¯\_(ツ)_/¯
	always #1 clk = !clk; //t1
	
	initial begin
	for (j = 0; j < 10; j=j+1) begin
		i = str[j];
		#2 //t2 - wait for the combi. circuit
		$display ("input: %b, output: %2b",i,o);
	end


	#1 $finish;
	end

endmodule