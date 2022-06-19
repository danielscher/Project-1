module Datapath(
	input         clk, reset,
	input         memtoreg,
	input         dobranch,
	input         alusrcbimm,
	input  [4:0]  destreg,
	input         regwrite,
	input         jump,
	input  [2:0]  alucontrol,
	output        zero,
	output [31:0] pc,
	input  [31:0] instr,
	output [31:0] aluout,
	output [31:0] writedata,
	input  [31:0] readdata
);
	wire [31:0] pc;
	wire [31:0] signimm;
	wire [31:0] srca, srcb, srcbimm;
	wire [31:0] result;
	reg LUISrc;
	wire [31:0] upperimm;

	//LUI Control: check if opcode of LUI instr
	always @* begin
		if (instr[31:26] == 6'b001111) begin
			LUISrc = 1;
		end
		else LUISrc = 0;
	end
	
	// Fetch: Pass PC to instruction memory and update PC
	ProgramCounter pcenv(clk, reset, dobranch, signimm, jump, instr[25:0], pc);

	// Execute:
	// (a) Select operand
	SignExtension se(instr[15:0], signimm);

	//instantiation of LUI
	LoadUpperImmediate LUI (signimm, upperimm);

	/*READ!: next line of code represents an additional MUX between the signimm and the mux for Srcbimm/Srcb
	enabling it will cause the xxx value to be given to the LUI breaking everything. */	

	//EXTENSION: if LUISrc is set signimm is upperimm
	//assign signimm = LUISrc ? upperimm : signimm;

	assign srcbimm = alusrcbimm ? signimm : srcb;
	// (b) Perform computation in the ALU
	ArithmeticLogicUnit alu(srca, srcbimm, alucontrol, aluout, zero);
	// (c) Select the correct result
	assign result = memtoreg ? readdata : aluout;

	// Memory: Data word that is transferred to the data memory for (possible) storage
	assign writedata = srcb;

	// Write-Back: Provide operands and write back the result
	RegisterFile gpr(clk, regwrite, instr[25:21], instr[20:16],
				   destreg, result, srca, srcb);

	
	
endmodule

module ProgramCounter(
	input         clk,
	input         reset,
	input         dobranch,
	input  [31:0] branchoffset,
	input         dojump,
	input  [25:0] jumptarget,
	output [31:0] progcounter
);
	reg  [31:0] pc;
	wire [31:0] incpc, branchpc, nextpc;

	// Increment program counter by 4 (word aligned)
	Adder pcinc(.a(pc), .b(32'b100), .cin(1'b0), .y(incpc));
	// Calculate possible (PC-relative) branch target
	Adder pcbranch(.a(incpc), .b({branchoffset[29:0], 2'b00}), .cin(1'b0), .y(branchpc));
	// Select the next value of the program counter
	assign nextpc = dojump   ? {incpc[31:28], jumptarget, 2'b00} :
					dobranch ? branchpc :
							   incpc;

	// The program counter memory element
	always @(posedge clk)
	begin
		if (reset) begin // Initialize with address 0x00400000
			pc <= 'h00400000;
		end else begin
			pc <= nextpc;
		end
	end

	// Output
	assign progcounter = pc;

endmodule

module RegisterFile(
	input         clk,
	input         we3,
	input  [4:0]  ra1, ra2, wa3,
	input  [31:0] wd3,
	output [31:0] rd1, rd2
);
	reg [31:0] registers[31:0];

	always @(posedge clk)
		if (we3) begin
			registers[wa3] <= wd3;
		end

	assign rd1 = (ra1 != 0) ? registers[ra1] : 0;
	assign rd2 = (ra2 != 0) ? registers[ra2] : 0;
endmodule

module Adder(
	input  [31:0] a, b,
	input         cin,
	output [31:0] y,
	output        cout
);
	assign {cout, y} = a + b + cin;
endmodule

module SignExtension(
	input  [15:0] a,
	output [31:0] y
);
	assign y = {{16{a[15]}}, a};
	initial $display("imm : %b \n signed immediate:%b",a,y);
endmodule

module ArithmeticLogicUnit(
	input  [31:0] a, b,
	input  [2:0]  alucontrol,
	output [31:0] result,
	output        zero
);
reg [31:0] resreg;
reg [31:0] HI ;
reg [31:0] LO ;
wire [63:0] prod;
wire [31:0] wirehi;
wire [31:0] wirelo;


	//encoding of each operation for the alucontrol.
	parameter
				SLT = 3'b000,
				SUB = 3'b001,
				ADD = 3'b101, 
				OR = 3'b110,
				AND = 3'b111,
				MFHI = 3'b010,
				MFLO = 3'b010,
				MUL = 3'b100;

	//assigns the result of each operation to the result output. 
	 always @* begin
		case (alucontrol)
			SLT:	resreg = a < b ? 1 : 0;
			SUB: 	resreg = a - b;
			ADD: 	resreg = a + b;
			OR: 	resreg = a | b;
			AND: 	resreg =  a & b;
			MUL: 	{HI,LO} = a * b; //stores two halves of prod in HI,LO
			MFLO:	resreg = wirelo; //connects LO to output.
			MFHI: 	resreg = wirehi; //connect HI to output.
		endcase
	 end

	//connects HI,LO regs to respc. wires
	assign wirehi = HI;
	assign wirelo = LO;
	//connect the result reg to the result output
	assign result = resreg;
	//zero is 1 iff result is 32'b0.
	assign zero = result ? 0 : 1;

endmodule
module LoadUpperImmediate (
	input [31:0] signimm,
	output [31:0] upperimm
);
	reg [31:0] res;
	//concatenates 16 zero to the unsigned imm. 
	assign upperimm = {signimm[15:0], {16{1'b0}}};
	initial $display("upper immediate:%b",upperimm);

endmodule