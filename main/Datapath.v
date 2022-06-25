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
	//wire [31:0] pc;
	wire [31:0] signimm;
	wire [31:0] srca, srcb, srcbimm;
	wire [31:0] result;
	
	//necessary for the extension of the datapath.
	reg LUISrc,DivMulSrc;
	wire [31:0] upperimm;
	wire JR;
	wire [2:0] alu2control;
	wire [31:0] alu2out;
	wire divBusy;

	//LUI Control: check if opcode of LUI instr
	always @* begin
		if (instr[31:26] == 6'b001111) begin
			LUISrc = 1;
		end
		else LUISrc = 0;
	end


	//decoder for the MUL, DIV, MFHI, MFLO.
	wire alu2Src;
	Alu2Decoder muldivdecoder(instr,alu2control,alu2Src);
	//Jump Register Control unit
	//controls the jumptarget in case of a JR.
	//since decoder doesn't handle the jumptarget.  
	JRControl jrc(instr,JR);
	wire [31:0] c = (pc);
	// Fetch: Pass PC to instruction memory and update PC
	ProgramCounter pcenv(clk, reset, dobranch, signimm, (divBusy ? 1'b1 : jump), (JR ? srca[27:2] : (divBusy ? c[27:2] : instr[25:0])),pc);
	// Execute:
	// (a) Select operand
	SignExtension se(instr[15:0], signimm);

	//instantiation of LUI
	LoadUpperImmediate LUI (signimm, upperimm);	

	//EXTENSION: if LUISrc is set to 1 and alusrcbimm then alu gets the upperimm otherwise the signimm or the rd2.
	assign srcbimm = alusrcbimm ? (LUISrc ? upperimm : signimm) : srcb;
	// (b) Perform computation in the ALU
	ArithmeticLogicUnit alu(srca, srcbimm, alucontrol, aluout, zero);
	ArithmeticLogicUnit2 alu2(clk, alu2control, srca, srcbimm, alu2out,divBusy);
	// (c) Select the correct result
	//EXTENSION: if memtoreg and jump are sets takes the next pc.
	assign result = memtoreg ? readdata : (jump ? (pc+4) : (alu2Src ? alu2out : aluout)); // modified this line to chose between divalu out and alu out

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
endmodule

module ArithmeticLogicUnit(
	input  [31:0] a, b,
	input  [2:0]  alucontrol,
	output [31:0] result,
	output        zero
);
reg [31:0] resreg;
wire [63:0] prod;
wire [31:0] wirehi;
wire [31:0] wirelo;


	//encoding of each operation for the alucontrol.
	parameter
				SLT = 3'b000,
				SUB = 3'b001,
				ADD = 3'b101, 
				OR = 3'b110,
				AND = 3'b111;

	//assigns the result of each operation to the result output. 
	 always @* begin
		case (alucontrol)
			SLT:	resreg = a < b ? 1 : 0;
			SUB: 	resreg = a - b;
			ADD: 	resreg = a + b;
			OR: 	resreg = a | b;
			AND: 	resreg =  a & b;
		endcase
	 end


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
endmodule
module JRControl (
	input [31:0] instr,
	output JR
);

assign JR = (instr[5:0] == 6'b001000);
	
endmodule


module Alu2Decoder (
	input [31:0] instr,
	output reg [2:0] alu2control,
	output reg DivMulSrc
);
wire [5:0] op = instr[31:26];
wire [5:0] funct = instr[5:0];

always @*
	begin
		case (op)
			6'b000000: // R-type instruction
				begin
					case (funct)
						6'b011001:begin
							alu2control = 3'b001; // multiplication (MULTU)
							DivMulSrc = 1;
						end 
						6'b011011:begin
							alu2control = 3'b010; // division (DIVU)
							DivMulSrc = 1;
						end 
						6'b010000:begin
							DivMulSrc = 1;
							alu2control = 3'b011; // mfhi
						end 
						6'b010010:begin
							DivMulSrc = 1;
							alu2control = 3'b100; // mflo
						end 
						default:begin
							alu2control = 3'b000; // undefined //
							DivMulSrc = 0;
						end 
					endcase
				end
			default: begin 
				alu2control = 3'b000;
				DivMulSrc = 0;
			end
		endcase

	end
	
endmodule

module ArithmeticLogicUnit2 (
	input clk,
	input [2:0] control,
	input [31:0] a,
	input [31:0] b,
	output [31:0] out,
	output reg divBusy
);
reg start;
reg [31:0] HI;
reg [31:0] LO;
reg [31:0] res;
wire [31:0] wirehi,wirelo,qlo,rhi;
reg [31:0] templo;
reg [31:0] temphi;

Division div(clk,start,a,b,qlo,rhi,busy);

always @(*) begin
		case (control)
			1: 	begin
				divBusy = 0;
				start = 0;
				{HI,LO} = a * b; //stores two halves of prod in HI,LO
			end
			2:	begin
				start <= 1;
				divBusy <= 0;
				end
			3:	begin 
				divBusy = busy;
				start = 0;
				res = wirelo;
			end //connects LO to output.
			4:begin
				divBusy = busy;
				start = 0;
				res = wirehi; //connect HI to output.
			end 	
			default: begin
				divBusy <= 0;
				start <= 0;
			end
		endcase
		if (busy) {HI,LO} <= {rhi,qlo};
		
end


assign wirelo = LO;
assign wirehi = HI;
assign out = res;

endmodule

module Division(
    input         clock,
    input         start,
    input  [31:0] a,
    input  [31:0] b,
    output [31:0] q,
    output [31:0] r,
	output reg busy
);
    parameter n = 31;
	reg [31:0] r_curr;
    reg [31:0] i;
    reg [31:0] sharedReg;
    reg [31:0] B;
    reg [31:0] A;
    
    initial begin
		i = -1;
        r_curr = 0;
        sharedReg = 0;
		busy = 0;
        //B = b;
    end

    always @(posedge clock && start) begin
        sharedReg = a;
        i = 31;
        B = b;
        A = a;
        r_curr = 0;
		busy = 1;
    end

	always @(i > 31) begin
		busy <= 0;
	end

    always @(posedge clock && (i < 32)) begin

        r_curr = r_curr << 1;
		r_curr = r_curr + sharedReg[i];

        if (r_curr < B) begin
            sharedReg[i] = 0;
        end else begin
            sharedReg[i] = 1;
            r_curr = r_curr - B;

        end
        i = i - 1;
    end
    assign q = sharedReg;
    assign r = r_curr;

endmodule
