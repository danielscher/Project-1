module Decoder(
	input     [31:0] instr,      // Instruction word
	input            zero,       // Does the current operation in the datapath return 0 as result?
	output reg       memtoreg,   // Use the loaded word instead of the ALU result as result
	output reg       memwrite,   // Write to the data memory
	output reg       dobranch,   // Perform a relative jump
	output reg       alusrcbimm, // Use the immediate value as second operand
	output reg [4:0] destreg,    // Number of the target register to (possibly) be written 
	output reg       regwrite,   // Write to the target register
	output reg       dojump,     // Perform an absolute jump
	output reg [2:0] alucontrol  // ALU control bits
);
	// Extract the primary and secondary opcode
	wire [5:0] op = instr[31:26];
	wire [5:0] funct = instr[5:0];

	always @*
	begin
		case (op)
			6'b000000: // R-type instruction
				begin
					regwrite = 1;
					destreg = instr[15:11];
					alusrcbimm = 0;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = (funct == 6'b001000) ? 1 : 0;
						case (funct)
							6'b100001: alucontrol = 3'b101; // addition
							6'b100011: alucontrol = 3'b001; // substruction
							6'b100100: alucontrol = 3'b111; // logical and
							6'b100101: alucontrol = 3'b110; // logical or
							6'b101011: alucontrol = 3'b000; // set less than
							//6'b011001: alucontrol = 3'b100; // multiplication
							//6'b010000,
							//6'b010010: alucontrol = {3'b01,funct[1]}; // mflo,mfhi (retrieves lo ,hi)
							default:   alucontrol = 3'b101; // undefined //
						endcase
				end
			6'b100011, // Load data word from memory
			6'b101011: // Store data word
				begin
					regwrite = ~op[3];
					destreg = instr[20:16];
					alusrcbimm = 1;
					dobranch = 0;
					memwrite = op[3];
					memtoreg = 1;
					dojump = 0;
					alucontrol = 3'b101;// TODO: check // Effective address: Base register + offset
				end
			6'b000101,
			6'b000100: // Branch Equal
				begin
					regwrite = 0;
					destreg = 5'bx;
					alusrcbimm = 0;
					dobranch = zero ^ op[0] ; // Equality test 
					memwrite = 0;
					memtoreg = 0;
					dojump = 0;
					alucontrol =3'b001; // Subtraction
				end
			6'b001001: // Addition immediate unsigned
				begin
					regwrite = 1;
					destreg = instr[20:16];
					alusrcbimm = 1;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = 0;
					alucontrol = 3'b101; // Addition
				end
			6'b000010: // Jump immediate
				begin
					regwrite = 0;
					destreg = 5'bx;
					alusrcbimm = 0;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = 1;
					alucontrol = 5'bx;  
				end


			//---------------------extended part---------------------
			6'b001111: // Load Upper Immediate   lui $t1 111 -> 111 || 0**16 
				begin 
					regwrite = 1;
					destreg = instr[20:16];
					alusrcbimm = 1;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = 0;
					alucontrol = 3'b101;
				end
			6'b001101: // Or Immediate (bitwise OR against an imm)
				begin
					regwrite = 1;
					destreg = instr[20:16];
					alusrcbimm = 1;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = 0;
					alucontrol = 3'b110;
				end
			6'b000011: //Jump and Link (jal)
				begin
					regwrite = 1;
					destreg = 5'd31; // stores current pc on reg #31
					alusrcbimm = 0;
					dobranch = 0;
					memwrite = 0;
					memtoreg = 0;
					dojump = 1;
					alucontrol = 5'bx;
				end
			//---------------------extended part end---------------------


			default: // Default case
				begin
					regwrite = 1'bx;
					destreg = 5'bx;
					alusrcbimm = 1'bx;
					dobranch = 1'bx;
					memwrite = 1'bx;
					memtoreg = 1'bx;
					dojump = 1'bx;
					alucontrol = 3'b101; // TODO
				end
		endcase
	end
endmodule

