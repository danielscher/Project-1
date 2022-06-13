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
					dojump = 0;
					case (funct)
						6'b100001: alucontrol = // TODO // addition unsigned
						6'b100011: alucontrol = // TODO // subtraction unsigned
						6'b100100: alucontrol = // TODO // and
						6'b100101: alucontrol = // TODO // or
						6'b101011: alucontrol = // TODO // set-less-than unsigned
						default:   alucontrol = // TODO // undefined
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
					alucontrol = // TODO // Effective address: Base register + offset
				end
			6'b000100: // Branch Equal
				begin
					regwrite = 0;
					destreg = 5'bx;
					alusrcbimm = 0;
					dobranch = zero; // Equality test
					memwrite = 0;
					memtoreg = 0;
					dojump = 0;
					alucontrol = // TODO // Subtraction
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
					alucontrol = // TODO // Addition
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
					alucontrol = // TODO
				end
			default: // Default case
				begin
					regwrite = 1'bx;
					destreg = 5'bx;
					alusrcbimm = 1'bx;
					dobranch = 1'bx;
					memwrite = 1'bx;
					memtoreg = 1'bx;
					dojump = 1'bx;
					alucontrol = // TODO
				end
		endcase
	end
endmodule

