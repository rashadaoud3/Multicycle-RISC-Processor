// Updated ALU module for 16-bit RISC Processor
module ALU (
	input wire [15:0] operandA,
	input wire [15:0] operandB,   
	input wire [3:0] Function, // Adjusted function width to match ISA (4 bits for opcode, 3 bits for R-type function)
	output reg [15:0] res,
	output reg zero, // Zero flag for comparison instructions
	output reg takeBranch
);        

// R-type operations
parameter AND = 4'b0000;
parameter ADD = 4'b0001;
parameter SUB = 4'b0010;
parameter SLL = 4'b0011; // Shift left logical
parameter SRL = 4'b0100; // Shift right logical

// I-type operations
parameter ADDI = 4'b0011; // ADD immediate
parameter ANDI = 4'b0010; // AND immediate
parameter LW = 4'b0101;   // Load word
parameter SW = 4'b0110;   // Store word
parameter BEQ = 4'b0111;  // Branch if equal
parameter BNE = 4'b1000;  // Branch if not equal

// J-type operations
parameter JMP = 4'b0001;  // Jump
parameter CALL = 4'b0010; // Call
parameter RET = 4'b0011;  // Return

always @* begin     
	res = 16'b0;
	takeBranch = 1'b0;
	zero = 1'b0;

	case(Function)
		// R-type operations
		AND: res = operandA & operandB;
		ADD: res = operandA + operandB;
		SUB: res = operandA - operandB;
		SLL: res = operandA << operandB[3:0];
		SRL: res = operandA >> operandB[3:0];

		// I-type operations
		ADDI: res = operandA + operandB;
		ANDI: res = operandA & operandB;
		LW: res = operandA + operandB; // Address calculation for load
		SW: res = operandA + operandB; // Address calculation for store
		BEQ: takeBranch = (operandA == operandB) ? 1 : 0;
		BNE: takeBranch = (operandA != operandB) ? 1 : 0;

		// J-type operations
		JMP: res = operandB; // Jump to address in operandB
		CALL: res = operandB; // Call subroutine (store return address externally)
		RET: res = operandA; // Return to address in operandA

		default: res = 16'b0; // Default case for unsupported operations
	endcase

	// Zero flag for ALU results
	zero = (res == 16'b0) ? 1 : 0;
end
endmodule
