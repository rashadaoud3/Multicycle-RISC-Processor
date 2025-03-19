 module ControlUnit(
	input wire clk,
	input wire [3:0] opcode,
	input wire m,
	output reg PCen,
	output reg memWrite,
	output reg memRead,
	output reg REGen,
	output reg sign5,
	output reg wrByte,
	output reg signW2B,
	output reg [1:0] RA,
	output reg [1:0] RB,
	output reg [1:0] RW,
	output reg [1:0] PCsrc,
	output reg [1:0] BUSWsrc,
	output reg opAsrc,
	output reg [1:0] opBsrc,
	output reg dataInsrc,
	output reg [4:0] Function, 
	output wire [2:0] R7
);	

assign R7 = 3'b111;  // R7 register is always 3'b111

// Define opcodes for different instructions
parameter AND = 5'b00000;
parameter ADD = 5'b00010;
parameter SUB = 5'b00100;
parameter ADDI = 5'b00110;
parameter ANDI = 5'b01000;
parameter LW = 5'b01010;
parameter LBu = 5'b01100;
parameter LBs = 5'b01101;
parameter SW = 5'b01110;
parameter BGT = 5'b10000;
parameter BGTZ = 5'b10001;
parameter BLT = 5'b10010;
parameter BLTZ = 5'b10011;
parameter BEQ = 5'b10100;
parameter BEQZ = 5'b10101;
parameter BNE = 5'b10110;
parameter BNEZ = 5'b10111;
parameter JMP = 5'b11000;
parameter CALL = 5'b11010;
parameter RET = 5'b11100;
parameter Sv = 5'b11110;	

// Define stages
parameter start = 0;
parameter RS = 1;
parameter IF = 2;
parameter ID = 3;
parameter EX = 4;
parameter MEM = 5;
parameter WB = 6;

reg [2:0] stage = start;
reg [4:0] temp;

always @(posedge clk) begin
	case (stage)
		start: begin
			PCen <= 0;
			memWrite <= 0;
			memRead <= 0;
			REGen <= 0;
			stage <= IF;
		end
		
		RS: begin
			PCen <= 1;
			memWrite <= 0;
			memRead <= 0;
			REGen <= 0;
			stage <= IF;
		end	
		
		IF: begin
			PCen <= 0;
			memWrite <= 0;
			memRead <= 0;
			REGen <= 0;
			stage <= ID;
		end	
		
		ID: begin
			PCen <= 0;
			PCsrc <= 2'b00;
			memWrite <= 0;
			memRead <= 0;
			REGen <= 0;
			
			temp = {opcode, 1'b0};
			
			if (opcode[3:0] == 4'b0110 || (opcode[3:0] > 4'b0111 && opcode[3:0] < 4'b1100)) begin
				temp[0] = m;
			end
				
			case(temp)
				AND: begin
					REGen <= 0;
					RA <= 2'b00;
					RB <= 2'b00;
					RW <= 2'b00;
					BUSWsrc <= 2'b00;
					opAsrc <= 0;
					opBsrc <= 2'b00;
					Function <= temp;
					stage <= EX;
				end
				
				ADD: begin
					REGen <= 0;
					RA <= 2'b00;
					RB <= 2'b00;
					RW <= 2'b00;
					BUSWsrc <= 2'b00;
					opAsrc <= 0;
					opBsrc <= 0;
					Function <= temp;
					stage <= EX;
				end	
				
				SUB: begin
					REGen <= 0;
					RA <= 2'b00;
					RB <= 2'b00;
					RW <= 2'b00;
					BUSWsrc <= 2'b00;
					opAsrc <= 0;
					opBsrc <= 2'b00;
					Function <= temp;
					stage <= EX;
				end	
				
				// Handle ADDI and other instructions...
				// This structure is consistent with your original code
				
				default: begin
					// Default handling for unimplemented or undefined instructions
				end
			endcase	
		end
		
		EX: begin  
			case (temp)
				AND: begin
					REGen <= 1;
					stage <= WB;
				end
				
				ADD: begin
					REGen <= 1;
					stage <= WB;
				end	
				
				SUB: begin
					REGen <= 1;
					stage <= WB;
				end
				
				// Handle other instruction executions...
				
				default: begin
					// Handle unexpected cases if necessary
				end
			endcase
		end
		
		MEM: begin 
			memWrite <= 0;
			memRead <= 0;
			case (temp)
				LW: begin
					stage <= WB;
				end
				// Handle other memory operations...
				
				default: begin
					// Handle unexpected cases
				end
			endcase
		end
		
		WB: begin
			REGen <= 0;
			stage <= RS;
		end
	endcase
end

endmodule
