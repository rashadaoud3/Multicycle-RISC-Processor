module ALUControl(
    input wire [5:0] opcode,    // Opcode (e.g., for R-type or I-type instructions)
    input wire [5:0] funct,     // Funct field (only for R-type instructions)
    output reg [3:0] alu_op     // ALU operation control signals
);

always @* begin
    case (opcode)
        6'b000000: begin  // R-type instructions
            case (funct)
                6'b100000: alu_op = 4'b0010; // ADD
                6'b100010: alu_op = 4'b0110; // SUB
                6'b100100: alu_op = 4'b0000; // AND
                6'b100101: alu_op = 4'b0001; // OR
                default: alu_op = 4'b1111;   // Invalid funct code
            endcase
        end
        6'b001000: alu_op = 4'b0010; // ADD for I-type (addi)
        6'b100011: alu_op = 4'b0010; // ADD for I-type (lw)
        6'b101011: alu_op = 4'b0010; // ADD for I-type (sw)
        default: alu_op = 4'b1111;   // Invalid opcode
    endcase
end

endmodule
