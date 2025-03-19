// Instruction Register (IR) Module
module IR (
    input wire [15:0] instruction, 
    output reg [3:0] opcode,
    output reg mode,
    output reg [2:0] rd,
    output reg [2:0] rs1,
    output reg [4:0] imm
);

always @(*) begin
    opcode = instruction[15:12]; // Extract opcode
    mode = instruction[11];      // Extract mode bit
    rd = instruction[10:8];      // Extract destination register
    rs1 = instruction[7:5];      // Extract source register 1
    imm = instruction[4:0];      // Extract immediate value
end
endmodule

// Testbench for Instruction Register
module tb_IR;
    reg [15:0] instruction;
    wire [3:0] opcode;
    wire mode;
    wire [2:0] rd;
    wire [2:0] rs1;
    wire [4:0] imm;

    IR uut (
        .instruction(instruction),
        .opcode(opcode),
        .mode(mode),
        .rd(rd),
        .rs1(rs1),
        .imm(imm)
    );

    initial begin
        $monitor("instruction=%h, opcode=%b, mode=%b, rd=%b, rs1=%b, imm=%b", 
                 instruction, opcode, mode, rd, rs1, imm);
        instruction = 16'b0011_000_101_00001; // Example instruction
        #10 instruction = 16'b0110_001_110_00110; // Another example
        #10 $finish;
    end
endmodule
