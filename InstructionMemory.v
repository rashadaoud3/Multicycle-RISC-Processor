// Updated Instruction Memory Module for 16-bit RISC Processor
module InstructionMemory (
    input wire [15:0] inputPC,
    output reg [15:0] instruction
);

reg [15:0] memory [255:0]; // 256 words of 16-bit instruction memory

initial begin
    // Initialize the instruction memory with a test program
    memory[0] = 16'b0011_000_000_000001; // ADDI R0, R0, 1
    memory[1] = 16'b0111_001_000_000010; // SW R1, R0, 2
    memory[2] = 16'b0001_001_000_000000; // ADD R1, R0, R0
    memory[3] = 16'b0110_010_000_000100; // LW R2, R0, 4
    memory[4] = 16'b1010_001_010_000001; // BEQ R1, R2, 1
    memory[5] = 16'b1100_000_000_000011; // JMP to address 3
    memory[6] = 16'b1101_000_000_000100; // CALL to address 4
    memory[7] = 16'b1110_000_000_000000; // RET
end

// Fetch the instruction from memory
always @(*) begin
    instruction = memory[inputPC[15:0]];
end
endmodule

// Testbench for Instruction Memory
module tb_InstructionMemory;
    reg [15:0] inputPC;
    wire [15:0] instruction;

    InstructionMemory uut (
        .inputPC(inputPC),
        .instruction(instruction)
    );

    initial begin
        $monitor("inputPC=%h, instruction=%h", inputPC, instruction);
        inputPC = 16'h0000;
        #10 inputPC = 16'h0001;
        #10 inputPC = 16'h0002;
        #10 inputPC = 16'h0003;
        #10 $finish;
    end
endmodule
