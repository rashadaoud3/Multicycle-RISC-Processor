// Updated Data Memory Module for 16-bit RISC Processor
module DataMemory (
    input wire clk,
    input wire [15:0] address,
    input wire [15:0] datain,
    input wire memread,
    input wire memwrite, 
    output reg [15:0] dataout
);             

reg [15:0] memory [255:0]; // 256 words of 16-bit data memory

// Initialize memory with zeros
initial begin
    integer i;
    for (i = 0; i < 256; i = i + 1) begin
        memory[i] = 16'd0;
    end
end

// Memory write logic
always @(posedge clk) begin
    if (memwrite) begin
        memory[address] <= datain;
    end
end       

// Memory read logic
always @(*) begin
    if (memread) begin
        dataout = memory[address];
    end else begin
        dataout = 16'd0;
    end
end

endmodule
