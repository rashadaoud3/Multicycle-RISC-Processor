module PCinc(
    input wire [15:0] inputPC,
    output reg [15:0] outPC
);

always @* begin
    // Increment inputPC by 2 for each clock cycle
    outPC = inputPC + 2;
end

endmodule






module tb_PCinc;
    reg [15:0] inputPC;
    wire [15:0] outPC;

    // Instantiate the PCinc module
    PCinc uut (
        .inputPC(inputPC),
        .outPC(outPC)
    );

    initial begin
        // Initialize inputPC and simulate changes
        inputPC = 16'h0000;  // Start with PC = 0x0000
        #10 inputPC = 16'h0002; // After 10 time units, inputPC becomes 0x0002
        #10 inputPC = 16'h0004; // After another 10 time units, inputPC becomes 0x0004
        #10 $finish;  // Finish simulation
    end

    initial begin
        // Monitor inputPC and outPC to see changes
        $monitor("Time: %0t, inputPC: %h, outPC: %h", $time, inputPC, outPC);
    end
endmodule
