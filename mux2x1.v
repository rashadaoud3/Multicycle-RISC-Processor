module mux2x1(
    input wire [15:0] a,       // First input (16-bit)
    input wire [15:0] b,       // Second input (16-bit)
    input wire sel,            // Select signal (1-bit)
    output wire [15:0] out     // Output (16-bit)
);

assign out = (sel == 1'b0) ? a : b; // If sel is 0, select 'a'; if sel is 1, select 'b'

endmodule
