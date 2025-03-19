module mux3x1(
    input wire [15:0] a,       // First input (16-bit)
    input wire [15:0] b,       // Second input (16-bit)
    input wire [15:0] c,       // Third input (16-bit)
    input wire [1:0] sel,      // 2-bit select signal
    output wire [15:0] out     // Output (16-bit)
);

assign out = (sel == 2'b00) ? a :  // If sel is 00, select 'a'
             (sel == 2'b01) ? b :  // If sel is 01, select 'b'
             c;                     // If sel is 10 or 11, select 'c'

endmodule
