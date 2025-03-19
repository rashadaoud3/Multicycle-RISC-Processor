module mux4x1(
    input wire [15:0] a,       // First input (16-bit)
    input wire [15:0] b,       // Second input (16-bit)
    input wire [15:0] c,       // Third input (16-bit)
    input wire [15:0] d,       // Fourth input (16-bit)
    input wire [1:0] sel,      // 2-bit select signal
    output wire [15:0] out     // Output (16-bit)
);

assign out = (sel == 2'b00) ? a :  // If sel is 00, select 'a'
             (sel == 2'b01) ? b :  // If sel is 01, select 'b'
             (sel == 2'b10) ? c :  // If sel is 10, select 'c'
             d;                     // If sel is 11, select 'd'

endmodule
