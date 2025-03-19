module extend8(
    input wire [7:0] in,        // 8-bit input value
    output reg [15:0] out       // 16-bit zero-extended output value
);

always @* begin
    out[15:8] = 8'b0;           // Zero-extend by setting the upper 8 bits to 0
    out[7:0] = in[7:0];         // Copy the 8-bit input to the lower 8 bits of the output
end

endmodule
