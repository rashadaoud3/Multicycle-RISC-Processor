module extend12(
    input wire [11:0] in,        // 12-bit input value
    output reg [15:0] out       // 16-bit zero-extended output value
);

always @* begin
    out[15:12] = 4'b0;           // Zero-extend by setting the upper 4 bits to 0
    out[11:0] = in[11:0];        // Copy the 12-bit input to the lower 12 bits of the output
end

endmodule

