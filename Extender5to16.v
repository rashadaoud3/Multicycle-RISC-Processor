module extend5(
    input wire [4:0] in,        // 5-bit input value
    input wire sign,            // Sign extension flag (1 for sign-extend, 0 for zero-extend)
    output reg [15:0] out       // 16-bit output value
);

always @* begin
    if (sign) begin
        // Sign extension: replicate the sign bit (in[4]) to extend the 5-bit value to 16 bits
        out[4:0] = in[4:0];         // Copy the 5-bit input to the lower part of the output
        out[15:5] = {11{in[4]}};    // Extend the sign bit (in[4]) to the upper part of the output
    end
    else begin
        // Zero extension: zero-extend the 5-bit input to 16 bits
        out[15:5] = 11'b0;          // Fill the upper 11 bits with zeros
        out[4:0] = in[4:0];         // Copy the 5-bit input to the lower part of the output
    end
end

endmodule
