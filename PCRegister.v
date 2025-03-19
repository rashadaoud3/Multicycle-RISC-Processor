module PCreg(
    input clk,	
    input wire en,
    input wire [15:0] in,
    output reg [15:0] out
);

reg [15:0] register;

initial begin
    register = 16'b0;  // Initialize register to zero
end		  	

// On rising edge of clock, update register if 'en' is high
always @(posedge clk) begin
    if (en) begin
        register <= in;  // Use non-blocking assignment for proper sequential behavior
    end	  
end		  	

// Continuously drive the 'out' with the value of 'register'
always @* begin
    out = register; 
end

endmodule










module tb_PCre;
    reg clk, en;
    reg [15:0] in;
    wire [15:0] out;

    // Instantiate the PCreg module
    PCreg uut (
        .clk(clk),
        .en(en),
        .in(in),
        .out(out)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // Clock period of 10 time units
    end

    // Test sequence
    initial begin
        en = 1; in = 16'h0001;  // Enable register and load value 0x0001
        #10 in = 16'h0002;  // Change input to 0x0002 after 10 time units
        #10 en = 0; in = 16'h0003;  // Disable register, don't load new value
        #10 en = 1; in = 16'h0004;  // Re-enable register and load value 0x0004
        #10 $finish;  // End simulation
    end
endmodule


