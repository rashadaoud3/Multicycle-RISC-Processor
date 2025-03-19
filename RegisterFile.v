module registers(
    input clk,
    input rst,            // Asynchronous reset
    input wire en,        // Enable signal for writing
    input wire [2:0] RA,  // Read Address A
    input wire [2:0] RB,  // Read Address B
    input wire [2:0] RW,  // Write Address
    input wire [15:0] BUSW,  // Write Data
    output reg [15:0] BUSA,  // Read Data A
    output reg [15:0] BUSB   // Read Data B
);

    // 8 registers of 16 bits
    reg [15:0] register [7:0];  
    integer i;

    // Asynchronous reset (clear registers at start)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i = i + 1) begin
                register[i] <= 16'd0;
            end
        end else if (en && (RW != 3'b000)) begin  // Ensure no write to register 0
            register[RW] <= BUSW;  // Write to register if enabled
        end
    end

    // Read operations (synchronously with the clock)
    always @(posedge clk) begin
        BUSA <= register[RA];  // Output data for read address A
        BUSB <= register[RB];  // Output data for read address B
    end
endmodule

// Testbench for Register File
module tb_registers;
    reg clk, rst, en;
    reg [2:0] RA, RB, RW;
    reg [15:0] BUSW;
    wire [15:0] BUSA, BUSB;

    // Instantiate the Register File
    registers uut (
        .clk(clk),
        .rst(rst),
        .en(en),
        .RA(RA),
        .RB(RB),
        .RW(RW),
        .BUSW(BUSW),
        .BUSA(BUSA),
        .BUSB(BUSB)
    );

    // Clock generation (period of 10)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Stimulus
    initial begin
        // Initialize signals
        rst = 1; en = 1; RW = 3'b000; BUSW = 16'hAAAA;
        #10 rst = 0;  // Release reset

        // Write to register 1 (RW=1)
        #10 RW = 3'b001; BUSW = 16'h5555; 

        // Read from registers
        #10 RA = 3'b001; RB = 3'b000;
        #10 RW = 3'b010; BUSW = 16'h1234; 

        // Read again after writing to register 2 (RW=2)
        #10 RA = 3'b010; RB = 3'b001;
        #10 $finish;
    end
endmodule
