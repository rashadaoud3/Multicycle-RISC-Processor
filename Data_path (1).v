module tb_datapath;
	// Inputs
	reg clk;
	reg rst;
	// Outputs
	wire [2:0] State;

	
	data_path uut (
		.clock(clk), 
		.rst(rst), 
		.State(State)
	);

	always #5 assign clk = ~clk;

	initial begin
		// Initialize Inputs
		clk = 0;
		rst = 1;

		// Wait 100 ns for global reset to finish
		#50;
		rst = 0;
        
		 
		
		
		
		
		

	end
      
endmodule
	







module data_path(

		input clock,
		input rst,
		output [2:0] State
    );
	
	// control wires
	wire IRWrite;
	wire PCWrite;
	wire ALUZeroCond;
	wire PCWriteCond;
	wire IorD;
	wire MemWrite;
	wire MemToReg;
	wire ReadACond;
	wire RegWrite;
	wire ALUSrcA;
	wire NNR;
	wire  MemRead;
	wire RegDst; 
	wire out_and; 
	wire out_or;
	wire [1:0] ReadBCond;
	wire [2:0] PCSrc;
	wire [1:0] ALUOp;
	wire [1:0] ALUSrcB;
	wire [15:0] alu_res;
	wire [15:0] alu_out;
	wire [15:0] jump_out;
	wire [15:0] RR_out;
	wire [15:0] For_out; 
	wire [15:0] out_mux5;
	wire[15:0] out_pc; 
	wire [15:0] address;
	wire[15:0] write_memory;
	wire[15:0] data_memory;
	wire[15:0] instr; 
	wire [3:0] opcod;
	wire [2:0]reg_s,reg_t,reg_d,funct;
	wire[5:0] immed;
	wire [8:0] j_imm,j_imm_out; 
	wire [15:0]data_MDR;
	wire [2:0]spf_out; 
	wire [2:0]m2_out; // output mux spf reg
	wire [2:0]m3_out;
	wire [15:0]m4_out; 
	wire [15:0] busA_out,busB_out; // out of reg file  
	wire [15:0] regA_out,regB_out;
	wire [15:0]m5_out,m6_out;
	wire[15:0] Extender_out,shift_out; 
	wire [2:0] Ctrl_out;  // out of alu control
	
	
						  
	
		module reg_alu_out(
			input clk,
			input rst,
			input [15:0] ALUIn,
			output reg [15:0] ALUOut
	    );
		 
		always @(posedge clk)
			begin
			if (rst)
				begin
				ALUOut <= 16'd0;
				end
			else
				begin
				ALUOut <= ALUIn;
				end
			end
			
	endmodule			 

	
	module mux5to1 (
    input wire data0, data1, data2, data3, data4,  // 1-bit input data lines
    input wire [2:0] sel,                          // 3-bit select line
    output wire out                                // Output of the multiplexer
);
    // Assign the output based on the select line using a case statement
    assign out = (sel == 3'b000) ? data0 :
                 (sel == 3'b001) ? data1 :
                 (sel == 3'b010) ? data2 :
                 (sel == 3'b011) ? data3 :
                 (sel == 3'b100) ? data4 :
                 1'b0; // Default case for unused select values
endmodule 


	module mux2to1 (
    input wire data0, data1,  // 1-bit input data lines
    input wire  sel,                          // 3-bit select line
    output wire out                                
);
    // Assign the output based on the select line using a case statement
    assign out = (sel == 1'b0) ? data0 :
                 (sel == 1'b1) ? data1 :
                
                 1'b0; // Default case for unused select values
endmodule 



module pc_register (
    input clk,
    input reset,   
	input PCWrite,

    input [15:0] next_pc, 
    output reg [15:0] pc
	
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 16'b0; // Initialize PC to 0 on reset
        end 
			else if (PCWrite)
 begin
            pc <= next_pc; 
        end
    end

endmodule
	   
module Memory (
    input wire clk,
    input wire [15:0] addrs,       // Address for instruction or data
    input wire [15:0] write_data,    // Data to be written to memory
    input wire mem_write,            // Control signal for memory write
    input wire mem_read,             // Control signal for memory read
    output reg [15:0] read_data,     // Data read from memory
    output reg [15:0] instruction    // Instruction read from memory
);

    reg [15:0] memory [255:0];       // 256 words of 16-bit memory

    // Initialize the instruction memory with a test program
    initial begin
        memory[0] = 16'b0011_000_000_000001; // ADDI R0, R0, 1
        memory[1] = 16'b0111_001_000_000010; // SW R1, R0, 2
        memory[2] = 16'b0001_001_000_000000; // ADD R1, R0, R0
        memory[3] = 16'b0110_010_000_000100; // LW R2, R0, 4
        memory[4] = 16'b1010_001_010_000001; // BEQ R1, R2, 1
        memory[5] = 16'b1100_000_000_000011; // JMP to address 3
        memory[6] = 16'b1101_000_000_000100; // CALL to address 4
        memory[7] = 16'b1110_000_000_000000; // RET
        // Additional initialization for data memory if needed
    end

    // Fetch instruction and read/write data
    always @(posedge clk) begin
        if (mem_read) begin
            read_data <= memory[addrs];   // Read data from memory
        end
        if (mem_write) begin
            memory[addrs] <= write_data;  // Write data to memory
        end
        instruction <= memory[addrs];     // Fetch instruction from memory
    end
endmodule	   
// Instruction Register (IR) Module
module IR (
    input wire [15:0] instruction, 
    output reg [3:0] opcode,
    output reg [2:0] rdt,
    output reg [2:0] rs,funct,
    output reg [5:0] imm ,
	output reg [8:0] Jump_Imm  ,

	input clk,
	input rst,
	input IRWrite
);
		always @(posedge clk or posedge rst)
		begin
		if (rst)begin
			opcode <= 4'b0;
			rdt <= 3'b0;
			rs <=3'b0;
			imm <=6'b0;
			Jump_Imm <=9'b0;
			funct<=3'b0;
		end			   
		
		


   else if (IRWrite)  begin
    opcode <=instruction[15:12]; // Extract opcode
    rdt <= instruction[11:9];      // 1 sourse
    rs <= instruction[8:6];      //  2 sour or dest
    imm <= instruction[5:0];      // Extract immediate value
	Jump_Imm <=instruction[11:3]; 
	funct<=instruction[2:0];
end
end
endmodule

module reg_memory_data(
		input clk,
		input rst,
		input [15:0] dataIn,
		output reg [15:0] dataOut
    );
	 
	
	 
	always @(posedge clk)
		begin
		if (rst)
			begin
			dataOut <= 16'hFFFF;
			end
		else
			begin
			dataOut <= dataIn;
			end
		end
		
endmodule 
module spf_reg (
	input clk,
		input rst,
		input [15:0] RT_dec,   // decr by 1
		output reg [15:0] out
    );
	
	   always @(posedge clk)
		begin
		if (rst)
			begin
			out <= 16'hFFFF;
			end
		else
			begin
			out <= RT_dec-1;
			end
		end
	
	endmodule	
	
	module registers(
    input clk,
    input rst,            // Asynchronous reset
    input wire en,        // Enable signal for writing	(reg write)
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


module reg_AB (
	input clk,
		input rst,
		input [15:0]in ,   
		output reg [15:0] out
    );
	
	   always @(posedge clk)
		begin
		if (rst)
			begin
			out <= 16'hFFFF;
			end
		else
			begin
			out <= in;
			end
		end
	
	endmodule
	
module mux4to1 (
    input wire data0, data1, data2, data3,  // 1-bit input data lines
    input wire [1:0] sel,                          // 3-bit select line
    output wire out                                // Output of the multiplexer
);
    // Assign the output based on the select line using a case statement
    assign out = (sel == 2'b00) ? data0 :
                 (sel == 2'b01) ? data1 :
                 (sel == 2'b10) ? data2 :
                 (sel == 2'b11) ? data3 :
                 
                 1'b0; // Default case for unused select values
endmodule 	

module Extender(
	input[5:0] extender_in,
	
	output  reg [15:0] extender_out );
	
	always @* begin 
		if ( extender_in[5])
			extender_out = {10'h3FF, extender_in};
		else 
			extender_out = {10'h000,extender_in};
	end
	
endmodule
	
module ShiftLeft1 (
    input [15:0] in,       // 15-bit input (sign-extended immediate)
    output [15:0] out      // 15-bit output (shifted immediate)
);

    assign out = in << 1;  // Shift left by 1 bit (equivalent to multiplying by 2)

endmodule
	

module ALU (
input [15:0] ALu_in1,Alu_in2,
output reg [15:0] Alu_out,	
output reg zero, 
input [2:0]Alu_control
);

always @(*)
	case(Alu_control[2:0])
		3'b000:Alu_out<= ALu_in1 & Alu_in2;	  
		3'b001:Alu_out<= ALu_in1 + Alu_in2;
		3'b010:Alu_out<= ALu_in1 - Alu_in2;	
	    3'b011:Alu_out<= ALu_in1 << Alu_in2;
		3'b100:Alu_out<= ALu_in1 >> Alu_in2;
		3'b101:	Alu_out<= ALu_in1 | Alu_in2;
		default: Alu_out <= 16'b0;               // Default case 
			
	   endcase 
  assign zero = (Alu_control == 3'b010) ? (Alu_out == 16'b0) : 1'b0; 
				  
		
	  endmodule	
	  
	  
	  
 module Alu_control(
	 output reg[2:0] ALU_Ctrl,
	 input [2:0] funct,
	 input[1:0] Alu_op);	
	 
	 parameter	   // from alu.v
	 AND =3'b000,
	 ADD =3'b001,
	 SUB=3'b010	,
	 SLL=3'b011	,
	 SRL=3'b100;
	
	 
	 
        always @(*) begin

			case (Alu_op)
		    2'b10: begin  // R-type instruction (opcode == 000000) 
		     case (funct)
			   3'b000:ALU_Ctrl=AND;
			   3'b001:ALU_Ctrl=ADD;
			   3'b010:ALU_Ctrl=SUB;
			   3'b11:ALU_Ctrl=SLL;
			   3'b100:ALU_Ctrl=SRL;
			  default: ALU_Ctrl = 3'bxxx; // Undefined

			 endcase
			end  
			// I -type instruction 
			
			2'b11:ALU_Ctrl=AND;
			2'b00:ALU_Ctrl=ADD;
			2'b01:ALU_Ctrl=SUB;
		
		default: ALU_Ctrl = 3'bxxx; // Undefined
						
			
			
			endcase
		end
 endmodule
 
 

 module RR_reg (
	input clk,
		input rst,
		input [15:0]in ,   
		output reg [15:0] out
    );
	
	   always @(posedge clk)
		begin
		if (rst)
			begin
			out <= 16'hFFFF;
			end
		else
			begin
			out <= in+1;
			end
		end
	
	endmodule
	
	
	
	
	
	module main_control(
		
   input [3:0] opcode,   // 6-bit opcode from instruction
   input [2:0] funct,
   input clk,reset,	 
   output reg RegWrite,
   output reg MemRead,
   output reg MemWrite,
   output reg MemToReg,	// Control signal for writing memory data to registers
   output reg ALUSrcA,
   output reg RegDst, 	
   output reg ALUZeroCond,
   output reg [2:0]PCSrc,
   output reg IRWrite,
   output reg [1:0] ALUSrcB,
   output reg [1:0] ALUOp,  // ALU operation control (2 bits)        
   output reg IorD,        // Control signal for memory address source 
   output reg PCWriteCond, 
   output reg NNR, // for loop
   output reg PCWrite   
	   
   );
   
   // States for the multicycle control
    parameter FETCH = 3'b000,
              DECODE = 3'b001,
              EXECUTE = 3'b010,
              MEMORY = 3'b011,
              WRITEBACK = 3'b100; 
			  
   reg [2:0] state, next_state;
  
// Control signals logic based on the state
    always @(*) begin
        // Default values for all control signals
        PCWrite = 0;
        MemRead = 0;
        MemWrite = 0;
		PCWriteCond=0;
        IorD = 0;
        MemToReg = 0;
        IRWrite = 0;
        RegDst = 0;
        RegWrite = 0;
        ALUSrcA = 0;
        ALUSrcB = 2'b00;
        ALUOp = 2'b00;
        PCSrc = 3'b000;
		ALUZeroCond=0;
		
			   case (state)
            FETCH: begin
               PCWrite = 1;
			   IorD=0;
			   MemRead=1;
			   MemWrite =0;
			   IRWrite =1; 
			   PCSrc=3'b000;
			   ALUOp=2'b00;
			   ALUSrcB=2'b01; 
			   ALUSrcA=0;
			   RegWrite=0;
			   
			  
            end	
			
			DECODE: begin	  
				
			   ALUOp=2'b00;
			   ALUSrcB=2'b11;
			   ALUSrcA=0;
			   
			end
			
			EXECUTE: begin
			case (opcode)	
				4'b0000: begin // R-type instruction
					ALUOp=2'b10;
					ALUSrcB=2'b00;
					ALUSrcA=1;
			        RegDst = 1;      
                    RegWrite = 1;     
                    end
					
					4'b0010:begin // ANDI
						
					 RegDst =0;
					 ALUSrcA= 1;
					 ALUSrcB=2'b10;
					 ALUOp=2'b11;
					 RegWrite=1;
					 MemToReg=0;
					end
					
					
					4'b0011:begin // ADDI
						
					 RegDst =0;
					 ALUSrcA= 1;
					 ALUSrcB=2'b10;
					 ALUOp=2'b00;
					 RegWrite=1;
					 MemToReg=0;
					end
					
					
				 4'b0100: begin // LW instruction 
					 ALUOp=2'b00;
					 ALUSrcB=2'b10;
					 ALUSrcA=1;
				 end
				 
				  4'b0101: begin // SW instruction 
					 ALUOp=2'b00;
					 ALUSrcB=2'b10;
					 ALUSrcA=1;
				 end
				 4'b1000:begin// for loop	
					 NNR=0;
					 PCSrc=3'b100;
					 
					 
						
						
					end	
				 
			   	 4'b0110: begin //BEQ	
						
						ALUOp=2'b01;//sub
						ALUSrcB=2'b00;
						ALUSrcA =1;
						PCSrc=3'b001;
						PCWriteCond=1;
						ALUZeroCond=1;
						IorD=0;
						end
			   
			   	   4'b0111:begin //BNE
						ALUOp=2'b01;//sub
						ALUSrcB=2'b00;
						ALUSrcA =1;
						PCSrc=3'b001;
						PCWriteCond=1;
						ALUZeroCond=1;
						IorD=0;
						  
						  
			   			  end
			   	   4'b0001: begin // jump  
				     case (funct)  
						 
				     3'b000:begin  //JMP Offset
			   		 PCSrc=3'b010;
					PCWrite=1;
					end
					
					
					3'b001: begin  //CALL Offset
					 PCSrc=3'b010;
					PCWrite=1;	
					end
					
					
					3'b010:begin   //RET
						PCWrite=1;
						PCSrc=3'b011;
						end
					
						
					
					
					endcase
				 end
			endcase		
            end
			
			
			MEMORY: begin 
				
			case (opcode) 
				
				4'b0100: begin // LW instruction 
					MemRead=1;
					IorD =1;
					IRWrite=0;
				end
				
				
				4'b0101: begin // SW instruction
					MemWrite =1;
					IorD=1;
				end
				
			
			endcase
			end	
			
			  WRITEBACK: begin
		    case (opcode) 
			4'b0000: begin // R-type instruction
					MemToReg=0;
					RegWrite =1;  
					RegDst =1;
					
					
			end
			
			
			4'b0100: begin // LW instruction
				RegWrite=1;
				MemToReg=1;
				RegDst=0;
			end
			endcase
        end	
		endcase
    end
	
	
	// State transition logic
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= FETCH;
        else
            state <= next_state;
    end
				
				
				
		 // Next state logic
    always @(*) begin
        case (state)
            FETCH: next_state = DECODE;
            DECODE: next_state = (opcode == 4'b0100 || opcode == 4'b0101) ? MEMORY : EXECUTE;
            EXECUTE: next_state = (opcode == 4'b0100) ? MEMORY : WRITEBACK;
            MEMORY: next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default: next_state = FETCH;
        endcase
    end

endmodule		
				
				

					
				

					
	
	
	and  gate1(out_and,ALUZeroCond,PCWriteCond);	

	or  gate2(out_or,out_and,PCWrite); 
	
	pc_register reg1(.clk(clock)
	,.reset(rst)
	,.PCWrite(out_or)
	,.next_pc(out_mux5)
	,.pc(out_pc)) ;		 
	
	
	ShiftLeft1 shift2(.in(j_imm),.out(j_imm_out)); 
	assign jump_out={j_imm_out ,out_pc[15:9]};  // jump target
	 assign alu_out=out_pc+Extender_out;//branch target
	
	 mux5to1 mux5 (
	    .data0(alu_res), 
	    .data1(alu_out), 
	    .data2(jump_out), 
	    .data3(RR_out), 
	    .data4(For_out), 
	    .sel(PCSrc), 
	    .out(out_mux5)
	);
	
	mux2to1 m1(.data0(out_pc),
	.data1(alu_out),
	.sel(IorD),
	.out(address)); 
	
	Memory memory (.clk(clock),
	.addrs(address) ,
	.mem_write(MemWrite),
	.mem_read(MemRead),
	.write_data(write_memory),
	.read_data(data_memory),
	.instruction(instr)); 
	
	
	IR reg2(.instruction(instr),
	.opcode(opcod),
	. clk(clock),
	.rst(rst),
	.rdt(reg_t),
	.rs(reg_s),
	.imm(immed),
	.Jump_Imm(j_imm),
	.IRWrite(IRWrite),
	.funct(funct)); 
	
	 reg_memory_data MDR(. clk(clock),
	.rst(rst),
	.dataIn(data_memory),
	.dataOut(data_MDR)); 
	
	spf_reg reg3(. clk(clock),
	.rst(rst),
	.RT_dec(reg_t),
	.out(spf_out));
	
	mux2to1 m2 (.data0(reg_t),.data1(spf_out),.sel(NNR),.out(m2_out));	
	mux2to1 m3 (.data0(m2_out),.data1(reg_d),.sel(RegDst),.out(m3_out));
	mux2to1 m4 (.data0(alu_out),.data1(data_MDR),.sel(MemToReg),.out(m4_out));
	
	
	registers reg_file(. clk(clock),
	.rst(rst),
	.en(RegWrite),
	.RA(reg_s),
	.RB(reg_t),
	.RW(m3_out),
	.BUSW(m4_out),
	.BUSA(busA_out),
	.BUSB(busB_out)
	);
	
	reg_AB reg_A(
	. clk(clock),
	.rst(rst),
	.in(busA_out),
	.out(regA_out)
	);
	
	reg_AB reg_B(
	. clk(clock),
	.rst(rst),
	.in(busB_out),
	.out(regB_out)
	);
	  
	mux2to1 m5 (.data0(out_pc),.data1(regA_out),.sel(ALUSrcA),.out(m5_out));   
	
	Extender extender (.extender_in(immed),.extender_out(Extender_out)); 
	ShiftLeft1 shift1(.in(Extender_out),.out(shift_out));
	
	
	mux4to1 m6 (.data0(regB_out),.data1(1),.data2(Extender_out),.data3(shift_out),.sel(ALUSrcB),.out(m6_out)) ;	
	
	Alu_control alu_ctr(
	.funct(funct),
	.Alu_op(ALUOp),
	.ALU_Ctrl(Ctrl_out)
	
	);
	
	
	ALU alu(
	.ALu_in1(m5_out),
	.Alu_in2(m6_out),
	.Alu_out(alu_res),
	.Alu_control(Ctrl_out),
	.zero(ALUZeroCond)
	
	) ;
	
	reg_alu_out alu_outt(
	. clk(clock),
	.rst(rst),
	.ALUIn(alu_res),
	.ALUOut(alu_out)
	
	); 
	
	RR_reg rr_reg(
	
	.clk(clock),
	.rst(rst),
	.in(out_pc),
	.out(RR_out)
	);
	
	main_control Main_ctr(
	
	   .opcode(opcod),   
	   .funct(funct),
	   .clk(clock),
	   .reset(rst),	 
	   .RegWrite(RegWrite),
	   .MemRead(MemRead),
	   .MemWrite(MemWrite),
	   .MemToReg(MemToReg),	
	   .ALUSrcA(ALUSrcA),
	   .RegDst(RegDst), 	
	   .PCSrc(PCSrc),
	   .IRWrite(IRWrite),
	   .ALUSrcB (ALUSrcB),
	   .ALUOp (ALUOp),        
	   .IorD(IorD),  
	   .NNR(NNR),
	   .ALUZeroCond(ALUZeroCond),
	   .PCWriteCond (PCWriteCond), 
	   .PCWrite (PCWrite)   
		   
	);
	
		
	
	
	
	endmodule