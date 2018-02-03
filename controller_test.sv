module controller_test();
	logic         clk;
	logic         reset;
	logic [31:12] Instr;
	logic [3:0]   ALUFlags;
	logic         PCWrite;
	logic         MemWrite;
	logic         RegWrite;
	logic         IRWrite;
	logic         AdrSrc;
	logic [1:0]   RegSrc;
	logic [1:0]   ALUSrcA;
	logic [1:0]   ALUSrcB;
	logic [1:0]   ResultSrc;
	logic [1:0]   ImmSrc;
	logic [1:0]   ALUControl;


	controller controller1(clk,reset,Instr,ALUFlags,PCWrite,MemWrite,RegWrite,IRWrite,
				AdrSrc,RegSrc,ALUSrcA,ALUSrcB,ResultSrc,ImmSrc,ALUControl);

	always
		begin
			clk <= 1; # 10; clk <= 0; #10;
		end

	initial

		begin
			reset <= 1; # 22; reset <= 0;

			//SUB R0, R15, R15
			Instr = 20'b1110_000_0010_0_1111_0000;
			ALUFlags = 4'b0000;
			//executed correctly!

			#70
			//ADD R2, R0, #5
			Instr = 20'b1110_001_0100_0_0000_0010;
			ALUFlags = 4'b0000;

			#80
			//LDR R2, [R0, #96]
			Instr = 20'b1110_010_1100_1_0000_0010;
			ALUFlags = 4'b0000;

			#100
			//BEQ
			Instr = 20'b0000_1010_0000_0000_0000;
			ALUFlags = 4'b0100;// Z = 1 and Branch shouldn't taken

			// //B
			// Instr = 20'b1110_1010_0000_0000_0000;
			// ALUFlags = 4'b0000;

			#60
			//STR R2, [R0, #100]
			Instr = 20'b1110_010_1100_0_0000_0010;

			#80
			//AND R5, R3, R4
			Instr = 20'b1110_000_0000_0_0011_0101;

			#80
			//ORR R4, R7, R2
			Instr = 20'b1110_000_1100_0_0111_0100;
			//570ns is enough and all instructions completed

		end
endmodule