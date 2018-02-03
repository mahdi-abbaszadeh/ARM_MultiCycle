// arm_multi.sv
// Multi-cycle implementation of a subset of ARMv4

// 16 32-bit registers
// Data-processing instructions
//   ADD, SUB, AND, ORR
//   INSTR <cond> <S> <Rd>, <Rn>, #immediate
//   INSTR <cond> <S> <Rd>, <Rn>, <Rm>
//    Rd <- <Rn> INSTR <Rm>	    	if (S) Update Status Flags
//    Rd <- <Rn> INSTR immediate	if (S) Update Status Flags
//   Instr[31:28] = cond
//   Instr[27:26] = Op = 00
//   Instr[25:20] = Funct
//                  [25]:    1 for immediate, 0 for register
//                  [24:21]: 0100 (ADD) / 0010 (SUB) /
//                           0000 (AND) / 1100 (ORR)
//                  [20]:    S (1 = update CPSR status Flags)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:8]  = 0000
//   Instr[7:0]   = immed_8  (for #immediate type) / 
//                  0000<Rm> (for register type)
//   
// Load/Store instructions
//   LDR, STR
//   INSTR <Rd>, [<Rn>, #offset]
//    LDR: Rd <- Mem[<Rn>+offset]
//    STR: Mem[<Rn>+offset] <- Rd
//   Instr[31:28] = cond
//   Instr[27:26] = Op = 01 
//   Instr[25:20] = Funct
//                  [25]:    0 (A)
//                  [24:21]: 1100 (P/U/B/W)
//                  [20]:    L (1 for LDR, 0 for STR)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:0]  = imm (zero extended)
//
// Branch instruction (PC <= PC + offset, PC holds 8 bytes past Branch
//   B
//   INSTR <target>
//    PC <- PC + 8 + imm << 2
//   Instr[31:28] = cond
//   Instr[27:25] = Op = 10
//   Instr[25:24] = Funct
//                  [25]: 1 (Branch)
//                  [24]: 0 (link)
//   Instr[23:0]  = offset (sign extend, shift left 2)
//   Note: no Branch delay slot on ARM
//
// Other:
//   R15 reads as PC+8
//   Conditional Encoding
//    cond  Meaning                       Flag
//    0000  Equal                         Z = 1
//    0001  Not Equal                     Z = 0
//    0010  Carry Set                     C = 1
//    0011  Carry Clear                   C = 0
//    0100  Minus                         N = 1
//    0101  Plus                          N = 0
//    0110  Overflow                      V = 1
//    0111  No Overflow                   V = 0
//    1000  Unsigned Higher               C = 1 & Z = 0
//    1001  Unsigned Lower/Same           C = 0 | Z = 1
//    1010  Signed greater/equal          N = V
//    1011  Signed less                   N != V
//    1100  Signed greater                N = V & Z = 0
//    1101  Signed less/equal             N != V | Z = 1
//    1110  Always                        any
//   Writes to register 15 (PC) are ignored 

module arm_multi(input  logic        clk, reset, 
           output logic [31:0] WriteData, Adr, 
           output logic        MemWrite,
           output logic[31:0] SrcA, SrcB, ALUResult,PC, Instr,
           output logic[3:0]  state);

  logic [31:0] ReadData;
  
  // instantiate processor and shared memory
  arm arm(clk, reset, MemWrite, Adr, 
          WriteData, ReadData, SrcA, SrcB, ALUResult, Instr, PC, state);
  mem mem(clk, MemWrite, Adr, WriteData, ReadData);
endmodule

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile.dat",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module arm(input  logic        clk, reset,
           output logic        MemWrite,
           output logic [31:0] Adr, WriteData,
           input  logic [31:0] ReadData,
           output logic[31:0] SrcA, SrcB, ALUResult, Instr, PC,
           output logic[3:0]  state);

  logic [3:0]  ALUFlags;
  logic        PCWrite, RegWrite, IRWrite;
  logic        AdrSrc;
  logic [1:0]  RegSrc, ALUSrcA, ALUSrcB, ImmSrc, ALUControl, ResultSrc;

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               PCWrite, MemWrite, RegWrite, IRWrite,
               AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,
               ImmSrc, ALUControl, state);
  datapath dp(clk, reset, Adr, WriteData, ReadData, Instr, ALUFlags,
              PCWrite, RegWrite, IRWrite,
              AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,
              ImmSrc, ALUControl, SrcA, SrcB, ALUResult, PC);
endmodule

module controller(input  logic         clk,
                  input  logic         reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic         PCWrite,
                  output logic         MemWrite,
                  output logic         RegWrite,
                  output logic         IRWrite,
                  output logic         AdrSrc,
                  output logic [1:0]   RegSrc,
                  output logic [1:0]   ALUSrcA,
                  output logic [1:0]   ALUSrcB,
                  output logic [1:0]   ResultSrc,
                  output logic [1:0]   ImmSrc,
                  output logic [1:0]   ALUControl,
                  output logic [3:0]   state);
                  
  logic [1:0] FlagW;
  logic       PCS, NextPC, RegW, MemW;
  
  decode dec(clk, reset, Instr[27:26], Instr[25:20], Instr[15:12],
             FlagW, PCS, NextPC, RegW, MemW,
             IRWrite, AdrSrc, ResultSrc, 
             ALUSrcA, ALUSrcB, ImmSrc, RegSrc, ALUControl, state);
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, NextPC, RegW, MemW,
               PCWrite, RegWrite, MemWrite);
endmodule

module decode(input  logic       clk, reset,
              input  logic [1:0] Op,
              input  logic [5:0] Funct,
              input  logic [3:0] Rd,
              output logic [1:0] FlagW,
              output logic       PCS, NextPC, RegW, MemW,
              output logic       IRWrite, AdrSrc,
              output logic [1:0] ResultSrc, ALUSrcA, ALUSrcB, 
              output logic [1:0] ImmSrc, RegSrc, ALUControl,
              output logic [3:0] state);

  logic       Branch, ALUOp;

  // Main FSM
  mainfsm fsm(clk, reset, Op, Funct, 
              IRWrite, AdrSrc, 
              ALUSrcA, ALUSrcB, ResultSrc,
              NextPC, RegW, MemW, Branch, ALUOp, state);

  // ADD CODE BELOW
  // Add code for the ALU Decoder and PC Logic.
  // Remember, you may reuse code from the book.
  


	// ALU Decoder
	always_comb
	if (ALUOp) begin // which DP Instr?
		case(Funct[4:1])
			4'b0100:	ALUControl = 2'b00; //ADD
			4'b0010: ALUControl = 2'b01; //SUB
			4'b0000:	ALUControl = 2'b10; //AND
			4'b1100: ALUControl = 2'b11; //OR
			default:	ALUControl = 2'bx;  // unimplemented
		endcase
		
		// update flags if S bit is set (C & V only for arith)
		FlagW[1] = Funct[0];
		FlagW[0] = Funct[0] & (ALUControl == 2'b00 | ALUControl == 2'b01);
	end
	else begin
		ALUControl = 2'b00; // add for non-DP instructions
		FlagW = 2'b00; // don't update Flags
	end



 	// PC Logic
	assign PCS = ((Rd == 4'b1111) & RegW) | Branch;

  // Add code for the Instruction Decoder (Instr Decoder) below.
  // Recall that the input to Instr Decoder is Op, and the outputs are
  // ImmSrc and RegSrc. We've completed the ImmSrc logic for you.

  // Instr Decoder
  assign ImmSrc = Op;
  assign RegSrc[0] = (Op == 10);
  assign RegSrc[1] = (Op == 01);


endmodule

module mainfsm(input  logic         clk,
               input  logic         reset,
               input  logic [1:0]   Op,
               input  logic [5:0]   Funct,
               output logic         IRWrite,
               output logic         AdrSrc,
               output logic [1:0]   ALUSrcA, ALUSrcB, ResultSrc,
               output logic         NextPC, RegW, MemW, Branch, ALUOp,
               output logic [3:0]   states);  
              
  typedef enum logic [3:0] {FETCH, DECODE, MEMADR, MEMRD, MEMWB, 
                            MEMWR, EXECUTER, EXECUTEI, ALUWB, BRANCH, UNKNOWN} statetype;
  
  statetype state, nextstate;
  logic [12:0] controls;

  assign states = state;
  
  // state register
  always @(posedge clk or posedge reset)
    if (reset) state <= FETCH;
    else
      state <= nextstate;
  
  // ADD CODE BELOW
  // Finish entering the next state logic below.  We've completed the 
  // first two states, FETCH and DECODE, for you.

  // next state logic
  always_comb
    casex(state)
      FETCH:                     nextstate = DECODE;
      DECODE: case(Op)
                2'b00: 
                  if (Funct[5])  nextstate = EXECUTEI;
                  else           nextstate = EXECUTER;
                2'b01:           nextstate = MEMADR;
                2'b10:           nextstate = BRANCH;
                default:         nextstate = UNKNOWN;
              endcase
      EXECUTER:                  nextstate = ALUWB;
      EXECUTEI:                  nextstate = ALUWB;
      MEMADR:     if (Funct[0])  nextstate = MEMRD;
                  else           nextstate = MEMWR;
      MEMRD:                     nextstate = MEMWB;
      default:                   nextstate = FETCH; 
    endcase
    
  // ADD CODE BELOW
  // Finish entering the output logic below.  We've entered the
  // output logic for the first two states, FETCH and DECODE, for you.

  // state-dependent output logic
  always_comb
    case(state)
      FETCH: 		controls = 13'b1_0001_0100_1100; 
      DECODE:  	controls = 13'b0_0000_0100_1100;      
      EXECUTER: controls = 13'b0_0000_0000_0001;
      EXECUTEI: controls = 13'b0_0000_0000_0011;
      ALUWB:		controls = 13'b0_0010_0000_0000;
      MEMADR:		controls = 13'b0_0000_0000_0010;
      MEMWR:		controls = 13'b0_0100_1000_0000;
      MEMRD:		controls = 13'b0_0000_1000_0000;
      MEMWB:		controls = 13'b0_0010_0010_0000;
      BRANCH:		controls = 13'b0_1000_0101_0010;
      default: 	controls = 13'bx_xxxx_xxxx_xxxx;
    endcase

  assign {NextPC, Branch, MemW, RegW, IRWrite,
          AdrSrc, ResultSrc,   
          ALUSrcA, ALUSrcB, ALUOp} = controls;
endmodule              

// ADD CODE BELOW
// Add code for the condlogic and condcheck modules. Remember, you may
// reuse code from the book.
module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, NextPC, RegW, MemW,
                 output logic       PCWrite, RegWrite, MemWrite);

  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx, CondEx_q;
  //CondEx_q is the result of CondExRegister


  // Delay writing flags until ALUWB state
  //flopr #(2)flagwritereg(clk, reset, FlagW&{2{CondEx}}, FlagWrite);
  
  assign FlagWrite = FlagW&{2{CondEx}};
	flopr #(1)CondExreg(clk, reset, CondEx, CondEx_q);
  // ADD CODE HERE
  

  //status register
  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], ALUFlags[3:2], Flags[3:2]);
	flopenr #(2)flagreg0(clk, reset, FlagWrite[0], ALUFlags[1:0], Flags[1:0]);

	//instance of condcheck
	condcheck c1 (Cond,Flags,CondEx);


	// write controls are conditional
	assign RegWrite = RegW & CondEx_q;
	assign MemWrite = MemW & CondEx_q;
	assign PCWrite = (PCS & CondEx_q) | NextPC;

endmodule    

module condcheck(input  logic [3:0] Cond,
                 input  logic [3:0] Flags,
                 output logic       CondEx);

  // ADD CODE HERE
  logic neg, zero, carry, overflow, ge;
	
	assign {neg, zero, carry, overflow} = Flags;
	assign ge = (neg == overflow);
	
	always_comb
		case(Cond)
			4'b0000:	CondEx = zero;
			4'b0001:	CondEx = ~zero;
			4'b0010:	CondEx = carry;
			4'b0011:	CondEx = ~carry;
			4'b0100:	CondEx = neg;
			4'b0101:	CondEx = ~neg;
			4'b0110:	CondEx = overflow;
			4'b0111:	CondEx = ~overflow;
			4'b1000:	CondEx = carry & ~zero;
			4'b1001:	CondEx = ~(carry & ~zero);
			4'b1010:	CondEx = ge;
			4'b1011:	CondEx = ~ge;
			4'b1100:	CondEx = ~zero & ge;
			4'b1101:	CondEx = ~(~zero & ge);
			4'b1110:	CondEx = 1'b1;
			default:	CondEx = 1'bx;
		endcase

endmodule


// ADD CODE BELOW
// Complete the datapath module below.
// The datapath unit is a structural SystemVerilog module. That is,
// it is composed of instances of its sub-modules. For example,
// the instruction register is instantiated as a 32-bit flopenr.
// The other submodules are likewise instantiated. 

module datapath(input  logic        clk, reset,
                output logic [31:0] Adr, WriteData,
                input  logic [31:0] ReadData,
                output logic [31:0] Instr,
                output logic [3:0]  ALUFlags,
                input  logic        PCWrite, RegWrite,
                input  logic        IRWrite,
                input  logic        AdrSrc, 
                input  logic [1:0]  RegSrc, 
                input  logic [1:0]  ALUSrcA, ALUSrcB, ResultSrc,
                input  logic [1:0]  ImmSrc, ALUControl,
                output logic[31:0] SrcA, SrcB, ALUResult, PC);

  logic [31:0] ExtImm, Result;
  logic [31:0] Data, RD1, RD2, A, ALUOut;
  logic [3:0]  RA1, RA2;

  // Your datapath hardware goes below. Instantiate each of the 
  // submodules that you need. Remember that you can reuse hardware
  // from the book. Be sure to give your instantiated modules 
  // applicable names such as pcreg (PC register), adrmux 
  // (Address Mux), etc. so that your code is easier to understand.

  // ADD CODE HERE
  
  //pc register
  flopenr #(32)pcreg(clk, reset, PCWrite, Result, PC);

  //address mux
  mux2 #(32)adrmux(PC, Result, AdrSrc, Adr);

  //IR Register
  flopenr #(32)ir_reg(clk, reset, IRWrite, ReadData, Instr);

  //ReadData Register
  flopr #(32)read_data_reg(clk, reset, ReadData, Data);

  //RA1 input mux
  mux2 #(4)RA1_mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
	
	//RA2 input mux
  mux2 #(4)RA2_mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);

  //register file
  RegisterFile regfile(clk, RegWrite, RA1, RA2, Instr[15:12], Result, Result, RD1, RD2);

  //Extendtion unit
  extend extend_unit(Instr[23:0], ImmSrc, ExtImm);

  //RD1 register
  flopr #(32)rd1_reg(clk, reset, RD1, A);

  //RD2 register
  flopr #(32)rd2_reg(clk, reset, RD2, WriteData);

  //SrcA mux
  mux3 #(32)SrcA_mux(A, PC, ALUOut, ALUSrcA, SrcA);

  //SrcB mux
  mux3 #(32)SrcB_mux(WriteData, ExtImm, 32'b100, ALUSrcB, SrcB);

  //ALU
  alu alu1(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);

  //ALUResult Register
  flopr #(32)alu_result_reg(clk, reset, ALUResult, ALUOut);

  //Result mux
  mux3 #(32)result_mux(ALUOut, Data, ALUResult, ResultSrc, Result);

endmodule


// ADD CODE BELOW
// Add needed building blocks below (i.e., parameterizable muxes, 
// registers, etc.). Remember, you can reuse code from the book.
// We've also provided a parameterizable 3:1 mux below for your 
// convenience.

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module mux2 #(parameter WIDTH = 8)(
	input logic [WIDTH-1:0] d0, d1,
	input logic s, 
	output logic [WIDTH-1:0] y
	);
	
	assign y = s ? d1 : d0;
endmodule

module RegisterFile(
	input logic clk, we3,
	input logic [3:0] ra1, ra2, wa3,
	input logic [31:0] wd3, r15,
	output logic [31:0] rd1, rd2
	);
	
	logic [31:0] rf[14:0];
	
	// three ported register file
	// read two ports combinationally
	// write third port on rising edge of clock 
	// register 15 reads PC + 8 instead
	
	always_ff @(posedge clk) 
		if (we3) rf[wa3] <= wd3;
	
	assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
	assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule

module extend(
	input logic [23:0] Instr, 
	input logic [1:0] ImmSrc,
	output logic [31:0] ExtImm
	);
	
	always_comb case(ImmSrc)
					// 8-bit unsigned immediate
		2'b00: 	ExtImm = {24'b0, Instr[7:0]};
					// 12-bit unsigned immediate
		2'b01: 	ExtImm = {20'b0, Instr[11:0]};
					// 24-bit two's complement shifted branch
		2'b10: 	ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; 
		default: ExtImm = 32'bx; // undefined
	endcase 
endmodule

module alu(
	input logic [31:0] A,B,
	input logic [1:0] ALUControl,
	output logic [31:0] Result,
	output logic [3:0] flags
	);
	
	logic cout;
	logic N,Z,C,V;
	
	always_comb
		case (ALUControl)
			2'b00: {cout, Result} = A + B;
			2'b01: {cout, Result} = A - B;
			2'b10:
			begin
					Result = A & B;
					cout = 1'b0;
			end
			2'b11:
			begin
					Result = A | B;
					cout = 1'b0;
			end
			default:
			begin
					Result = 32'bx;
					cout = 1'bx;
			end
		endcase
	assign Z = &(~Result);
	assign N = Result[31];
	assign C = cout & ~ALUControl[1];
	assign V = ~(ALUControl[0] ^ A[31] ^ B[31]) & (Result[31] ^ A[31]) & ~ALUControl[1];
	assign flags = {N,Z,C,V};
endmodule 