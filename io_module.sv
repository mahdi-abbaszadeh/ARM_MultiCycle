module io_module (
	input logic clk_50, reset,
	output logic [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
	output logic LCD_ON,    // LCD Power ON/OFF
	output logic LCD_BLON,    // LCD Back Light ON/OFF
	output logic LCD_RW,    // LCD Read/Write Select, 0 = Write, 1 = Read
	output logic LCD_EN,    // LCD Enable
	output logic LCD_RS,    // LCD Command/Data Select, 0 = Command, 1 = Data
	inout [7:0] LCD_DATA    // LCD Data bus 8 bits 
);
	logic clk;
	logic [3:0] state;
	logic [27:0] counter;

	always_ff @(posedge clk_50)
		if(~reset) begin
			counter <= 28'b0;
			clk = 1'b0;
		end

		else if(counter[27] == 1'b1)begin
			counter <= 29'b0;
			clk <= ~clk;
		end
		else begin
			counter <= counter + 1;
		end

	arm_multi arm(clk, ~reset, WriteData, Adr, MemWrite, 
		SrcA, SrcB, ALUResult, PC, Instr, state);

	logic[31:0] SrcA, SrcB, ALUResult, PC, Instr;
	//src B
	seven_seg_driver s0(SrcB[3:0], HEX0);
	seven_seg_driver s1(SrcB[7:4], HEX1);

	//src A
	seven_seg_driver s2(SrcA[3:0], HEX2);
	seven_seg_driver s3(SrcA[7:4], HEX3);

	// ALU result
	seven_seg_driver s4(ALUResult[3:0], HEX4);
	seven_seg_driver s5(ALUResult[7:4], HEX5);

	//PC
	seven_seg_driver s6(PC[3:0], HEX6);
	seven_seg_driver s7(PC[7:4], HEX7);

	//lcd driver
	lcd_driver lcd(clk_50, Instr, state, LCD_ON, LCD_BLON, LCD_RW, LCD_EN, LCD_RS, LCD_DATA);



endmodule