module seven_seg_driver (
	input logic [3:0] in,
	output logic[6:0] out);
	
	logic[6:0] n_out;
	assign out = ~n_out;
	
	always_comb
		case (in)
			4'b0000:	n_out = 7'b0111111;
			4'b0001:	n_out = 7'b0000110;
			4'b0010:	n_out = 7'b1011011;
			4'b0011:	n_out = 7'b1001111;
			4'b0100:	n_out = 7'b1100110;
			4'b0101:	n_out = 7'b1101101;
			4'b0110:	n_out = 7'b1111101;
			4'b0111:	n_out = 7'b0000111;
			4'b1000:	n_out = 7'b1111111;
			4'b1001:	n_out = 7'b1100111;
			4'b1010:	n_out = 7'b1110111;
			4'b1011:	n_out = 7'b1111100;
			4'b1100:	n_out = 7'b0111001;
			4'b1101:	n_out = 7'b1011110;
			4'b1110:	n_out = 7'b1111001;
			4'b1111:	n_out = 7'b1110001;
		endcase
endmodule