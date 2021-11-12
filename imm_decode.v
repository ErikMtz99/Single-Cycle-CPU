module immdecode(input [31:0] in, //[31:7]
                 input immControl,
				 output reg [31:0] immOp);
				 
	wire [6:0] opcode;		
	
	assign opcode = in[6:0];	 

	always @ (*)
	begin
	  if (immControl) begin
		case(opcode)
		7'b0110011: immOp = 32'b0; //ADD, SUB, etc (R)
		7'b0001011: immOp = 32'b0; //ADUQ (R)
		7'b0010011: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //ADDI (I)
		7'b0110111: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //LUI This one i am not sure (I)
		7'b1100111: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //JALR (I)
		7'b0000011: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //LW (I)		
		7'b1100011: immOp = {20'b0000_0000_0000_0000_0000, in[31],in[7],in[30:25],in[11:8]}; //BEQ (B)
		7'b0100011: immOp = {20'b0000_0000_0000_0000_0000, in[31:25],in[11:7]}; //SW (S)
		7'b1101111: immOp = {12'b0000_0000_0000, in[31],in[19:12],in[20],in[30:21]}; //JAL (J)
		7'b0010111: immOp = 32'b0; //AUIPC (R) ?
		default: immOp = 32'b0;
		endcase
	  end
	end

endmodule				 
		//13/54 Lecture 04