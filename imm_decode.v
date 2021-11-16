module immdecode(input [31:0] in, //[31:7]
                 input immControl,
				 output reg [31:0] immOp,
				 output reg neg);
				 
	wire [6:0] opcode;
	//wire [11:0]component2;		
	
	assign opcode = in[6:0];	 
    //assign component2 = in[31:20];
	always @ (*)
	begin
	neg = 1'b0;
	  if (immControl) begin
		case(opcode)
		7'b0110011: immOp = 32'b0; //ADD, SUB, etc (R)
		7'b0001011: immOp = 32'b0; //ADUQ (R)
		7'b0010011: begin
					   if (in[31]) begin //if MSB = 1, negative number 2nd complement
						  //component2 = ~component2 + 1'b1;
						  immOp = {20'b0000_0000_0000_0000_0000, ~(in[31:20])+1'b1}; //ADDI (I)
						  neg = 1'b1;
					   end else begin
		                      immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //ADDI (I)
						      neg = 1'b0;
		                   end
					end
		7'b0110111: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //LUI This one i am not sure (I)
		7'b1100111: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //JALR (I)
		7'b0000011: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //LW (I)		
		7'b1100011: immOp = {20'b0000_0000_0000_0000_0000, in[31],in[7],in[30:25],in[11:8]}; //BEQ (B)
		7'b0100011: immOp = {20'b0000_0000_0000_0000_0000, in[31:25],in[11:7]}; //SW (S)
		7'b1101111: immOp = {12'b0000_0000_0000, in[31],in[19:12],in[20],in[30:21]}; //JAL (J)
		7'b0010111: immOp = {20'b0000_0000_0000_0000_0000, in[31:20]}; //AUIPC (R) ?
		default: immOp = 32'b0;
		endcase
	  end
	end

endmodule				 
		//13/54 Lecture 04

module test();
    reg [31:0] in; 
	reg immControl;
	wire [31:0] immOp;
	wire neg;	
  
immdecode immdecode_test(in, immControl, immOp, neg);  
  initial begin
    $dumpfile("test");
    $dumpvars;
	in = 32'b000000000000_00000_000000000110011; //add
	immControl = 0;
	#20;
	immControl = 1;
	#20;
	in = 32'b000000000101_01010_000000000010011; //addi 5 + 10
	#20;
	in = 32'b111111111001_01010_000000000010011; //addi  -7 + 10
	#20;
    #160 $finish;
  end

  //always #1 clk = ~clk;
  //always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule