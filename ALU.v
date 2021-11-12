module alu (input [31:0] sA,sB,
            input [2:0] control,
            output reg [31:0] result,
			output reg zero);
			
     always @ (*)
        case(control)
          0: result = sA + sB; //SUM
          1: result = sA - sB; //MINUS
		  2: begin //ADDU.QB SUM
		        result[31:24] =  sA[31:24] +  sB[31:24];
				result[23:16] =  sA[23:16] +  sB[23:16];
			    result[15:8] =  sA[15:8] +  sB[15:8];
				result[7:0] =  sA[7:0] +  sB[7:0];
			 end
		  3: result = sA & sB; //AND
		  4: result = sA < sB; //LESS THAN
		  5: result = {sB[31:12],12'b0000_0000_0000}; //THIS IS FOR LOAD UPPER INMEDIATE
		  6: result = sA << sB; //shift left //fix!
		  7: result = sA >> sB; //shift right //fix!
          default: result = 0;
        endcase
		
      always @ (*)
        if (result) zero = 0; //check if even if the result gives zero (not by branching) it has to give 1 the signal
        else zero  = 1;
endmodule

module test();
  reg [31:0] sA, sB;
  reg [2:0] c;
  wire [31:0] result;
  wire zero,x;
  
  alu alu_test(sA,sB,c,result,zero);
  
  initial begin
    $dumpfile("test");
    $dumpvars;
    sA = 15;
    sB = 7;
    c = 0;
	#10;
	sA = 15;
	sB = 3;
	c = 1;
	#10;
	sA = 10;
	sB = 10;
	c = 1;
	#10;
	sA = 24'b011110101010101010101010;
	sB = 24'b010000010101011110100110;
	c = 2;
	#10;
	c = 3;
	#10;
	c = 4;
	#10;
	c = 5;
	sB = 32'b11111111110000000000000000111111;
	#10;
	c = 6;
	#10;
	c = 7;
    #10 $finish;
  end


  always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule
