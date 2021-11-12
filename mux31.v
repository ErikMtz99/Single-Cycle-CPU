module mux31 (input [31:0] d0,d1,d2,
                   input [1:0] select ,
                   output reg [31:0] y);
     always @ (*)
        case(select)
          0: y = d0;
          1: y = d1;
          2: y = d2;
          default: y = 0; 
        endcase
endmodule

module test();
  reg [31:0] d0, d1, d2;
  reg [1:0] select;
  wire [31:0] y;
  
  mux31 mux31_test(d0,d1,d2,select,y);
  
  initial begin
    $dumpfile("test");
    $dumpvars;
	d0 = 100;
	d1 = 3434343434;
	d2 = 2;
	select = 00;
	#20;
	select = 01;
	#20;
	select = 10;
	#20;
	select = 11;
	#20;
	
    #160 $finish;
  end

  //always #1 clk = ~clk;
  //always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule