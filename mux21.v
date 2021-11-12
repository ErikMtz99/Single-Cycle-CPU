module mux21 (input[31:0] d0, d1,
              input select,
              output reg [31:0] y);
   always @ (*)
	if (select) y = d0;
        else y = d1;
endmodule


module test();
  reg [31:0] d0, d1;
  reg select;
  wire [31:0] y;
  
  mux21 mux21_test(d0,d1,select,y);
  
  initial begin
    $dumpfile("test");
    $dumpvars;
	d0 = 100;
	d1 = 2;
	select = 0;
	#20;
	select = 1;
	#20;
	
    #160 $finish;
  end

  //always #1 clk = ~clk;
  //always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule