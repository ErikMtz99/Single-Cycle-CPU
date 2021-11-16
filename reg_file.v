module reg_file (input [4:0] A1, A2, A3,
            input [31:0] WD3,
			input WE3, clk,
            output reg [31:0] RD1, RD2);
			
	 reg [31:0] rf[31:0]; //32 registers: Array of vectors 

	 always @ (posedge clk) // Write port
	    begin
        rf[0] = 0;  //Register 0 hardwired to 0
	    RD1 = rf[A1]; 
		RD2 = rf[A2];
	    if (WE3) begin
		    rf[A3] = WD3;
		end	
		end
endmodule