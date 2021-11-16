module control_unit (input [31:0] instr, //funct7 [31:25], funct3 [14:12], opcode [6:0]
                     input neg,
                     output reg BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite, 
					 output reg [2:0] ALUControl,
					 output reg ALUSrc, immControl,s_auipc);
					 
     reg [11:0]concat_code;
	 wire [6:0] funct7, opcode;
	 wire [2:0] funct3;
	 
	 assign funct7 = instr[31:25];
	 assign funct3 = instr[14:12];
	 assign opcode = instr[6:0];


	 always @ (*) 
	    begin
        case (opcode)
        7'b0110011: begin 
					case (funct3)
						3'b000: begin
								case(funct7)
									7'b0000000: concat_code = 12'b000100_000_00_0; //ADD
									7'b0100000: concat_code = 12'b000100_001_00_0; //SUB
								endcase
								end
						3'b001: concat_code = 12'b000000_000_00_0; //SLL
						3'b010: concat_code = 12'b000100_100_00_0; //SLT
						3'b101: begin
						        case(funct7)
									7'b0000000: concat_code = 12'b000000_000_00_0; //SRL
									7'b0100000: concat_code = 12'b000000_000_00_0; //SRA
								endcase
						        end
						
						3'b111: concat_code = 12'b000100_011_00_0; //AND
					endcase
			        end
		7'b0010011: begin
		               if (neg) concat_code = 12'b000100_001_11_0; //ADDI negative immediate number
					   else concat_code = 12'b000100_000_11_0; //ADDI positive immediate number
		            end
		7'b0001011: concat_code = 12'b000100_010_00_0; //ADDUQB
		7'b1100011: concat_code = 12'b001000_001_01_0; //BEQ
		7'b0000011: concat_code = 12'b000110_000_11_0; //LW
		7'b0100011: concat_code = 12'b000001_000_11_0; //SW
		7'b0110111: concat_code = 12'b000100_101_11_0; //LUI
		7'b1101111: concat_code = 12'b010100_000_01_0; //JAL
		7'b1100111: concat_code = 12'b100100_000_11_0; //JALR
		7'b0010111: concat_code = 12'b000100_000_01_1; //AUIPC
        default: concat_code = 12'b000000_000_00_0;
        endcase		
		end
	 
	 always @ (*) 
	 begin
	 BranchJalr = concat_code[11];
	 BranchJal = concat_code[10];
	 BranchBeq = concat_code[9];
	 RegWrite = concat_code[8];
	 MemToReg = concat_code[7];
	 MemWrite = concat_code[6];
	 ALUControl = concat_code[5:3];
	 ALUSrc = concat_code[2];
	 immControl = concat_code[1];
	 s_auipc = concat_code[0];
	 end
	 
endmodule

module test();
    reg [31:0] instr; //funct7 [31:25], funct3 [14:12], opcode [6:0]
	reg neg;
    wire BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite; 
	wire [2:0] ALUControl;
	wire ALUSrc, immControl,s_auipc;
	
	//wire [31:25] funct7;
	//wire [2:0] funct3;
	//wire [6:0] opcode;
  
control_unit control_unit_test(instr, neg, BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite, ALUControl, ALUSrc, immControl, s_auipc);  
  initial begin
    $dumpfile("test");
    $dumpvars;
	instr = 32'b00000000000000000000000000110011; //add
	neg = 0;
	#20;
	neg = 1;
	instr = 32'b00000000000000000000000000010011; //addi
	#20;
	neg = 0;
	instr = 32'b00000000000000000000000000010011; //addi
	#20;
    #160 $finish;
  end

  //always #1 clk = ~clk;
  //always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule