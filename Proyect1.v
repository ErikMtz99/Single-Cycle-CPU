module top (input         clk, reset,
		    output [31:0] data_to_mem, address_to_mem,
		    output        write_enable);

	wire [31:0] pc, instruction, data_from_mem;

	inst_mem  imem(pc[7:2], instruction);
	data_mem  dmem(clk, write_enable, address_to_mem, data_to_mem, data_from_mem);
	processor CPU(clk, reset, pc, instruction, write_enable, address_to_mem, data_to_mem, data_from_mem);
	
endmodule

//-------------------------------------------------------------------
module data_mem (input clk, we,
		 input  [31:0] address, wd,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];

	initial begin
		$readmemh ("memfile_data.hex",RAM,0,63);
	end

	assign rd=RAM[address[31:2]]; // word aligned

	always @ (posedge clk)
		if (we)
			RAM[address[31:2]]<=wd;
endmodule

//-------------------------------------------------------------------
module inst_mem (input  [5:0]  address, //I can change this input value to support more instructions. We only use 6 spaces because we have 64 instructions 2^6=64
		 output [31:0] rd);

	reg [31:0] RAM[63:0];
	initial begin
		$readmemh ("memfile_inst.hex",RAM,0,63);
	end
	assign rd=RAM[address]; // word aligned
endmodule

//-------------------------------------------------------------------
module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem //WD3
                );
    //... write your code here ...
	wire BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite, ALUSrc, immControl;
    wire [2:0] ALUControl;
	wire [31:0] rs1, rs2, SrcB;
	wire [31:0] immOp;
	wire [31:0] ALUout;
	wire zero;
	wire [31:0] PCn, mux3_out, res;
	wire [1:0] signal_1;
    wire signal_2;
	wire [31:0] sum_signal_1, sum_signal_2;
	
	always @ (posedge clk)
	begin
		if (reset) begin
		    PC = 32'b0;
		else 
		    PC = PCn;
		end	
	end
	
	reg_file  regfile(instruction[19:15],instruction[24:20],instruction[11:7],res, RegWrite, clk, rs1, rs2);
	control_unit control_u(instruction, BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite, ALUControl, ALUSrc, immControl);
	
	mux21 mux_1(rs2, immOp, ALUSrc, SrcB);
	
	alu  alu_main(rs1, SrcB, ALUControl, ALUout, zero);
	immdecode immc(instruction, immControl, immOp);
	
	assign signal_1[0] = BranchJalr;
	assign signal_1[1] = (BranchBeq & zero) | BranchJal;
	
	assign signal_2 = BranchJal | BranchJalr;
	
	assign sum_siganl_1 = PC + 4;
	assign sum_signal_2 = immOp +PC;
	
	mux31 mux_2(sum_signal_1,ALUout,sum_signal_2,signal_1,PCn); //y goes to instruction memory!!
    
	mux21 mux_3(ALUout, sum_signal_1, signal_2, mux3_out);
	mux21 mux_4(mux3_out, data_from_mem, MemToReg, res);

	

	
	
endmodule

//-------------------------------------------------------------------
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
        if (result) zero = 0;
        else zero  = 1;
endmodule
//----------------------------------------------------------------------
module control_unit (input [31:0] instr, //funct7 [31:25], funct3 [14:12], opcode [6:0]
                     output reg BranchJalr, BranchJal, BranchBeq, RegWrite, MemToReg, MemWrite, 
					 output reg [2:0] ALUControl,
					 output reg ALUSrc, immControl);
					 
     reg [9:0]concat_code;
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
									7'b0000000: concat_code = 11'b000100_000_00; //ADD
									7'b0100000: concat_code = 11'b000100_001_00; //SUB
								endcase
								end
						3'b001: concat_code = 11'b000000_000_00; //SLL
						3'b010: concat_code = 11'b000100_100_00; //SLT
						3'b101: begin
						        case(funct7)
									7'b0000000: concat_code = 11'b000000_000_00; //SRL
									7'b0100000: concat_code = 11'b000000_000_00; //SRA
								endcase
						        end
						
						3'b111: concat_code = 11'b000100_011_00; //AND
					endcase
			        end
		7'b0010011: concat_code = 11'b000100_000_11; //ADDI
		7'b0001011: concat_code = 11'b000100_010_00; //ADDUQB
		7'b1100011: concat_code = 11'b001000_001_01; //BEQ
		7'b0000011: concat_code = 11'b000110_000_11; //LW
		7'b0100011: concat_code = 11'b000001_000_11; //SW
		7'b0110111: concat_code = 11'b000100_101_11; //LUI
		7'b1101111: concat_code = 11'b010100_000_01; //JAL
		7'b1100111: concat_code = 11'b100100_000_11; //JALR
		7'b0010111: concat_code = 11'b000000_000_00; //AUIPC
        default: concat_code = 11'b000000_000_00;
        endcase		
		end
	 
	 always @ (*) 
	 begin
	 BranchJalr = concat_code[10];
	 BranchJal = concat_code[9];
	 BranchBeq = concat_code[8];
	 RegWrite = concat_code[7];
	 MemToReg = concat_code[6];
	 MemWrite = concat_code[5];
	 ALUControl = concat_code[4:2];
	 ALUSrc = concat_code[1];
	 immControl = concat_code[0];
	 end
	 
endmodule
//----------------------------------------------------------------------
module immdecode(input [31:0] in, //[31:7]
                 input immControl,
				 output reg [31:0] immOp);
				 
	wire [6:0] opcode;		
	
	assign opcode = in[6:0];	 

	always @ (immControl)
	begin
		case(opcode)
		7'b0110011: immOp = 32'b0; //ADD, SUB, etc
		7'b0010011: immOp = {in[31:20],20'b0000_0000_0000_0000_0000}; //ADDI
		7'b0001011: immOp = 32'b0; //ADUQ
		7'b1100011: immOp = {in[31],in[7],in[30:25],in[11:8],20'b0000_0000_0000_0000_0000}; //BEQ
		7'b0000011: immOp = {in[31:20],20'b0000_0000_0000_0000_0000}; //LW
		7'b0100011: immOp = {in[31:25],in[11:7],20'b0000_0000_0000_0000_0000}; //SW
		7'b0110111: immOp = {in[31:20],20'b0000_0000_0000_0000_0000}; //LUI This one i am not sure
		7'b1101111: immOp = {in[31],in[19:12],in[20],in[30:21],12'b0000_0000_0000}; //JAL
		7'b1100111: immOp = {in[31:20],20'b0000_0000_0000_0000_0000}; //JALR
		7'b0010111: immOp = 32'b0;
		default: immOp = 32'b0;
		endcase
	end

endmodule		
//------------------------------------------------------------------------
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
//------------------------------------------------------------------------
module mux21 (input[31:0] d0, d1,
              input select,
              output reg [31:0] y);
   always @ (*)
	if (select) y = d0;
        else y = d1;
endmodule

//------------------------------------------------------------------------
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
//------------------------------------------------------------------------

module test();
  reg clk, reset;
  wire [31:0] data_to_mem, address_to_mem;
  wire write_enable;
  
  top top_test(clk, reset, data_to_mem, address_to_mem, write_enable);
  
  initial begin
    $dumpfile("test");
    $dumpvars;
	clk=0;
    reset=0;
	data_to_mem=0;
	address_to_mem=0;
	write_enable=0;
	#10;
	write_enable=1;
    #160 $finish;
  end

  always #1 clk = ~clk;
  //always @(x) $display( "The value of x was changed. Time=%3d, x=%b. Inputs: sA=%b, sB=%b, alu=%b.",$time, x,sA,sB,c);
endmodule
		