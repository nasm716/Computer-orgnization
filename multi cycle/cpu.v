`timescale 1ns/100ps

`include "opcodes.v"
`include "constants.v"

module cpu (
    output readM, // read from memory
    output writeM, // write to memory
    output [`WORD_SIZE-1:0] address, // current address for data
    inout [`WORD_SIZE-1:0] data, // data being input or output
    input inputReady, // indicates that data is ready from the input port
    input reset_n, // active-low RESET signal
    input clk, // clock signal
    
    // for debuging/testing purpose
    output [`WORD_SIZE-1:0] num_inst, // number of instruction during execution
    output [`WORD_SIZE-1:0] output_port, // this will be used for a "WWD" instruction
    output is_halted // 1 if the cpu is halted
);
  reg halted=0;
  reg [15:0]MDR;
  reg [15:0]inst;
  reg readM_in; //decision making read memroy or not;
 // reg writeM_in=0; //decision making write memroy or not;
  reg [15:0]num_inst_in=0;
  reg data_in;
  assign readM=readM_in;
  assign num_inst=num_inst_in;
   // for address
  reg [15:0]PC;
   // for microseq
    wire IF;
    wire PC_PC1;
    wire PC_ALUOUT;
    wire PC_PC31;
    wire IR_MEM;
    wire A_RF;
    wire B_RF;
    wire ALUOut_AB;
    wire ALUOut_Asign;
    wire ALUOut_PCsign;
    wire MDR_MEM;
    wire MEM_B;
    wire RF_ALU1st;
    wire RF_ALU2nd;
    wire RF_MDR1st;
    wire RF2_PC;
    wire PC_RS;
    wire isWWD;
    wire isHLT;
    wire [3:0]opcode;
    wire [5:0]func_code;
    assign opcode[3:0]=inst[15:12];
    assign func_code[5:0]=inst[5:0];  
    microseq microseq(clk, inst[15:0],reset_n,opcode[3:0],func_code[5:0],IF,PC_PC1,PC_ALUOUT,PC_PC15,IR_MEM,A_RF,B_RF,ALUOut_AB,ALUOut_Asign,ALUOut_PCsign,MDR_MEM,
 MEM_B,RF_ALU1st, RF_ALU2nd,RF_MDR1st, RF2_PC,PC_RS,isWWD,isHLT);

   //for define control
    wire ALUSrcA;
    wire IorD;
    wire IRWrite;
    wire PCWrite;
    wire PCWriteCond;
    wire [1:0]ALUSrcB;
    wire [1:0]PCSrc;
    wire MemRead;
    wire MemWrite;
    wire MemtoReg;
    wire RegDst;
   // wire Jump;
    wire [4:0]ALUOp;
    wire ALUSrc;
    wire RegWrite;      
   control control(opcode[3:0],func_code[5:0],PC_PC1,PC_ALUOUT, PC_PC15,IR_MEM,ALUOut_AB,ALUOut_Asign, ALUOut_PCsign, MDR_MEM,MEM_B,RF_ALU1st, RF_ALU2nd, RF_MDR1st, RF2_PC,
PC_RS,ALUSrcA,IorD,IRWrite, PCWrite, PCWriteCond,ALUSrcB[1:0],PCSrc[1:0],RegDst, RegWrite,MemRead,MemWrite,MemtoReg,ALUOp[3:0]);
   // for define RF
   wire [1:0] addr1;
   wire [1:0] addr2;
   wire [1:0] addr3;
   wire [15:0] data1;
   wire [15:0] data2;
   wire [15:0] data3;
   reg [15:0] next_PC; //for JAL JRL instruction
   assign addr1=inst[11:10];
   assign addr2=inst[9:8];
   assign addr3=RF2_PC?2'b10:(RegDst?inst[7:6]:inst[9:8]); //for write back pc to rf $2
   RF RF(RegWrite,clk,reset_n,addr1[1:0],addr2[1:0],addr3[1:0],data1[15:0],data2[15:0],data3[15:0]);
   // for define ALU
   reg [15:0] A;
   reg [15:0] B;
   wire [15:0]ALU1;
   wire [15:0]ALU2;
   wire Cin;
   wire [3:0]OP;
   wire Cout;
   wire [15:0]C;
   wire bcond;
   //for define ALUOut
    reg [15:0]ALUOut;
   //we finish declare all unit!!!
   
   //let's start connect line and mux!!!   
   //for PC Change
     wire [15:0]Jump_addr={PC[15:14],inst[11:0]};
     wire [15:0]PCLine=(PCSrc[1])?Jump_addr:((PCSrc[0])?ALUOut[15:0]:C[15:0]); // PC뒤에 연결된 MUX
     wire ChangePC=(bcond&PCWriteCond)|PCWrite;
     //for Address
     wire [15:0]address_in=(IorD)?ALUOut[15:0]:PC[15:0];
     assign address=address_in;
   //for signextend, 원래는 2bit shift 해야하지만, 1instruction이 1개에 들어가므로 여기서는 shift하면 안된다.
   wire [15:0]sign_ex={{8{inst[7]}},inst[7:0]};
   wire [15:0]shift_0=(sign_ex<<0);
   //for ALU1, ALU2 MUX
   assign ALU1[15:0]=(ALUSrcA)?A:address_in;
   assign ALU2[15:0]=(ALUSrcB[1])?((ALUSrcB[0])?shift_0:sign_ex):((ALUSrcB[0])?1:B);
   assign Cin=0;
   assign OP=ALUOp;
   ALU ALU( ALU1[15:0], ALU2[15:0], Cin, OP[3:0],Cout,C[15:0],bcond);
   //for writeback mux
   assign data3[15:0]=(RF2_PC)?next_PC:(MemtoReg)?MDR:ALUOut[15:0]; //원래의 mux에 JRL, JAL을 수행하기 위해 next_PC를 추가했다.
   //for save (MEM<-B)
   assign data = writeM ? B : `WORD_SIZE'bz;
   assign writeM=MemWrite;
   // for isWWD
   assign output_port=isWWD?data1:0;
   //for HLT
   assign is_halted=halted;
   always @(posedge clk) begin
      if(!reset_n) begin 
      PC<=0; //
      num_inst_in<=0;
      inst<=16'b0; //in Now there are no ISA responding instruction=16'b0 
      readM_in<=0; 
      end
      else if(!halted) begin
      if(IF)begin //Instruction fatch start
          readM_in<=1; //require signal
      end
      if(inputReady)begin
        readM_in<=0;
      end
      if(IRWrite)begin 
      inst<=data;
      num_inst_in<=num_inst+1;
      end
      if(MemRead)begin
          readM_in<=1; //require signal
      end
      //In this part, update clk syncronsly
      if(MDR_MEM) MDR<=data; //MDR to data
      if(ChangePC) PC<=PCLine[15:0];// change PC
      if(PC_PC1) next_PC<=PCLine[15:0];// save next PC for JAL, JRL
      if(A_RF) A[15:0]<=data1[15:0];
      if(B_RF) B[15:0]<=data2[15:0];
      if(ALUOut_AB|ALUOut_PCsign|ALUOut_Asign)ALUOut[15:0]<=C[15:0];
      //ALUOut update! 
      if(isHLT)halted<=1; 
       //after halt we don't fatch instruction! 
      end
    end
endmodule
module microseq( //instruction name; wrie to destination<-start is dest_st
        input clk,
        input [15:0]inst,
        input reset_n,
        input wire [3:0]opcode,
        input wire [5:0]func_code, 
        output wire IF,
        output wire PC_PC1,
        output wire PC_ALUOUT,
        output wire PC_PC15,
        output wire IR_MEM,
        output wire A_RF,
        output wire B_RF,
        output wire ALUOut_AB,
        output wire ALUOut_Asign,
        output wire ALUOut_PCsign,
        output wire MDR_MEM,
        output wire MEM_B,
        output wire RF_ALU1st,
        output wire RF_ALU2nd,
        output wire RF_MDR1st,
        output wire RF2_PC, 
        output wire PC_RS,
        output wire isWWD,
        output wire isHLT
        ); 
        reg [3:0]state;
        reg [1:0]instype; // 00->R and I about register 01->LW, 10->SW, 11->branch or jump,
        // IF stage(0 1 2 3)->ID stage(4)->EX stage(5 6)->MEM stage(7 8 9 10)-> Write back stage (11)
        assign IF=(state==0);     //require instruction
        assign IR_MEM=(state==1); //fatch instruction
        assign PC_PC1=(state==1); //PC=PC+1
        //ID stage
        assign A_RF=(state==4);
        assign B_RF=(state==4);
        assign ALUOut_PCsign=(state==4);  
        //EX stage
        assign ALUOut_AB=(state==5)&((opcode==15)|(opcode<4)); // R type and branch and JAL and JRL 정확히 겹치지는 않지만 A만 인가하면 되므로 신호를 합치겠다
        assign ALUOut_Asign=((state==5)&(opcode<9)&(opcode>3)); // for I type and jump 
        assign PC_ALUOUT=(state==5)&(opcode<4); //for conditonal branch
        assign PC_PC15=(state==5)&(opcode>8&opcode<11); //pc relativ jump
        assign PC_RS=(state==5)&(opcode==15)&((func_code==25)|(func_code==26));//for JPR and JRL
        //MEM stage
        assign MDR_MEM=((state==7)|(state==8))&(instype==1); //inst lw
        assign MEM_B=((state==7)|(state==8))&(instype==2);  //inst sw
        //WB stage
        assign RF_ALU1st=(state==11)&((opcode==15)&(func_code<8)); //R type execpt special case JPL은 추가해야할수도 있음
        assign RF_ALU2nd=(state==11)&(opcode>3&opcode<9); //I type execpt lw sw
        assign RF_MDR1st=(state==11)&(instype==1); //Lw
        assign RF2_PC=(state==11)&((opcode==10)|((opcode==15)&(func_code==26)));//for JAL and JRL
        //any time!
        assign isWWD=(opcode==15)&(func_code==28); //isWWD  2 mean ID stage
        assign isHLT=(opcode==15)&(func_code==29); //isHLT
        //state diagram about micro seq 
        // R, I type and JAL, JPR: IF->ID->EX->WB->IF
        // SW: IF->ID->EX->MEM->IF
        // LW: IF->ID->EX->MEM->WB->IF
        // branch or jump: IF->ID->EX->MEM->WB->IF
        always @(posedge clk) begin
         if(!reset_n) begin
           state<=0;
         end
         else begin
         if(state==4) begin //after ID stage 
          if((opcode==15)|(opcode>3)&(opcode<7))instype<=0;
          else if(opcode==7) instype<=1;
          else if(opcode==8) instype<=2;
          else if((opcode>=0&opcode<4)|opcode==9|opcode==10) instype<=3;
          state<=state+1;
         end
         else if(state==6)begin //after ex stage
          if(instype==3 & opcode==9) state<=0; //instype=Br or JMP-> go to IF;
          else if(instype==0 & opcode==10)state<=11; //instype=R or I or JAL-> go to WB;
          else state<=state+1;
         end
         else if(state==10)begin //after MEM
             if(instype==1) state<=state+1; //instype=LW->go to WB;
             else if(instype==2)state<=0; //instype=SW->go to IF stage!
             else state<=state+1; //It's unless if it's works normally!
         end
         else if(state==11)begin //after MEM
            state<=0; //go to IF;
         end
         else state<=state+1;
         end
         end
 endmodule
 
module control( //ALU control도 합쳐서 여기안에 들어가있다.
        input wire [3:0]opcode,
        input wire [5:0]func_code,
        input wire PC_PC1,
        input wire PC_ALUOUT,
        input wire PC_PC15, //jump!
        input wire IR_MEM,
        input wire ALUOut_AB,
        input wire ALUOut_Asign,
        input wire ALUOut_PCsign,
        input wire MDR_MEM,
        input wire MEM_B,
        input wire RF_ALU1st, //I type WB 1st mean rt
        input wire RF_ALU2nd, //R type WB 2nd mean rd
        input wire RF_MDR1st, //I type LW 
        input wire RF2_PC,
        input wire PC_RS,
        output wire ALUSrcA,
        output wire IorD,
        output wire IRWrite,
        output wire PCWrite,
        output wire PCWriteCond,
        output wire [1:0]ALUSrcB,
        output wire [1:0]PCSrc,
        output wire RegDst,
        output wire RegWrite,
        output wire MemRead,
        output wire MemWrite,
        output wire MemtoReg,
        output wire [3:0]ALUOp
        ); 
            wire ALUOpisadd;
            assign ALUSrcA= ALUOut_AB|ALUOut_Asign;
            assign IorD=MDR_MEM|MEM_B;
            assign IRWrite=IR_MEM;
            assign PCWrite=(PC_PC1|PC_PC15|PC_RS);
            assign PCWriteCond=PC_ALUOUT;
            //secode mean source of B, ALUOUT_Asign maen A(ALU 1)+sign(ALU2)
            assign ALUSrcB[1:0]=(ALUOut_Asign|ALUOut_PCsign)?(ALUOut_Asign?2'b10:2'b11):(PC_PC1?2'b01:2'b00);
            assign PCSrc[1]=PC_PC15; //jump
            assign PCSrc[0]=PC_ALUOUT; 
            assign RegDst=(opcode==15);//R type of I type
            assign RegWrite=RF_ALU1st|RF_ALU2nd|RF_MDR1st|RF2_PC;  
            assign MemRead=MDR_MEM;
            assign MemWrite=MEM_B;
            assign MemtoReg=RF_MDR1st;
            assign ALUOpisadd=(opcode==7)|(opcode==8)|ALUOut_PCsign|PC_PC1; //ALU 에서원하는 계산이 add인가?
            assign ALUOp[3:0]=ALUOpisadd?4'b0:(((opcode==15)&(func_code<8))?func_code[3:0]:((opcode<7)?{1'b1,opcode[2:0]}:15));
            // 일반적인 R타입인가? 아니면 I type인가(SWLW)? 아니면 그것도 아닌가(ex.JPR JAL) 
       endmodule

module RF(
       input RegWrite,
       input clk,
       input reset_n,
       input [1:0] addr1,
       input [1:0] addr2,
       input [1:0] addr3,
       output [15:0] data1,
       output [15:0] data2,
       input [15:0] data3
       );
       reg [63:0] regs; //register저장할 메모리 선언
       wire [15:0] reg_wires [3 : 0]; // reg1,reg2,reg3,reg4
       
       assign reg_wires[0] = regs[15:0];
       assign reg_wires[1] = regs[31:16];
       assign reg_wires[2] = regs[47:32];
       assign reg_wires[3] = regs[63:48];
       assign data1=reg_wires[addr1]; //read: data1에 (addr1)번째 reg값 할당
       assign data2=reg_wires[addr2]; //read: data2에 (addr2)번째 reg값 할당
       always @(posedge clk) begin
          if(!reset_n) begin //sync reset
            regs=64'b0; 
           end
           else begin
           if(RegWrite) begin
            case(addr3)
             0: regs[15:0]=data3;
             1: regs[31:16]=data3;
             2: regs[47:32]=data3;
             3: regs[63:48]=data3;
             // write=1, reset=1이면 write: (addr1)번째 reg에 data3값 할당
             endcase
           end
           end
       end
endmodule

module ALU(
   input [15:0] ALU1,
   input [15:0] ALU2,
   input Cin,
   input  [3:0]OP,
   output Cout,
   output [15:0] C,
   output bcond
   );
   reg [16:0] temp;  //계산값을 저장하기 위한 17bit 메모리
   reg bcond_in; //bcond를 계산하기 위한 메모리
   assign Cout=temp[16];
   assign C=temp[15:0];
   assign bcond=bcond_in;
   always @(*) begin
    case(OP)
     0: begin //OP_ADD
     temp=ALU1+ALU2+Cin;
     bcond_in=0;
     end
     1: begin  //OP_SUB
     temp=ALU1+(~ALU2)+2+(~Cin);
     bcond_in=0;
     end
     2: begin //OP_AND
     temp=ALU1&ALU2;
     temp[16]=0;
     bcond_in=0;
     end
     3: begin //OP_OR
     temp=(ALU1|ALU2);
     temp[16]=0;
     bcond_in=0;
     end     
     4: begin //OP_NOT
     temp=~ALU1;
     temp[16]=0;
     bcond_in=0;
     end
     5: begin //OP_TCP
     temp=~ALU1+1;
     temp[16]=0;
     bcond_in=0;
     end
     6: begin //OP_SHL
      temp=ALU1<<1;
      temp[16]=0;
      bcond_in=0;
     end 
     7: begin //OP_SHR
     temp=ALU1>>1;
     temp[15]=ALU1[15];
      bcond_in=0;
     end
     8: begin //OP_BNE
      bcond_in=!(ALU1[15:0]==ALU2[15:0]);
      end
     9: begin //OP_BEQ
      bcond_in=(ALU1==ALU2);
      end
     10: begin //OP_BGZ
      bcond_in=(ALU1[15]==0 & ALU1>0);//signbit is postive and ALU1 is non zero? 
      end
     11: begin //OP_BLZ
      bcond_in=(ALU1[15]==1); //signbit is negative?
      end
     12: begin //OP_ADI or LWD or SWD
     temp=ALU1+ALU2+Cin;
     bcond_in=0;
     end
     13: begin //OP_ORI
     temp=(ALU1|ALU2);
     temp[16]=0;
     bcond_in=0;
     end 
     14: begin //LHI
     temp=ALU2<<8;
     bcond_in=0;
     end
     15: begin // the others thing
        temp[15:0]=ALU1[15:0];
        bcond_in=0;
      end
     default: begin
     end
    endcase 
   end
endmodule

