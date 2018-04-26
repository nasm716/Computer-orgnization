`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/15 16:44:52
// Design Name: 
// Module Name: ALU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module ALU(
    input [15:0] A,
    input [15:0] B,
    input Cin,
    input [3:0] OP,
    output Cout,
    output [15:0] C
    );
    reg [16:0] temp;  //계산값을 저장하기 위한 17bit 메모리
    assign Cout=temp[16];
    assign C=temp[15:0];
    always @(*) begin
     case(OP)
      0: begin //OP_ADD
      temp=A+B+Cin;
      end
      1: begin  //OP_ADD
      temp=A+(~B)+2+(~Cin);
      end
      2: begin //OP_ID
      temp=A;
      end
      3: begin //OP_NAND
      temp=~(A&B);
      temp[16]=0;
      end     
      4: begin //OP_NOR
      temp=~(A|B);
      temp[16]=0;
      end
      5: begin //OP_XONR
      temp=~(A^B);
      temp[16]=0;
      end
      6: begin //OP_NOT
      temp=~A;
      temp[16]=0;
      end 
      7: begin //OP_AND
      temp=A&B;
      temp[16]=0;
      end 
      8: begin //OP_OR
      temp=A|B;
      temp[16]=0;
      end
      9: begin //OP_XOR
      temp=A^B;
      temp[16]=0;
      end 
      10: begin //OP_LRS
      temp=A>>1;
      end
      11: begin //OP_ARS
      temp=A>>1;
      temp[15]=A[15];
      end
      12: begin //OP_RR
      temp=A>>1;
      temp[15]=A[0];
      end
      13: begin //OP_LLS
      temp=A<<1;
      temp[16]=0;
      end
      14: begin //OP_ALS
      temp=A<<1;
      temp[0]=A[0];
      temp[16]=0;
      end
      15: begin //OP_RL
      temp=A<<1;
      temp[0]=A[15];
      temp[16]=0;
       end  
      default: begin
      end
     endcase 
    end
    
endmodule
