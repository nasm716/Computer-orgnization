`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/23 03:13:23
// Design Name: 
// Module Name: rf
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


module RF(
    input write,
    input clk,
    input reset_n,
    input [1:0] addr1,
    input [1:0] addr2,
    input [1:0] addr3,
    output [15:0] data1,
    output [15:0] data2,
    input [15:0] data3
    );
    reg [63:0] regs; //register������ �޸� ����
    wire [15:0] reg_wires [3 : 0]; // reg1,reg2,reg3,reg4
    
    assign reg_wires[0] = regs[15:0];
    assign reg_wires[1] = regs[31:16];
    assign reg_wires[2] = regs[47:32];
    assign reg_wires[3] = regs[63:48];
    assign data1=reg_wires[addr1]; //read: data1�� (addr1)��° reg�� �Ҵ�
    assign data2=reg_wires[addr2]; //read: data2�� (addr2)��° reg�� �Ҵ�
    always @(posedge clk) begin
       if(!reset_n) begin //sync reset
         regs=64'b0; 
        end
        else begin
        if(write) begin
         case(addr3)
          0: regs[15:0]=data3;
          1: regs[31:16]=data3;
          2: regs[47:32]=data3;
          3: regs[63:48]=data3;
          // write=1, reset=1�̸� write: (addr1)��° reg�� data3�� �Ҵ�
          endcase
        end
        end
    end 
endmodule
