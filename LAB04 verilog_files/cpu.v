///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: cpu.v
// Author: 
// Description: 

// DEFINITIONS
`define WORD_SIZE 16    // data and address word size

// MODULE DECLARATION
module cpu (
    output readM,                       // read from memory
    output [`WORD_SIZE-1:0] address,    // current address for data
    inout [`WORD_SIZE-1:0] data,        // data being input or output
    input inputReady,                   // indicates that data is ready from the input port
    input reset_n,                      // active-low RESET signal
    input clk,                          // clock signal
  
    // for debuging/testing purpose
    output [`WORD_SIZE-1:0] num_inst,   // number of instruction during execution
    output [`WORD_SIZE-1:0] output_port // this will be used for a "WWD" instruction
);
  
    // Datapath - control Unit
    wire [3:0] opcode;
    wire [5:0] func_code;
    wire RegDst, Jump, ALUSrc, RegWrite, isWWD;
    wire ALUOp;
    
    control_unit Control (
        .reset_n (reset_n),
        .opcode (opcode),
        .func_code (func_code),
        .RegDst (RegDst),
        .Jump (Jump),
        .ALUOp(ALUOp), // it's ALUOpertaion when initial
        .ALUSrc (ALUSrc), 
        .RegWrite (RegWrite),
        .isWWD (isWWD)
        ); 
        
    datapath #(.WORD_SIZE (`WORD_SIZE)) 
        DP (
        .clk(clk),
        .reset_n (reset_n),
        .inputReady (inputReady),
        .data (data),
        .readM (readM),
        .address (address),
        .num_inst (num_inst),
        .output_port (output_port),
        
        .RegDst (RegDst),
        .RegWrite (RegWrite),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),    
        .Jump (Jump),
        .isWWD (isWWD),
        
        .opcode(opcode),
        .func_code (func_code)
        );    
         
  // ... fill in the rest of the code

endmodule
//////////////////////////////////////////////////////////////////////////
