// OPCODES
`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_IJALR 7'b1100111
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111

// FUNC7 - ADD
`define FUNC7_ADD 7'b0000000
`define FUNC7_SUB 7'b0100000

// ALU Codes
`define ALU_ADD  4'b0000
`define ALU_SUB  4'b0001
`define ALU_AND  4'b0010
`define ALU_OR   4'b0011
`define ALU_XOR  4'b0100
`define ALU_SLL  4'b0101
`define ALU_SRL  4'b0110
`define ALU_SRA  4'b0111
`define ALU_SLT  4'b1000
`define ALU_SLTU 4'b1001

// B Type Codes
`define BTYPE_BEQ  3'b000
`define BTYPE_BNE  3'b001
`define BTYPE_BLT  3'b100
`define BTYPE_BGE  3'b101
`define BTYPE_BLTU 3'b110
`define BTYPE_BGEU 3'b111

// Forwarding Unit
`define FORWARD_ORG 2'b00
`define FORWARD_MEM 2'b01
`define FORWARD_WB  2'b10

// Store Types
`define STORE_SB  2'b00
`define STORE_SH  2'b01
`define STORE_SW  2'b10
`define STORE_DEF 2'b11

// Load Types
`define LOAD_LB  3'b000
`define LOAD_LH  3'b001   // FIXED NAME
`define LOAD_LW  3'b010
`define LOAD_LBU 3'b011
`define LOAD_LHU 3'b100
`define LOAD_DEF 3'b111

// Constants
`define ZERO_32BIT  32'h00000000
`define ZERO_12BIT  12'h000

// BTB State
`define STRONG_NOT_TAKEN 2'b00
`define WEAK_NOT_TAKEN   2'b01
`define STRONG_TAKEN     2'b10
`define WEAK_TAKEN       2'b11

module decode_stage(
    input clk,
    input rst,
    input id_flush,
    input [31:0] instruction_in,
    input reg_file_wr_en,
    input [4:0] reg_file_wr_addr,
    input [31:0] reg_file_wr_data,
    
    output [31:0] op1,
    output [31:0] op2,
    output [4:0] rs1,
    output [4:0] rs2,
    output [4:0] rd,
    output reg [31:0] immediate,
    output [6:0] opcode,
    output alu_src,
    output invalid_inst,
    output [6:0] func7,
    output [2:0] func3,
    output mem_write,
    output mem_read,
    output [2:0] mem_load_type,
    output[1:0] mem_store_type,
    output wb_reg_file
);
    wire [31:0] instruction;

    assign instruction = id_flush ? 32'h00000000 : instruction_in;

    assign opcode = instruction[6:0];
    assign rd = instruction[11:7];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign func7 = instruction[31:25];
    assign func3 = instruction[14:12];

    always @(*) begin
        case (opcode)
            `OPCODE_STYPE: 
                immediate = {{20{instruction[31]}},instruction[31:25],instruction[11:7]}; 
            `OPCODE_JTYPE: 
                immediate = {{11{instruction[31]}},instruction[31],instruction[19:12],instruction[20],instruction[30:21],1'b0};
            `OPCODE_BTYPE: 
                immediate = {{19{instruction[31]}},instruction[31],instruction[7],instruction[30:25],instruction[11:8],1'b0};
            `OPCODE_UTYPE: 
                immediate = {instruction[31:12],`ZERO_12BIT};
            `OPCODE_AUIPC: 
                immediate = {instruction[31:12],`ZERO_12BIT};
            `OPCODE_ITYPE:       
                immediate = {{20{instruction[31]}},instruction[31:20]};
              default: immediate =0 ;
        endcase
    end
    endmodule
    
