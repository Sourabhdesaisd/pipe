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
//`timescale 1ns/1ps

module decode_controller_pipelined (
    input  wire [6:0] opcode,
    input  wire [2:0] func3,
    input  wire [6:0] func7,

    // Datapath control outputs
    output reg        ex_alu_src,      // 1 -> use immediate as ALU operand 2
    output reg        mem_write,       // store to memory
    output reg        mem_read,        // load from memory
    output reg [2:0]  mem_load_type,   // LOAD_LB/LH/LW/LBU/LHU/LOAD_DEF
    output reg [1:0]  mem_store_type,  // STORE_SB/SH/SW/STORE_DEF

    // Register-file write enable and mem-to-reg
    output reg        wb_reg_file,     // write enable for regfile
    output reg        memtoreg,        // 1 -> writeback data comes from memory (loads)

    // Branch / Jump / PC control hints
    output reg        branch,          // B-type
    output reg        jal,             // JAL
    output reg        jalr,            // JALR (I-type)
    output reg        auipc,           // AUIPC (U-type)
    output reg        lui,             // LUI (U-type)

    // Simple ALU control (map to ALU_* defines you already have)
    output reg [3:0]  alu_ctrl         // ALU_ADD, ALU_SUB, ALU_AND, ...
);

    // Local opcode class wires
    wire r_type  = (opcode == `OPCODE_RTYPE);
    wire i_type  = (opcode == `OPCODE_ITYPE);
    wire iload   = (opcode == `OPCODE_ILOAD);
    wire stype   = (opcode == `OPCODE_STYPE);
    wire btype   = (opcode == `OPCODE_BTYPE);
    wire jtype   = (opcode == `OPCODE_JTYPE);
    wire auipc_o = (opcode == `OPCODE_AUIPC);
    wire utype   = (opcode == `OPCODE_UTYPE);
    wire jalr_o  = (opcode == `OPCODE_IJALR);

    always @(*) begin
        // default control values
        ex_alu_src    = 1'b0;
        mem_write     = 1'b0;
        mem_read      = 1'b0;
        mem_load_type = `LOAD_DEF;
        mem_store_type= `STORE_DEF;
        wb_reg_file   = 1'b0;
        memtoreg      = 1'b0;
        branch        = 1'b0;
        jal           = 1'b0;
        jalr          = 1'b0;
        auipc         = 1'b0;
        lui           = 1'b0;
        alu_ctrl      = `ALU_ADD; // default

        // classify opcode -> set enables & wb_reg_file
        if (r_type) begin
            ex_alu_src  = 1'b0;
            wb_reg_file = 1'b1;   // ALU result -> regfile
            memtoreg    = 1'b0;
        end
        else if (i_type) begin
            // I-type arithmetic (immediate used)
            ex_alu_src  = 1'b1;
            wb_reg_file = 1'b1;   // ALU result -> regfile
            memtoreg    = 1'b0;
        end
        else if (iload) begin
            // Loads: address calculation uses ALU (rs1 + imm), read from memory, writeback from memory
            ex_alu_src  = 1'b1;
            mem_read    = 1'b1;
            wb_reg_file = 1'b1;
            memtoreg    = 1'b1;   // select memory for WB
        end
        else if (stype) begin
            ex_alu_src  = 1'b1;
            mem_write   = 1'b1;
            wb_reg_file = 1'b0;
            memtoreg    = 1'b0;
        end
        else if (btype) begin
            branch      = 1'b1;
            ex_alu_src  = 1'b0;
            wb_reg_file = 1'b0;
            memtoreg    = 1'b0;
        end
        else if (jtype) begin
            jal         = 1'b1;
            wb_reg_file = 1'b1;   // PC+4 -> regfile
            memtoreg    = 1'b0;
        end
        else if (jalr_o) begin
            jalr        = 1'b1;
            ex_alu_src  = 1'b1;
            wb_reg_file = 1'b1;   // PC+4 -> regfile
            memtoreg    = 1'b0;
        end
        else if (utype) begin
            // LUI: write imm<<12 to rd (ALU/imm result)
            lui         = 1'b1;
            wb_reg_file = 1'b1;
            memtoreg    = 1'b0;
            // ex_alu_src intentionally left 0 (design convention)
        end
        else if (auipc_o) begin
            auipc       = 1'b1;
            wb_reg_file = 1'b1;
            memtoreg    = 1'b0;
            // ex_alu_src intentionally left 0 (design convention)
        end

        // mem load/store type decode
// mem load type decode
if (mem_read) begin
    case (func3)
        3'b000: mem_load_type = `LOAD_LB;   // LB
        3'b001: mem_load_type = `LOAD_LH;   // LH
        3'b010: mem_load_type = `LOAD_LW;   // LW
        3'b100: mem_load_type = `LOAD_LBU;  // LBU (func3 == 100)
        3'b101: mem_load_type = `LOAD_LHU;  // LHU (func3 == 101)
        default: mem_load_type = `LOAD_DEF;
    endcase
end
        
        if (mem_write) begin
            case (func3)
                3'b000: mem_store_type = `STORE_SB;
                3'b001: mem_store_type = `STORE_SH;
                3'b010: mem_store_type = `STORE_SW;
                default: mem_store_type = `STORE_DEF;
            endcase
        end

        // ALU control
        if (r_type) begin
            case (func3)
                3'b000: alu_ctrl = (func7 == `FUNC7_SUB) ? `ALU_SUB : `ALU_ADD;
                3'b001: alu_ctrl = `ALU_SLL;
                3'b010: alu_ctrl = `ALU_SLT;
                3'b011: alu_ctrl = `ALU_SLTU;
                3'b100: alu_ctrl = `ALU_XOR;
                3'b101: alu_ctrl = (func7 == `FUNC7_SUB) ? `ALU_SRA : `ALU_SRL;
                3'b110: alu_ctrl = `ALU_OR;
                3'b111: alu_ctrl = `ALU_AND;
                default: alu_ctrl = `ALU_ADD;
            endcase
        end
        else if (iload) begin
            // Loads use ADD for address calculation
            alu_ctrl = `ALU_ADD;
        end
        else if (i_type || jalr_o) begin
            // I-type immediates (shifts handled by func7)
            case (func3)
                3'b000: alu_ctrl = `ALU_ADD;
                3'b001: alu_ctrl = `ALU_SLL;
                3'b010: alu_ctrl = `ALU_SLT;
                3'b011: alu_ctrl = `ALU_SLTU;
                3'b100: alu_ctrl = `ALU_XOR;
                3'b101: alu_ctrl = (func7 == `FUNC7_SUB) ? `ALU_SRA : `ALU_SRL;
                3'b110: alu_ctrl = `ALU_OR;
                3'b111: alu_ctrl = `ALU_AND;
                default: alu_ctrl = `ALU_ADD;
            endcase
        end
        else if (btype) begin
            alu_ctrl = `ALU_SUB;
        end
        else begin
            alu_ctrl = `ALU_ADD;
        end
    end // always
endmodule

