`define ZERO_12    12'h000
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

module pc_jump(
    input  [31:0] pc,
    input  [31:0] imm,    // immediate (pass imm_out)
    input  [31:0] rs1,    // register-file rs1

    input  [6:0]  opcode,
    input  [2:0]  func3,

    input         carry_flag,
    input         zero_flag,
    input         negative_flag,
    input         overflow_flag,

    input         predictedTaken,

    output [31:0] update_pc,
    output [31:0] jump_addr,
    output        modify_pc,
    output        update_btb
);

    wire [31:0] input_a;
    wire [31:0] adder_out;
    wire [31:0] pc_inc;

    wire        jump_inst, branch_inst;
    wire        jalr_inst;
    wire        branch_taken;
    wire        jump_en;

    // decode types
    assign jalr_inst   = (opcode == `OPCODE_IJALR);
    assign jump_inst   = (opcode == `OPCODE_JTYPE) || jalr_inst;
    assign branch_inst = (opcode == `OPCODE_BTYPE);

    // BTB update for any control-flow
    assign update_btb  = jump_inst || branch_inst;

    // branch condition (from flags)
    wire beq  = (func3 == `BTYPE_BEQ);
    wire bne  = (func3 == `BTYPE_BNE);
    wire blt  = (func3 == `BTYPE_BLT);
    wire bge  = (func3 == `BTYPE_BGE);
    wire bltu = (func3 == `BTYPE_BLTU);
    wire bgeu = (func3 == `BTYPE_BGEU);

    assign branch_taken =
           (beq  &&  zero_flag)                          ||
           (bne  && ~zero_flag)                          ||
           (blt  && (negative_flag != overflow_flag))    || // signed < 
           (bge  && (negative_flag == overflow_flag))    || // signed >=
           (bltu &&  carry_flag)                         || // unsigned <
           (bgeu && ~carry_flag);                           // unsigned >=

    // jump / branch enable
    assign jump_en   = jump_inst || (branch_inst && branch_taken);

    // mispredict detection
    assign modify_pc = jump_en ^ predictedTaken;

    // target address
    assign input_a   = jalr_inst ? rs1 : pc;
    assign adder_out = input_a + imm;

    // JALR must clear LSB
    assign jump_addr = jalr_inst ? (adder_out & 32'hFFFF_FFFE) : adder_out;

    // next PC on mispredict
    assign pc_inc = pc + 32'h4;

    assign update_pc = modify_pc ? (jump_en ? jump_addr : pc_inc)
                                 : pc_inc;
endmodule
