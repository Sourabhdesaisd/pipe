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

module simple_alu (
    input  wire [31:0] rs1,
    input  wire [31:0] rs2,
    input  wire [31:0] imm_out,
    input  wire        ex_alu_src,
    input  wire [3:0]  alu_ctrl,
    input  wire [6:0]  opcode,
    input  wire [2:0]  func3,
    input  wire [6:0]  func7,

    output reg  [31:0] result_alu,
    output wire        carry_flag,
    output wire        zero_flag,
    output wire        negative_flag,
    output wire        overflow_flag
);

    // Actual ALU Operand2 from mux
    wire [31:0] rs2_final;

    alu_operand_mux mux_op2 (
        .rs2(rs2),
        .imm_out(imm_out),
        .ex_alu_src(ex_alu_src),
        .rs2_final(rs2_final)
    );

    // SUB uses two’s complement
    wire [31:0] b_sel = (alu_ctrl == `ALU_SUB) ? ~rs2_final : rs2_final;
    wire        cin_sel = (alu_ctrl == `ALU_SUB);

    wire [31:0] add_res;
    wire add_c, add_z, add_n, add_o;

    ripple_carry_adder32 add_u (
        .a(rs1),
        .b(b_sel),
        .cin(cin_sel),
        .sum(add_res),
        .carry_flag(add_c),
        .zero_flag(add_z),
        .negative_flag(add_n),
        .overflow_flag(add_o)
    );

    wire [31:0] cmp_res;
    comparator_unit32 cmp_u (.rs1(rs1), .rs2(rs2_final), .opcode(opcode), .func3(func3), .Y(cmp_res));

    wire [31:0] log_res;
    logical_unit32 log_u (.rs1(rs1), .rs2(rs2_final), .opcode(opcode), .func3(func3), .Y(log_res));

    wire [31:0] shift_res;
    shifter_unit32 sh_u (.rs1(rs1), .rs2(rs2_final), .opcode(opcode), .func3(func3), .func7(func7), .imm_out(imm_out), .Y(shift_res));

    // Final ALU result
  /*  always @(*) begin
        case (alu_ctrl)
            `ALU_ADD:  result_alu = add_res;
            `ALU_SUB:  result_alu = add_res;
            `ALU_SLT:  result_alu = cmp_res;
            `ALU_SLTU: result_alu = cmp_res;
            `ALU_XOR:  result_alu = log_res;
            `ALU_OR:   result_alu = log_res;
            `ALU_AND:  result_alu = log_res;
            `ALU_SLL:  result_alu = shift_res;
            `ALU_SRL:  result_alu = shift_res;
            `ALU_SRA:  result_alu = shift_res;
            default:   result_alu = add_res;
        endcase
    end */

    assign carry_flag    = add_c;
    assign zero_flag     = add_z;
    assign negative_flag = add_n;
    assign overflow_flag = add_o;

endmodule

