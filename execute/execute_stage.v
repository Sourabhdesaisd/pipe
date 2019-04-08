
module execute_top (
    // inputs (from decode stage)
    input  wire [31:0] pc,
    input  wire [31:0] rs1,
    input  wire [31:0] rs2,        // raw register-file rs2
    input  wire [31:0] imm_out,    // immediate from decode
    input  wire        ex_alu_src, // select ALU operand2 (1 = imm, 0 = rs2)
    input  wire [3:0]  alu_ctrl,   // ALU control code
    input  wire [6:0]  opcode,
    input  wire [2:0]  func3,
    input  wire [6:0]  func7,
    input  wire        predictedTaken, // branch predictor decision

    // alu outputs (to MEM/WB)
    output wire [31:0] result_alu,
    output wire        carry_flag,
    output wire        zero_flag,
    output wire        negative_flag,
    output wire        overflow_flag,

    // forwarded operand (to MEM stage for store data or for debugging)
    output wire [31:0] rs2_final,

    // branch/jump outputs (to IF/BTB/IF stage)
    output wire [31:0] update_pc,
    output wire [31:0] jump_addr,
    output wire        modify_pc,
    output wire        update_btb
);

    // External operand mux instance (for forwarding the exact operand used by ALU)
    // NOTE: simple_alu also contains/instantiates its own alu_operand_mux in your supplied code,
    // so you will have two muxes in the synthesized design unless you choose to refactor.
    alu_operand_mux u_operand_mux (
        .rs2(rs2),
        .imm_out(imm_out),
        .ex_alu_src(ex_alu_src),
        .rs2_final(rs2_final)
    );

    // Instantiate the ALU.
    // Keep using your simple_alu as-is (it also will internally instantiate its own mux).
    // We pass original rs2 and imm_out & ex_alu_src to simple_alu so it behaves unchanged.
    simple_alu u_simple_alu (
        .rs1(rs1),
        .rs2(rs2),
        .imm_out(imm_out),
        .ex_alu_src(ex_alu_src),
        .alu_ctrl(alu_ctrl),
        .opcode(opcode),
        .func3(func3),
        .func7(func7),

        .result_alu(result_alu),
        .carry_flag(carry_flag),
        .zero_flag(zero_flag),
        .negative_flag(negative_flag),
        .overflow_flag(overflow_flag)
    );

    // Instantiate PC/branch/jump unit
    pc_jump u_pc_jump (
        .pc(pc),
        .imm(imm_out),
        .rs1(rs1),

        .opcode(opcode),
        .func3(func3),

        .carry_flag(carry_flag),
        .zero_flag(zero_flag),
        .negative_flag(negative_flag),
        .overflow_flag(overflow_flag),

        .predictedTaken(predictedTaken),

        .update_pc(update_pc),
        .jump_addr(jump_addr),
        .modify_pc(modify_pc),
        .update_btb(update_btb)
    );

endmodule

