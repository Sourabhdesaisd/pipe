module mux_alu_src (
    input  [31:0] rs2,
    input  [31:0] imm_out,
    input         ex_alu_src,      // 0 ? take rs2, 1 ? take imm
    output [31:0] alu_imm_rs2
);
    assign alu_imm_rs2 = (ex_alu_src) ? imm_out : rs2;
endmodule

