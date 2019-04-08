module alu_operand_mux (
    input  wire [31:0] rs2,        // from register file
    input  wire [31:0] imm_out,    // from decode
    input  wire        ex_alu_src, // 1 ? use imm_out
    output wire [31:0] rs2_final   // selected operand2 for ALU
);

    assign rs2_final = ex_alu_src ? imm_out : rs2;

endmodule

