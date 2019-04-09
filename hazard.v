
// ---------------------------
// Minimal defines (used by hazard/forwarding/others)
// Keep these in a central file if you already have one.
// ---------------------------
`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_IJALR 7'b1100111
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111

// -------------------------------------------------
// hazard_unit (as provided earlier, compile-safe)
// -------------------------------------------------
module hazard_unit (
    input  wire [4:0] id_rs1,
    input  wire [4:0] id_rs2,
    input  wire [6:0] opcode,
    input  wire [4:0] ex_rd,
    input  wire       ex_load_inst,
    input  wire       jump_branch_taken,
    input  wire       invalid_inst,
    input  wire       modify_pc,
    output reg  if_id_pipeline_flush,
    output reg  if_id_pipeline_en,
    output reg  id_ex_pipeline_flush,
    output reg  id_ex_pipeline_en,
    output reg  pc_en,
    output reg  load_stall
);
    wire id_rs2_used = (opcode == `OPCODE_RTYPE) ||
                       (opcode == `OPCODE_STYPE) ||
                       (opcode == `OPCODE_BTYPE);

    wire id_rs1_used = (opcode == `OPCODE_ITYPE) ||
                       (opcode == `OPCODE_ILOAD) ||
                       (opcode == `OPCODE_IJALR) ||
                        id_rs2_used;

    wire rs1_hazard = id_rs1_used && (id_rs1 != 5'b00000) && (id_rs1 == ex_rd);
    wire rs2_hazard = id_rs2_used && (id_rs2 != 5'b00000) && (id_rs2 == ex_rd);
    wire load_hazard = ex_load_inst && (ex_rd != 5'b00000) && (rs1_hazard || rs2_hazard);

    always @(*) begin
        if_id_pipeline_flush = 1'b0;
        if_id_pipeline_en    = 1'b1;
        id_ex_pipeline_flush = 1'b0;
        id_ex_pipeline_en    = 1'b1;
        pc_en                = 1'b1;
        load_stall           = 1'b0;

        if (jump_branch_taken && modify_pc) begin
            if_id_pipeline_flush = 1'b1;
            id_ex_pipeline_flush = 1'b1;
            pc_en = 1'b1;
        end
        else if (load_hazard) begin
            if_id_pipeline_en = 1'b0;
            id_ex_pipeline_en = 1'b0;
            pc_en = 1'b0;
            load_stall = 1'b1;
        end
        else if (invalid_inst) begin
            id_ex_pipeline_flush = 1'b1;
        end
    end
endmodule



