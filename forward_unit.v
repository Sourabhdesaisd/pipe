// forwarding_unit.v
// Produces forwarding selects for EX operands.
// Priority: EX/MEM (most recent) -> MEM/WB -> original
/*
module forwarding_unit (
    input  wire [4:0] rs1,            // source reg index (EX stage)
    input  wire [4:0] rs2,            // source reg index (EX stage)
    input  wire [4:0] rd_mem,         // dest reg in EX/MEM (or mem stage)
    input  wire [4:0] rd_wb,          // dest reg in MEM/WB (WB stage)
    input  wire       reg_file_wr_mem,// EX/MEM will write back?
    input  wire       reg_file_wr_wb, // MEM/WB will write back?

    output reg  [1:0] operand_a_cntl, // 00 = original, 01 = forward from MEM, 10 = forward from WB
    output reg  [1:0] operand_b_cntl
);

    // Valid destination flags (rd != x0 and write-enable true)
    wire valid_rd_mem = (rd_mem != 5'b00000) && reg_file_wr_mem;
    wire valid_rd_wb  = (rd_wb  != 5'b00000) && reg_file_wr_wb;

    // Matches
    wire rs1_matches_mem = valid_rd_mem && (rs1 == rd_mem);
    wire rs1_matches_wb  = valid_rd_wb  && (rs1 == rd_wb);
    wire rs2_matches_mem = valid_rd_mem && (rs2 == rd_mem);
    wire rs2_matches_wb  = valid_rd_wb  && (rs2 == rd_wb);

    // Priority: MEM (most recent) over WB
    always @(*) begin
        if (rs1_matches_mem)
            operand_a_cntl = 2'b01; // forward from MEM stage
        else if (rs1_matches_wb)
            operand_a_cntl = 2'b10; // forward from WB stage
        else
            operand_a_cntl = 2'b00; // use original value from ID/EX

        if (rs2_matches_mem)
            operand_b_cntl = 2'b01; // forward from MEM stage
        else if (rs2_matches_wb)
            operand_b_cntl = 2'b10; // forward from WB stage
        else
            operand_b_cntl = 2'b00; // use original value from ID/EX
    end

endmodule
  */


 // -------------------------------------------------
// forwarding_unit (as provided earlier, compile-safe)
// -------------------------------------------------
module forwarding_unit (
    input  wire [4:0] rs1,
    input  wire [4:0] rs2,
    input  wire [4:0] rd_mem,
    input  wire [4:0] rd_wb,
    input  wire       reg_file_wr_mem,
    input  wire       reg_file_wr_wb,
    output reg  [1:0] operand_a_cntl,
    output reg  [1:0] operand_b_cntl
);
    wire valid_rd_mem = (rd_mem != 5'b00000) && reg_file_wr_mem;
    wire valid_rd_wb  = (rd_wb  != 5'b00000) && reg_file_wr_wb;

    wire rs1_matches_mem = valid_rd_mem && (rs1 == rd_mem);
    wire rs1_matches_wb  = valid_rd_wb  && (rs1 == rd_wb);
    wire rs2_matches_mem = valid_rd_mem && (rs2 == rd_mem);
    wire rs2_matches_wb  = valid_rd_wb  && (rs2 == rd_wb);

    always @(*) begin
        if (rs1_matches_mem)
            operand_a_cntl = 2'b01;
        else if (rs1_matches_wb)
            operand_a_cntl = 2'b10;
        else
            operand_a_cntl = 2'b00;

        if (rs2_matches_mem)
            operand_b_cntl = 2'b01;
        else if (rs2_matches_wb)
            operand_b_cntl = 2'b10;
        else
            operand_b_cntl = 2'b00;
    end
endmodule


