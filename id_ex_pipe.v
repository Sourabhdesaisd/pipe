module id_ex_pipeline (
    input  wire        clk,
    input  wire        rst,
    input  wire        id_ex_flush,

    // ---------------------------
    // Inputs from Decode stage
    // ---------------------------
    input   [31:0] rs1,
    input   [31:0] rs2,
    input   [31:0] imm_out,
    input   [4:0]  rd,
    input   [6:0]  opcode_in,
    input   [2:0]  func3_in,
    input   [6:0]  func7_in,

    input          ex_alu_src_in,
    input          mem_write_in,
    input          mem_read_in,
    input   [2:0]  mem_load_type_in,
    input   [1:0]  mem_store_type_in,
    input          wb_reg_file_in,
    input          memtoreg_in,
    input          branch_in,
    input          jal_in,
    input          jalr_in,
    input          auipc_in,
    input          lui_in,
    input   [3:0]  alu_ctrl_in,

    // ---------------------------
    // Outputs to Execute stage
    // ---------------------------
    output reg [31:0] rs1_out,
    output reg [31:0] rs2_out,
    output reg [31:0] imm_out,
    output reg [4:0]  rd_out,
    output reg [6:0]  opcode_out,
    output reg [2:0]  func3_out,
    output reg [6:0]  func7_out,

    output reg        ex_alu_src_out,
    output reg        mem_write_out,
    output reg        mem_read_out,
    output reg [2:0]  mem_load_type_out,
    output reg [1:0]  mem_store_type_out,
    output reg        wb_reg_file_out,
    output reg        memtoreg_out,
    output reg        branch_out,
    output reg        jal_out,
    output reg        jalr_out,
    output reg        auipc_out,
    output reg        lui_out,
    output reg [3:0]  alu_ctrl_out
);

    always @(posedge clk or posedge rst) begin
        if (rst || id_ex_flush) begin
            // Clear all outputs
            rs1_out <= 32'b0;
            rs2_out <= 32'b0;
            imm_out <= 32'b0;
            rd_out  <= 5'b0;
            opcode_out <= 7'b0;
            func3_out <= 3'b0;
            func7_out <= 7'b0;

            ex_alu_src_out <= 0;
            mem_write_out  <= 0;
            mem_read_out   <= 0;
            mem_load_type_out <= 3'b0;
            mem_store_type_out <= 2'b0;
            wb_reg_file_out <= 0;
            memtoreg_out <= 0;
            branch_out <= 0;
            jal_out <= 0;
            jalr_out <= 0;
            auipc_out <= 0;
            lui_out <= 0;
            alu_ctrl_out <= 4'b0;
        end
        else begin
            // Latch values
            rs1_out <= rs1;
            rs2_out <= rs2;
            imm_out <= imm_out;
            rd_out  <= rd;
            opcode_out <= opcode_in;
            func3_out <= func3_in;
            func7_out <= func7_in;

            ex_alu_src_out <= ex_alu_src_in;
            mem_write_out  <= mem_write_in;
            mem_read_out   <= mem_read_in;
            mem_load_type_out <= mem_load_type_in;
            mem_store_type_out <= mem_store_type_in;
            wb_reg_file_out <= wb_reg_file_in;
            memtoreg_out <= memtoreg_in;
            branch_out <= branch_in;
            jal_out <= jal_in;
            jalr_out <= jalr_in;
            auipc_out <= auipc_in;
            lui_out <= lui_in;
            alu_ctrl_out <= alu_ctrl_in;
        end
    end

endmodule

