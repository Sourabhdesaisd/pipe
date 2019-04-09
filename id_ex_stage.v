module id_ex_reg(
    input  wire        clk,
    input  wire        rst,
    input  wire        en,       // enable capture; tie 1 if no stall logic
    input  wire        flush,    // when asserted, insert bubble (clear control signals)

    // Optional PC from IF/ID (connect if available)
    input  wire [31:0] pc_id,
    output reg  [31:0] pc_ex,

    // Instruction fields (from ID)
    input  wire [6:0]  opcode,
    input  wire [2:0]  func3,
    input  wire [6:0]  func7,
    input  wire [4:0]  rd,
    input  wire [4:0]  rs1,
    input  wire [4:0]  rs2,
    input  wire [31:0] imm_out,

    // Register file read data (from ID)
    input  wire [31:0] rs1_data,
    input  wire [31:0] rs2_data,

    // Control signals (from controller in ID)
    input  wire        ex_alu_src,
    input  wire        mem_write,
    input  wire        mem_read,
    input  wire [2:0]  mem_load_type,
    input  wire [1:0]  mem_store_type,
    input  wire        wb_reg_file,
    input  wire        memtoreg,
    input  wire        branch,
    input  wire        jal,
    input  wire        jalr,
    input  wire        auipc,
    input  wire        lui,
    input  wire [3:0]  alu_ctrl,

    // Outputs to EX stage (suffixed _ex)
    output reg  [6:0]  opcode_ex,
    output reg  [2:0]  func3_ex,
    output reg  [6:0]  func7_ex,
    output reg  [4:0]  rd_ex,
    output reg  [4:0]  rs1_ex,
    output reg  [4:0]  rs2_ex,
    output reg  [31:0] imm_ex,

    output reg  [31:0] rs1_data_ex,
    output reg  [31:0] rs2_data_ex,

    output reg         ex_alu_src_ex,
    output reg         mem_write_ex,
    output reg         mem_read_ex,
    output reg  [2:0]  mem_load_type_ex,
    output reg  [1:0]  mem_store_type_ex,
    output reg         wb_reg_file_ex,
    output reg         memtoreg_ex,
    output reg         branch_ex,
    output reg         jal_ex,
    output reg         jalr_ex,
    output reg         auipc_ex,
    output reg         lui_ex,
    output reg  [3:0]  alu_ctrl_ex
);

    // Safe defaults for controls and data (used on reset/flush)
    localparam [31:0] ZERO32 = 32'h00000000;
    localparam [6:0]  ZERO7  = 7'h00;
    localparam [4:0]  ZERO5  = 5'h00;
    localparam [3:0]  ZERO4  = 4'h0;
    localparam [2:0]  ZERO3  = 3'h0;
    localparam [1:0]  ZERO2  = 2'h0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // reset all pipeline registers to safe values
            pc_ex             <= ZERO32;

            opcode_ex         <= ZERO7;
            func3_ex          <= ZERO3;
            func7_ex          <= ZERO7;
            rd_ex             <= ZERO5;
            rs1_ex            <= ZERO5;
            rs2_ex            <= ZERO5;
            imm_ex            <= ZERO32;

            rs1_data_ex       <= ZERO32;
            rs2_data_ex       <= ZERO32;

            ex_alu_src_ex     <= 1'b0;
            mem_write_ex      <= 1'b0;
            mem_read_ex       <= 1'b0;
            mem_load_type_ex  <= ZERO3;
            mem_store_type_ex <= ZERO2;
            wb_reg_file_ex    <= 1'b0;
            memtoreg_ex       <= 1'b0;
            branch_ex         <= 1'b0;
            jal_ex            <= 1'b0;
            jalr_ex           <= 1'b0;
            auipc_ex          <= 1'b0;
            lui_ex            <= 1'b0;
            alu_ctrl_ex       <= ZERO4;
        end
        else if (!en) begin
            // hold current values (stall)
            // nothing to do here - keep previous registers
        end
        else begin
            if (flush) begin
                // Insert bubble: clear control signals and data -> safe NOP in EX
                pc_ex             <= ZERO32;

                opcode_ex         <= ZERO7;
                func3_ex          <= ZERO3;
                func7_ex          <= ZERO7;
                rd_ex             <= ZERO5;
                rs1_ex            <= ZERO5;
                rs2_ex            <= ZERO5;
                imm_ex            <= ZERO32;

                rs1_data_ex       <= ZERO32;
                rs2_data_ex       <= ZERO32;

                ex_alu_src_ex     <= 1'b0;
                mem_write_ex      <= 1'b0;
                mem_read_ex       <= 1'b0;
                mem_load_type_ex  <= ZERO3;
                mem_store_type_ex <= ZERO2;
                wb_reg_file_ex    <= 1'b0;
                memtoreg_ex       <= 1'b0;
                branch_ex         <= 1'b0;
                jal_ex            <= 1'b0;
                jalr_ex           <= 1'b0;
                auipc_ex          <= 1'b0;
                lui_ex            <= 1'b0;
                alu_ctrl_ex       <= ZERO4;
            end
            else begin
                // Normal capture from ID stage
                pc_ex             <= pc_id;

                opcode_ex         <= opcode;
                func3_ex          <= func3;
                func7_ex          <= func7;
                rd_ex             <= rd;
                rs1_ex            <= rs1;
                rs2_ex            <= rs2;
                imm_ex            <= imm_out;

                rs1_data_ex       <= rs1_data;
                rs2_data_ex       <= rs2_data;

                ex_alu_src_ex     <= ex_alu_src;
                mem_write_ex      <= mem_write;
                mem_read_ex       <= mem_read;
                mem_load_type_ex  <= mem_load_type;
                mem_store_type_ex <= mem_store_type;
                wb_reg_file_ex    <= wb_reg_file;
                memtoreg_ex       <= memtoreg;
                branch_ex         <= branch;
                jal_ex            <= jal;
                jalr_ex           <= jalr;
                auipc_ex          <= auipc;
                lui_ex            <= lui;
                alu_ctrl_ex       <= alu_ctrl;
            end
        end
    end

endmodule

