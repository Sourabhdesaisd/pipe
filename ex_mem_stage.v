module ex_mem_reg (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,       // enable capture; tie 1 if no stall logic
    input  wire        flush,    // when asserted, insert bubble (clear control signals)

    // From EX stage (inputs)
    input  wire [31:0] alu_result_ex,
    input  wire        zero_flag_ex,
    input  wire        negative_flag_ex,
    input  wire        carry_flag_ex,
    input  wire        overflow_flag_ex,

    input  wire [31:0] rs2_data_ex,        // store data (from ID/EX)
    input  wire [4:0]  rd_ex,              // destination reg index (from ID/EX)

    // control signals from EX (from id_ex_reg)
    input  wire        mem_write_ex,
    input  wire        mem_read_ex,
    input  wire [2:0]  mem_load_type_ex,
    input  wire [1:0]  mem_store_type_ex,
    input  wire        wb_reg_file_ex,
    input  wire        memtoreg_ex,

    // Branch / jump / BTB info from EX
    input  wire        branch_ex,
    input  wire        jal_ex,
    input  wire        jalr_ex,
    input  wire        modify_pc_ex,      // mispredict detected in EX
    input  wire [31:0] update_pc_ex,      // next PC if branch/jump or PC+4
    input  wire [31:0] jump_addr_ex,      // jump target calculated in EX
    input  wire        update_btb_ex,     // whether to update BTB

    // Outputs to MEM stage (suffixed _mem)
    output reg  [31:0] alu_result_mem,
    output reg         zero_flag_mem,
    output reg         negative_flag_mem,
    output reg         carry_flag_mem,
    output reg         overflow_flag_mem,

    output reg  [31:0] rs2_data_mem,
    output reg  [4:0]  rd_mem,

    output reg         mem_write_mem,
    output reg         mem_read_mem,
    output reg  [2:0]  mem_load_type_mem,
    output reg  [1:0]  mem_store_type_mem,
    output reg         wb_reg_file_mem,
    output reg         memtoreg_mem,

    output reg         branch_mem,
    output reg         jal_mem,
    output reg         jalr_mem,
    output reg         modify_pc_mem,
    output reg  [31:0] update_pc_mem,
    output reg  [31:0] jump_addr_mem,
    output reg         update_btb_mem
);

    // Safe default constants
    localparam [31:0] ZERO32 = 32'h00000000;
    localparam [6:0]  ZERO7  = 7'h00;
    localparam [4:0]  ZERO5  = 5'h00;
    localparam [3:0]  ZERO4  = 4'h0;
    localparam [2:0]  ZERO3  = 3'h0;
    localparam [1:0]  ZERO2  = 2'h0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // clear all pipeline outputs on reset
            alu_result_mem     <= ZERO32;
            zero_flag_mem      <= 1'b0;
            negative_flag_mem  <= 1'b0;
            carry_flag_mem     <= 1'b0;
            overflow_flag_mem  <= 1'b0;

            rs2_data_mem       <= ZERO32;
            rd_mem             <= ZERO5;

            mem_write_mem      <= 1'b0;
            mem_read_mem       <= 1'b0;
            mem_load_type_mem  <= ZERO3;
            mem_store_type_mem <= ZERO2;
            wb_reg_file_mem    <= 1'b0;
            memtoreg_mem       <= 1'b0;

            branch_mem         <= 1'b0;
            jal_mem            <= 1'b0;
            jalr_mem           <= 1'b0;
            modify_pc_mem      <= 1'b0;
            update_pc_mem      <= ZERO32;
            jump_addr_mem      <= ZERO32;
            update_btb_mem     <= 1'b0;
        end
        else if (!en) begin
            // hold (stall) - do nothing, keep prior values
        end
        else begin
            if (flush) begin
                // Insert bubble: clear control signals & data
                alu_result_mem     <= ZERO32;
                zero_flag_mem      <= 1'b0;
                negative_flag_mem  <= 1'b0;
                carry_flag_mem     <= 1'b0;
                overflow_flag_mem  <= 1'b0;

                rs2_data_mem       <= ZERO32;
                rd_mem             <= ZERO5;

                mem_write_mem      <= 1'b0;
                mem_read_mem       <= 1'b0;
                mem_load_type_mem  <= ZERO3;
                mem_store_type_mem <= ZERO2;
                wb_reg_file_mem    <= 1'b0;
                memtoreg_mem       <= 1'b0;

                branch_mem         <= 1'b0;
                jal_mem            <= 1'b0;
                jalr_mem           <= 1'b0;
                modify_pc_mem      <= 1'b0;
                update_pc_mem      <= ZERO32;
                jump_addr_mem      <= ZERO32;
                update_btb_mem     <= 1'b0;
            end
            else begin
                // Normal capture of EX outputs
                alu_result_mem     <= alu_result_ex;
                zero_flag_mem      <= zero_flag_ex;
                negative_flag_mem  <= negative_flag_ex;
                carry_flag_mem     <= carry_flag_ex;
                overflow_flag_mem  <= overflow_flag_ex;

                rs2_data_mem       <= rs2_data_ex;
                rd_mem             <= rd_ex;

                mem_write_mem      <= mem_write_ex;
                mem_read_mem       <= mem_read_ex;
                mem_load_type_mem  <= mem_load_type_ex;
                mem_store_type_mem <= mem_store_type_ex;
                wb_reg_file_mem    <= wb_reg_file_ex;
                memtoreg_mem       <= memtoreg_ex;

                branch_mem         <= branch_ex;
                jal_mem            <= jal_ex;
                jalr_mem           <= jalr_ex;
                modify_pc_mem      <= modify_pc_ex;
                update_pc_mem      <= update_pc_ex;
                jump_addr_mem      <= jump_addr_ex;
                update_btb_mem     <= update_btb_ex;
            end
        end
    end

endmodule

