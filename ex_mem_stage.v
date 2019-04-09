// ex_mem_reg.v  (updated to include pc_ex -> pc_mem forwarding)
/*module ex_mem_reg (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        flush,

    // EX inputs
    input  wire [31:0] alu_result_ex,
    input  wire        zero_flag_ex,
    input  wire        negative_flag_ex,
    input  wire        carry_flag_ex,
    input  wire        overflow_flag_ex,

    input  wire [31:0] rs2_data_ex,
    input  wire [4:0]  rd_ex,

    input  wire        mem_write_ex,
    input  wire        mem_read_ex,
    input  wire [2:0]  mem_load_type_ex,
    input  wire [1:0]  mem_store_type_ex,
    input  wire        wb_reg_file_ex,
    input  wire        memtoreg_ex,

    input  wire        branch_ex,
    input  wire        jal_ex,
    input  wire        jalr_ex,
    input  wire        modify_pc_ex,
    input  wire [31:0] update_pc_ex,
    input  wire [31:0] jump_addr_ex,
    input  wire        update_btb_ex,

    // NEW: forward PC from EX into MEM stage
    input  wire [31:0] pc_ex,

    // MEM outputs (captured)
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
    output reg         update_btb_mem,

    // NEW: pc forwarded to MEM (pc_mem)
    output reg  [31:0] pc_mem
);

    // safe default constants
    localparam [31:0] ZERO32 = 32'h00000000;
    localparam [4:0]  ZERO5  = 5'h00;
    localparam [3:0]  ZERO4  = 4'h0;
    localparam [2:0]  ZERO3  = 3'h0;
    localparam [1:0]  ZERO2  = 2'h0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
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

            pc_mem             <= ZERO32;
        end
        else if (!en) begin
            // hold values
        end
        else begin
            if (flush) begin
                // bubble
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

                pc_mem             <= ZERO32;
            end else begin
                // normal capture
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

                pc_mem             <= pc_ex;
            end
        end
    end

endmodule


*/


// -------------------------------------------------
// ex_mem_reg
// Captures EX stage outputs and forwards to MEM stage.
// Provide standard signals required by forwarding and memory.
// -------------------------------------------------
module ex_mem_reg (
    input  wire         clk,
    input  wire         rst,
    input  wire         en,
    input  wire         flush,

    // inputs from EX stage
    input  wire [31:0]  alu_result_ex,
    input  wire         zero_flag_ex,
    input  wire         negative_flag_ex,
    input  wire         carry_flag_ex,
    input  wire         overflow_flag_ex,

    input  wire [31:0]  rs2_data_ex,        // for stores
    input  wire [4:0]   rd_ex,              // destination register index

    input  wire         mem_write_ex,
    input  wire         mem_read_ex,
    input  wire [2:0]   mem_load_type_ex,
    input  wire [1:0]   mem_store_type_ex,
    input  wire         wb_reg_file_ex,
    input  wire         memtoreg_ex,

    input  wire         branch_ex,
    input  wire         jal_ex,
    input  wire         jalr_ex,

    input  wire         modify_pc_ex,
    input  wire [31:0]  update_pc_ex,
    input  wire [31:0]  jump_addr_ex,
    input  wire         update_btb_ex,

    // pc forwarded
    input  wire [31:0]  pc_ex,

    // outputs to MEM stage
    output reg  [31:0]  alu_result_mem,
    output reg          zero_flag_mem,
    output reg          negative_flag_mem,
    output reg          carry_flag_mem,
    output reg          overflow_flag_mem,

    output reg  [31:0]  rs2_data_mem,
    output reg  [4:0]   rd_mem,

    output reg          mem_write_mem,
    output reg          mem_read_mem,
    output reg  [2:0]   mem_load_type_mem,
    output reg  [1:0]   mem_store_type_mem,
    output reg          wb_reg_file_mem,
    output reg          memtoreg_mem,

    output reg          branch_mem,
    output reg          jal_mem,
    output reg          jalr_mem,

    output reg          modify_pc_mem,
    output reg  [31:0]  update_pc_mem,
    output reg  [31:0]  jump_addr_mem,
    output reg          update_btb_mem,

    output reg  [31:0]  pc_mem
);
    // synchronous capture, with enable/flush
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_mem    <= 32'b0;
            zero_flag_mem     <= 1'b0;
            negative_flag_mem <= 1'b0;
            carry_flag_mem    <= 1'b0;
            overflow_flag_mem <= 1'b0;

            rs2_data_mem      <= 32'b0;
            rd_mem            <= 5'b0;

            mem_write_mem     <= 1'b0;
            mem_read_mem      <= 1'b0;
            mem_load_type_mem <= 3'b111;
            mem_store_type_mem<= 2'b11;
            wb_reg_file_mem   <= 1'b0;
            memtoreg_mem      <= 1'b0;

            branch_mem        <= 1'b0;
            jal_mem           <= 1'b0;
            jalr_mem          <= 1'b0;

            modify_pc_mem     <= 1'b0;
            update_pc_mem     <= 32'b0;
            jump_addr_mem     <= 32'b0;
            update_btb_mem    <= 1'b0;

            pc_mem            <= 32'b0;
        end
        else if (en) begin
            if (flush) begin
                // convert into NOP values
                alu_result_mem    <= 32'b0;
                zero_flag_mem     <= 1'b0;
                negative_flag_mem <= 1'b0;
                carry_flag_mem    <= 1'b0;
                overflow_flag_mem <= 1'b0;

                rs2_data_mem      <= 32'b0;
                rd_mem            <= 5'b0;

                mem_write_mem     <= 1'b0;
                mem_read_mem      <= 1'b0;
                mem_load_type_mem <= 3'b111;
                mem_store_type_mem<= 2'b11;
                wb_reg_file_mem   <= 1'b0;
                memtoreg_mem      <= 1'b0;

                branch_mem        <= 1'b0;
                jal_mem           <= 1'b0;
                jalr_mem          <= 1'b0;

                modify_pc_mem     <= 1'b0;
                update_pc_mem     <= 32'b0;
                jump_addr_mem     <= 32'b0;
                update_btb_mem    <= 1'b0;

                pc_mem            <= 32'b0;
            end
            else begin
                alu_result_mem    <= alu_result_ex;
                zero_flag_mem     <= zero_flag_ex;
                negative_flag_mem <= negative_flag_ex;
                carry_flag_mem    <= carry_flag_ex;
                overflow_flag_mem <= overflow_flag_ex;

                rs2_data_mem      <= rs2_data_ex;
                rd_mem            <= rd_ex;

                mem_write_mem     <= mem_write_ex;
                mem_read_mem      <= mem_read_ex;
                mem_load_type_mem <= mem_load_type_ex;
                mem_store_type_mem<= mem_store_type_ex;
                wb_reg_file_mem   <= wb_reg_file_ex;
                memtoreg_mem      <= memtoreg_ex;

                branch_mem        <= branch_ex;
                jal_mem           <= jal_ex;
                jalr_mem          <= jalr_ex;

                modify_pc_mem     <= modify_pc_ex;
                update_pc_mem     <= update_pc_ex;
                jump_addr_mem     <= jump_addr_ex;
                update_btb_mem    <= update_btb_ex;

                pc_mem            <= pc_ex;
            end
        end
    end
endmodule

