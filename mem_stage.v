// mem_stage.v
module mem_stage (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,       // pipeline enable (stall control)
    input  wire        flush,    // pipeline flush for bubbles

    // Inputs from EX/MEM register (connect these from ex_mem_reg outputs)
    input  wire [31:0] alu_result_mem,
    input  wire        zero_flag_mem,
    input  wire        negative_flag_mem,
    input  wire        carry_flag_mem,
    input  wire        overflow_flag_mem,

    input  wire [31:0] rs2_data_mem,
    input  wire [4:0]  rd_mem,

    input  wire        mem_write_mem,
    input  wire        mem_read_mem,
    input  wire [2:0]  mem_load_type_mem,
    input  wire [1:0]  mem_store_type_mem,
    input  wire        wb_reg_file_mem,
    input  wire        memtoreg_mem,

    input  wire        branch_mem,
    input  wire        jal_mem,
    input  wire        jalr_mem,
    input  wire        modify_pc_mem,
    input  wire [31:0] update_pc_mem,
    input  wire [31:0] jump_addr_mem,
    input  wire        update_btb_mem,

    // Outputs to MEM/WB register (exposed as wires here; they will be captured in mem_wb_reg)
    output wire [31:0] alu_result_for_wb,
    output wire [31:0] load_wb_data,     // data coming from memory (if memtoreg)
    output wire [4:0]  rd_for_wb,
    output wire        wb_reg_file_out,
    output wire        memtoreg_out,

    // Branch outputs forwarded to top (so IF can react)
    output wire        modify_pc_out,
    output wire [31:0] update_pc_out,
    output wire [31:0] jump_addr_out,
    output wire        update_btb_out
);

    // Connect mem inputs to your data_memory_unit
    wire [31:0] wb_data_from_mem; // wb_data output from data_memory_unit

    data_memory_unit u_data_mem (
        .clk        (clk),
        .mem_read   (mem_read_mem),
        .mem_write  (mem_write_mem),
        .store_type (mem_store_type_mem),
        .load_type  (mem_load_type_mem),
        .alu_result (alu_result_mem),
        .rs2        (rs2_data_mem),
        .memtoreg   (memtoreg_mem),
        .wb_data    (wb_data_from_mem)
    );

    // Pass through to MEM/WB register (we'll instantiate mem_wb_reg below)
    // For now wire outputs (mem_wb_reg will capture them)
    assign alu_result_for_wb = alu_result_mem;
    assign load_wb_data      = wb_data_from_mem;
    assign rd_for_wb         = rd_mem;
    assign wb_reg_file_out   = wb_reg_file_mem;
    assign memtoreg_out      = memtoreg_mem;

    // Branch outputs
    assign modify_pc_out = modify_pc_mem;
    assign update_pc_out = update_pc_mem;
    assign jump_addr_out = jump_addr_mem;
    assign update_btb_out= update_btb_mem;

endmodule

