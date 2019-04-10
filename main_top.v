// rv32i_core.v

module rv32i_core(
    input  wire        clk,
    input  wire        rst,

    // Observability
    output wire [31:0] wb_result_out,
    output wire [31:0] ex_result_out,
    output wire        pc_en_out
);

    // ------------------------------------------------------------------
    // IF stage wires
    // ------------------------------------------------------------------
    wire [31:0] pc_if;
    wire [31:0] inst_if;

    // BTB ? IF
    wire [31:0] btb_target_pc;
    wire        btb_valid;
    wire        btb_predictTaken;

    // EX -> IF jump signals (from EX/MEM later)
    wire [31:0] ex_pc_jump_addr;
    wire        ex_jump_en;

    // PC control driven by hazard unit
    wire        pc_en;
    assign pc_en_out = pc_en;

    // IF/ID pipeline control from hazard unit
    wire if_id_pipeline_en;
    wire if_id_pipeline_flush;

    // IF/ID wires
    wire [31:0] pc_id;
    wire [31:0] inst_id;
    wire        id_flush; // forwarded from hazard logic / control

    // ------------------------------------------------------------------
    // ID stage / top_decode
    // (top_decode includes its own register_file and expects writeback ports)
    // ------------------------------------------------------------------
    wire        wb_wr_en;
    wire [4:0]  wb_wr_addr;
    wire [31:0] wb_wr_data;

    // decode outputs
    wire [6:0]  opcode_id;
    wire [2:0]  func3_id;
    wire [6:0]  func7_id;
    wire [4:0]  rd_id;
    wire [4:0]  rs1_id;
    wire [4:0]  rs2_id;
    wire [31:0] imm_id;
    wire [31:0] rs1_data_id;
    wire [31:0] rs2_data_id;

    // control signals
    wire        ex_alu_src_id;
    wire        mem_write_id;
    wire        mem_read_id;
    wire [2:0]  mem_load_type_id;
    wire [1:0]  mem_store_type_id;
    wire        wb_reg_file_id;
    wire        memtoreg_id;
    wire        branch_id;
    wire        jal_id;
    wire        jalr_id;
    wire        auipc_id;
    wire        lui_id;
    wire [3:0]  alu_ctrl_id;

    // ------------------------------------------------------------------
    // ID/EX pipeline stage wires
    // ------------------------------------------------------------------
    wire [31:0] pc_ex;
    wire [6:0]  opcode_ex;
    wire [2:0]  func3_ex;
    wire [6:0]  func7_ex;
    wire [4:0]  rd_ex;
    wire [4:0]  rs1_ex;
    wire [4:0]  rs2_ex;
    wire [31:0] imm_ex;
    wire [31:0] rs1_data_ex;
    wire [31:0] rs2_data_ex;

    wire        ex_alu_src_ex;
    wire        mem_write_ex;
    wire        mem_read_ex;
    wire [2:0]  mem_load_type_ex;
    wire [1:0]  mem_store_type_ex;
    wire        wb_reg_file_ex;
    wire        memtoreg_ex;
    wire        branch_ex;
    wire        jal_ex;
    wire        jalr_ex;
    wire        auipc_ex;
    wire        lui_ex;
    wire [3:0]  alu_ctrl_ex;

    // ------------------------------------------------------------------
    // Forwarding / hazard wires
    // ------------------------------------------------------------------
    wire [1:0] operand_a_cntl;
    wire [1:0] operand_b_cntl;
    wire        load_stall;
    wire        id_ex_pipeline_en;
    wire        id_ex_pipeline_flush;

    // compute final WB value (for forwarding)
    wire [31:0] alu_result_wb;
    wire [31:0] mem_load_data_wb;
    wire [31:0] wb_final_value;
    wire        memtoreg_wb;
    wire [4:0]  rd_wb;
    wire        wb_reg_file_wb;

    assign wb_final_value = memtoreg_wb ? mem_load_data_wb : alu_result_wb;

    // ------------------------------------------------------------------
    // EX stage wires
    // ------------------------------------------------------------------
    wire [31:0] alu_result_ex;
    wire        zero_flag_ex;
    wire        negative_flag_ex;
    wire        carry_flag_ex;
    wire        overflow_flag_ex;

    // forwarding sources (connected later)
    wire [31:0] data_forward_mem;
    wire [31:0] data_forward_wb;

    // EX outputs used by EX/MEM
    wire [31:0] pc_jump_addr_ex; // internal EX-calculated jump target
    wire        update_btb_ex;

    // ------------------------------------------------------------------
    // EX/MEM pipeline outputs (to MEM stage)
    // ------------------------------------------------------------------
    wire [31:0] alu_result_mem;
    wire [31:0] rs2_data_mem;
    wire [4:0]  rd_mem;
    wire        mem_write_mem;
    wire        mem_read_mem;
    wire [2:0]  mem_load_type_mem;
    wire [1:0]  mem_store_type_mem;
    wire        wb_reg_file_mem;
    wire        memtoreg_mem;
    wire        branch_mem;
    wire        jal_mem;
    wire        jalr_mem;
    wire        modify_pc_mem;
    wire [31:0] update_pc_mem;
    wire [31:0] jump_addr_mem;
    wire        update_btb_mem;
    wire [31:0] pc_mem; // forwarded PC from EX stage

    // ------------------------------------------------------------------
    // MEM/WB outputs (already declared above for WB mux)
    // ------------------------------------------------------------------
    // alu_result_wb, mem_load_data_wb, rd_wb, wb_reg_file_wb, memtoreg_wb already declared

    // ------------------------------------------------------------------
    // Instantiate IF stage (pc, inst_mem, pc_update or if_stage_top)
    // ------------------------------------------------------------------
    // Using your if_stage_top (which contains pc, pc_update and inst_mem)
    if_stage_top if_stage_inst (
        .clk(clk),
        .rst(rst),
        .pc_en(pc_en),

        .btb_target_pc(btb_target_pc),
        .btb_pc_valid(btb_valid),
        .btb_pc_predictTaken(btb_predictTaken),

        .pc_jump_addr(ex_pc_jump_addr),
        .jump_en(ex_jump_en),

        .pc_out(pc_if),
        .instruction_out(inst_if)
    );

    // IF/ID pipeline register
    if_id_reg if_id_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(if_id_pipeline_en),

        .pc(pc_if),
        .instruction(inst_if),

        .pc_id(pc_id),
        .instruction_id(inst_id)
    );

    // ------------------------------------------------------------------
    // Decode (top_decode contains register file). Provide it WB ports.
    // ------------------------------------------------------------------
    top_decode top_decode_inst (
        .clk(clk),
        .rst(rst),
        .instruction_in(inst_id),
        .id_flush(if_id_pipeline_flush),   // flush used as id_flush here

        .wb_wr_en(wb_reg_file_wb),
        .wb_wr_addr(rd_wb),
        .wb_wr_data(wb_final_value),

        .opcode(opcode_id),
        .func3(func3_id),
        .func7(func7_id),
        .rd(rd_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .imm_out(imm_id),

        .rs1_data(rs1_data_id),
        .rs2_data(rs2_data_id),

        .ex_alu_src(ex_alu_src_id),
        .mem_write(mem_write_id),
        .mem_read(mem_read_id),
        .mem_load_type(mem_load_type_id),
        .mem_store_type(mem_store_type_id),
        .wb_reg_file(wb_reg_file_id),
        .memtoreg(memtoreg_id),
        .branch(branch_id),
        .jal(jal_id),
        .jalr(jalr_id),
        .auipc(auipc_id),
        .lui(lui_id),
        .alu_ctrl(alu_ctrl_id)
    );

    // ------------------------------------------------------------------
    // Hazard unit - connects decode signals and EX destination/loads
    // ------------------------------------------------------------------
    hazard_unit hazard_unit_inst (
        .id_rs1(rs1_id),
        .id_rs2(rs2_id),
        .opcode(opcode_id),
        .ex_rd(rd_ex),
        .ex_load_inst(mem_read_ex),      // is EX stage currently a load?
        .jump_branch_taken(branch_ex & zero_flag_ex), // approximate: branch_ex & zero_flag_ex ; you may adapt
        .invalid_inst(1'b0),
        .modify_pc(modify_pc_mem),

        .if_id_pipeline_flush(if_id_pipeline_flush),
        .if_id_pipeline_en(if_id_pipeline_en),
        .id_ex_pipeline_flush(id_ex_pipeline_flush),
        .id_ex_pipeline_en(id_ex_pipeline_en),
        .pc_en(pc_en),
        .load_stall(load_stall)
    );

    // ------------------------------------------------------------------
    // ID/EX pipeline register
    // ------------------------------------------------------------------
    id_ex_reg id_ex_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(id_ex_pipeline_en),
        .flush(id_ex_pipeline_flush),

        .pc_id(pc_id),
        .pc_ex(pc_ex),

        .opcode(opcode_id),
        .func3(func3_id),
        .func7(func7_id),
        .rd(rd_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .imm_out(imm_id),

        .rs1_data(rs1_data_id),
        .rs2_data(rs2_data_id),

        .ex_alu_src(ex_alu_src_id),
        .mem_write(mem_write_id),
        .mem_read(mem_read_id),
        .mem_load_type(mem_load_type_id),
        .mem_store_type(mem_store_type_id),
        .wb_reg_file(wb_reg_file_id),
        .memtoreg(memtoreg_id),
        .branch(branch_id),
        .jal(jal_id),
        .jalr(jalr_id),
        .auipc(auipc_id),
        .lui(lui_id),
        .alu_ctrl(alu_ctrl_id) //,

       /* // outputs to EX
        .opcode_ex(opcode_ex),
        .func3_ex(func3_ex),
        .func7_ex(func7_ex),
        .rd_ex(rd_ex),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .imm_ex(imm_ex),

        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex),

        .ex_alu_src_ex(ex_alu_src_ex),
        .mem_write_ex(mem_write_ex),
        .mem_read_ex(mem_read_ex),
        .mem_load_type_ex(mem_load_type_ex),
        .mem_store_type_ex(mem_store_type_ex),
        .wb_reg_file_ex(wb_reg_file_ex),
        .memtoreg_ex(memtoreg_ex),
        .branch_ex(branch_ex),
        .jal_ex(jal_ex),
        .jalr_ex(jalr_ex),
        .auipc_ex(auipc_ex),
        .lui_ex(lui_ex),
        .alu_ctrl_ex(alu_ctrl_ex)*/
    );

    // ------------------------------------------------------------------
    // Forwarding unit (decides from EX/MEM or MEM/WB)
    // ------------------------------------------------------------------
    forwarding_unit forwarding_unit_inst (
        .rs1(rs1_ex),
        .rs2(rs2_ex),
        .rd_mem(rd_mem),
        .rd_wb(rd_wb),
        .reg_file_wr_mem(wb_reg_file_mem),
        .reg_file_wr_wb(wb_reg_file_wb),
        .operand_a_cntl(operand_a_cntl),
        .operand_b_cntl(operand_b_cntl)
    );

    // data_forward_mem/from EX/MEM is alu_result_mem (below) and data_forward_wb is wb_final_value
    assign data_forward_mem = alu_result_mem;
    assign data_forward_wb  = wb_final_value;

    // ------------------------------------------------------------------
    // Execute stage: connect forwarding/control inputs and outputs
    // ------------------------------------------------------------------
    execute_stage execute_stage_inst (
        .pc(pc_ex),
        .op1(rs1_data_ex),
        .op2(rs2_data_ex),
        .pipeline_flush(ex_forward_pipeline_flush), // you can tie 0 or derive from control
        .immediate(imm_ex),
        .func7(func7_ex),
        .func3(func3_ex),
        .opcode(opcode_ex),
        .ex_alu_src(ex_alu_src_ex),
        .predictedTaken(1'b0),
        .invalid_inst(1'b0),
        .ex_wb_reg_file(wb_reg_file_ex),
        .alu_rd_in(rd_ex),

        .operand_a_forward_cntl(operand_a_cntl),
        .operand_b_forward_cntl(operand_b_cntl),
        .data_forward_mem(data_forward_mem),
        .data_forward_wb(data_forward_wb),

        .result_alu(alu_result_ex),
        .zero_flag(zero_flag_ex),
        .negative_flag(negative_flag_ex),
        .carry_flag(carry_flag_ex),
        .overflow_flag(overflow_flag_ex),

        .op1_selected(),        // debug outputs left unconnected here
        .op2_selected(),
        .op2_after_alu_src(),

        // jump/branch outputs
        .pc_jump_addr(pc_jump_addr_ex),
        .jump_en(ex_jump_en),
        .update_btb(update_btb_ex),
        .calc_jump_addr(),      // optional
        .wb_rd(),               // optional
        .wb_reg_file()
    );

    // Connect EX -> IF jump outputs (EX stage may assert jump and target)
    assign ex_pc_jump_addr = pc_jump_addr_ex;
    assign ex_jump_en      = ex_jump_en;

    // ------------------------------------------------------------------
    // EX/MEM pipeline register: capture EX outputs for MEM stage
    // ------------------------------------------------------------------
    ex_mem_reg ex_mem_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(1'b1),
        .flush(1'b0),

        .alu_result_ex(alu_result_ex),
        .zero_flag_ex(zero_flag_ex),
        .negative_flag_ex(negative_flag_ex),
        .carry_flag_ex(carry_flag_ex),
        .overflow_flag_ex(overflow_flag_ex),

        .rs2_data_ex(rs2_data_ex),
        .rd_ex(rd_ex),

        .mem_write_ex(mem_write_ex),
        .mem_read_ex(mem_read_ex),
        .mem_load_type_ex(mem_load_type_ex),
        .mem_store_type_ex(mem_store_type_ex),
        .wb_reg_file_ex(wb_reg_file_ex),
        .memtoreg_ex(memtoreg_ex),

        .branch_ex(branch_ex),
        .jal_ex(jal_ex),
        .jalr_ex(jalr_ex),

        .modify_pc_ex(update_btb_ex),    // use update_btb_ex as modify flag path (ok to wire like this)
        .update_pc_ex(imm_ex),           // NOTE: imm_ex used as placeholder; real update_pc should come from EX calculation
        .jump_addr_ex(pc_jump_addr_ex),
        .update_btb_ex(update_btb_ex),

        .pc_ex(pc_ex),

        // outputs to MEM stage
        .alu_result_mem(alu_result_mem),
        .zero_flag_mem(), .negative_flag_mem(), .carry_flag_mem(), .overflow_flag_mem(),

        .rs2_data_mem(rs2_data_mem),
        .rd_mem(rd_mem),

        .mem_write_mem(mem_write_mem),
        .mem_read_mem(mem_read_mem),
        .mem_load_type_mem(mem_load_type_mem),
        .mem_store_type_mem(mem_store_type_mem),
        .wb_reg_file_mem(wb_reg_file_mem),
        .memtoreg_mem(memtoreg_mem),

        .branch_mem(branch_mem),
        .jal_mem(jal_mem),
        .jalr_mem(jalr_mem),

        .modify_pc_mem(modify_pc_mem),
        .update_pc_mem(update_pc_mem),
        .jump_addr_mem(jump_addr_mem),
        .update_btb_mem(update_btb_mem),

        .pc_mem(pc_mem)
    );

    // ------------------------------------------------------------------
    // MEM stage: top-level data memory subsystem (data_mem_top)
    // We use mem_stage wrapper you provided that instantiates data_mem_top
    // ------------------------------------------------------------------
    mem_stage mem_stage_inst (
        .clk(clk),
        .rst(rst),
        .en(1'b1),
        .flush(1'b0),

        .alu_result_mem(alu_result_mem),
        .zero_flag_mem(1'b0),
        .negative_flag_mem(1'b0),
        .carry_flag_mem(1'b0),
        .overflow_flag_mem(1'b0),

        .rs2_data_mem(rs2_data_mem),
        .rd_mem(rd_mem),

        .mem_write_mem(mem_write_mem),
        .mem_read_mem(mem_read_mem),
        .mem_load_type_mem(mem_load_type_mem),
        .mem_store_type_mem(mem_store_type_mem),
        .wb_reg_file_mem(wb_reg_file_mem),
        .memtoreg_mem(memtoreg_mem),

        .branch_mem(branch_mem),
        .jal_mem(jal_mem),
        .jalr_mem(jalr_mem),
        .modify_pc_mem(modify_pc_mem),
        .update_pc_mem(update_pc_mem),
        .jump_addr_mem(jump_addr_mem),
        .update_btb_mem(update_btb_mem),

        .alu_result_for_wb(alu_result_wb),
        .load_wb_data(mem_load_data_wb),
        .rd_for_wb(rd_wb),
        .wb_reg_file_out(wb_reg_file_wb),
        .memtoreg_out(memtoreg_wb),

        .modify_pc_out(),   // expose if needed
        .update_pc_out(),
        .jump_addr_out(),
        .update_btb_out()
    );

    // ------------------------------------------------------------------
    // MEM/WB pipeline register
    // ------------------------------------------------------------------
    mem_wb_reg mem_wb_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(1'b1),
        .flush(1'b0),

        .alu_result_for_wb(alu_result_wb),
        .load_wb_data(mem_load_data_wb),
        .rd_for_wb(rd_wb),
        .wb_reg_file_in(wb_reg_file_wb),
        .memtoreg_in(memtoreg_wb),

        .pc_plus4_in(pc_mem + 32'h4),
        .pc_to_reg_in(jal_mem || jalr_mem),

        .alu_result_wb(alu_result_wb),
        .mem_load_data_wb(mem_load_data_wb),
        .rd_wb(rd_wb),
        .wb_reg_file_wb(wb_reg_file_wb),
        .memtoreg_wb(memtoreg_wb),

        .pc_plus4_wb(),
        .pc_to_reg_wb()
    );

    // ------------------------------------------------------------------
    // Writeback mux -> drive register file write ports in top_decode
    // ------------------------------------------------------------------
    writeback_mux writeback_mux_inst (
        .alu_result_wb(alu_result_wb),
        .mem_load_data_wb(mem_load_data_wb),
        .pc_plus4_wb(32'b0),
        .memtoreg_wb(memtoreg_wb),
        .pc_to_reg_wb(1'b0),
        .wr_data(wb_wr_data)
    );

    assign wb_wr_en   = wb_reg_file_wb;
    assign wb_wr_addr = rd_wb;

    // Expose outputs for observability
    assign wb_result_out = wb_wr_data;
    assign ex_result_out = alu_result_ex;

endmodule

