// rv32i_core.v
// Full RV32I pipeline top (IF -> ID -> EX -> MEM -> WB).
// Uses the modules from your project: inst_mem, pc_reg, pc_update, if_id_reg,
// top_decode (contains its own register_file and exposes wb_wr_*), id_ex_reg,
// execute_stage, ex_mem_reg (updated), data_memory_unit, mem_wb_reg (updated), writeback_mux.
/*
module main_top(
    input  wire        clk,
    input  wire        rst,

    // Observability (optional)
    output wire [31:0] wb_result,   // writeback result visible externally
    output wire [31:0] ex_result,   // ALU result visible externally
    output wire        pc_en_out    // PC enable (for external observation)
);

    // -------------------------------------------------------
    // Simple config: no BTB/predictor and no hazard logic yet.
    // Those signals are present as wires and defaulted to safe values.
    // -------------------------------------------------------

    // IF stage wires
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] instruction;
    wire        read_en = 1'b1;

    // Predictor / BTB wires (not used yet)
    wire [31:0] btb_target_pc = 32'h0;
    wire        btb_pc_valid  = 1'b0;
    wire        btb_pc_predictTaken = 1'b0;
    wire        btb_update = 1'b0;
    wire [31:0] btb_update_target = 32'h0;

    // EX->IF jump signals (from execute stage / later from EX/MEM pipeline)
    wire [31:0] ex_pc_jump_addr;
    wire        ex_jump_en;
    assign ex_pc_jump_addr = 32'h0;
    assign ex_jump_en = 1'b0;

    // Simple pipeline control (no stalls by default)
    wire pc_en = 1'b1;
    wire if_id_en = 1'b1;
    wire id_ex_en = 1'b1;
    wire ex_mem_en = 1'b1;
    wire mem_wb_en = 1'b1;

    // expose pc_en externally
    assign pc_en_out = pc_en;

    // -------------------------------------------------------
    // IF: PC register, next-PC selection, instruction memory
    // -------------------------------------------------------
    pc u_pc (
        .clk     (clk),
        .rst     (rst),
        .next_pc (next_pc),
        .pc_en   (pc_en),
        .pc      (pc)
    );

    pc_update u_pc_update (
        .pc                 (pc),
        .pc_jump_addr       (ex_pc_jump_addr),
        .btb_target_pc      (btb_target_pc),
        .btb_pc_valid       (btb_pc_valid),
        .btb_pc_predictTaken(btb_pc_predictTaken),
        .jump_en            (ex_jump_en),
        .next_pc            (next_pc)
    );

    inst_mem u_inst_mem (
        .pc          (pc),
        .read_en     (read_en),
        .instruction (instruction)
    );

    // IF/ID pipeline register (keeps your original names)
    wire [31:0] pc_id;
    wire [31:0] instruction_id;
    if_id_reg u_if_id_reg (
        .clk         (clk),
        .rst         (rst),
        .en          (if_id_en),
        .pc          (pc),
        .instruction (instruction),
        .pc_id       (pc_id),
        .instruction_id (instruction_id)
    );

    // -------------------------------------------------------
    // ID stage (top_decode which contains register_file)
    // top_decode expects writeback ports: wb_wr_en, wb_wr_addr, wb_wr_data
    // We will drive these from the MEM/WB stage later.
    // -------------------------------------------------------

    // writeback wires (driven in WB)
    wire        wb_wr_en;
    wire [4:0]  wb_wr_addr;
    wire [31:0] wb_wr_data;

    // decode outputs
    wire [6:0]  opcode;
    wire [2:0]  func3;
    wire [6:0]  func7;
    wire [4:0]  rd_idx;
    wire [4:0]  rs1_idx;
    wire [4:0]  rs2_idx;
    wire [31:0] imm_out;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

    // control signals from decode
    wire        ex_alu_src;
    wire        mem_write;
    wire        mem_read;
    wire [2:0]  mem_load_type;
    wire [1:0]  mem_store_type;
    wire        wb_reg_file;
    wire        memtoreg;
    wire        branch;
    wire        jal;
    wire        jalr;
    wire        auipc;
    wire        lui;
    wire [3:0]  alu_ctrl;

    top_decode u_top_decode (
        .clk            (clk),
        .rst            (rst),
        .instruction_in (instruction_id),
        .id_flush       (1'b0),         // no flush by default
        .wb_wr_en       (wb_wr_en),
        .wb_wr_addr     (wb_wr_addr),
        .wb_wr_data     (wb_wr_data),

        // outputs
        .opcode         (opcode),
        .func3          (func3),
        .func7          (func7),
        .rd             (rd_idx),
        .rs1            (rs1_idx),
        .rs2            (rs2_idx),
        .imm_out        (imm_out),
        .rs1_data       (rs1_data),
        .rs2_data       (rs2_data),

        // control
        .ex_alu_src     (ex_alu_src),
        .mem_write      (mem_write),
        .mem_read       (mem_read),
        .mem_load_type  (mem_load_type),
        .mem_store_type (mem_store_type),
        .wb_reg_file    (wb_reg_file),
        .memtoreg       (memtoreg),
        .branch         (branch),
        .jal            (jal),
        .jalr           (jalr),
        .auipc          (auipc),
        .lui            (lui),
        .alu_ctrl       (alu_ctrl)
    );

    // -------------------------------------------------------
    // ID/EX pipeline register
    // -------------------------------------------------------
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

    id_ex_reg u_id_ex_reg (
        .clk        (clk),
        .rst        (rst),
        .en         (id_ex_en),
        .flush      (1'b0),

        .pc_id      (pc_id),
        .pc_ex      (pc_ex),

        .opcode     (opcode),
        .func3      (func3),
        .func7      (func7),
        .rd         (rd_idx),
        .rs1        (rs1_idx),
        .rs2        (rs2_idx),
        .imm_out    (imm_out),

        .rs1_data   (rs1_data),
        .rs2_data   (rs2_data),

        .ex_alu_src (ex_alu_src),
        .mem_write  (mem_write),
        .mem_read   (mem_read),
        .mem_load_type (mem_load_type),
        .mem_store_type(mem_store_type),
        .wb_reg_file (wb_reg_file),
        .memtoreg   (memtoreg),
        .branch     (branch),
        .jal        (jal),
        .jalr       (jalr),
        .auipc      (auipc),
        .lui        (lui),
        .alu_ctrl   (alu_ctrl),

        .opcode_ex  (opcode_ex),
        .func3_ex   (func3_ex),
        .func7_ex   (func7_ex),
        .rd_ex      (rd_ex),
        .rs1_ex     (rs1_ex),
        .rs2_ex     (rs2_ex),
        .imm_ex     (imm_ex),

        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex),

        .ex_alu_src_ex    (ex_alu_src_ex),
        .mem_write_ex     (mem_write_ex),
        .mem_read_ex      (mem_read_ex),
        .mem_load_type_ex (mem_load_type_ex),
        .mem_store_type_ex(mem_store_type_ex),
        .wb_reg_file_ex   (wb_reg_file_ex),
        .memtoreg_ex      (memtoreg_ex),
        .branch_ex        (branch_ex),
        .jal_ex           (jal_ex),
        .jalr_ex          (jalr_ex),
        .auipc_ex         (auipc_ex),
        .lui_ex           (lui_ex),
        .alu_ctrl_ex      (alu_ctrl_ex)
    );

    // -------------------------------------------------------
    // EX stage
    // -------------------------------------------------------
    wire [31:0] alu_result_ex;
    wire        zero_flag_ex;
    wire        negative_flag_ex;
    wire        carry_flag_ex;
    wire        overflow_flag_ex;

    wire [31:0] jump_addr_ex;
    wire [31:0] update_pc_ex;
    wire        modify_pc_ex;
    wire        update_btb_ex;

    // predictedTaken tie 0 for now
    wire predictedTaken_ex = 1'b0;

   execute_stage u_execute_stage (
        .pc_ex            (pc_ex),
        .rs1_data_ex      (rs1_data_ex),
        .rs2_data_ex      (rs2_data_ex),
        .imm_ex           (imm_ex),
        .ex_alu_src_ex    (ex_alu_src_ex),
        .opcode_ex        (opcode_ex),
        .func3_ex         (func3_ex),
        .func7_ex         (func7_ex),
        .predictedTaken_ex(predictedTaken_ex),

        .alu_result_ex    (alu_result_ex),
        .zero_flag_ex     (zero_flag_ex),
        .negative_flag_ex (negative_flag_ex),
        .carry_flag_ex    (carry_flag_ex),
        .overflow_flag_ex (overflow_flag_ex),

        .jump_addr_ex     (jump_addr_ex),
        .update_pc_ex     (update_pc_ex),
        .modify_pc_ex     (modify_pc_ex),
        .update_btb_ex    (update_btb_ex)
    );
execute_stage u_execute_stage (
    .pc                 (ex_pc),
    .op1                (ex_op1),
    .op2                (ex_op2),
    .pipeline_flush     (ex_forward_pipeline_flush),

    .immediate          (ex_immediate),
    .func7              (ex_func7),
    .func3              (ex_func3),
    .opcode             (ex_opcode),
    .ex_alu_src         (ex_alu_src),
    .predictedTaken     (ex_pred_taken),
    .invalid_inst       (invalid_inst),
    .ex_wb_reg_file     (ex_wb_reg_file),
    .alu_rd_in          (ex_wb_rd),

    // forwarding inputs (new)
    .operand_a_forward_cntl (operand_a_cntl),
    .operand_b_forward_cntl (operand_b_cntl),
    .data_forward_mem       (data_forward_mem),
    .data_forward_wb        (data_forward_wb),

    // outputs
    .result_alu         (ex_result),
    .zero_flag          (),    // if you want flags use wires
    .negative_flag      (),
    .carry_flag         (),
    .overflow_flag      (),

    .op1_selected       (),    // optional
    .op2_selected       (),    // optional
    .op2_after_alu_src  (),

    .pc_jump_addr       (ex_if_pc_jump_addr),
    .jump_en            (ex_jump_en),
    .update_btb         (btb_update),
    .calc_jump_addr     (btb_update_target),

    .wb_rd              (alu_rd),
    .wb_reg_file        (alu_wb)
);

forwarding_unit forwarding_unit_inst (
    .rs1(ex_rs1),               // EX stage rs1 index (from ID/EX)
    .rs2(ex_rs2),               // EX stage rs2 index
    .rd_mem(rd_mem),            // EX/MEM dest index (from ex_mem_reg)
    .rd_wb(rd_wb),              // MEM/WB dest index (from mem_wb_reg)
    .reg_file_wr_mem(wb_reg_file_mem), // EX/MEM write-enable
    .reg_file_wr_wb(wb_reg_file_wb),   // MEM/WB write-enable
    .operand_a_cntl(operand_a_cntl),
    .operand_b_cntl(operand_b_cntl)
);


    // expose EX result for external observe
    assign ex_result = alu_result_ex;

    // -------------------------------------------------------
    // EX/MEM pipeline register (capture EX outputs -> MEM stage)
    // Using updated ex_mem_reg that includes pc_ex input to forward pc to MEM
    // -------------------------------------------------------
    wire [31:0] alu_result_mem;
    wire        zero_flag_mem;
    wire        negative_flag_mem;
    wire        carry_flag_mem;
    wire        overflow_flag_mem;
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
    wire [31:0] pc_mem;

    ex_mem_reg u_ex_mem_reg (
        .clk                (clk),
        .rst                (rst),
        .en                 (ex_mem_en),
        .flush              (1'b0),

        .alu_result_ex      (alu_result_ex),
        .zero_flag_ex       (zero_flag_ex),
        .negative_flag_ex   (negative_flag_ex),
        .carry_flag_ex      (carry_flag_ex),
        .overflow_flag_ex   (overflow_flag_ex),

        .rs2_data_ex        (rs2_data_ex),
        .rd_ex              (rd_ex),

        .mem_write_ex       (mem_write_ex),
        .mem_read_ex        (mem_read_ex),
        .mem_load_type_ex   (mem_load_type_ex),
        .mem_store_type_ex  (mem_store_type_ex),
        .wb_reg_file_ex     (wb_reg_file_ex),
        .memtoreg_ex        (memtoreg_ex),

        .branch_ex          (branch_ex),
        .jal_ex             (jal_ex),
        .jalr_ex            (jalr_ex),
        .modify_pc_ex       (modify_pc_ex),
        .update_pc_ex       (update_pc_ex),
        .jump_addr_ex       (jump_addr_ex),
        .update_btb_ex      (update_btb_ex),

        .pc_ex              (pc_ex),        // forwarded pc

        // outputs
        .alu_result_mem     (alu_result_mem),
        .zero_flag_mem      (zero_flag_mem),
        .negative_flag_mem  (negative_flag_mem),
        .carry_flag_mem     (carry_flag_mem),
        .overflow_flag_mem  (overflow_flag_mem),

        .rs2_data_mem       (rs2_data_mem),
        .rd_mem             (rd_mem),

        .mem_write_mem      (mem_write_mem),
        .mem_read_mem       (mem_read_mem),
        .mem_load_type_mem  (mem_load_type_mem),
        .mem_store_type_mem (mem_store_type_mem),
        .wb_reg_file_mem    (wb_reg_file_mem),
        .memtoreg_mem       (memtoreg_mem),

        .branch_mem         (branch_mem),
        .jal_mem            (jal_mem),
        .jalr_mem           (jalr_mem),
        .modify_pc_mem      (modify_pc_mem),
        .update_pc_mem      (update_pc_mem),
        .jump_addr_mem      (jump_addr_mem),
        .update_btb_mem     (update_btb_mem),

        .pc_mem             (pc_mem)
    );

// forwarding control wires
wire [1:0] operand_a_cntl;
wire [1:0] operand_b_cntl;

// forwarded data sources
wire [31:0] data_forward_mem;   // from EX/MEM (alu_result_mem)
wire [31:0] data_forward_wb;    // from MEM/WB (final WB value)

// from EX/MEM (ensure these names come out of your ex_mem_reg or ex_mem_pipeline)
wire [31:0] alu_result_mem;     // ALU result in EX/MEM
wire [4:0]  rd_mem;             // dest reg in EX/MEM (aka alu_rd)
wire        wb_reg_file_mem;    // EX/MEM will write back?

// from MEM/WB (ensure mem_wb_reg outputs these names)
wire [31:0] alu_result_wb;      // ALU result captured in MEM/WB
wire [31:0] mem_load_data_wb;   // Load result captured in MEM/WB
wire        memtoreg_wb;        // MEM/WB memtoreg select
wire [4:0]  rd_wb;              // dest reg in MEM/WB
wire        wb_reg_file_wb;     // MEM/WB will write back?
    
  
 // -------------------------------------------------------
    // MEM stage: data memory unit
    // -------------------------------------------------------
    wire [31:0] wb_data_from_mem;
    data_memory_unit u_data_memory_unit (
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

    // -------------------------------------------------------
    // MEM/WB pipeline register (capture MEM outputs -> WB stage)
    // Updated to carry pc+4 and pc_to_reg
    // -------------------------------------------------------
    wire [31:0] alu_result_wb;
    wire [31:0] mem_load_data_wb;
    wire [4:0]  rd_wb;
    wire        wb_reg_file_wb;  
    wire        memtoreg_wb;   
    wire [31:0] pc_plus4_wb;
    wire        pc_to_reg_wb;

    // compute pc+4 in MEM stage
    wire [31:0] pc_plus4_calc = pc_mem + 32'h4;

    mem_wb_reg u_mem_wb_reg (
        .clk                (clk),
        .rst                (rst),
        .en                 (mem_wb_en),
        .flush              (1'b0),

        .alu_result_for_wb  (alu_result_mem),
        .load_wb_data       (wb_data_from_mem),
        .rd_for_wb          (rd_mem),
        .wb_reg_file_in     (wb_reg_file_mem),
        .memtoreg_in        (memtoreg_mem),

        .pc_plus4_in        (pc_plus4_calc),
        .pc_to_reg_in       (jal_mem || jalr_mem),

        .alu_result_wb      (alu_result_wb),
        .mem_load_data_wb   (mem_load_data_wb),
        .rd_wb              (rd_wb),
        .wb_reg_file_wb     (wb_reg_file_wb),
        .memtoreg_wb        (memtoreg_wb),

        .pc_plus4_wb        (pc_plus4_wb),
        .pc_to_reg_wb       (pc_to_reg_wb)
    );

    // -------------------------------------------------------
    // Writeback mux and top-level writeback connections
    // -------------------------------------------------------
    wire [31:0] wb_wr_data_w;
    writeback_mux u_writeback_mux (
        .alu_result_wb    (alu_result_wb),
        .mem_load_data_wb (mem_load_data_wb),
        .pc_plus4_wb      (pc_plus4_wb),
        .memtoreg_wb      (memtoreg_wb),
        .pc_to_reg_wb     (pc_to_reg_wb),
        .wr_data          (wb_wr_data_w)
    );

    // Drive top_decode's writeback ports (it contains the register_file)
    assign wb_wr_en   = wb_reg_file_wb;
    assign wb_wr_addr = rd_wb;
    assign wb_wr_data = wb_wr_data_w;

    // Also expose external WB result and ALU result
    assign wb_result = wb_wr_data_w;
    assign ex_result = alu_result_ex;

endmodule


*/
// -------------------------------------------------
// rv32i_core (updated top) - uses the pipeline regs above
// NOTE: This top still expects your other stage modules to exist.
// -------------------------------------------------
module rv32i_core(
    input  wire clk,
    input  wire rst,
    output wire [31:0] wb_result,
    output wire [31:0] ex_result,
    output wire pc_en_out
);
    // -- IF/ID wires (assumed by your other modules)
    wire [31:0] if_instruction, id_instruction;
    wire [31:0] if_pc, id_pc;
    wire        id_flush;
    wire        id_pred_taken;

    // -- ID/EX wires (assumed to be created by id_ex_pipeline)
    wire [31:0] ex_pc;
    wire [31:0] ex_op1;
    wire [31:0] ex_op2;
    wire [4:0]  ex_rs1;
    wire [4:0]  ex_rs2;
    wire [4:0]  ex_wb_rd;
    wire [31:0] ex_immediate;
    wire [6:0]  ex_opcode;
    wire        ex_alu_src;
    wire [6:0]  ex_func7;
    wire [2:0]  ex_func3;
    wire        ex_mem_read;
    wire        ex_wb_reg_file;
    wire        ex_pred_taken;

    // -- EX/MEM wires (produced by ex_mem_reg above)
    wire [31:0] alu_result_mem;
    wire [4:0]  rd_mem;
    wire        wb_reg_file_mem;
    wire [31:0] pc_mem;

    // -- MEM/WB wires (produced by mem_wb_reg above)
    wire [31:0] alu_result_wb;
    wire [31:0] mem_load_data_wb;
    wire [4:0]  rd_wb;
    wire        wb_reg_file_wb;
    wire        memtoreg_wb;
    wire [31:0] pc_plus4_wb;
    wire        pc_to_reg_wb;

    // forwarding control wires
    wire [1:0] operand_a_cntl;
    wire [1:0] operand_b_cntl;
    wire [31:0] data_forward_mem;
    wire [31:0] data_forward_wb;

    // hazard signals exposed
    wire if_id_pipeline_flush;
    wire if_id_pipeline_en;
    wire id_ex_pipeline_flush;
    wire id_ex_pipeline_en;
    wire invalid_inst;
    wire load_stall;
    wire ex_forward_pipeline_flush;
    wire ex_jump_en;
    wire [31:0] ex_if_pc_jump_addr;
    wire btb_update;
    wire [31:0] btb_update_target;
    wire [31:0] ex_result_wire;

    // wires for mem stage outputs (assumed by your mem_stage/mem_wb_pipeline)
    wire [31:0] mem_read_data;
    wire [31:0] mem_calculated_result;

    // ---------------------------
    // Instantiate forwarding unit (needs ex_rs1/ex_rs2, rd_mem, rd_wb)
    // ---------------------------
    forwarding_unit forwarding_unit_inst (
        .rs1(ex_rs1),
        .rs2(ex_rs2),
        .rd_mem(rd_mem),
        .rd_wb(rd_wb),
        .reg_file_wr_mem(wb_reg_file_mem),
        .reg_file_wr_wb(wb_reg_file_wb),
        .operand_a_cntl(operand_a_cntl),
        .operand_b_cntl(operand_b_cntl)
    );

    // ---------------------------
    // Instantiate hazard unit
    // (ID signals must be connected inside your top - here we assume decode_stage exports them)
    // ---------------------------
    // We will assume your decode_stage produces id_rs1, id_rs2 and id_opcode and exposes invalid_inst
    // Connect these in your actual top to the hazard_unit inputs.
    wire [4:0] id_rs1;
    wire [4:0] id_rs2;
    wire [6:0] id_opcode;
    hazard_unit hazard_unit_inst (
        .id_rs1(id_rs1),
        .id_rs2(id_rs2),
        .opcode(id_opcode),
        .ex_rd(ex_wb_rd),             // ID/EX -> EX destination
        .ex_load_inst(ex_mem_read),   // ID/EX -> ex_mem_read
        .jump_branch_taken(ex_jump_en),
        .invalid_inst(invalid_inst),
        .modify_pc(ex_jump_en),
        .if_id_pipeline_flush(if_id_pipeline_flush),
        .if_id_pipeline_en(if_id_pipeline_en),
        .id_ex_pipeline_flush(id_ex_pipeline_flush),
        .id_ex_pipeline_en(id_ex_pipeline_en),
        .pc_en(pc_en_out),
        .load_stall(load_stall)
    );

    // ---------------------------
    // Compute data_forward_wb (final WB value) - used by forwarding unit
    // ---------------------------
    assign data_forward_mem = alu_result_mem;
    assign data_forward_wb  = memtoreg_wb ? mem_load_data_wb : alu_result_wb;

    // ---------------------------
    // ****************************************************************************
    // NOTE: The following block shows where to connect ex_mem_reg and mem_wb_reg.
    // Your pipeline already has an EX stage and MEM stage. Ensure when you instantiate
    // ex_mem_reg and mem_wb_reg that you connect the exact signals used there.
    // ****************************************************************************
    // Example instantiation (if you are capturing EX stage outputs into ex_mem_reg)
    // (If you already have ex_mem_pipeline module in your project, you can replace it with this ex_mem_reg)
    // ---------------------------

    // --- EX stage should produce alu_result_ex, flags, rs2_data_ex, rd_ex, control signals, pc_ex
    wire [31:0] alu_result_ex;
    wire        zero_flag_ex;
    wire        negative_flag_ex;
    wire        carry_flag_ex;
    wire        overflow_flag_ex;
    wire [31:0] rs2_data_ex;
    wire [4:0]  rd_ex;
    wire        mem_write_ex;
    wire [2:0]  mem_load_type_ex;
    wire [1:0]  mem_store_type_ex;
    wire        wb_reg_file_ex;
    wire        memtoreg_ex;
    wire        branch_ex;
    wire        jal_ex;
    wire        jalr_ex;
    wire        modify_pc_ex;
    wire [31:0] update_pc_ex;
    wire [31:0] jump_addr_ex;
    wire        update_btb_ex;

    // instantiate ex_mem_reg to capture outputs produced by your execute stage
    ex_mem_reg ex_mem_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(id_ex_pipeline_en),
        .flush(id_ex_pipeline_flush),

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

        .modify_pc_ex(modify_pc_ex),
        .update_pc_ex(update_pc_ex),
        .jump_addr_ex(jump_addr_ex),
        .update_btb_ex(update_btb_ex),

        .pc_ex(ex_pc),

        .alu_result_mem(alu_result_mem),
        .zero_flag_mem(), .negative_flag_mem(), .carry_flag_mem(), .overflow_flag_mem(),
        .rs2_data_mem(), .rd_mem(rd_mem),
        .mem_write_mem(mem_write_mem),
        .mem_read_mem(), .mem_load_type_mem(), .mem_store_type_mem(),
        .wb_reg_file_mem(wb_reg_file_mem),
        .memtoreg_mem(memtoreg_wb), // note: here we directly map memtoreg to mem_wb_reg later

        .branch_mem(), .jal_mem(), .jalr_mem(),
        .modify_pc_mem(), .update_pc_mem(), .jump_addr_mem(), .update_btb_mem(),
        .pc_mem(pc_mem)
    );

    // ---------------------------
    // The MEM stage in your design should produce:
    //   mem_read_data, mem_calculated_result and eventually pass alu_result_for_wb and load_wb_data to mem_wb_reg
    // Example mem_wb_reg instantiation shown below:
    // ---------------------------

    // wires that mem_stage will produce for WB
    wire [31:0] alu_result_for_wb;
    wire [31:0] load_wb_data;
    wire [4:0]  rd_for_wb;
    wire        wb_reg_file_in;
    wire        memtoreg_in;
    wire [31:0] pc_plus4_in;
    wire        pc_to_reg_in;

    // instantiate mem_wb_reg
    mem_wb_reg mem_wb_reg_inst (
        .clk(clk),
        .rst(rst),
        .en(1'b1), // you should gate this with mem_wb pipeline enable if you have one
        .flush(1'b0),

        .alu_result_for_wb(alu_result_for_wb),
        .load_wb_data(load_wb_data),
        .rd_for_wb(rd_for_wb),
        .wb_reg_file_in(wb_reg_file_in),
        .memtoreg_in(memtoreg_in),

        .pc_plus4_in(pc_plus4_in),
        .pc_to_reg_in(pc_to_reg_in),

        .alu_result_wb(alu_result_wb),
        .mem_load_data_wb(mem_load_data_wb),
        .rd_wb(rd_wb),
        .wb_reg_file_wb(wb_reg_file_wb),
        .memtoreg_wb(memtoreg_wb),

        .pc_plus4_wb(pc_plus4_wb),
        .pc_to_reg_wb(pc_to_reg_wb)
    );

    // ---------------------------
    // Finally: map the top-level outputs to wires
    // ---------------------------
    assign wb_result = data_forward_wb; // final value written back (this is convenient; your real WB stage produces this)
    assign ex_result = ex_result_wire;

    // pc_en_out already driven by hazard_unit
    //
    //
    //
    //
    // -------------------------------------------------------
    // Writeback mux and top-level writeback connections
    // -------------------------------------------------------
    wire [31:0] wb_wr_data_w;
    writeback_mux u_writeback_mux (
        .alu_result_wb    (alu_result_wb),
        .mem_load_data_wb (mem_load_data_wb),
        .pc_plus4_wb      (pc_plus4_wb),
        .memtoreg_wb      (memtoreg_wb),
        .pc_to_reg_wb     (pc_to_reg_wb),
        .wr_data          (wb_wr_data_w)
    );

    // Drive top_decode's writeback ports (it contains the register_file)
    assign wb_wr_en   = wb_reg_file_wb;
    assign wb_wr_addr = rd_wb;
    assign wb_wr_data = wb_wr_data_w;


endmodule

