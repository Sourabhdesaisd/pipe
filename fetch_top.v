module if_stage_top(
    input  wire        clk,
    input  wire        rst,
    input  wire        pc_en,

    // BTB / predictor input signals
    input  wire [31:0] btb_target_pc,
    input  wire        btb_pc_valid,
    input  wire        btb_pc_predictTaken,

    // Signals from execute stage
    input  wire [31:0] pc_jump_addr,
    input  wire        jump_en,

    // Outputs to ID stage
    output wire [31:0] pc_out,
    output wire [31:0] instruction_out
);

    wire [31:0] pc_current;
    wire [31:0] pc_next;
    wire [31:0] instruction;

    // Program Counter
    pc pc_inst (
        .clk     (clk),
        .rst     (rst),
        .next_pc (pc_next),
        .pc_en   (pc_en),
        .pc      (pc_current)
    );

    // Next PC selection logic
    pc_update pc_update_inst (
        .pc                 (pc_current),
        .pc_jump_addr       (pc_jump_addr),
        .btb_target_pc      (btb_target_pc),
        .btb_pc_valid       (btb_pc_valid),
        .btb_pc_predictTaken(btb_pc_predictTaken),
        .jump_en            (jump_en),
        .next_pc            (pc_next)
    );

    // Instruction Memory
    inst_mem inst_mem_inst (
        .pc         (pc_current),
        .read_en    (1'b1),         // always fetch
        .instruction(instruction)
    );

    assign pc_out = pc_current;
    assign instruction_out = instruction;

endmodule

