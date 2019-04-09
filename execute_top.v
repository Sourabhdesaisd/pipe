/*module execute_top (
    input  [31:0] rs1,     // Read data 1 from register file
    input  [31:0] rs2,     // Read data 2 from register file
    input  [31:0] imm_out,          // immediate from decoder
    input         ex_alu_src,       // comes from main control unit
    input  [6:0]  opcode,
    input  [2:0]  funct3,
    input  [6:0]  funct7,

    output [31:0] alu_result,
    output        zero_flag,
    output        negative_flag,
    output        carry_flag,
    output        overflow_flag
);

    // ------------- ex_alu_src mux -------------
    wire [31:0] alu_src2;

    mux_alu_src u_mux (
        .rs2   (rs2),
        .imm_out   (imm_out),
        .ex_alu_src     (ex_alu_src),
        .alu_imm_rs2 (alu_src2)
    );

    // ------------- ALU control -------------
    wire [3:0] alu_ctrl;

    alu_control u_alu_ctrl (
        .opcode (opcode),
        .funct3 (funct3),
        .funct7 (funct7),
        .alu_ctrl (alu_ctrl)
    );

    // ------------- ALU core -------------
    alu_top32 u_alu (
        .rs1          (rs1),
        .rs2          (alu_src2),     // <-- input selected by mux
        .alu_ctrl     (alu_ctrl),
        .alu_result   (alu_result),
        .zero_flag    (zero_flag),
        .negative_flag(negative_flag),
        .carry_flag   (carry_flag),
        .overflow_flag(overflow_flag)
    );

endmodule*/

/*
module execute_stage (
    // From ID/ID->EX pipeline (connected to id_ex_reg outputs)
    input  wire [31:0] pc_ex,             // pc_ex from id_ex_reg (optional, used for AUIPC/JAL)
    input  wire [31:0] rs1_data_ex,       // register read data (rs1)
    input  wire [31:0] rs2_data_ex,       // register read data (rs2)
    input  wire [31:0] imm_ex,            // immediate from decoder
    input  wire        ex_alu_src_ex,     // select imm vs rs2
    input  wire [6:0]  opcode_ex,
    input  wire [2:0]  func3_ex,
    input  wire [6:0]  func7_ex,

    // Predictor input (if available). Tie 0 if none.
    input  wire        predictedTaken_ex, // prediction from BTB/predictor for this PC (optional)

    // Outputs to subsequent stages / IF (EX/MEM stage will capture these)
    output wire [31:0] alu_result_ex,     // ALU result
    output wire        zero_flag_ex,
    output wire        negative_flag_ex,
    output wire        carry_flag_ex,
    output wire        overflow_flag_ex,

    // Branch/Jump outputs
    output wire [31:0] jump_addr_ex,      // calculated target
    output wire [31:0] update_pc_ex,      // next PC in case of mispredict (or PC+4)
    output wire        modify_pc_ex,      // mispredict detected (1 -> real outcome != prediction)
    output wire        update_btb_ex      // whether BTB should be updated (branch/jump)
);

    // Internal wires
    wire [31:0] alu_src2_ex;
    wire [3:0]  alu_ctrl_ex;
    wire [31:0] alu_result_int;
    wire        zf, nf, cf, of;

    // ------------- ALU source selection -------------
    // Uses your mux_alu_src module (select imm or rs2)
    mux_alu_src u_mux_alu_src (
        .rs2      (rs2_data_ex),
        .imm_out  (imm_ex),
        .ex_alu_src (ex_alu_src_ex),
        .alu_imm_rs2 (alu_src2_ex)
    );

    // ------------- ALU control -------------
    // Uses your alu_control module to map opcode/func3/func7 -> alu_ctrl
    alu_control u_alu_ctrl (
        .opcode  (opcode_ex),
        .funct3  (func3_ex),
        .funct7  (func7_ex),
        .alu_ctrl(alu_ctrl_ex)
    );

    // ------------- ALU core -------------
    // Uses your alu_top32 (which internally instantiates arithmetic/logical/shift/compare)
    alu_top32 u_alu_top32 (
        .rs1           (rs1_data_ex),
        .rs2           (alu_src2_ex),
        .alu_ctrl      (alu_ctrl_ex),
        .alu_result    (alu_result_int),
        .zero_flag     (zf),
        .negative_flag (nf),
        .carry_flag    (cf),
        .overflow_flag (of)
    );

    // Connect ALU outputs to EX outputs
    assign alu_result_ex    = alu_result_int;
    assign zero_flag_ex     = zf;
    assign negative_flag_ex = nf;
    assign carry_flag_ex    = cf;
    assign overflow_flag_ex = of;

    // ------------- PC / Jump logic -------------
    // Use your pc_jump module to compute jump target, mispredict and BTB update
    pc_jump u_pc_jump (
        .pc            (pc_ex),
        .imm           (imm_ex),
        .rs1           (rs1_data_ex),
        .opcode        (opcode_ex),
        .func3         (func3_ex),

        .carry_flag    (cf),
        .zero_flag     (zf),
        .negative_flag (nf),
        .overflow_flag (of),

        .predictedTaken(predictedTaken_ex),

        .update_pc     (update_pc_ex),
        .jump_addr     (jump_addr_ex),
        .modify_pc     (modify_pc_ex),
        .update_btb    (update_btb_ex)
    );

endmodule */

// execute_stage.v
// Execute stage with forwarding muxes, ALU control, ALU core and PC/jump calculation.
// Ports follow the names used earlier in the top-level integration.

module execute_stage (
    // basic pipeline inputs
    input  wire [31:0] pc,               // PC forwarded from ID/EX
    input  wire [31:0] op1,              // ex_op1 (rs1 value forwarded from ID/EX)
    input  wire [31:0] op2,              // ex_op2 (rs2 value forwarded from ID/EX)
    input  wire        pipeline_flush,   // flush (bubble) control for EX stage

    // instruction fields / control
    input  wire [31:0] immediate,        // imm_ex
    input  wire [6:0]  func7,
    input  wire [2:0]  func3,
    input  wire [6:0]  opcode,
    input  wire        ex_alu_src,       // use immediate as operand2 when 1
    input  wire        predictedTaken,   // branch predictor hint (not used here, but kept)
    input  wire        invalid_inst,     // if set, treat as NOP
    input  wire        ex_wb_reg_file,   // will write to regfile after EX
    input  wire [4:0]  alu_rd_in,        // destination rd coming into EX (ID/EX->EX)

    // forwarding controls and sources
    input  wire [1:0]  operand_a_forward_cntl, // 00 original, 01 from MEM, 10 from WB
    input  wire [1:0]  operand_b_forward_cntl,
    input  wire [31:0] data_forward_mem,  // from EX/MEM (ALU result)
    input  wire [31:0] data_forward_wb,   // from MEM/WB (final wb value)

    // outputs
    output reg  [31:0] result_alu,        // ALU result (to EX/MEM)
    output reg         zero_flag,
    output reg         negative_flag,
    output reg         carry_flag,
    output reg         overflow_flag,

    // forwarded debug / pipeline outputs (selected operands)
    output reg  [31:0] op1_selected,      // after forwarding selection
    output reg  [31:0] op2_selected,      // after forwarding selection (before imm mux)
    output reg  [31:0] op2_after_alu_src, // final operand2 fed to ALU (after imm mux)

    // jump/branch outputs
    output wire [31:0] pc_jump_addr,      // jump target computed
    output wire        jump_en,           // true if branch/jump taken
    output wire        update_btb,        // whether BTB should be updated
    output wire [31:0] calc_jump_addr,    // branch target (same as pc_jump_addr)
    // Writeback outputs
    output wire [4:0]  wb_rd,             // destination index to write in EX/MEM
    output wire        wb_reg_file        // will write to regfile (EX/MEM)
);

    // internal wires
    wire [31:0] a_src;   // final ALU operand A
    wire [31:0] b_src;   // final ALU operand B (after imm selection)
    wire [3:0]  alu_ctrl;

    // ALU flag wires (from arithmetic unit)
    wire zf, nf, cf, of;

    // ------------------------------------------------------------
    // Forwarding selection (choose op1/op2 from EX/MEM or MEM/WB or original)
    // ------------------------------------------------------------
    always @(*) begin
        // op1 selection priority: MEM -> WB -> original
        case (operand_a_forward_cntl)
            2'b01: op1_selected = data_forward_mem;
            2'b10: op1_selected = data_forward_wb;
            default: op1_selected = op1;
        endcase

        // op2 selection PRIOR to ALU-SRC mux
        case (operand_b_forward_cntl)
            2'b01: op2_selected = data_forward_mem;
            2'b10: op2_selected = data_forward_wb;
            default: op2_selected = op2;
        endcase
    end

    // ------------------------------------------------------------
    // ALU-src mux (choose immediate or forwarded rs2)
    // ------------------------------------------------------------
    always @(*) begin
        if (ex_alu_src)
            op2_after_alu_src = immediate;
        else
            op2_after_alu_src = op2_selected;
    end

    // ------------------------------------------------------------
    // ALU control generation
    // Use alu_control module (same as earlier)
    // ------------------------------------------------------------
    // instantiate alu_control (combinational)
    wire [3:0] alu_ctrl_w;
    alu_control u_alu_control (
        .opcode  (opcode),
        .funct3  (func3),
        .funct7  (func7),
        .alu_ctrl(alu_ctrl_w)
    );

    // ------------------------------------------------------------
    // ALU core instantiation
    // Use alu_top32 (which internally uses arithmetic, logic, shift, compare units)
    // ------------------------------------------------------------
    wire [31:0] alu_result_w;
    wire        zf_w, nf_w, cf_w, of_w;

    alu_top32 u_alu_top (
        .rs1           (op1_selected),
        .rs2           (op2_after_alu_src),
        .alu_ctrl      (alu_ctrl_w),
        .alu_result    (alu_result_w),
        .zero_flag     (zf_w),
        .negative_flag (nf_w),
        .carry_flag    (cf_w),
        .overflow_flag (of_w)
    );

    // ------------------------------------------------------------
    // Handle invalid instruction or pipeline flush: treat as NOP
    // If pipeline_flush or invalid_inst set, produce zeroed outputs and no writeback.
    // ------------------------------------------------------------
    always @(*) begin
        if (pipeline_flush || invalid_inst) begin
            result_alu      = 32'h00000000;
            zero_flag       = 1'b0;
            negative_flag   = 1'b0;
            carry_flag      = 1'b0;
            overflow_flag   = 1'b0;
            // still drive operand selections for debug visibility
            // keep op1_selected/op2_selected as chosen above
            // final op2 after alu_src is already set above
        end
        else begin
            result_alu      = alu_result_w;
            zero_flag       = zf_w;
            negative_flag   = nf_w;
            carry_flag      = cf_w;
            overflow_flag   = of_w;
        end
    end

    // ------------------------------------------------------------
    // PC / Jump calculation
    // Use pc_jump module (combinational) that decides branch/jump taken and provides update PC
    // pc_jump expects flags and opcode/func3 and returns update_pc/jump_addr/modify_pc/update_btb
    // ------------------------------------------------------------
    wire [31:0] update_pc_w;
    wire [31:0] jump_addr_w;
    wire        modify_pc_w;
    wire        update_btb_w;

    pc_jump u_pc_jump (
        .pc             (pc),
        .imm            (immediate),
        .rs1            (op1_selected),
        .opcode         (opcode),
        .func3          (func3),
        .carry_flag     (carry_flag),
        .zero_flag      (zero_flag),
        .negative_flag  (negative_flag),
        .overflow_flag  (overflow_flag),
        .predictedTaken (predictedTaken),

        .update_pc      (update_pc_w),
        .jump_addr      (jump_addr_w),
        .modify_pc      (modify_pc_w),
        .update_btb     (update_btb_w)
    );

    // expose jump outputs
    assign pc_jump_addr = jump_addr_w;
    assign jump_en      = (modify_pc_w) ? 1'b1 : 1'b0; // if modify_pc indicates a real change, indicate jump_en
    assign update_btb   = update_btb_w;
    assign calc_jump_addr = jump_addr_w;

    // ------------------------------------------------------------
    // Final writeback signals
    // ------------------------------------------------------------
    // Pass through destination rd and write-enable from ID/EX stage (these were inputs)
    assign wb_rd       = alu_rd_in;
    assign wb_reg_file = ex_wb_reg_file;

endmodule



/*

module tb_execute_top;

    // Inputs to execute_top
    reg  [31:0] rs1;
    reg  [31:0] rs2;
    reg  [31:0] imm_out;
    reg         ex_alu_src;   // 0 -> rs2, 1 -> imm_out
    reg  [6:0]  opcode;
    reg  [2:0]  funct3;
    reg  [6:0]  funct7;

    // Outputs from execute_top
    wire [31:0] alu_result;
    wire        zero_flag;
    wire        negative_flag;
    wire        carry_flag;
    wire        overflow_flag;

    // DUT
    execute_top dut (
        .rs1           (rs1),
        .rs2           (rs2),
        .imm_out       (imm_out),
        .ex_alu_src    (ex_alu_src),
        .opcode        (opcode),
        .funct3        (funct3),
        .funct7        (funct7),
        .alu_result    (alu_result),
        .zero_flag     (zero_flag),
        .negative_flag (negative_flag),
        .carry_flag    (carry_flag),
        .overflow_flag (overflow_flag)
    );

    // Simple self-check task (checks result, prints flags)
    task CHECK;
        input [31:0]   exp;
        input [127:0]  name;    // label for the case
        begin
            #1;  // wait for combinational logic
            if (alu_result === exp) begin
                $display("PASS : %-12s | result=%0d (0x%08h)  Z=%b N=%b C=%b O=%b",
                         name, alu_result, alu_result,
                         zero_flag, negative_flag, carry_flag, overflow_flag);
            end else begin
                $display("FAIL : %-12s | result=%0d (0x%08h) exp=%0d (0x%08h)  Z=%b N=%b C=%b O=%b",
                         name, alu_result, alu_result, exp, exp,
                         zero_flag, negative_flag, carry_flag, overflow_flag);
            end
        end
    endtask

    initial begin
        $display("=========== EXECUTE_TOP (ALU + MUX + ALU_CTRL) TEST BEGIN ==========");

        //------------------------------------------------------------------
        // R-type: ADD  rd = rs1 + rs2   (no carry / overflow expected)
        //------------------------------------------------------------------
        rs1        = 32'd10;
        rs2        = 32'd20;
        imm_out    = 32'd0;
        ex_alu_src = 1'b0;         // use rs2
        opcode     = 7'b0110011;   // R-type
        funct3     = 3'b000;       // ADD/SUB
        funct7     = 7'b0000000;   // ADD
        CHECK(32'd30, "ADD");

        //------------------------------------------------------------------
        // R-type: SUB  rd = rs1 - rs2
        //------------------------------------------------------------------
        rs1        = 32'd40;
        rs2        = 32'd15;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB
        CHECK(32'd25, "SUB");

        //------------------------------------------------------------------
        // R-type: AND
        //------------------------------------------------------------------
        rs1        = 32'hF0F0_00FF;
        rs2        = 32'h0FF0_F0F0;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b111;       // AND
        funct7     = 7'b0000000;
        CHECK(32'h00F0_00F0, "AND");

        //------------------------------------------------------------------
        // R-type: OR
        //------------------------------------------------------------------
        rs1        = 32'hA0A0_0000;
        rs2        = 32'h00A0_0A0A;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b110;       // OR
        funct7     = 7'b0000000;
        CHECK(32'hA0A0_0A0A, "OR");

        //------------------------------------------------------------------
        // R-type: SLT (signed)
        //------------------------------------------------------------------
        rs1        = -5;           // 0xFFFF_FFFB
        rs2        =  8;           // 0x0000_0008
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b010;       // SLT
        funct7     = 7'b0000000;
        CHECK(32'd1, "SLT");

        //------------------------------------------------------------------
        // I-type: ADDI  (mux selects imm_out)
        //------------------------------------------------------------------
        rs1        = 32'd10;
        rs2        = 32'd123;      // ignored
        imm_out    = 32'd4;
        ex_alu_src = 1'b1;         // use immediate
        opcode     = 7'b0010011;   // I-type
        funct3     = 3'b000;       // ADDI
        funct7     = 7'b0000000;
        CHECK(32'd14, "ADDI");

        //------------------------------------------------------------------
        // I-type: ORI
        //------------------------------------------------------------------
        rs1        = 32'h0000_FF00;
        imm_out    = 32'h0000_00F0;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b110;       // ORI
        funct7     = 7'b0000000;
        CHECK(32'h0000_FFF0, "ORI");

        //------------------------------------------------------------------
        // I-type: SLLI  (shift left logical immediate)
        //------------------------------------------------------------------
        rs1        = 32'h0000_0003;
        imm_out    = 32'd4;        // shamt in [4:0]
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b001;       // SLLI
        funct7     = 7'b0000000;
        CHECK(32'h0000_0030, "SLLI");

        //------------------------------------------------------------------
        // I-type: SRLI  (logical right shift imm)
        //------------------------------------------------------------------
        rs1        = 32'h0000_0080;
        imm_out    = 32'd3;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b101;       // SRLI/SRAI
        funct7     = 7'b0000000;   // SRLI
        CHECK(32'h0000_0010, "SRLI");

        //------------------------------------------------------------------
        // I-type: SRAI  (arithmetic right shift imm)
        //------------------------------------------------------------------
        rs1        = 32'hFFFF_FF80;  // -128
        imm_out    = 32'd3;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b101;         // SRLI/SRAI
        funct7     = 7'b0100000;     // SRAI
        CHECK(32'hFFFF_FFF0, "SRAI");

        //------------------------------------------------------------------
        // EXTRA: check ZERO flag explicitly (result = 0)
        //------------------------------------------------------------------
        rs1        = 32'd25;
        rs2        = 32'd25;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB
        #1;
        $display("CHECK ZERO  | result=%0d Z=%b N=%b C=%b O=%b",
                 alu_result, zero_flag, negative_flag, carry_flag, overflow_flag);

        //------------------------------------------------------------------
        // EXTRA: generate a negative result to see N flag
        //------------------------------------------------------------------
        rs1        = 32'd5;
        rs2        = 32'd20;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB -> 5-20 = -15
        #1;
        $display("CHECK NEG   | result=%0d (0x%08h) Z=%b N=%b C=%b O=%b",
                 alu_result, alu_result, zero_flag, negative_flag,
                 carry_flag, overflow_flag);

        //------------------------------------------------------------------
        // EXTRA: big signed ADD to observe overflow_flag (depends on your logic)
        //------------------------------------------------------------------
        rs1        = 32'h7FFF_FFFF;   // largest +ve
        rs2        = 32'd1;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0000000;      // ADD
        #1;
        $display("CHECK OVF   | result=%0d (0x%08h) Z=%b N=%b C=%b O=%b",
                 alu_result, alu_result, zero_flag, negative_flag,
                 carry_flag, overflow_flag);

        $display("=========== EXECUTE_TOP TEST END ===========");
        $finish;
    end

endmodule
*/

