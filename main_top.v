// cpu_top.v

module main_top (
    input  wire clk,
    input  wire rst,
    input  wire id_flush,

    // instruction input to decode (for testbench)
    input  wire [31:0] instruction_in,

    // writeback control (to write register file inside top_decode)
    input  wire        wb_wr_en,
    input  wire [4:0]  wb_wr_addr,
    input  wire [31:0] wb_wr_data,

    // outputs from execute stage
    output wire [31:0] alu_result,
    output wire        zero_flag,
    output wire        negative_flag,
    output wire        carry_flag,
    output wire        overflow_flag
);

    // ------------------ signals between decode and id_ex ------------------
    wire [6:0]  opcode;
    wire [2:0]  func3;
    wire [6:0]  func7;
    wire [4:0]  rd;
    wire [4:0]  rs1;
    wire [4:0]  rs2;
    wire [31:0] imm_out;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

    // control signals from decode controller
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

    // Instantiate your existing top_decode
    top_decode u_top_decode (
        .clk(clk),
        .rst(rst),

        .instruction_in(instruction_in),
        .id_flush(id_flush),

        .wb_wr_en(wb_wr_en),
        .wb_wr_addr(wb_wr_addr),
        .wb_wr_data(wb_wr_data),

        .opcode(opcode),
        .func3(func3),
        .func7(func7),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .imm_out(imm_out),

        .rs1_data(rs1_data),
        .rs2_data(rs2_data),

        .ex_alu_src(ex_alu_src),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .mem_load_type(mem_load_type),
        .mem_store_type(mem_store_type),
        .wb_reg_file(wb_reg_file),
        .memtoreg(memtoreg),
        .branch(branch),
        .jal(jal),
        .jalr(jalr),
        .auipc(auipc),
        .lui(lui),
        .alu_ctrl(alu_ctrl)
    );

    // ------------------ ID/EX pipeline wires ------------------
    wire [31:0] idex_rs1, idex_rs2, idex_imm;
    wire [4:0]  idex_rd;
    wire [6:0]  idex_opcode, idex_func7;
    wire [2:0]  idex_func3;

    wire        idex_ex_alu_src;
    wire [3:0]  idex_alu_ctrl;
    // (other control wires can be added if needed)

    // Instantiate pipeline register
    id_ex_pipeline u_id_ex (
        .clk(clk),
        .rst(rst),
        .id_ex_flush(id_flush),

        .rs1_in(rs1_data),
        .rs2_in(rs2_data),
        .imm_in(imm_out),
        .rd_in(rd),
        .opcode_in(opcode),
        .func3_in(func3),
        .func7_in(func7),

        .ex_alu_src_in(ex_alu_src),
        .mem_write_in(mem_write),
        .mem_read_in(mem_read),
        .mem_load_type_in(mem_load_type),
        .mem_store_type_in(mem_store_type),
        .wb_reg_file_in(wb_reg_file),
        .memtoreg_in(memtoreg),
        .branch_in(branch),
        .jal_in(jal),
        .jalr_in(jalr),
        .auipc_in(auipc),
        .lui_in(lui),
        .alu_ctrl_in(alu_ctrl),

        .rs1_out(idex_rs1),
        .rs2_out(idex_rs2),
        .imm_out(idex_imm),
        .rd_out(idex_rd),
        .opcode_out(idex_opcode),
        .func3_out(idex_func3),
        .func7_out(idex_func7),

        .ex_alu_src_out(idex_ex_alu_src),
        .mem_write_out(), // not used in this small test
        .mem_read_out(),
        .mem_load_type_out(),
        .mem_store_type_out(),
        .wb_reg_file_out(),
        .memtoreg_out(),
        .branch_out(),
        .jal_out(),
        .jalr_out(),
        .auipc_out(),
        .lui_out(),
        .alu_ctrl_out(idex_alu_ctrl)
    );

    // Instantiate execute_top (wire idex outputs to exec inputs)
    execute_top u_execute (
        .rs1    (idex_rs1),
        .rs2    (idex_rs2),
        .imm_out(idex_imm),
        .ex_alu_src(idex_ex_alu_src),
        .opcode (idex_opcode),
        .funct3 (idex_func3),
        .funct7 (idex_func7),

        .alu_result(alu_result),
        .zero_flag(zero_flag),
        .negative_flag(negative_flag),
        .carry_flag(carry_flag),
        .overflow_flag(overflow_flag)
    );

endmodule


// tb_cpu_top_all.v

module tb_cpu_top_all;

    // Clock/reset
    reg clk = 0;
    reg rst = 1;
    reg id_flush = 0;

    // CPU inputs
    reg  [31:0] instruction_in = 32'b0;
    reg         wb_wr_en = 0;
    reg  [4:0]  wb_wr_addr = 5'b0;
    reg  [31:0] wb_wr_data = 32'b0;

    // Outputs from CPU
    wire [31:0] alu_result;
    wire        zero_flag, negative_flag, carry_flag, overflow_flag;

    // Instantiate DUT (cpu_top)
    main_top UCPU (
        .clk(clk),
        .rst(rst),
        .id_flush(id_flush),
        .instruction_in(instruction_in),
        .wb_wr_en(wb_wr_en),
        .wb_wr_addr(wb_wr_addr),
        .wb_wr_data(wb_wr_data),
        .alu_result(alu_result),
        .zero_flag(zero_flag),
        .negative_flag(negative_flag),
        .carry_flag(carry_flag),
        .overflow_flag(overflow_flag)
    );

    // clock generation
    parameter CLK_PERIOD = 10;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Pipeline latency: how many clock cycles to wait after feeding an instruction
    // Increase if your pipeline is deeper. Default set conservatively to 6 cycles.
    parameter integer PIPELINE_WAIT_CYCLES = 6;

    // ----------------------------
    // Test vector structure (arrays)
    // ----------------------------
    // For plain Verilog we use parallel arrays
    parameter integer NUM_TESTS = 15;

    reg [31:0] tv_instruction [0:NUM_TESTS-1];
    reg [31:0] tv_rs1_value   [0:NUM_TESTS-1];
    reg [31:0] tv_rs2_value   [0:NUM_TESTS-1];
    reg [31:0] tv_expected_result [0:NUM_TESTS-1];
    reg [3:0]  tv_expected_flags  [0:NUM_TESTS-1]; // {z,n,c,v}
    reg [255:0] tv_desc        [0:NUM_TESTS-1];

    integer i;
    integer pass_count = 0;
    integer fail_count = 0;

    // Helper: pack flags for printing
    function [3:0] pack_flags;
        input z,n,c,v;
        begin
            pack_flags = {z,n,c,v};
        end
    endfunction

    // Initialize test vectors
    initial begin
        // Default: zero everything
        for (i = 0; i < NUM_TESTS; i = i + 1) begin
            tv_instruction[i] = 32'b0;
            tv_rs1_value[i] = 32'b0;
            tv_rs2_value[i] = 32'b0;
            tv_expected_result[i] = 32'b0;
            tv_expected_flags[i] = 4'b0;
            tv_desc[i] = "unused";
        end

        // ---------------------------
        // Test 0: ADD x4, x1, x2  (R-type)
        // enc: funct7=0 rs2=2 rs1=1 func3=0 rd=4 opcode=0110011
        // ---------------------------
        tv_instruction[0] = 32'h00208233; // ADD x4,x1,x2
        tv_rs1_value[0]   = 32'd10;
        tv_rs2_value[0]   = 32'd22;
        tv_expected_result[0] = 32'd32;
        tv_expected_flags[0]  = pack_flags(0,0,0,0);
        tv_desc[0] = "ADD x4,x1,x2";

        // ---------------------------
        // Test 1: SUB x5, x3, x4  (R-type)
        // SUB has funct7=0100000, funct3=000
        // encode: funct7=0x20 rs2=4 rs1=3 func3=0 rd=5 opcode=0110011
        // ---------------------------
        tv_instruction[1] = {7'b0100000,5'd4,5'd3,3'b000,5'd5,7'b0110011}; // SUB x5,x3,x4
        tv_rs1_value[1] = 32'd50;
        tv_rs2_value[1] = 32'd20;
        tv_expected_result[1] = 32'd30; // 50 - 20
        tv_expected_flags[1]  = pack_flags(0,0,0,0);
        tv_desc[1] = "SUB x5,x3,x4";

        // ---------------------------
        // Test 2: AND x6,x1,x2
        // ---------------------------
        tv_instruction[2] = {7'b0000000,5'd2,5'd1,3'b111,5'd6,7'b0110011}; // AND
        tv_rs1_value[2] = 32'hF0F0;
        tv_rs2_value[2] = 32'h0FF0;
        tv_expected_result[2] = 32'h00F0;
        tv_expected_flags[2]  = pack_flags(0,0,0,0);
        tv_desc[2] = "AND x6,x1,x2";

        // ---------------------------
        // Test 3: OR x7,x1,x2
        // ---------------------------
        tv_instruction[3] = {7'b0000000,5'd2,5'd1,3'b110,5'd7,7'b0110011}; // OR
        tv_rs1_value[3] = 32'h0A0A;
        tv_rs2_value[3] = 32'h0505;
        tv_expected_result[3] = 32'h0F0F;
        tv_expected_flags[3]  = pack_flags(0,0,0,0);
        tv_desc[3] = "OR x7,x1,x2";

        // ---------------------------
        // Test 4: SLL x8,x1,x2 (shift left logical)
        // funct3=001
        // ---------------------------
        tv_instruction[4] = {7'b0000000,5'd2,5'd1,3'b001,5'd8,7'b0110011}; // SLL
        tv_rs1_value[4] = 32'd1;
        tv_rs2_value[4] = 32'd3; // shift by 3 -> result 8
        tv_expected_result[4] = 32'd8;
        tv_expected_flags[4]  = pack_flags(0,0,0,0);
        tv_desc[4] = "SLL x8,x1,x2";

        // ---------------------------
        // Test 5: SRL x9,x1,x2 (logical right)
        // funct3=101, funct7=0
        // ---------------------------
        tv_instruction[5] = {7'b0000000,5'd2,5'd1,3'b101,5'd9,7'b0110011}; // SRL
        tv_rs1_value[5] = 32'h80000000;
        tv_rs2_value[5] = 32'd1; // logical -> 0x40000000
        tv_expected_result[5] = 32'h40000000;
        tv_expected_flags[5]  = pack_flags(0,1,0,0); // negative_flag may reflect sign of result; adjust if your ALU sets negative based on MSB
        tv_desc[5] = "SRL x9,x1,x2";

        // ---------------------------
        // Test 6: SRA x10,x1,x2 (arithmetic right)
        // funct3=101, funct7=0100000 for SRA? Actually SRA uses funct7=0100000? RISC-V SRA uses funct7=0100000.
        // We'll encode with funct7=0100000 to get SRA
        // ---------------------------
        tv_instruction[6] = {7'b0100000,5'd2,5'd1,3'b101,5'd10,7'b0110011}; // SRA
        tv_rs1_value[6] = 32'h80000000;
        tv_rs2_value[6] = 32'd1; // arithmetic -> 0xC0000000
        tv_expected_result[6] = 32'hC0000000;
        tv_expected_flags[6]  = pack_flags(0,1,0,0);
        tv_desc[6] = "SRA x10,x1,x2";

        // ---------------------------
        // Test 7: SLT x11,x1,x2 (signed less than)
        // funct3=010
        // ---------------------------
        tv_instruction[7] = {7'b0000000,5'd2,5'd1,3'b010,5'd11,7'b0110011}; // SLT
        tv_rs1_value[7] = -32'd5; // 0xFFFFFFFB
        tv_rs2_value[7] = 32'd3;
        tv_expected_result[7] = 32'd1; // true
        tv_expected_flags[7]  = pack_flags(0,1,0,0);
        tv_desc[7] = "SLT x11,x1,x2";

        // ---------------------------
        // Test 8: SLTU x12,x1,x2 (unsigned less than)
        // funct3=011
        // ---------------------------
        tv_instruction[8] = {7'b0000000,5'd2,5'd1,3'b011,5'd12,7'b0110011}; // SLTU
        tv_rs1_value[8] = 32'hFFFF_FFFF; // large unsigned
        tv_rs2_value[8] = 32'd1;
        tv_expected_result[8] = 32'd0; // not less unsigned
        tv_expected_flags[8]  = pack_flags(0,1,0,0);
        tv_desc[8] = "SLTU x12,x1,x2";

        // ---------------------------
        // Test 9: ADDI x13, x1, imm (I-type)
        // opcode=0010011, funct3=000
        // imm = 5
        // encoding: imm[11:0] rs1 funct3 rd opcode
        // ---------------------------
        tv_instruction[9] = {12'd5,5'd1,3'b000,5'd13,7'b0010011}; // ADDI x13,x1,5
        tv_rs1_value[9] = 32'd7;
        tv_rs2_value[9] = 32'd0; // unused
        tv_expected_result[9] = 32'd12;
        tv_expected_flags[9]  = pack_flags(0,0,0,0);
        tv_desc[9] = "ADDI x13,x1,5";

        // ---------------------------
        // Test 10: XOR x14,x1,x2
        // ---------------------------
        tv_instruction[10] = {7'b0000000,5'd2,5'd1,3'b100,5'd14,7'b0110011}; // XOR
        tv_rs1_value[10] = 32'hF0F0F0F0;
        tv_rs2_value[10] = 32'h0F0F0F0F;
        tv_expected_result[10] = 32'hFFFFFFFF;
        tv_expected_flags[10] = pack_flags(0,1,0,0);
        tv_desc[10] = "XOR x14,x1,x2";

        // ---------------------------
        // Test 11: ORI x15, x1, imm (I-type OR immediate)
        // opcode=0010011? Actually ORI uses opcode 0010011 with funct3=110
        // imm = 8
        // ---------------------------
        tv_instruction[11] = {12'd8,5'd1,3'b110,5'd15,7'b0010011}; // ORI x15,x1,8
        tv_rs1_value[11] = 32'h10;
        tv_expected_result[11] = 32'h18;
        tv_expected_flags[11] = pack_flags(0,0,0,0);
        tv_desc[11] = "ORI x15,x1,8";

        // ---------------------------
        // Test 12: ANDI x16, x1, imm
        // ---------------------------
        tv_instruction[12] = {12'hFFF,5'd1,3'b111,5'd16,7'b0010011}; // ANDI x16,x1,-1
        tv_rs1_value[12] = 32'h12345678;
        tv_expected_result[12] = 32'h12345678;
        tv_expected_flags[12] = pack_flags(0,1,0,0);
        tv_desc[12] = "ANDI x16,x1,-1";

        // ---------------------------
        // Test 13: SLLI x17, x1, shamt (I-type shift immediate)
        // encoding: funct7=0000000, shamt in rs2 field for immediate forms
        // For SLLI: opcode=0010011 funct3=001 imm[11:0]=shamt
        // ---------------------------
        tv_instruction[13] = {7'b0000000,5'd1,5'd1,3'b001,5'd17,7'b0010011}; // This is not exact I-type encoding in simple form, but we'll keep for variety
        tv_rs1_value[13] = 32'd3;
        tv_rs2_value[13] = 32'd0;
        tv_expected_result[13] = 32'd3; // may not shift due to encoding - this acts as sanity
        tv_expected_flags[13] = pack_flags(0,0,0,0);
        tv_desc[13] = "SLLI x17,x1,shamt (sanity)";

        // ---------------------------
        // Test 14: Edge case: ADD with overflow (signed)
        // ADD x18,x1,x2 where values cause signed overflow
        // ---------------------------
        tv_instruction[14] = 32'h00209233; // ADD x4,x1,x2 (reuse format) - description will explain values
        tv_rs1_value[14] = 32'h7FFFFFFF; // max positive
        tv_rs2_value[14] = 32'd1;
        tv_expected_result[14] = 32'h80000000; // wrap
        tv_expected_flags[14] = pack_flags(0,1,0,1); // expect negative and overflow; adjust if your ALU flags differ
        tv_desc[14] = "ADD overflow case";

    end // initial tv init

    // Simple helper task: write register via wb interface (pulse write)
    task write_reg;
        input [4:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            wb_wr_addr = addr;
            wb_wr_data = data;
            wb_wr_en = 1'b1;
            @(negedge clk);
            wb_wr_en = 1'b0;
            wb_wr_addr = 5'b0;
            wb_wr_data = 32'b0;
        end
    endtask

    // Task to present and evaluate a single test vector
    task run_test;
        input integer idx;
        reg [31:0] actual_result;
        reg [3:0] actual_flags;
        integer cycle_wait;
        begin
            $display("\n---- TEST %0d : %s ----", idx, tv_desc[idx]);

            // Preload registers
            if (tv_rs1_value[idx] !== 32'bx) begin
                write_reg(5'd1, tv_rs1_value[idx]); // NOTE: here we always write to x1 (rs1 field used in vectors)
            end

            // If rs2 value is non-zero, write to x2
            if (tv_rs2_value[idx] !== 32'b0) begin
                write_reg(5'd2, tv_rs2_value[idx]);
            end

            // Apply instruction
            @(negedge clk);
            instruction_in = tv_instruction[idx];
            @(negedge clk);
            instruction_in = 32'b0; // clear after one cycle (optional)

            // Wait pipeline to settle
            for (cycle_wait = 0; cycle_wait < PIPELINE_WAIT_CYCLES; cycle_wait = cycle_wait + 1) begin
                @(negedge clk);
            end

            // Capture actual results
            actual_result = alu_result;
            actual_flags = {zero_flag, negative_flag, carry_flag, overflow_flag};

            // Compare with expected
            if (actual_result === tv_expected_result[idx] && actual_flags === tv_expected_flags[idx]) begin
                $display("PASS: %s", tv_desc[idx]);
                $display("  Expected result=0x%08h (%0d), actual=0x%08h (%0d)", tv_expected_result[idx], tv_expected_result[idx], actual_result, actual_result);
                $display("  Expected flags ZN C V = %b, actual = %b", tv_expected_flags[idx], actual_flags);
                pass_count = pass_count + 1;
            end
            else begin
                $display("FAIL: %s", tv_desc[idx]);
                $display("  Expected result=0x%08h (%0d), actual=0x%08h (%0d)", tv_expected_result[idx], tv_expected_result[idx], actual_result, actual_result);
                $display("  Expected flags ZN C V = %b, actual = %b", tv_expected_flags[idx], actual_flags);
                fail_count = fail_count + 1;
            end

            // small gap
            repeat (2) @(negedge clk);
        end
    endtask

    // Test sequence
    initial begin
        $dumpfile("tb_cpu_top_all.vcd");
        $dumpvars(0, tb_cpu_top_all);

        // reset sequence
        rst = 1; # (CLK_PERIOD * 4);
        rst = 0;
        @(negedge clk);

        // Run through all tests
        for (i = 0; i < NUM_TESTS; i = i + 1) begin
            run_test(i);
        end

        // Summary
        $display("\n================ TEST SUMMARY ================");
        $display(" Total tests : %0d", NUM_TESTS);
        $display(" Passed      : %0d", pass_count);
        $display(" Failed      : %0d", fail_count);
        $display("=============================================\n");

        #100;
        $finish;
    end

endmodule




