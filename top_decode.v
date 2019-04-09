/*module top_decode(
    input          clk,
    input          rst,

    // Instion fetch / ID inputs
    input   [31:0] instruction_in,
    input          id_flush,

    // Writk port
    input          wb_wr_en,
    input   [4:0]  wb_wr_addr,
    input   [31:0] wb_wr_data,

    // Instion fields
    output  [6:0]  opcode,
    output  [2:0]  func3,
    output  [6:0]  func7,
    output  [4:0]  rd,
    output  [4:0]  rs1_addr,
    output  [4:0]  rs2_addr,
    output  [31:0] imm_out,

    // Regi file outputs
    output  [31:0] rs1_data,
    output  [31:0] rs2_data,

    // Contsignals
    output         ex_alu_src,
    output         mem_write,
    output         mem_read,
    output  [2:0]  mem_load_type,
    output  [1:0]  mem_store_type,
    output         wb_reg_file,
    output         memtoreg,
    output         branch,
    output         jal,
    output         jalr,
    output         auipc,
    output         lui,
    output  [3:0]  alu_ctrl
);

    // Internal connections
    wire [6:0]  opcode_w;
    wire [2:0]  func3_w;
    wire [6:0]  func7_w;
    wire [4:0]  rd_w;
    wire [4:0]  rs1_w;
    wire [4:0]  rs2_w;
    wire [31:0] imm_w;

    wire        ex_alu_src_w;
    wire        mem_write_w;
    wire        mem_read_w;
    wire [2:0]  mem_load_type_w;
    wire [1:0]  mem_store_type_w;
    wire        wb_reg_file_w;
    wire        memtoreg_w;
    wire        branch_w;
    wire        jal_w;
    wire        jalr_w;
    wire        auipc_w;
    wire        lui_w;
    wire [3:0]  alu_ctrl_w;

    // ------------------------------
    // Decode unit
    // ------------------------------
    decode_unit u_decode_unit (
        .instruction_in(instruction_in),
        .id_flush(id_flush),

        .opcode(opcode_w),
        .func3(func3_w),
        .func7(func7_w),
        .rd(rd_w),
        .rs1(rs1_w),
        .rs2(rs2_w),
        .imm_out(imm_w)
    );

    // ------------------------------
    // Controller
    // ------------------------------
    decode_controller_pipelined u_decode_ctrl (
        .opcode(opcode_w),
        .func3(func3_w),
        .func7(func7_w),

        .ex_alu_src(ex_alu_src_w),
        .mem_write(mem_write_w),
        .mem_read(mem_read_w),
        .mem_load_type(mem_load_type_w),
        .mem_store_type(mem_store_type_w),
        .wb_reg_file(wb_reg_file_w),
        .memtoreg(memtoreg_w),
        .branch(branch_w),
        .jal(jal_w),
        .jalr(jalr_w),
        .auipc(auipc_w),
        .lui(lui_w),
        .alu_ctrl(alu_ctrl_w)
    );

    // ------------------------------
    // Register file
    // ------------------------------
    register_file u_regfile (
        .clk(clk),

        .wr_en(wb_wr_en),
        .wr_addr(wb_wr_addr),
        .wr_data(wb_wr_data),

        .rs1_addr(rs1_w),
        .rs2_addr(rs2_w),

        .rs1(rs1_data),
        .rs2(rs2_data)
    );

    // ------------------------------
    // Assign outputs (no _o suffix)
    // ------------------------------
    assign opcode        = opcode_w;
    assign func3         = func3_w;
    assign func7         = func7_w;
    assign rd            = rd_w;
    assign rs1_addr           = rs1_w;
    assign rs2           = rs2_w;
    assign imm_out       = imm_w;

    assign ex_alu_src    = ex_alu_src_w;
    assign mem_write     = mem_write_w;
    assign mem_read      = mem_read_w;
    assign mem_load_type = mem_load_type_w;
    assign mem_store_type= mem_store_type_w;
    assign wb_reg_file   = wb_reg_file_w;
    assign memtoreg      = memtoreg_w;

    assign branch        = branch_w;
    assign jal           = jal_w;
    assign jalr          = jalr_w;
    assign auipc         = auipc_w;
    assign lui           = lui_w;
    assign alu_ctrl      = alu_ctrl_w;

endmodule*/

module top_decode(
    input  wire        clk,
    input  wire        rst,

    // Instruction fetch / ID inputs
    input  wire [31:0] instruction_in,
    input  wire        id_flush,

    // Writeback port (connect WB stage here)
    input  wire        wb_wr_en,
    input  wire [4:0]  wb_wr_addr,
    input  wire [31:0] wb_wr_data,

    // Instruction fields (outputs)
    output wire [6:0]  opcode,
    output wire [2:0]  func3,
    output wire [6:0]  func7,
    output wire [4:0]  rd,
    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output wire [31:0] imm_out,

    // Register file outputs
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data,

    // Control signals
    output wire        ex_alu_src,
    output wire        mem_write,
    output wire        mem_read,
    output wire [2:0]  mem_load_type,
    output wire [1:0]  mem_store_type,
    output wire        wb_reg_file,
    output wire        memtoreg,
    output wire        branch,
    output wire        jal,
    output wire        jalr,
    output wire        auipc,
    output wire        lui,
    output wire [3:0]  alu_ctrl
);

    // Internal wires (clean names)
    wire [6:0]  opcode_w;
    wire [2:0]  func3_w;
    wire [6:0]  func7_w;
    wire [4:0]  rd_w;
    wire [4:0]  rs1_w;
    wire [4:0]  rs2_w;
    wire [31:0] imm_w;

    wire        ex_alu_src_w;
    wire        mem_write_w;
    wire        mem_read_w;
    wire [2:0]  mem_load_type_w;
    wire [1:0]  mem_store_type_w;
    wire        wb_reg_file_w;
    wire        memtoreg_w;
    wire        branch_w;
    wire        jal_w;
    wire        jalr_w;
    wire        auipc_w;
    wire        lui_w;
    wire [3:0]  alu_ctrl_w;

    // ------------------------------
    // Decode unit
    // ------------------------------
    decode_unit u_decode_unit (
        .instruction_in(instruction_in),
        .id_flush(id_flush),

        .opcode(opcode_w),
        .func3(func3_w),
        .func7(func7_w),
        .rd(rd_w),
        .rs1(rs1_w),
        .rs2(rs2_w),
        .imm_out(imm_w)
    );

    // ------------------------------
    // Controller
    // ------------------------------
    decode_controller_pipelined u_decode_ctrl (
        .opcode(opcode_w),
        .func3(func3_w),
        .func7(func7_w),

        .ex_alu_src(ex_alu_src_w),
        .mem_write(mem_write_w),
        .mem_read(mem_read_w),
        .mem_load_type(mem_load_type_w),
        .mem_store_type(mem_store_type_w),
        .wb_reg_file(wb_reg_file_w),
        .memtoreg(memtoreg_w),
        .branch(branch_w),
        .jal(jal_w),
        .jalr(jalr_w),
        .auipc(auipc_w),
        .lui(lui_w),
        .alu_ctrl(alu_ctrl_w)
    );

    // ------------------------------
    // Register file
    // ------------------------------
    register_file u_regfile (
        .clk(clk),

        .wr_en(wb_wr_en),
        .wr_addr(wb_wr_addr),
        .wr_data(wb_wr_data),

        .rs1_addr(rs1_w),
        .rs2_addr(rs2_w),

        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );

    // ------------------------------
    // Assign outputs (top-level)
    // ------------------------------
    assign opcode        = opcode_w;
    assign func3         = func3_w;
    assign func7         = func7_w;
    assign rd            = rd_w;
    assign rs1           = rs1_w;
    assign rs2           = rs2_w;
    assign imm_out       = imm_w;

    assign ex_alu_src    = ex_alu_src_w;
    assign mem_write     = mem_write_w;
    assign mem_read      = mem_read_w;
    assign mem_load_type = mem_load_type_w;
    assign mem_store_type= mem_store_type_w;
    assign wb_reg_file   = wb_reg_file_w;
    assign memtoreg      = memtoreg_w;

    assign branch        = branch_w;
    assign jal           = jal_w;
    assign jalr          = jalr_w;
    assign auipc         = auipc_w;
    assign lui           = lui_w;
    assign alu_ctrl      = alu_ctrl_w;

endmodule

/*
module tb_simple_passfail();

    // clock
    reg clk = 0;
    always #5 clk = ~clk;

    // DUT inputs
    reg [31:0] instruction_in;
    reg id_flush;

    // WB interface (simulate writeback)
    reg wb_wr_en;
    reg [4:0] wb_wr_addr;
    reg [31:0] wb_wr_data;

    // DUT outputs (no _o suffix)
    wire [6:0]  opcode;
    wire [2:0]  func3;
    wire [6:0]  func7;
    wire [4:0]  rd;
    wire [4:0]  rs1;
    wire [4:0]  rs2;
    wire [31:0] imm_out;

    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

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

    // instantiate DUT (assumes top_decode_if_id exists)
    top_decode dut (
        .clk(clk),
        .rst(1'b0),
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

    // Helper instruction builders (same style as before)
    function [31:0] make_rtype(input [6:0] opc, input [6:0] f7, input [2:0] f3, input [4:0] r1, input [4:0] r2, input [4:0] rd_in);
        make_rtype = {f7, r2, r1, f3, rd_in, opc};
    endfunction

    function [31:0] make_itype(input [6:0] opc, input [11:0] imm12, input [2:0] f3, input [4:0] r1, input [4:0] rd_in, input [6:0] dummy);
        make_itype = {imm12, r1, f3, rd_in, opc};
    endfunction

    function [31:0] make_stype(input [6:0] opc, input [11:0] imm12, input [2:0] f3, input [4:0] r1, input [4:0] r2);
        make_stype = {imm12[11:5], r2, r1, f3, imm12[4:0], opc};
    endfunction

    function [31:0] make_btype(input [6:0] opc, input [12:0] imm13, input [2:0] f3, input [4:0] r1, input [4:0] r2);
        make_btype = {imm13[12], imm13[10:5], r2, r1, f3, imm13[4:1], imm13[11], opc};
    endfunction

    function [31:0] make_utype(input [6:0] opc, input [19:0] imm20, input [4:0] rd_in);
        make_utype = {imm20, rd_in, opc};
    endfunction

    function [31:0] make_jtype(input [6:0] opc, input [20:0] imm21, input [4:0] rd_in);
        make_jtype = {imm21[20], imm21[10:1], imm21[11], imm21[19:12], rd_in, opc};
    endfunction

    // constants (kept in sync with your controller)
    localparam OPC_RTYPE = 7'b0110011;
    localparam OPC_ITYPE = 7'b0010011;
    localparam OPC_ILOAD = 7'b0000011;
    localparam OPC_STYPE = 7'b0100011;
    localparam OPC_BTYPE = 7'b1100011;
    localparam OPC_JAL   = 7'b1101111;
    localparam OPC_JALR  = 7'b1100111;
    localparam OPC_AUIPC = 7'b0010111;
    localparam OPC_LUI   = 7'b0110111;

    // ALU control encodings (must match decode_controller_pipelined)
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLL  = 4'b0101;
    localparam ALU_SRL  = 4'b0110;
    localparam ALU_SRA  = 4'b0111;
    localparam ALU_SLT  = 4'b1000;
    localparam ALU_SLTU = 4'b1001;

    // load/store types
    localparam LOAD_LB  = 3'b000;
    localparam LOAD_LH  = 3'b001;
    localparam LOAD_LW  = 3'b010;
    localparam LOAD_LBU = 3'b011;
    localparam LOAD_LHU = 3'b100;

    localparam STORE_SB = 2'b00;
    localparam STORE_SH = 2'b01;
    localparam STORE_SW = 2'b10;

    // small set of funct3/funct7 constants used in tests
    localparam F3_ADD_SUB = 3'b000;
    localparam F3_SLL     = 3'b001;
    localparam F3_SRL_SRA = 3'b101;

    localparam F7_ADD = 7'b0000000;
    localparam F7_SUB = 7'b0100000;

    // bookkeeping for pass/fail
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num = 0;

    // helper to check equality and print pass/fail
    task check_and_report;
        input string test_name;
        input bit condition;
        begin
            test_num = test_num + 1;
            if (condition) begin
                pass_count = pass_count + 1;
                $display("Test %0d PASS: %s", test_num, test_name);
            end else begin
                fail_count = fail_count + 1;
                $display("Test %0d FAIL: %s", test_num, test_name);
            end
        end
    endtask

    // Main test sequence (linear)
    initial begin
        // init
        instruction_in = 32'h00000013; // NOP
        id_flush = 0;
        wb_wr_en = 0;
        wb_wr_addr = 0;
        wb_wr_data = 0;
        #20;

        // Test 1: R-type ADD -> expect wb_reg_file=1, ex_alu_src=0, alu_ctrl=ALU_ADD
        instruction_in = make_rtype(OPC_RTYPE, F7_ADD, F3_ADD_SUB, 5'd1, 5'd2, 5'd3);
        #8;
        check_and_report("R-type ADD (wb_reg=1, ex_alu_src=0, alu=ADD)",
            (wb_reg_file == 1'b1) && (ex_alu_src == 1'b0) && (alu_ctrl == ALU_ADD));

        // Test 2: R-type SUB -> expect alu_ctrl = ALU_SUB, wb_reg_file=1
        instruction_in = make_rtype(OPC_RTYPE, F7_SUB, F3_ADD_SUB, 5'd5, 5'd6, 5'd4);
        #8;
        check_and_report("R-type SUB (alu=SUB, wb_reg=1)",
            (alu_ctrl == ALU_SUB) && (wb_reg_file == 1'b1));

        // Test 3: R-type SLL -> expect alu_ctrl = ALU_SLL
        instruction_in = make_rtype(OPC_RTYPE, F7_ADD, F3_SLL, 5'd8, 5'd9, 5'd7);
        #8;
        check_and_report("R-type SLL (alu=SLL)",
            (alu_ctrl == ALU_SLL));

        // Test 4: I-type ADDI -> expect ex_alu_src=1, wb_reg_file=1, alu_ctrl=ALU_ADD
        instruction_in = make_itype(OPC_ITYPE, 12'h007, F3_ADD_SUB, 5'd11, 5'd10, 7'b0);
        #8;
        check_and_report("I-type ADDI (ex_alu_src=1, wb_reg=1, alu=ADD)",
            (ex_alu_src == 1'b1) && (wb_reg_file == 1'b1) && (alu_ctrl == ALU_ADD));

        // Test 5: I-type SRLI -> expect ex_alu_src=1, alu_ctrl=ALU_SRL
        instruction_in = make_itype(OPC_ITYPE, 12'h003, F3_SRL_SRA, 5'd13, 5'd12, 7'b0);
        #8;
        check_and_report("I-type SRLI (ex_alu_src=1, alu=SRL)",
            (ex_alu_src == 1'b1) && (alu_ctrl == ALU_SRL));

        // Test 6: Load LW x14, 8(x15) -> expect mem_read=1, memtoreg=1, wb_reg_file=1, mem_load_type=LOAD_LW, ex_alu_src=1
        instruction_in = make_itype(OPC_ILOAD, 12'h008, 3'b010, 5'd15, 5'd14, 7'b0);
        #8;
        check_and_report("LW (mem_read=1, memtoreg=1, load_type=LW, ex_alu_src=1)",
            (mem_read == 1'b1) && (memtoreg == 1'b1) && (wb_reg_file == 1'b1) && (mem_load_type == LOAD_LW) && (ex_alu_src == 1'b1));

        // Test 7: Store SW x16, 12(x17) -> expect mem_write=1, mem_store_type=STORE_SW
        instruction_in = make_stype(OPC_STYPE, 12'd12, 3'b010, 5'd17, 5'd16);
        #8;
        check_and_report("SW (mem_write=1, store_type=SW)",
            (mem_write == 1'b1) && (mem_store_type == STORE_SW));

        // Test 8: Branch BEQ -> expect branch=1, alu_ctrl=ALU_SUB
        instruction_in = make_btype(OPC_BTYPE, 13'b000000001000, 3'b000, 5'd18, 5'd19);
        #8;
        check_and_report("BEQ (branch=1, alu=SUB)",
            (branch == 1'b1) && (alu_ctrl == ALU_SUB));

        // Test 9: JAL -> expect jal=1, wb_reg_file=1
        instruction_in = make_jtype(OPC_JAL, 21'd16, 5'd20);
        #8;
        check_and_report("JAL (jal=1, wb_reg=1)",
            (jal == 1'b1) && (wb_reg_file == 1'b1));

        // Test 10: JALR -> expect jalr=1, ex_alu_src=1, wb_reg_file=1
        instruction_in = make_itype(OPC_JALR, 12'h004, 3'b000, 5'd22, 5'd21, 7'b0);
        #8;
        check_and_report("JALR (jalr=1, ex_alu_src=1, wb_reg=1)",
            (jalr == 1'b1) && (ex_alu_src == 1'b1) && (wb_reg_file == 1'b1));

        // Test 11: LUI -> expect lui=1, wb_reg_file=1
        instruction_in = make_utype(OPC_LUI, 20'h12345, 5'd23);
        #8;
        check_and_report("LUI (lui=1, wb_reg=1)",
            (lui == 1'b1) && (wb_reg_file == 1'b1));

        // Test 12: AUIPC -> expect auipc=1, wb_reg_file=1
        instruction_in = make_utype(OPC_AUIPC, 20'h00010, 5'd24);
        #8;
        check_and_report("AUIPC (auipc=1, wb_reg=1)",
            (auipc == 1'b1) && (wb_reg_file == 1'b1));

        // Test 13: Forwarding: write wb_wr x5 then issue read rs1=x5, expect rs1_data == wb_wr_data
        wb_wr_en = 1;
        wb_wr_addr = 5;
        wb_wr_data = 32'hDEADBEEF;
        #10;               // allow write clock edge to occur (register_file writes on posedge)
        wb_wr_en = 0;
        // issue an instruction that reads rs1=5
        instruction_in = make_itype(OPC_ITYPE, 12'h000, F3_ADD_SUB, 5'd5, 5'd1, 7'b0);
        #8;
        check_and_report("Forwarding: read rs1==x5 after WB (rs1_data==DEADBEEF)",
            (rs1_data == 32'hDEADBEEF));

        // Summary
        #5;
        $display("\n=== TESTSUMMARY ===");
        $display("Total tests: %0d  Passed: %0d  Failed: %0d", test_num, pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED.");
        else $display("SOME TESTS FAILED.");
        #10;
        $finish;
    end

    // dump
    initial begin
        $dumpfile("tb_simple_passfail.vcd");
        $dumpvars(0, tb_simple_passfail);
    end

endmodule
*/

