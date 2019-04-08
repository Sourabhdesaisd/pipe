
// Reuse the same defines as your design (copied minimal set used by the DUT)
// OPCODES
`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_IJALR 7'b1100111
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111

// FUNC7
`define FUNC7_ADD 7'b0000000
`define FUNC7_SUB 7'b0100000

// ALU Codes
`define ALU_ADD  4'b0000
`define ALU_SUB  4'b0001
`define ALU_AND  4'b0010
`define ALU_OR   4'b0011
`define ALU_XOR  4'b0100
`define ALU_SLL  4'b0101
`define ALU_SRL  4'b0110
`define ALU_SRA  4'b0111
`define ALU_SLT  4'b1000
`define ALU_SLTU 4'b1001

// Store Types
`define STORE_SB  2'b00
`define STORE_SH  2'b01
`define STORE_SW  2'b10
`define STORE_DEF 2'b11

// Load Types
`define LOAD_LB  3'b000
`define LOAD_LH  3'b001
`define LOAD_LW  3'b010
`define LOAD_LBU 3'b011
`define LOAD_LHU 3'b100
`define LOAD_DEF 3'b111

// Instantiate the DUT (assumes decode_controller_pipelined is in compile)
module tb_decode_controller_pipelined;

    // DUT inputs
    reg  [6:0] opcode;
    reg  [2:0] func3;
    reg  [6:0] func7;

    // DUT outputs
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

    // expected signals (for checking)
    reg        exp_ex_alu_src;
    reg        exp_mem_write;
    reg        exp_mem_read;
    reg [2:0]  exp_mem_load_type;
    reg [1:0]  exp_mem_store_type;
    reg        exp_wb_reg_file;
    reg        exp_memtoreg;
    reg        exp_branch;
    reg        exp_jal;
    reg        exp_jalr;
    reg        exp_auipc;
    reg        exp_lui;
    reg [3:0]  exp_alu_ctrl;

    // counters
    integer pass_count = 0;
    integer fail_count = 0;
    integer testno = 0;

    // Instantiate DUT
    decode_controller_pipelined dut (
        .opcode(opcode),
        .func3(func3),
        .func7(func7),

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

    // helper task to compare expected vs actual and print one-line summary
    task check_and_report(input string name);
        begin
            testno = testno + 1;
            if ( ex_alu_src === exp_ex_alu_src &&
                 mem_write  === exp_mem_write &&
                 mem_read   === exp_mem_read &&
                 mem_load_type === exp_mem_load_type &&
                 mem_store_type === exp_mem_store_type &&
                 wb_reg_file === exp_wb_reg_file &&
                 memtoreg === exp_memtoreg &&
                 branch === exp_branch &&
                 jal === exp_jal &&
                 jalr === exp_jalr &&
                 auipc === exp_auipc &&
                 lui === exp_lui &&
                 alu_ctrl === exp_alu_ctrl ) begin
                $display("%3d PASS : %s  (opc=%07b f3=%03b f7=%07b) -> alu_ctrl=%02h, ex_alu_src=%b, memR=%b memW=%b loadType=%03b storeType=%02b wb=%b memtoreg=%b br=%b jal=%b jalr=%b auipc=%b lui=%b",
                    testno, name, opcode, func3, func7, alu_ctrl, ex_alu_src, mem_read, mem_write, mem_load_type, mem_store_type, wb_reg_file, memtoreg, branch, jal, jalr, auipc, lui);
                pass_count = pass_count + 1;
            end else begin
                $display("%3d FAIL : %s  (opc=%07b f3=%03b f7=%07b)", testno, name, opcode, func3, func7);
                $display("       expected: alu_ctrl=%02h, ex_alu_src=%b, memR=%b memW=%b loadType=%03b storeType=%02b wb=%b memtoreg=%b br=%b jal=%b jalr=%b auipc=%b lui=%b",
                    exp_alu_ctrl, exp_ex_alu_src, exp_mem_read, exp_mem_write, exp_mem_load_type, exp_mem_store_type, exp_wb_reg_file, exp_memtoreg, exp_branch, exp_jal, exp_jalr, exp_auipc, exp_lui);
                $display("       actual  : alu_ctrl=%02h, ex_alu_src=%b, memR=%b memW=%b loadType=%03b storeType=%02b wb=%b memtoreg=%b br=%b jal=%b jalr=%b auipc=%b lui=%b",
                    alu_ctrl, ex_alu_src, mem_read, mem_write, mem_load_type, mem_store_type, wb_reg_file, memtoreg, branch, jal, jalr, auipc, lui);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // set all expected default zeros helper
    task set_expected_defaults;
        begin
            exp_ex_alu_src = 1'b0;
            exp_mem_write  = 1'b0;
            exp_mem_read   = 1'b0;
            exp_mem_load_type = `LOAD_DEF;
            exp_mem_store_type= `STORE_DEF;
            exp_wb_reg_file = 1'b0;
            exp_memtoreg = 1'b0;
            exp_branch = 1'b0;
            exp_jal = 1'b0;
            exp_jalr = 1'b0;
            exp_auipc = 1'b0;
            exp_lui = 1'b0;
            exp_alu_ctrl = `ALU_ADD;
        end
    endtask

    initial begin
        $display("\n=== decode_controller_pipelined testbench ===\n");
        // small delay to let things settle
        #1;

        // ---------- Tests ----------
        // 1) R-type ADD (func3=000, func7=0000000)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        // expected for R-type add: ex_alu_src=0, wb=1, alu_ctrl = ALU_ADD
        exp_ex_alu_src = 1'b0;
        exp_wb_reg_file = 1'b1;
        exp_memtoreg = 1'b0;
        exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("R - ADD");

        // 2) R-type SUB (func3=000, func7=0100000)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b000; func7 = `FUNC7_SUB;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SUB;
        #1; check_and_report("R - SUB");

        // 3) R-type XOR (func3=100)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b100; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_XOR;
        #1; check_and_report("R - XOR");

        // 4) R-type OR (func3=110)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b110; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_OR;
        #1; check_and_report("R - OR");

        // 5) R-type AND (func3=111)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b111; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_AND;
        #1; check_and_report("R - AND");

        // 6) R-type SLL (func3=001)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b001; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLL;
        #1; check_and_report("R - SLL");

        // 7) R-type SRL (func3=101 func7=0000000)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b101; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SRL;
        #1; check_and_report("R - SRL");

        // 8) R-type SRA (func3=101 func7=0100000)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b101; func7 = `FUNC7_SUB;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SRA;
        #1; check_and_report("R - SRA");

        // 9) R-type SLT (func3=010)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b010; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLT;
        #1; check_and_report("R - SLT");

        // 10) R-type SLTU (func3=011)
        set_expected_defaults();
        opcode = `OPCODE_RTYPE; func3 = 3'b011; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLTU;
        #1; check_and_report("R - SLTU");

        // ---------- I-type arithmetic (ADDI, XORI, ORI, ANDI, SLTI, SLTIU, SLLI, SRLI, SRAI) ----------
        // 11) ADDI (func3=000)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("I - ADDI");

        // 12) XORI (func3=100)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b100; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_XOR;
        #1; check_and_report("I - XORI");

        // 13) ORI (func3=110)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b110; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_OR;
        #1; check_and_report("I - ORI");

        // 14) ANDI (func3=111)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b111; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_AND;
        #1; check_and_report("I - ANDI");

        // 15) SLTI (func3=010)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b010; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLT;
        #1; check_and_report("I - SLTI");

        // 16) SLTIU (func3=011)
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b011; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLTU;
        #1; check_and_report("I - SLTIU");

        // 17) SLLI (func3=001) shift-immediate -> SLL
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b001; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SLL;
        #1; check_and_report("I - SLLI");

        // 18) SRLI (func3=101, func7=0000000) -> SRL
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b101; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SRL;
        #1; check_and_report("I - SRLI");

        // 19) SRAI (func3=101, func7=0100000) -> SRA
        set_expected_defaults();
        opcode = `OPCODE_ITYPE; func3 = 3'b101; func7 = `FUNC7_SUB;
        exp_ex_alu_src = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SRA;
        #1; check_and_report("I - SRAI");

        // ---------- Loads (I-load) ----------
        // 20) LB (func3=000)
        set_expected_defaults();
        opcode = `OPCODE_ILOAD; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_read = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b1; exp_mem_load_type = `LOAD_LB; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("LD - LB");

        // 21) LH (func3=001)
        set_expected_defaults();
        opcode = `OPCODE_ILOAD; func3 = 3'b001; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_read = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b1; exp_mem_load_type = `LOAD_LH; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("LD - LH");

        // 22) LW (func3=010)
        set_expected_defaults();
        opcode = `OPCODE_ILOAD; func3 = 3'b010; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_read = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b1; exp_mem_load_type = `LOAD_LW; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("LD - LW");

        // 23) LBU (func3=100)
        set_expected_defaults();
        opcode = `OPCODE_ILOAD; func3 = 3'b100; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_read = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b1; exp_mem_load_type = `LOAD_LBU; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("LD - LBU");

        // 24) LHU (func3=101)
        set_expected_defaults();
        opcode = `OPCODE_ILOAD; func3 = 3'b101; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_read = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b1; exp_mem_load_type = `LOAD_LHU; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("LD - LHU");

        // ---------- Stores (S-type) ----------
        // 25) SB (func3=000)
        set_expected_defaults();
        opcode = `OPCODE_STYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_write = 1'b1; exp_mem_store_type = `STORE_SB; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("ST - SB");

        // 26) SH (func3=001)
        set_expected_defaults();
        opcode = `OPCODE_STYPE; func3 = 3'b001; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_write = 1'b1; exp_mem_store_type = `STORE_SH; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("ST - SH");

        // 27) SW (func3=010)
        set_expected_defaults();
        opcode = `OPCODE_STYPE; func3 = 3'b010; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b1; exp_mem_write = 1'b1; exp_mem_store_type = `STORE_SW; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("ST - SW");

        // ---------- Branches (B-type) ----------
        // 28) BEQ (func3=000)
        set_expected_defaults();
        opcode = `OPCODE_BTYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_branch = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SUB;
        #1; check_and_report("BR - BEQ");

        // 29) BNE (func3=001)
        set_expected_defaults();
        opcode = `OPCODE_BTYPE; func3 = 3'b001; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_branch = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SUB;
        #1; check_and_report("BR - BNE");

        // 30) BLT (func3=100)
        set_expected_defaults();
        opcode = `OPCODE_BTYPE; func3 = 3'b100; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_branch = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SUB;
        #1; check_and_report("BR - BLT");

        // 31) BGE (func3=101)
        set_expected_defaults();
        opcode = `OPCODE_BTYPE; func3 = 3'b101; func7 = `FUNC7_ADD;
        exp_ex_alu_src = 1'b0; exp_branch = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_SUB;
        #1; check_and_report("BR - BGE");

        // ---------- Jumps ----------
        // 32) JAL (J-type)
        set_expected_defaults();
        opcode = `OPCODE_JTYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_jal = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("J - JAL");

        // 33) JALR (I-type with jalr opcode)
        set_expected_defaults();
        opcode = `OPCODE_IJALR; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_jalr = 1'b1; exp_wb_reg_file = 1'b1; exp_ex_alu_src = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("J - JALR");

        // ---------- U-type ----------
        // 34) LUI
        set_expected_defaults();
        opcode = `OPCODE_UTYPE; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_lui = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("U - LUI");

        // 35) AUIPC
        set_expected_defaults();
        opcode = `OPCODE_AUIPC; func3 = 3'b000; func7 = `FUNC7_ADD;
        exp_auipc = 1'b1; exp_wb_reg_file = 1'b1; exp_memtoreg = 1'b0; exp_alu_ctrl = `ALU_ADD;
        #1; check_and_report("U - AUIPC");

        // ---------- finished ----------
        #1;
        $display("\nTest summary: passed = %0d, failed = %0d (total = %0d)\n", pass_count, fail_count, pass_count+fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED - inspect output lines above for details");

        $finish;
    end

endmodule

