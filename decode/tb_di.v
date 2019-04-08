/*
// Keep these defines in sync with decode_unit.v (if you used different names change here)
`define ZERO_12    12'h000

`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111

module tb_decode_unit_simple;

    // TB signals
    reg  [31:0] instruction_in;
    reg         id_flush;
    wire [6:0]  opcode;
    wire [2:0]  func3;
    wire [6:0]  func7;
    wire [4:0]  rd;
    wire [4:0]  rs1;
    wire [4:0]  rs2;
    wire [31:0] imm_out;

    // Instantiate the decode_unit (assumes decode_unit.v present)
    decode_unit dut (
        .instruction_in(instruction_in),
        .id_flush(id_flush),
        .opcode(opcode),
        .func3(func3),
        .func7(func7),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .imm_out(imm_out)
    );

   // initial begin

   //     $shm_open("wave.shm") ;
  //      $shm_probe("ACTMF") ;

 //   end


    // helper task to print header
    initial begin
       $shm_open("wave.shm") ;
        $shm_probe("ACTMF") ;

        $display("\n=== decode_unit simple testbench ===");
        $display("Fields: INST_HEX | exp_opcode func3 func7 rd rs1 rs2 imm | act_opcode func3 func7 rd rs1 rs2 imm | RESULT");
        $display("-------------------------------------------------------------------------------------------------------------");
    end

    // ---------- TESTS ----------
    initial begin
        id_flush = 0;

        // ----- 1) R-type: ADD (func7=0, func3=000, rd=5, rs1=2, rs2=3, opcode=0110011)
        instruction_in = {7'b0000000, 5'd3, 5'd2, 3'b000, 5'd5, `OPCODE_RTYPE}; // 31:25 func7,24:20 rs2,19:15 rs1,14:12 f3,11:7 rd,6:0 opc
        #1;
        $display("R-ADD  %08h | exp: %07b %03b %07b %02d %02d %02d %08h | act: %07b %03b %07b %02d %02d %02d %08h | %s",
            instruction_in,
            instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
            instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
            32'h00000000,
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===instruction_in[6:0] && func3===instruction_in[14:12] && func7===instruction_in[31:25] &&
              rd===instruction_in[11:7] && rs1===instruction_in[19:15] && rs2===instruction_in[24:20] && imm_out==32'h0)
              ? "PASS":"FAIL")
        );

        // ----- 2) R-type: SUB (func7=0100000)
        instruction_in = {7'b0100000, 5'd4, 5'd6, 3'b000, 5'd7, `OPCODE_RTYPE};
        #1;
        $display("R-SUB  %08h | exp: %07b %03b %07b %02d %02d %02d %08h | act: %07b %03b %07b %02d %02d %02d %08h | %s",
            instruction_in,
            instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
            instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
            32'h00000000,
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===instruction_in[6:0] && func3===instruction_in[14:12] && func7===instruction_in[31:25] &&
              rd===instruction_in[11:7] && rs1===instruction_in[19:15] && rs2===instruction_in[24:20] && imm_out==32'h0)
              ? "PASS":"FAIL")
        );

        // ----- 3) I-type: ADDI imm = 12'h00A  (instr[31:20] = 0x00A, rs1=2, rd=5, opcode=0010011)
        instruction_in = {12'h00A, 5'd2, 3'b000, 5'd5, `OPCODE_ITYPE};
        #1;
        $display("I-ADDI %08h | exp: %07b %03b %07b %02d %02d %02d %08h | act: %07b %03b %07b %02d %02d %02d %08h | %s",
            instruction_in,
            instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
            instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
            {{20{instruction_in[31]}}, instruction_in[31:20]},
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===instruction_in[6:0] && func3===instruction_in[14:12] &&
              rd===instruction_in[11:7] && rs1===instruction_in[19:15] &&
              imm_out=={{20{instruction_in[31]}}, instruction_in[31:20]}) ? "PASS":"FAIL")
        );

        // ----- 4) Load: LW imm = 12'h010 rs1=1 rd=8 opcode = ILOAD (0000011), func3=010
        instruction_in = {12'h010, 5'd1, 3'b010, 5'd8, `OPCODE_ILOAD};
        #1;
        $display("I-LW   %08h | exp: %07b %03b %07b %02d %02d %02d %08h | act: %07b %03b %07b %02d %02d %02d %08h | %s",
            instruction_in,
            instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
            instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
            {{20{instruction_in[31]}}, instruction_in[31:20]},
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===instruction_in[6:0] && func3===instruction_in[14:12] &&
              rd===instruction_in[11:7] && rs1===instruction_in[19:15] &&
              imm_out=={{20{instruction_in[31]}}, instruction_in[31:20]}) ? "PASS":"FAIL")
        );

        // ----- 5) Store: SW imm = 12'h00F -> instr[31:25]=imm[11:5], instr[11:7]=imm[4:0]
        // Build S-format: instr = {imm[11:5], rs2, rs1, func3, imm[4:0], opcode}
        // Use imm=12'h00F (0x0F) => imm[11:5]=7'b0000000, imm[4:0]=5'b01111
        instruction_in = {7'b0000000, 5'd3, 5'd1, 3'b010, 5'b01111, `OPCODE_STYPE}; // rs2=3, rs1=1, func3=010
        #1;
        $display("S-SW   %08h | exp: %07b %03b %07b %02d %02d %02d %08h | act: %07b %03b %07b %02d %02d %02d %08h | %s",
            instruction_in,
            instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
            instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
            {{20{instruction_in[31]}}, instruction_in[31:25], instruction_in[11:7]},
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===instruction_in[6:0] && func3===instruction_in[14:12] &&
              rs1===instruction_in[19:15] && rs2===instruction_in[24:20] &&
              imm_out=={{20{instruction_in[31]}}, instruction_in[31:25], instruction_in[11:7]}) ? "PASS":"FAIL")
        );

        // ----- 6) Branch: BEQ imm=4 (encode as B-format)
        // B-format fields: instr[31]=imm[12], instr[7]=imm[11], instr[30:25]=imm[10:5], instr[11:8]=imm[4:1], LSB=0
        // For imm=4 => imm[4:1]=0010 (bits 4..1), imm[10:5]=0 etc.
        instruction_in = {1'b0, 6'b000000, 5'd3, 5'd2, 3'b000, 4'b0010, 1'b0, `OPCODE_BTYPE};
        #1;
        $display("B-BEQ  %08h | exp imm=%08h | act imm=%08h | %s",
            instruction_in,
            {{19{instruction_in[31]}}, instruction_in[31], instruction_in[7], instruction_in[30:25], instruction_in[11:8], 1'b0},
            imm_out,
            ((imm_out=={{19{instruction_in[31]}}, instruction_in[31], instruction_in[7], instruction_in[30:25], instruction_in[11:8], 1'b0}) ? "PASS":"FAIL")
        );

        // ----- 7) J-type: JAL (small positive immediate). Example construct:
        // J-format layout: [31]=imm[20], [19:12]=imm[19:12], [20]=imm[11], [30:21]=imm[10:1], lsb=0
        // We'll set imm such that instr[19:12] = 8'b00000001 -> small jump
        instruction_in = {1'b0, 8'b00000001, 1'b0, 10'b0000000000, 5'd5, `OPCODE_JTYPE};
        #1;
        $display("J-JAL  %08h | exp imm=%08h | act imm=%08h | %s",
            instruction_in,
            {{11{instruction_in[31]}}, instruction_in[31], instruction_in[19:12], instruction_in[20], instruction_in[30:21], 1'b0},
            imm_out,
            ((imm_out=={{11{instruction_in[31]}}, instruction_in[31], instruction_in[19:12], instruction_in[20], instruction_in[30:21], 1'b0}) ? "PASS":"FAIL")
        );

        // ----- 8) U-type: LUI (imm_high = 20'h000AB, rd=5)
        instruction_in = {20'h000AB, 5'd5, `OPCODE_UTYPE};
        #1;
        $display("U-LUI  %08h | exp imm=%08h | act imm=%08h | %s",
            instruction_in,
            {instruction_in[31:12], 12'h000},
            imm_out,
            ((imm_out=={instruction_in[31:12], 12'h000}) ? "PASS":"FAIL")
        );

        // ----- 9) U-type: AUIPC (imm_high = 20'h00123, rd=6)
        instruction_in = {20'h00123, 5'd6, `OPCODE_AUIPC};
        #1;
        $display("U-AUIPC%08h | exp imm=%08h | act imm=%08h | %s",
            instruction_in,
            {instruction_in[31:12], 12'h000},
            imm_out,
            ((imm_out=={instruction_in[31:12], 12'h000}) ? "PASS":"FAIL")
        );

        // ----- 10) id_flush test: when id_flush=1 instruction should be treated as zero
        instruction_in = 32'hdeadbeef; // garbage input
        id_flush = 1;
        #1;
        $display("ID_FLUSH %08h | exp opcode=0000000 func3=000 func7=0000000 rd=0 rs1=0 rs2=0 imm=0 | act: opc=%07b f3=%03b f7=%07b rd=%02d rs1=%02d rs2=%02d imm=%08h | %s",
            instruction_in,
            opcode, func3, func7, rd, rs1, rs2, imm_out,
            ((opcode===7'b0000000 && func3===3'b000 && func7===7'b0000000 && rd===5'd0 && rs1===5'd0 && rs2===5'd0 && imm_out===32'h0) ? "PASS":"FAIL")
        );
        id_flush = 0;

        $display("\n=== End of tests ===\n");
        $finish;
    end

endmodule
`timescale 1ns/1ps

`define ZERO_12    12'h000

`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111
`define OPCODE_IJALR 7'b1100111 */

module tb_decode_unit_all_expected;

    reg  [31:0] instruction_in;
    reg         id_flush;
    wire [6:0]  opcode;
    wire [2:0]  func3;
    wire [6:0]  func7;
    wire [4:0]  rd;
    wire [4:0]  rs1;
    wire [4:0]  rs2;
    wire [31:0] imm_out;

    // DUT (use your decode_unit that outputs imm_out and fields)
    decode_unit dut (
        .instruction_in(instruction_in),
        .id_flush(id_flush),
        .opcode(opcode),
        .func3(func3),
        .func7(func7),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .imm_out(imm_out)
    );

    // compute expected immediate in TB using same decoding rules as decode_unit
    function automatic [31:0] expected_imm(input [31:0] inst);
        reg [6:0] opc;
        begin
            opc = inst[6:0];
            case (opc)
                `OPCODE_ITYPE, `OPCODE_ILOAD: expected_imm = {{20{inst[31]}}, inst[31:20]};
                `OPCODE_STYPE:                 expected_imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};
                `OPCODE_BTYPE:                 expected_imm = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                `OPCODE_JTYPE:                 expected_imm = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
                `OPCODE_UTYPE:                 expected_imm = {inst[31:12], 12'h000};
                `OPCODE_AUIPC:                 expected_imm = {inst[31:12], 12'h000};
                default:                       expected_imm = 32'h00000000;
            endcase
        end
    endfunction

    initial begin
      $shm_open("wave.shm") ;
        $shm_probe("ACTMF") ;

        $display("\n=== decode_unit: EXPECTED vs ACTUAL immediate (all instructions) ===");
        $display("INST_HEX  | opcode func3 func7 rd rs1 rs2 | EXPECTED_IMM   | ACTUAL_IMM     | RESULT");
        $display("-------------------------------------------------------------------------------------------");

        id_flush = 0;

        // ---------------- R-type instructions (expected imm = 0) ----------------
        instruction_in = {7'b0000000, 5'd3, 5'd2, 3'b000, 5'd5, `OPCODE_RTYPE}; // ADD
        #1;
        $display("ADD   %08h | %07b %03b %07b %02d %02d %02d | %08h | %08h | %s",
                 instruction_in, instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
                 instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
                 expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0100000, 5'd4, 5'd6, 3'b000, 5'd7, `OPCODE_RTYPE}; // SUB
        #1;
        $display("SUB   %08h | %07b %03b %07b %02d %02d %02d | %08h | %08h | %s",
                 instruction_in, instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
                 instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
                 expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000, 5'd8, 5'd7, 3'b100, 5'd4, `OPCODE_RTYPE}; // XOR
        #1;
        $display("XOR   %08h | %07b %03b %07b %02d %02d %02d | %08h | %08h | %s",
                 instruction_in, instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
                 instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
                 expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000, 5'd9, 5'd1, 3'b110, 5'd2, `OPCODE_RTYPE}; // OR
        #1;
        $display("OR    %08h | %07b %03b %07b %02d %02d %02d | %08h | %08h | %s",
                 instruction_in, instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
                 instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
                 expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000, 5'd10, 5'd11, 3'b111, 5'd1, `OPCODE_RTYPE}; // AND
        #1;
        $display("AND   %08h | %07b %03b %07b %02d %02d %02d | %08h | %08h | %s",
                 instruction_in, instruction_in[6:0], instruction_in[14:12], instruction_in[31:25],
                 instruction_in[11:7], instruction_in[19:15], instruction_in[24:20],
                 expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // shifts, slt, sltu
        instruction_in = {7'b0000000,5'd12,5'd13,3'b001,5'd3,`OPCODE_RTYPE}; #1;
        $display("SLL   %08h | f3=%03b f7=%07b | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, instruction_in[14:12], instruction_in[31:25], expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000,5'd14,5'd15,3'b101,5'd4,`OPCODE_RTYPE}; #1;
        $display("SRL   %08h | f3=%03b f7=%07b | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, instruction_in[14:12], instruction_in[31:25], expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0100000,5'd1,5'd2,3'b101,5'd6,`OPCODE_RTYPE}; #1;
        $display("SRA   %08h | f3=%03b f7=%07b | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, instruction_in[14:12], instruction_in[31:25], expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000,5'd7,5'd8,3'b010,5'd9,`OPCODE_RTYPE}; #1;
        $display("SLT   %08h | f3=%03b f7=%07b | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, instruction_in[14:12], instruction_in[31:25], expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000,5'd2,5'd3,3'b011,5'd4,`OPCODE_RTYPE}; #1;
        $display("SLTU  %08h | f3=%03b f7=%07b | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, instruction_in[14:12], instruction_in[31:25], expected_imm(instruction_in), imm_out,
                 (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- I-type ALU immediates ----------------
        instruction_in = {12'h00A, 5'd2, 3'b000, 5'd5, `OPCODE_ITYPE}; #1; // ADDI
        $display("ADDI  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h00B, 5'd2, 3'b100, 5'd5, `OPCODE_ITYPE}; #1; // XORI
        $display("XORI  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h00C, 5'd2, 3'b110, 5'd5, `OPCODE_ITYPE}; #1; // ORI
        $display("ORI   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h0FF, 5'd2, 3'b111, 5'd5, `OPCODE_ITYPE}; #1; // ANDI (negative immediate test)
        $display("ANDI  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h001, 5'd2, 3'b001, 5'd5, `OPCODE_ITYPE}; #1; // SLLI
        $display("SLLI  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

// ---------- SRLI and SRAI tests (correct func7 values) ----------
    // SRLI: func7 = 7'b0000000  (0x00), func3 = 3'b101, shamt = 5'd3, rs1=2, rd=5, opcode = ITYPE (0010011)
    instruction_in = {7'b0000000, 5'd3, 5'd2, 3'b101, 5'd5, `OPCODE_ITYPE}; // SRLI
    #1;
    $display("SRLI  %08h | exp func7=%07b (0x%02h) exp_shamt=%02d exp_imm=%08h | act func7=%07b act_imm=%08h | %s",
             instruction_in,
             instruction_in[31:25], instruction_in[31:25],
             instruction_in[24:20],
             {{20{instruction_in[31]}}, instruction_in[31:20]}, // expected I-type imm (but for shift imm uses shamt)
             func7, imm_out,
             ((func7 === 7'b0000000 && imm_out === {{20{instruction_in[31]}}, instruction_in[31:20]}) ? "PASS":"FAIL")
    );

    // SRAI: func7 = 7'b0100000 (0x20), func3 = 3'b101, shamt = 5'd3, rs1=2, rd=5, opcode = ITYPE (0010011)
    instruction_in = {7'b0100000, 5'd3, 5'd2, 3'b101, 5'd5, `OPCODE_ITYPE}; // SRAI
    #1;
    $display("SRAI  %08h | exp func7=%07b (0x%02h) exp_shamt=%02d exp_imm=%08h | act func7=%07b act_imm=%08h | %s",
             instruction_in,
             instruction_in[31:25], instruction_in[31:25],
             instruction_in[24:20],
             {{20{instruction_in[31]}}, instruction_in[31:20]},
             func7, imm_out,
             ((func7 === 7'b0100000 && imm_out === {{20{instruction_in[31]}}, instruction_in[31:20]}) ? "PASS":"FAIL")
    );
       

        instruction_in = {12'h00D,5'd2,3'b010,5'd5,`OPCODE_ITYPE}; #1; // SLTI
        $display("SLTI  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h00E,5'd2,3'b011,5'd5,`OPCODE_ITYPE}; #1; // SLTIU
        $display("SLTIU %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- Loads ----------------
        instruction_in = {12'h010, 5'd1, 3'b000, 5'd8, `OPCODE_ILOAD}; #1; // LB
        $display("LB    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h011, 5'd1, 3'b001, 5'd8, `OPCODE_ILOAD}; #1; // LH
        $display("LH    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h012, 5'd1, 3'b010, 5'd8, `OPCODE_ILOAD}; #1; // LW
        $display("LW    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h013, 5'd1, 3'b100, 5'd8, `OPCODE_ILOAD}; #1; // LBU
        $display("LBU   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h014, 5'd1, 3'b101, 5'd8, `OPCODE_ILOAD}; #1; // LHU
        $display("LHU   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- Stores ----------------
        instruction_in = {7'b0000000,5'd3,5'd1,3'b000,5'b01111,`OPCODE_STYPE}; #1; // SB
        $display("SB    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000,5'd3,5'd1,3'b001,5'b01111,`OPCODE_STYPE}; #1; // SH
        $display("SH    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {7'b0000000,5'd3,5'd1,3'b010,5'b01111,`OPCODE_STYPE}; #1; // SW
        $display("SW    %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- Branches ----------------
        instruction_in = {1'b0,6'b000000,5'd3,5'd2,3'b000,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BEQ imm=4
        $display("BEQ   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {1'b0,6'b000000,5'd4,5'd5,3'b001,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BNE
        $display("BNE   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {1'b0,6'b000000,5'd6,5'd7,3'b100,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BLT
        $display("BLT   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {1'b0,6'b000000,5'd8,5'd9,3'b101,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BGE
        $display("BGE   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {1'b0,6'b000000,5'd1,5'd2,3'b110,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BLTU
        $display("BLTU  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {1'b0,6'b000000,5'd3,5'd4,3'b111,4'b0010,1'b0,`OPCODE_BTYPE}; #1; // BGEU
        $display("BGEU  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- Jumps ----------------
        instruction_in = {1'b0, 8'b00000001, 1'b0, 10'b0000000000, 5'd5, `OPCODE_JTYPE}; #1; // JAL
        $display("JAL   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {12'h020, 5'd1, 3'b000, 5'd5, `OPCODE_IJALR}; #1; // JALR
        $display("JALR  %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- U-type ----------------
        instruction_in = {20'h000AB, 5'd5, `OPCODE_UTYPE}; #1; // LUI
        $display("LUI   %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        instruction_in = {20'h00123, 5'd6, `OPCODE_AUIPC}; #1; // AUIPC
        $display("AUIPC %08h | exp_imm=%08h | act_imm=%08h | %s",
                 instruction_in, expected_imm(instruction_in), imm_out, (imm_out === expected_imm(instruction_in) ? "PASS":"FAIL"));

        // ---------------- id_flush test ----------------
        instruction_in = 32'hdeadbeef; id_flush = 1; #1;
        $display("IDFLUSH %08h | exp all zeros imm=%08h | act imm=%08h | %s",
                 instruction_in, 32'h0, imm_out, (imm_out === 32'h0 ? "PASS":"FAIL"));
        id_flush = 0;

        $display("\n=== End of all tests ===\n");
        $finish;
    end

endmodule

