
// Minimal useful defines (you can remove/replace if you already have these)
`define ZERO_12    12'h000
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

// FUNC7 - ADD
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

// B Type Codes
`define BTYPE_BEQ  3'b000
`define BTYPE_BNE  3'b001
`define BTYPE_BLT  3'b100
`define BTYPE_BGE  3'b101
`define BTYPE_BLTU 3'b110
`define BTYPE_BGEU 3'b111

// Forwarding Unit
`define FORWARD_ORG 2'b00
`define FORWARD_MEM 2'b01
`define FORWARD_WB  2'b10

// Store Types
`define STORE_SB  2'b00
`define STORE_SH  2'b01
`define STORE_SW  2'b10
`define STORE_DEF 2'b11

// Load Types
`define LOAD_LB  3'b000
`define LOAD_LH  3'b001   // FIXED NAME
`define LOAD_LW  3'b010
`define LOAD_LBU 3'b011
`define LOAD_LHU 3'b100
`define LOAD_DEF 3'b111

// Constants
`define ZERO_32BIT  32'h00000000
`define ZERO_12BIT  12'h000

// BTB State
`define STRONG_NOT_TAKEN 2'b00
`define WEAK_NOT_TAKEN   2'b01
`define STRONG_TAKEN     2'b10
`define WEAK_TAKEN       2'b11
module decode_unit (
    input  wire [31:0] instruction_in,
    input  wire        id_flush,        // when asserted, treat instruction as NOP (all zeros)
    output wire [6:0]  opcode,
    output wire [2:0]  func3,
    output wire [6:0]  func7,
    output wire [4:0]  rd,
    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output reg  [31:0] imm_out          // decoded immediate (sign-extended where applicable)
);

    // Select either real instruction or zero when flushed
    wire [31:0] instr = id_flush ? 32'h00000000 : instruction_in;

    // Field extraction (combinational)
    assign opcode = instr[6:0];
    assign rd     = instr[11:7];
    assign func3  = instr[14:12];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign func7  = instr[31:25];

    // Immediate generation
    always @(*) begin
        case (opcode)
            // I-type (includes arithmetic immediates) and loads (I-format)
            `OPCODE_ITYPE, `OPCODE_ILOAD: begin
                // instr[31:20] sign-extended to 32 bits
                imm_out = {{20{instr[31]}}, instr[31:20]};
            end

            // S-type (store): imm[11:5]=instr[31:25], imm[4:0]=instr[11:7]
            `OPCODE_STYPE: begin
                imm_out = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end

            // B-type (branch): imm = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0} sign-extended
            `OPCODE_BTYPE: begin
                imm_out = {{19{instr[31]}},
                           instr[31],
                           instr[7],
                           instr[30:25],
                           instr[11:8],
                           1'b0};
            end

            // J-type (JAL): imm = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0} sign-extended
            `OPCODE_JTYPE: begin
                imm_out = {{11{instr[31]}},
                           instr[31],
                           instr[19:12],
                           instr[20],
                           instr[30:21],
                           1'b0};
            end

            // U-type (LUI): imm = instr[31:12] << 12 (no sign extension needed)
            `OPCODE_UTYPE: begin
                imm_out = {instr[31:12], `ZERO_12};
            end

            // AUIPC: same encoding as U-type
            `OPCODE_AUIPC: begin
                imm_out = {instr[31:12], `ZERO_12};
            end

            // For R-type and unknown opcodes, immediate = 0
            default: begin
                imm_out = 32'h00000000;
            end
        endcase
    end

endmodule

