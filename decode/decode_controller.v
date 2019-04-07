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

module decode_controller (
    input [6:0] opcode,
    input [2:0] func3,
    input [6:0] func7,
    output ex_alu_src,
    output mem_write,
    output mem_read,
    output reg [2:0] mem_load_type,
    output reg [1:0] mem_store_type,
    output wb_reg_file,
    output invalid_inst
);
    wire r_type_inst;
    wire i_type_inst;
    wire wb_inst;
    wire u_type_inst;
    wire b_type_inst;
    wire j_type_inst;
    wire aupic_inst;
    wire jalr_inst;

    assign wb_inst = (opcode == `OPCODE_RTYPE);
    assign r_type_inst = (wb_inst && (func7 == `FUNC7_ADD || func7 == `FUNC7_SUB));
    assign i_type_inst = (opcode == `OPCODE_ITYPE);
    assign mem_write = (opcode == `OPCODE_STYPE);
    assign mem_read = (opcode == `OPCODE_ILOAD);
    assign u_type_inst = (opcode == `OPCODE_UTYPE);
    assign b_type_inst = (opcode == `OPCODE_BTYPE);
    assign j_type_inst = (opcode == `OPCODE_JTYPE);
    assign aupic_inst = ( opcode == `OPCODE_AUIPC);
    assign jalr_inst = (opcode == `OPCODE_IJALR);

    assign ex_alu_src  = i_type_inst || mem_read || mem_write ||
                          u_type_inst ||aupic_inst || jalr_inst;

    assign wb_reg_file  = wb_inst || i_type_inst || mem_read ||
                          u_type_inst ||aupic_inst || jalr_inst || j_type_inst;
                         
    assign invalid_inst = !(r_type_inst || ex_alu_src ||
                            b_type_inst || j_type_inst);

    always @(*) begin
        mem_store_type = `STORE_DEF; // Disable writing
        if (mem_write) begin
            case (func3)
                3'b000: mem_store_type = `STORE_SB;
                3'b001: mem_store_type = `STORE_SH;
                3'b010: mem_store_type = `STORE_SW;
                default:mem_store_type = `STORE_DEF; // Disable writing
            endcase
        end
    end

    always @(*) begin
        mem_load_type = `LOAD_DEF; // Load full value
        if (mem_read) begin
            case (func3)
                3'b000: mem_load_type = `LOAD_LB;
                3'b001: mem_load_type = `LOAD_LH;
                3'b010: mem_load_type = `LOAD_LW;
                3'b100: mem_load_type = `LOAD_LBU;
                3'b101: mem_load_type = `LOAD_LHU;
                default:mem_load_type = `LOAD_DEF; // Load full value
            endcase
        end
    end

endmodule
