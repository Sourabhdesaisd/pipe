module shifter_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [6:0]  opcode,
    input  [2:0]  func3,
    input  [6:0]  func7,
    input  [31:0] imm_out,    // imm_out_out from decode
    output reg [31:0] Y
);

    localparam OPCODE_R = 7'b0110011;
    localparam OPCODE_I = 7'b0010011;
    reg [4:0] shamt;

    always @(*) begin
        if (opcode == OPCODE_R) shamt = rs2[4:0];
        else if (opcode == OPCODE_I) shamt = imm_out[4:0];
        else shamt = 5'd0;

        case (func3)
            3'b001: begin // SLL / SLLI
                if ((opcode == OPCODE_R && func7 == 7'b0000000) ||
                    (opcode == OPCODE_I && imm_out[11:5] == 7'b0000000))
                    Y = rs1 << shamt;
                else
                    Y = 32'b0;
            end

            3'b101: begin // SRL/SRLI/SRA/SRAI
                if ((opcode == OPCODE_R && func7 == 7'b0000000) ||
                    (opcode == OPCODE_I && imm_out[11:5] == 7'b0000000))
                    Y = rs1 >> shamt; // logical
                else if ((opcode == OPCODE_R && func7 == 7'b0100000) ||
                         (opcode == OPCODE_I && imm_out[11:5] == 7'b0100000))
                    Y = $signed(rs1) >>> shamt; // arithmetic
                else
                    Y = 32'b0;
            end

            default: Y = 32'b0;
        endcase
    end

endmodule

