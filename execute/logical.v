module logical_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [6:0]  opcode,
    input  [2:0]  func3,
    output reg [31:0] Y
);

    always @(*) begin
        case(func3)
            3'b100: Y = rs1 ^ rs2;  // XOR
            3'b110: Y = rs1 | rs2;  // OR
            3'b111: Y = rs1 & rs2;  // AND
            default: Y = 32'b0;
        endcase
    end
endmodule

