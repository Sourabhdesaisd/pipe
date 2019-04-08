module comparator_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [6:0]  opcode,
    input  [2:0]  func3,
    output reg [31:0] Y
);

    localparam OPCODE_R = 7'b0110011;
    localparam OPCODE_I = 7'b0010011;

    always @(*) begin
        case(func3)
            3'b010: // SLT/SLTI signed
                Y = (($signed(rs1) < $signed(rs2))) ? 32'd1 : 32'd0;

            3'b011: // SLTU/SLTIU unsigned
                Y = (rs1 < rs2) ? 32'd1 : 32'd0;

            default:
                Y = 32'd0;
        endcase
    end
endmodule

