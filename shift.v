module shift_unit32 (
    input  [31:0] rs1,       // RS1 value
    input  [31:0] rs2,       // RS2 or immediate (shift amount in lower 5 bits)
    input  [3:0]  alu_ctrl,  // From alu_control
    output reg [31:0] result_shift
);

    wire [4:0] shamt;
    assign shamt = rs2[4:0];  // RV32: shift amount = lower 5 bits

    always @(*) begin
        case (alu_ctrl)
            4'b0101: result_shift = rs1 << shamt;            // SLL, SLLI
            4'b0110: result_shift = rs1 >> shamt;            // SRL, SRLI (logical)
            4'b0111: result_shift = $signed(rs1) >>> shamt;  // SRA, SRAI (arithmetic)

            default: result_shift = 32'b0;
        endcase
    end

endmodule


/*
//`timescale 1ns/1ps

module tb_shift_unit32;

    reg  [31:0] rs1;
    reg  [31:0] rs2;
    reg  [3:0]  alu_ctrl;
    wire [31:0] result_shift;

    integer errors;

    shift_unit32 dut (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_shift(result_shift)
    );
initial begin
$shm_open("wave.shm");
$shm_probe("ACTMF");
end



    task check;
        input [31:0] a, b;
        input [3:0] ctrl;
        input [31:0] expected;
        input [50*8:1] name;
    begin
        rs1 = a;
        rs2 = b;
        alu_ctrl = ctrl;
        #1;

        if (result_shift !== expected) begin
            $display("FAIL: %0s  exp=%h got=%h", name, expected, result_shift);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s  value=%h", name, result_shift);
        end
    end
    endtask

    initial begin
        errors = 0;

        check(32'h0000_000F, 32'h0000_0002, 4'b0101, 32'h0000_003C, "SLL");
        check(32'hF000_0000, 32'h0000_0004, 4'b0110, 32'h0F00_0000, "SRL");
        check(32'hF000_0000, 32'h0000_0004, 4'b0111, 32'hFF00_0000, "SRA"); // Sign Extend

        // Summary
        if (errors == 0)
            $display("\n==== SHIFT UNIT TESTS PASSED ====\n");
        else
            $display("\n==== %0d TEST(S) FAILED ====\n", errors);

        $finish;
    end

endmodule*/

