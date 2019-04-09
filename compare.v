module compare_unit32 (
    input  [31:0] rs_1,        // RS1 value
    input  [31:0] rs_2,        // RS2 or immediate
    input  [3:0]  alu_ctrl,   // From alu_control unit
    output reg [31:0] result_cmp
);

    always @(*) begin
        case (alu_ctrl)
            4'b1000: // SLT (signed)
                result_cmp = ($signed(rs_1) < $signed(rs_2)) ? 32'b1 : 32'b0;

            4'b1001: // SLTU (unsigned)
                result_cmp = (rs_1 < rs_2) ? 32'b1 : 32'b0;

            default:
                result_cmp = 32'b0;
        endcase
    end

endmodule

/*//`timescale 1ns/1ps

module tb_compare_unit32;

    reg  [31:0] rs_1;
    reg  [31:0] rs_2;
    reg  [3:0]  alu_ctrl;
    wire [31:0] result_cmp;

    integer errors;

    compare_unit32 dut (
        .rs_1(rs_1),
        .rs_2(rs_2),
        .alu_ctrl(alu_ctrl),
        .result_cmp(result_cmp)
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
        rs_1 = a;
        rs_2 = b;
        alu_ctrl = ctrl;
        #1;

        if (result_cmp !== expected) begin
            $display("FAIL: %0s exp=%h got=%h", name, expected, result_cmp);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s value=%h", name, result_cmp);
        end
    end
    endtask

    initial begin
        errors = 0;

        // Signed SLT tests
        check(32'd5, 32'd10, 4'b1000, 32'b1, "SLT smaller");
        check(32'd10, 32'd5, 4'b1000, 32'b0, "SLT greater");
        check(-32'sd5, 32'd3, 4'b1000, 32'b1, "SLT negative < positive");

        // Unsigned SLTU tests
        check(32'h0000_0005, 32'h0000_000A, 4'b1001, 32'b1, "SLTU smaller");
        check(32'hFFFF_FFFF, 32'h0000_0001, 4'b1001, 32'b0, "SLTU unsigned check");

        // Summary
        if (errors == 0)
            $display("\n==== COMPARE UNIT TESTS PASSED ====\n");
        else
            $display("\n==== %0d TEST(S) FAILED ====\n", errors);

        $finish;
    end

endmodule
*/
