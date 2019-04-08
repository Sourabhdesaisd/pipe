module logical_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,  // from alu_control block
    output reg [31:0] result_alu
);

    always @(*) begin
        case (alu_ctrl)
            4'b0010: result_alu = rs1 & rs2; // AND / ANDI
            4'b0011: result_alu = rs1 | rs2; // OR / ORI
            4'b0100: result_alu = rs1 ^ rs2; // XOR / XORI

            default: result_alu = 32'b0;
        endcase
    end

endmodule

/*
module tb_logical_unit32;

    reg  [31:0] rs1;
    reg  [31:0] rs2;
    reg  [3:0]  alu_ctrl;
    wire [31:0] result_alu;

    integer errors;

    // Instantiate DUT
    logical_unit32 dut (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_alu(result_alu)
    );
initial begin
$shm_open("wave.shm");
$shm_probe("ACTMF");
end



    // Check Task
    task check;
        input [31:0] a, b;
        input [3:0] ctrl;
        input [31:0] expected;
        input [127:0] name;
    begin
        rs1 = a;
        rs2 = b;
        alu_ctrl = ctrl;
        #1; // allow settling

        if (result_alu !== expected) begin
            $display("FAIL: %0s  ctrl=%b  a=%h b=%h  expected=%h got=%h",
                    name, ctrl, a, b, expected, result_alu);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s  result=%h", name, result_alu);
        end
    end
    endtask

    initial begin
        errors = 0;

        // Test Vectors
        check(32'hA5A5_F0F0, 32'h0F0F_A5A5, 4'b0010, (32'hA5A5_F0F0 & 32'h0F0F_A5A5), "AND");
        check(32'h1234_5678, 32'hFFFF_0000, 4'b0011, (32'h1234_5678 | 32'hFFFF_0000), "OR");
        check(32'hAAAA_AAAA, 32'h5555_5555, 4'b0100, (32'hAAAA_AAAA ^ 32'h5555_5555), "XOR");

        // Default / Illegal alu_ctrl
        check(32'h1111_2222, 32'h3333_4444, 4'b1111, 32'b0, "DEFAULT");

        // Summary
        if (errors == 0)
            $display("\n==== ALL LOGICAL UNIT TESTS PASSED ====\n");
        else
            $display("\n==== %0d TEST(S) FAILED ====\n", errors);

        $finish;
    end

endmodule
*/
