module alu_top32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,     // control from ALU control block
    output [31:0] alu_result,
    output        zero_flag,
    output        negative_flag,
    output        carry_flag,
    output        overflow_flag
);

    // --- Outputs of sub-units ---
    wire [31:0] result_arith;
    wire [31:0] result_logic;
    wire [31:0] result_shift;
    wire [31:0] result_cmp;

    wire zf, nf, cf, of;

    // ---------- Arithmetic unit ----------
    arithmetic_unit32 u_arith (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_alu(result_arith),
        .zero_flag(zf),
        .carry_flag(cf),
        .negative_flag(nf),
        .overflow_flag(of)
    );

    // ---------- Logical unit ----------
    logical_unit32 u_logic (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_alu(result_logic)
    );

    // ---------- Shift unit ----------
    shift_unit32 u_shift (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_shift(result_shift)
    );

    // ---------- Compare unit ----------
    compare_unit32 u_cmp (
        .rs_1(rs1),
        .rs_2(rs2),
        .alu_ctrl(alu_ctrl),
        .result_cmp(result_cmp)
    );

    // ---------- Select output ----------
    reg [31:0] result_final;
    always @(*) begin
        case (alu_ctrl)
            4'b0000,          // ADD / ADDI / LOAD / STORE
            4'b0001,          // SUB
            4'b1010,          // LUI
            4'b1011:          // AUIPC
                result_final = result_arith;

            4'b0010,          // AND / ANDI
            4'b0011,          // OR / ORI
            4'b0100:          // XOR / XORI
                result_final = result_logic;

            4'b0101,          // SLL / SLLI
            4'b0110,          // SRL / SRLI
            4'b0111:          // SRA / SRAI
                result_final = result_shift;

            4'b1000,          // SLT
            4'b1001:          // SLTU
                result_final = result_cmp;

            default:
                result_final = 32'b0;
        endcase
    end

    assign alu_result     = result_final;
    assign zero_flag      = zf;
    assign carry_flag     = cf;
    assign negative_flag  = nf;
    assign overflow_flag  = of;

endmodule



/*

module tb_alu_top32;
    reg  [31:0] rs1, rs2;
    reg  [3:0]  alu_ctrl;
    wire [31:0] alu_result;
    wire zero_flag, negative_flag, carry_flag, overflow_flag;

    alu_top32 DUT (
        .rs1(rs1),
        .rs2(rs2),
        .alu_ctrl(alu_ctrl),
        .alu_result(alu_result),
        .zero_flag(zero_flag),
        .negative_flag(negative_flag),
        .carry_flag(carry_flag),
        .overflow_flag(overflow_flag)
    );

    task CHECK;
        input [31:0] exp;
        begin
            #1;
            if (alu_result === exp)
                $display("PASS  | alu_ctrl=%b  rs1=%0d  rs2=%0d  result=%0d",
                          alu_ctrl, rs1, rs2, alu_result);
            else
                $display("FAIL  | alu_ctrl=%b  rs1=%0d  rs2=%0d  result=%0d  expected=%0d",
                          alu_ctrl, rs1, rs2, alu_result, exp);
        end
    endtask

    initial begin
        $display("================ ALU TEST BEGIN ================");

        //------ ADD -------
        rs1=10; rs2=5; alu_ctrl=4'b0000; CHECK(15);

        //------ SUB -------
        rs1=20; rs2=7; alu_ctrl=4'b0001; CHECK(13);

        //------ AND -------
        rs1=32'hF0F0; rs2=32'h0FF0; alu_ctrl=4'b0010; CHECK(32'h00F0);

        //------ OR -------
        rs1=32'hA0A0; rs2=32'h0A0A; alu_ctrl=4'b0011; CHECK(32'hAAAA);

        //------ XOR -------
        rs1=32'hAAAA; rs2=32'h5555; alu_ctrl=4'b0100; CHECK(32'hFFFF);

        //------ SLL -------
        rs1=32'h1; rs2=32'd4; alu_ctrl=4'b0101; CHECK(32'h10);

        //------ SRL -------
        rs1=32'h80; rs2=32'd4; alu_ctrl=4'b0110; CHECK(32'h8);

        //------ SRA (arithmetic) -------
        rs1=32'hFFFF_FF80; rs2=32'd4; alu_ctrl=4'b0111; CHECK(32'hFFFF_FFF8);

        //------ SLT (signed) -------
        rs1=-5; rs2=10; alu_ctrl=4'b1000; CHECK(1);

        //------ SLTU (unsigned) -------
        rs1=32'hFFFF_FFF0; rs2=32'h10; alu_ctrl=4'b1001; CHECK(0);

        //------ LUI -------
        rs1=32'h1234_5678; rs2=32'hABCD_0000; alu_ctrl=4'b1010; CHECK(32'hABCD_0000);

        //------ AUIPC (PC + imm) -------
        rs1=32'h1000; rs2=32'd20; alu_ctrl=4'b1011; CHECK(32'h1014);

        //------ ADDI -------
        rs1=10; rs2=3; alu_ctrl=4'b0000; CHECK(13);

        //------ ANDI -------
        rs1=32'hFF00; rs2=32'h00F0; alu_ctrl=4'b0010; CHECK(32'h00F0);

        //------ ORI -------
        rs1=32'hFF00; rs2=32'h00F0; alu_ctrl=4'b0011; CHECK(32'hFFF0);

        //------ XORI -------
        rs1=32'hAAAA; rs2=32'h000F; alu_ctrl=4'b0100; CHECK(32'hAAA5);

        //------ SLLI -------
        rs1=32'h2; rs2=32'd3; alu_ctrl=4'b0101; CHECK(32'h10);

        //------ SRLI -------
        rs1=32'h20; rs2=32'd2; alu_ctrl=4'b0110; CHECK(32'h8);

        //------ SRAI -------
        rs1=32'hFFFF_FF80; rs2=32'd2; alu_ctrl=4'b0111; CHECK(32'hFFFF_FFE0);

        //------ FINAL ------
        $display("================ ALU TEST END ================");
        $finish;
    end
endmodule
*/
