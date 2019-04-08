// -------------------------------------------------------------
// 32-bit Arithmetic Unit with full flag support
// Handles: 
//   0000 : ADD / ADDI / address calc / LOAD / STORE / JALR
//   0001 : SUB / branch compare
//   1010 : LUI   (result = rs2, imm prepared outside)
//   1011 : AUIPC (PC + imm)
//
// Flags:
//   zero_flag      -> result_alu == 0
//   carry_flag     -> carry (ADD) / borrow (SUB) / 0 for LUI
//   negative_flag  -> result_alu[31]
//   overflow_flag  -> signed overflow for ADD/SUB
// -------------------------------------------------------------
module arithmetic_unit32 (
    input  [31:0] rs1,        // rs1 or PC
    input  [31:0] rs2,        // rs2 or immediate (or imm_prepared for LUI)
    input  [3:0]  alu_ctrl,   // from alu_control
    output reg [31:0] result_alu,
    output       zero_flag,
    output reg   carry_flag,
    output reg   negative_flag,
    output reg   overflow_flag
);

    // 33-bit extended add/sub (for carry detection)
    wire [32:0] add_ext;
    wire [32:0] sub_ext;

    assign add_ext = {1'b0, rs1} + {1'b0, rs2};
    assign sub_ext = {1'b0, rs1} - {1'b0, rs2};

    always @(*) begin
        // Defaults
        result_alu   = 32'b0;
        carry_flag     = 1'b0;
        negative_flag  = 1'b0;
        overflow_flag  = 1'b0;

        case (alu_ctrl)
            4'b0000: begin
                // ADD / ADDI / address calc / LOAD / STORE / JALR
                // ALU control must give:
                //   rs1 = rs1 (or PC for some address uses)
                //   rs2 = immediate or rs2
                result_alu  = add_ext[31:0];
                carry_flag    = add_ext[32];  // carry-out of MSB
            end

            4'b0001: begin
                // SUB / branch compare
                // usually rs1 = rs1, rs2 = rs2 or imm
                result_alu  = sub_ext[31:0];
                carry_flag    = sub_ext[32];  // acts as "borrow" indicator
            end

            4'b1010: begin
                // LUI : Load Upper Immediate
                // Control should already prepare rs2 as the final LUI value
                // (imm << 12 or whatever your immediate unit does).
                result_alu  = rs2;
                carry_flag    = 1'b0;         // no carry concept here
                // overflow_flag will remain 0 below for non-ADD/SUB
            end

            4'b1011: begin
                // AUIPC : PC + imm
                // Control must provide:
                //   rs1 = PC
                //   rs2 = imm
                result_alu  = add_ext[31:0];
                carry_flag    = add_ext[32];
            end

            default: begin
                result_alu  = 32'b0;
                carry_flag    = 1'b0;
            end
        endcase

        // Negative (sign bit)
        negative_flag = result_alu[31];

        // Overflow handling (signed 2's complement)
        case (alu_ctrl)
            4'b0000: begin
                // ADD signed overflow:
                //  - two positives give negative
                //  - two negatives give positive
                overflow_flag =
                      (~rs1[31] & ~rs2[31] &  result_alu[31]) |
                      ( rs1[31] &  rs2[31] & ~result_alu[31]);
            end

            4'b0001: begin
                // SUB signed overflow:
                //  - (+) - (-) gives negative
                //  - (-) - (+) gives positive
                overflow_flag =
                      ( rs1[31] & ~rs2[31] & ~result_alu[31]) |
                      (~rs1[31] &  rs2[31] &  result_alu[31]);
            end

            default: begin
                // For LUI, AUIPC, etc: no signed overflow concept here
                overflow_flag = 1'b0;
            end
        endcase
    end

    // Zero flag: result is exactly 0
    assign zero_flag = (result_alu == 32'b0);

endmodule


/*
//`timescale 1ns/1ps

module tb_arithmetic_unit32;

    reg  [31:0] rs1;
    reg  [31:0] rs2;
    reg  [3:0]  alu_ctrl;
    wire [31:0] result_alu;
    wire        zero_flag;
    wire        carry_flag;
    wire        negative_flag;
    wire        overflow_flag;

    integer error_count;

    // DUT
    arithmetic_unit32 dut (
        .rs1          (rs1),
        .rs2          (rs2),
        .alu_ctrl     (alu_ctrl),
        .result_alu (result_alu),
        .zero_flag    (zero_flag),
        .carry_flag   (carry_flag),
        .negative_flag(negative_flag),
        .overflow_flag(overflow_flag)
    );

    // Self-checking task
    task run_test;
        input integer test_num;
        input [31:0] t_rs1;
        input [31:0] t_rs2;
        input [3:0]  t_alu_ctrl;
        input [31:0] exp_result;
        input        exp_z;
        input        exp_c;
        input        exp_n;
        input        exp_v;
    begin
        rs1      = t_rs1;
        rs2      = t_rs2;
        alu_ctrl = t_alu_ctrl;

        #1; // small delay to let combinational logic settle

        if (result_alu   !== exp_result ||
            zero_flag      !== exp_z      ||
            carry_flag     !== exp_c      ||
            negative_flag  !== exp_n      ||
            overflow_flag  !== exp_v) begin

            $display("TEST %0d : FAIL", test_num);
            $display("  rs1 = 0x%08h, rs2 = 0x%08h, alu_ctrl = %b",
                      rs1, rs2, alu_ctrl);
            $display("  Expected: result=0x%08h Z=%b C=%b N=%b V=%b",
                      exp_result, exp_z, exp_c, exp_n, exp_v);
            $display("  Got     : result=0x%08h Z=%b C=%b N=%b V=%b",
                      result_alu, zero_flag, carry_flag,
                      negative_flag, overflow_flag);
            error_count = error_count + 1;
        end else begin
            $display("TEST %0d : PASS", test_num);
        end
    end
    endtask

    initial begin
        error_count = 0;

        // ---------------------------------------------------------
        // ADD / ADDI / Address calc (also used for LOAD/STORE/JALR)
        // alu_ctrl = 0000
        // ---------------------------------------------------------

        // Test 1: simple ADD: 5 + 7 = 12
        run_test(1,
                 32'd5, 32'd7,
                 4'b0000,
                 32'd12,    // result
                 1'b0,      // Z
                 1'b0,      // C
                 1'b0,      // N
                 1'b0);     // V

        // Test 2: ADD with carry out (wrap around)
        // 0xFFFF_FFFF + 1 = 0x0000_0000, carry=1
        run_test(2,
                 32'hFFFF_FFFF, 32'h0000_0001,
                 4'b0000,
                 32'h0000_0000, // result
                 1'b1,          // Z
                 1'b1,          // C
                 1'b0,          // N
                 1'b0);         // V

        // Test 3: ADD with signed overflow
        // 0x7FFF_FFFF (max +ve) + 1 = 0x8000_0000 (negative) -> overflow
        run_test(3,
                 32'h7FFF_FFFF, 32'h0000_0001,
                 4'b0000,
                 32'h8000_0000, // result
                 1'b0,          // Z
                 1'b0,          // C
                 1'b1,          // N
                 1'b1);         // V

        // ---------------------------------------------------------
        // SUB / Branch compare
        // alu_ctrl = 0001
        // ---------------------------------------------------------

        // Test 4: simple SUB: 10 - 3 = 7
        run_test(4,
                 32'd10, 32'd3,
                 4'b0001,
                 32'd7,     // result
                 1'b0,      // Z
                 1'b0,      // C (no borrow)
                 1'b0,      // N
                 1'b0);     // V

        // Test 5: SUB resulting in negative (borrow)
        // 3 - 10 = 0xFFFF_FFF9
        run_test(5,
                 32'd3, 32'd10,
                 4'b0001,
                 32'hFFFF_FFF9, // result
                 1'b0,          // Z
                 1'b1,          // C (borrow)
                 1'b1,          // N
                 1'b0);         // V

        // Test 6: SUB with signed overflow
        // 0x8000_0000 (-2^31) - 1 = 0x7FFF_FFFF -> overflow
        run_test(6,
                 32'h8000_0000, 32'h0000_0001,
                 4'b0001,
                 32'h7FFF_FFFF, // result
                 1'b0,          // Z
                 1'b0,          // C
                 1'b0,          // N
                 1'b1);         // V

        // ---------------------------------------------------------
        // LUI
        // alu_ctrl = 1010
        // rs2 should already be the final LUI value from imm unit
        // ---------------------------------------------------------

        run_test(7,
                 32'h0000_0000, 32'h1234_5000,
                 4'b1010,
                 32'h1234_5000, // result
                 1'b0,          // Z
                 1'b0,          // C
                 1'b0,          // N (MSB=0)
                 1'b0);         // V

        // ---------------------------------------------------------
        // AUIPC (PC + imm)
        // alu_ctrl = 1011
        // ---------------------------------------------------------

        run_test(8,
                 32'h0000_1000, 32'h0000_0004,  // PC + imm
                 4'b1011,
                 32'h0000_1004, // result
                 1'b0,          // Z
                 1'b0,          // C
                 1'b0,          // N
                 1'b0);         // V

        // ---------------------------------------------------------
        // Summary
        // ---------------------------------------------------------
        if (error_count == 0) begin
            $display("====================================================");
            $display(" ALL TESTS PASSED for arithmetic_unit32 ?");
            $display("====================================================");
        end else begin
            $display("====================================================");
            $display(" %0d TEST(S) FAILED for arithmetic_unit32 ?", error_count);
            $display("====================================================");
        end

        $finish;
    end

endmodule
*/

