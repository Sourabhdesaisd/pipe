module execute_top (
    input  [31:0] rs1,     // Read data 1 from register file
    input  [31:0] rs2,     // Read data 2 from register file
    input  [31:0] imm_out,          // immediate from decoder
    input         ex_alu_src,       // comes from main control unit
    input  [6:0]  opcode,
    input  [2:0]  funct3,
    input  [6:0]  funct7,

    output [31:0] alu_result,
    output        zero_flag,
    output        negative_flag,
    output        carry_flag,
    output        overflow_flag
);

    // ------------- ex_alu_src mux -------------
    wire [31:0] alu_src2;

    mux_alu_src u_mux (
        .rs2   (rs2),
        .imm_out   (imm_out),
        .ex_alu_src     (ex_alu_src),
        .alu_imm_rs2 (alu_src2)
    );

    // ------------- ALU control -------------
    wire [3:0] alu_ctrl;

    alu_control u_alu_ctrl (
        .opcode (opcode),
        .funct3 (funct3),
        .funct7 (funct7),
        .alu_ctrl (alu_ctrl)
    );

    // ------------- ALU core -------------
    alu_top32 u_alu (
        .rs1          (rs1),
        .rs2          (alu_src2),     // <-- input selected by mux
        .alu_ctrl     (alu_ctrl),
        .alu_result   (alu_result),
        .zero_flag    (zero_flag),
        .negative_flag(negative_flag),
        .carry_flag   (carry_flag),
        .overflow_flag(overflow_flag)
    );

endmodule



module tb_execute_top;

    // Inputs to execute_top
    reg  [31:0] rs1;
    reg  [31:0] rs2;
    reg  [31:0] imm_out;
    reg         ex_alu_src;   // 0 -> rs2, 1 -> imm_out
    reg  [6:0]  opcode;
    reg  [2:0]  funct3;
    reg  [6:0]  funct7;

    // Outputs from execute_top
    wire [31:0] alu_result;
    wire        zero_flag;
    wire        negative_flag;
    wire        carry_flag;
    wire        overflow_flag;

    // DUT
    execute_top dut (
        .rs1           (rs1),
        .rs2           (rs2),
        .imm_out       (imm_out),
        .ex_alu_src    (ex_alu_src),
        .opcode        (opcode),
        .funct3        (funct3),
        .funct7        (funct7),
        .alu_result    (alu_result),
        .zero_flag     (zero_flag),
        .negative_flag (negative_flag),
        .carry_flag    (carry_flag),
        .overflow_flag (overflow_flag)
    );

    // Simple self-check task (checks result, prints flags)
    task CHECK;
        input [31:0]   exp;
        input [127:0]  name;    // label for the case
        begin
            #1;  // wait for combinational logic
            if (alu_result === exp) begin
                $display("PASS : %-12s | result=%0d (0x%08h)  Z=%b N=%b C=%b O=%b",
                         name, alu_result, alu_result,
                         zero_flag, negative_flag, carry_flag, overflow_flag);
            end else begin
                $display("FAIL : %-12s | result=%0d (0x%08h) exp=%0d (0x%08h)  Z=%b N=%b C=%b O=%b",
                         name, alu_result, alu_result, exp, exp,
                         zero_flag, negative_flag, carry_flag, overflow_flag);
            end
        end
    endtask

    initial begin
        $display("=========== EXECUTE_TOP (ALU + MUX + ALU_CTRL) TEST BEGIN ==========");

        //------------------------------------------------------------------
        // R-type: ADD  rd = rs1 + rs2   (no carry / overflow expected)
        //------------------------------------------------------------------
        rs1        = 32'd10;
        rs2        = 32'd20;
        imm_out    = 32'd0;
        ex_alu_src = 1'b0;         // use rs2
        opcode     = 7'b0110011;   // R-type
        funct3     = 3'b000;       // ADD/SUB
        funct7     = 7'b0000000;   // ADD
        CHECK(32'd30, "ADD");

        //------------------------------------------------------------------
        // R-type: SUB  rd = rs1 - rs2
        //------------------------------------------------------------------
        rs1        = 32'd40;
        rs2        = 32'd15;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB
        CHECK(32'd25, "SUB");

        //------------------------------------------------------------------
        // R-type: AND
        //------------------------------------------------------------------
        rs1        = 32'hF0F0_00FF;
        rs2        = 32'h0FF0_F0F0;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b111;       // AND
        funct7     = 7'b0000000;
        CHECK(32'h00F0_00F0, "AND");

        //------------------------------------------------------------------
        // R-type: OR
        //------------------------------------------------------------------
        rs1        = 32'hA0A0_0000;
        rs2        = 32'h00A0_0A0A;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b110;       // OR
        funct7     = 7'b0000000;
        CHECK(32'hA0A0_0A0A, "OR");

        //------------------------------------------------------------------
        // R-type: SLT (signed)
        //------------------------------------------------------------------
        rs1        = -5;           // 0xFFFF_FFFB
        rs2        =  8;           // 0x0000_0008
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b010;       // SLT
        funct7     = 7'b0000000;
        CHECK(32'd1, "SLT");

        //------------------------------------------------------------------
        // I-type: ADDI  (mux selects imm_out)
        //------------------------------------------------------------------
        rs1        = 32'd10;
        rs2        = 32'd123;      // ignored
        imm_out    = 32'd4;
        ex_alu_src = 1'b1;         // use immediate
        opcode     = 7'b0010011;   // I-type
        funct3     = 3'b000;       // ADDI
        funct7     = 7'b0000000;
        CHECK(32'd14, "ADDI");

        //------------------------------------------------------------------
        // I-type: ORI
        //------------------------------------------------------------------
        rs1        = 32'h0000_FF00;
        imm_out    = 32'h0000_00F0;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b110;       // ORI
        funct7     = 7'b0000000;
        CHECK(32'h0000_FFF0, "ORI");

        //------------------------------------------------------------------
        // I-type: SLLI  (shift left logical immediate)
        //------------------------------------------------------------------
        rs1        = 32'h0000_0003;
        imm_out    = 32'd4;        // shamt in [4:0]
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b001;       // SLLI
        funct7     = 7'b0000000;
        CHECK(32'h0000_0030, "SLLI");

        //------------------------------------------------------------------
        // I-type: SRLI  (logical right shift imm)
        //------------------------------------------------------------------
        rs1        = 32'h0000_0080;
        imm_out    = 32'd3;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b101;       // SRLI/SRAI
        funct7     = 7'b0000000;   // SRLI
        CHECK(32'h0000_0010, "SRLI");

        //------------------------------------------------------------------
        // I-type: SRAI  (arithmetic right shift imm)
        //------------------------------------------------------------------
        rs1        = 32'hFFFF_FF80;  // -128
        imm_out    = 32'd3;
        ex_alu_src = 1'b1;
        opcode     = 7'b0010011;
        funct3     = 3'b101;         // SRLI/SRAI
        funct7     = 7'b0100000;     // SRAI
        CHECK(32'hFFFF_FFF0, "SRAI");

        //------------------------------------------------------------------
        // EXTRA: check ZERO flag explicitly (result = 0)
        //------------------------------------------------------------------
        rs1        = 32'd25;
        rs2        = 32'd25;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB
        #1;
        $display("CHECK ZERO  | result=%0d Z=%b N=%b C=%b O=%b",
                 alu_result, zero_flag, negative_flag, carry_flag, overflow_flag);

        //------------------------------------------------------------------
        // EXTRA: generate a negative result to see N flag
        //------------------------------------------------------------------
        rs1        = 32'd5;
        rs2        = 32'd20;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0100000;   // SUB -> 5-20 = -15
        #1;
        $display("CHECK NEG   | result=%0d (0x%08h) Z=%b N=%b C=%b O=%b",
                 alu_result, alu_result, zero_flag, negative_flag,
                 carry_flag, overflow_flag);

        //------------------------------------------------------------------
        // EXTRA: big signed ADD to observe overflow_flag (depends on your logic)
        //------------------------------------------------------------------
        rs1        = 32'h7FFF_FFFF;   // largest +ve
        rs2        = 32'd1;
        ex_alu_src = 1'b0;
        opcode     = 7'b0110011;
        funct3     = 3'b000;
        funct7     = 7'b0000000;      // ADD
        #1;
        $display("CHECK OVF   | result=%0d (0x%08h) Z=%b N=%b C=%b O=%b",
                 alu_result, alu_result, zero_flag, negative_flag,
                 carry_flag, overflow_flag);

        $display("=========== EXECUTE_TOP TEST END ===========");
        $finish;
    end

endmodule

