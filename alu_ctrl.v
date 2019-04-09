

//-------------------------------------------------
// RV32I ALU Control Unit
//-------------------------------------------------
module alu_control (
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg [3:0] alu_ctrl
);

    always @(*) begin
        case (opcode)

            // R-type
            7'b0110011: begin
                case (funct3)
                    3'b000: alu_ctrl = (funct7[5]) ? 4'b0001 : 4'b0000; // SUB / ADD
                    3'b111: alu_ctrl = 4'b0010; // AND
                    3'b110: alu_ctrl = 4'b0011; // OR
                    3'b100: alu_ctrl = 4'b0100; // XOR
                    3'b001: alu_ctrl = 4'b0101; // SLL
                    3'b101: alu_ctrl = (funct7[5]) ? 4'b0111 : 4'b0110; // SRA / SRL
                    3'b010: alu_ctrl = 4'b1000; // SLT
                    3'b011: alu_ctrl = 4'b1001; // SLTU
                    default: alu_ctrl = 4'b1111;
                endcase
            end

            // I-type ALU immediates
            7'b0010011: begin
                case (funct3)
                    3'b000: alu_ctrl = 4'b0000; // ADDI
                    3'b111: alu_ctrl = 4'b0010; // ANDI
                    3'b110: alu_ctrl = 4'b0011; // ORI
                    3'b100: alu_ctrl = 4'b0100; // XORI
                    3'b001: alu_ctrl = 4'b0101; // SLLI
                    3'b101: alu_ctrl = (funct7[5]) ? 4'b0111 : 4'b0110; // SRAI / SRLI
                    3'b010: alu_ctrl = 4'b1000; // SLTI
                    3'b011: alu_ctrl = 4'b1001; // SLTIU
                    default: alu_ctrl = 4'b1111;
                endcase
            end

            // Load / Store : address add
            7'b0000011, // LOAD
            7'b0100011: // STORE
                alu_ctrl = 4'b0000; // ADD

            // Branch : compare
            7'b1100011:
                alu_ctrl = 4'b0001; // SUB

            // JALR : rs1 + imm
            7'b1100111:
                alu_ctrl = 4'b0000; // ADD

            // LUI
            7'b0110111:
                alu_ctrl = 4'b1010;

            // AUIPC
            7'b0010111:
                alu_ctrl = 4'b1011;

            default:
                alu_ctrl = 4'b1111;
        endcase
    end
endmodule

/*
//-------------------------------------------------
// Testbench for alu_control
//-------------------------------------------------
module tb_alu_control;

    reg  [6:0] opcode;
    reg  [2:0] funct3;
    reg  [6:0] funct7;
    wire [3:0] alu_ctrl;

    integer errors;

    // DUT
    alu_control dut (
        .opcode (opcode),
        .funct3 (funct3),
        .funct7 (funct7),
        .alu_ctrl (alu_ctrl)
    );
    
initial begin
$shm_open("wave.shm");
$shm_probe("ACTMF");
end


    // simple check task
    task check;
        input [6:0] op;
        input [2:0] f3;
        input [6:0] f7;
        input [3:0] exp;
        input [127:0] name;
    begin
        opcode = op;
        funct3 = f3;
        funct7 = f7;
        #1; // allow combinational logic to settle
        if (alu_ctrl !== exp) begin
            $display("FAIL: %0s  opcode=%b funct3=%b funct7=%b  expected=%b got=%b",
                     name, op, f3, f7, exp, alu_ctrl);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s  alu_ctrl=%b", name, alu_ctrl);
        end
    end
    endtask

    initial begin
        errors = 0;

        // ---------------- R-type ----------------
        check(7'b0110011, 3'b000, 7'b0000000, 4'b0000, "ADD");
        check(7'b0110011, 3'b000, 7'b0100000, 4'b0001, "SUB");
        check(7'b0110011, 3'b111, 7'b0000000, 4'b0010, "AND");
        check(7'b0110011, 3'b110, 7'b0000000, 4'b0011, "OR");
        check(7'b0110011, 3'b100, 7'b0000000, 4'b0100, "XOR");
        check(7'b0110011, 3'b001, 7'b0000000, 4'b0101, "SLL");
        check(7'b0110011, 3'b101, 7'b0000000, 4'b0110, "SRL");
        check(7'b0110011, 3'b101, 7'b0100000, 4'b0111, "SRA");
        check(7'b0110011, 3'b010, 7'b0000000, 4'b1000, "SLT");
        check(7'b0110011, 3'b011, 7'b0000000, 4'b1001, "SLTU");

        // ---------------- I-type ----------------
        check(7'b0010011, 3'b000, 7'b0000000, 4'b0000, "ADDI");
        check(7'b0010011, 3'b111, 7'b0000000, 4'b0010, "ANDI");
        check(7'b0010011, 3'b110, 7'b0000000, 4'b0011, "ORI");
        check(7'b0010011, 3'b100, 7'b0000000, 4'b0100, "XORI");
        check(7'b0010011, 3'b001, 7'b0000000, 4'b0101, "SLLI");
        check(7'b0010011, 3'b101, 7'b0000000, 4'b0110, "SRLI");
        check(7'b0010011, 3'b101, 7'b0100000, 4'b0111, "SRAI");
        check(7'b0010011, 3'b010, 7'b0000000, 4'b1000, "SLTI");
        check(7'b0010011, 3'b011, 7'b0000000, 4'b1001, "SLTIU");

        // ------------- Load / Store -------------
        // LW
        check(7'b0000011, 3'b010, 7'b0000000, 4'b0000, "LW address");
        // SW
        check(7'b0100011, 3'b010, 7'b0000000, 4'b0000, "SW address");

        // ------------- Branch (all use SUB) -----
        check(7'b1100011, 3'b000, 7'b0000000, 4'b0001, "BEQ");
        check(7'b1100011, 3'b001, 7'b0000000, 4'b0001, "BNE");
        check(7'b1100011, 3'b100, 7'b0000000, 4'b0001, "BLT");
        check(7'b1100011, 3'b101, 7'b0000000, 4'b0001, "BGE");
        check(7'b1100011, 3'b110, 7'b0000000, 4'b0001, "BLTU");
        check(7'b1100011, 3'b111, 7'b0000000, 4'b0001, "BGEU");

        // ------------- JALR ---------------------
        check(7'b1100111, 3'b000, 7'b0000000, 4'b0000, "JALR address");

        // ------------- LUI / AUIPC --------------
        check(7'b0110111, 3'b000, 7'b0000000, 4'b1010, "LUI");
        check(7'b0010111, 3'b000, 7'b0000000, 4'b1011, "AUIPC");

        // ------------- Default / illegal --------
        check(7'b0000000, 3'b000, 7'b0000000, 4'b1111, "Illegal/default");

        // Summary
        if (errors == 0)
            $display("\n==== ALL ALU_CONTROL TESTS PASSED ====\n");
        else
            $display("\n==== %0d TEST(S) FAILED ====\n", errors);

        $finish;
    end

endmodule*/

