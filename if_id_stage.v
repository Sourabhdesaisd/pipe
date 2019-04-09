module if_id_reg(
    input  wire        clk,
    input  wire        rst,
    input  wire        en,            // enable capture; tie 1 if no stall logic

    // IF-stage inputs (kept original names)
    input  wire [31:0] pc,
    input  wire [31:0] instruction,

    // ID-stage outputs
    output reg  [31:0] pc_id,
    output reg  [31:0] instruction_id
);

    // On reset -> clear to known state. On enable -> capture. Else -> hold.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_id         <= 32'h00000000;
            instruction_id<= 32'h00000013; // NOP (ADDI x0,x0,0)
        end else if (en) begin
            pc_id         <= pc;
            instruction_id<= instruction;
        end
        // if en == 0 -> hold values (stall)
    end

endmodule

