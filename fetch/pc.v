module pc(
    input clk,
    input rst,
    input [31:0] next_pc,
    input pc_en,
    output reg [31:0] pc
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 32'h00000000;
        end else if (pc_en) begin
            pc <= next_pc;
        end
    end

endmodule


