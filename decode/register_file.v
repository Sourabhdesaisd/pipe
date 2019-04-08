module register_file (
    input clk,

    input wr_en,
    input [4:0] wr_addr,
    input [31:0] wr_data,

    input [4:0] rs1_addr,
    input [4:0] rs2_addr,

    output wire [31:0] rs1,
    output wire [31:0] rs2
);
    reg [31:0] reg_file [0:31];

    wire [31:0] rs1_forwarded;
    wire [31:0] rs2_forwarded;
    initial begin
        $readmemh("reg_mem.hex", reg_file);
    end

    // Operand forwarding if read and write addr are same
    assign rs1_forwarded = ((rs1_addr == wr_addr) && wr_en) ? wr_data : reg_file[rs1_addr];
    assign rs2_forwarded = ((rs2_addr == wr_addr) && wr_en) ? wr_data : reg_file[rs2_addr];

    // x0 is hardwired to 0
    assign rs1 = (rs1_addr == 0) ? 32'b0 : rs1_forwarded;
    assign rs2 = (rs2_addr == 0) ? 32'b0 : rs2_forwarded;

    // Write operation
    always @(posedge clk) begin
        if (wr_en && wr_addr != 0)
            reg_file[wr_addr] <= wr_data;
    end


endmodule
