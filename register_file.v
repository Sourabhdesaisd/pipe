/*module register_file (
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


endmodule*/


module register_file (
    input  wire        clk,

    // write port (from WB stage)
    input  wire        wr_en,
    input  wire [4:0]  wr_addr,
    input  wire [31:0] wr_data,

    // read addresses (from ID stage)
    input  wire [4:0]  rs1_addr,
    input  wire [4:0]  rs2_addr,

    // read data outputs
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data
);
    reg [31:0] reg_file [0:31];

    initial begin
        // Optional: initialize registers from a file if present
        // $readmemh("reg_mem.hex", reg_file);
        integer i;
        for (i = 0; i < 32; i = i + 1) reg_file[i] = 32'h00000000;
    end

    // Forwarding behavior: if a write happens to the same reg in the same cycle,
    // provide the write data to reads (combinational forwarding).
    // This simple scheme assumes write happens on posedge and reads are combinational.
    wire [31:0] rs1_comb = reg_file[rs1_addr];
    wire [31:0] rs2_comb = reg_file[rs2_addr];

    assign rs1_data = (rs1_addr == 5'd0) ? 32'h0 :
                      ((wr_en && (wr_addr == rs1_addr)) ? wr_data : rs1_comb);

    assign rs2_data = (rs2_addr == 5'd0) ? 32'h0 :
                      ((wr_en && (wr_addr == rs2_addr)) ? wr_data : rs2_comb);

    // Write operation (synchronous)
    always @(posedge clk) begin
        if (wr_en && (wr_addr != 5'd0)) begin
            reg_file[wr_addr] <= wr_data;
        end
    end

endmodule

