// mem_wb_reg.v  (updated to carry pc_plus4_in and pc_to_reg_in)
/*module mem_wb_reg (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        flush,

    // MEM stage inputs
    input  wire [31:0] alu_result_for_wb,
    input  wire [31:0] load_wb_data,
    input  wire [4:0]  rd_for_wb,
    input  wire        wb_reg_file_in,
    input  wire        memtoreg_in,

    // NEW: pc+4 + pc_to_reg inputs for JAL/JALR writeback
    input  wire [31:0] pc_plus4_in,
    input  wire        pc_to_reg_in,

    // outputs to WB stage
    output reg  [31:0] alu_result_wb,
    output reg  [31:0] mem_load_data_wb,
    output reg  [4:0]  rd_wb,
    output reg         wb_reg_file_wb,
    output reg         memtoreg_wb,

    // NEW outputs
    output reg  [31:0] pc_plus4_wb,
    output reg         pc_to_reg_wb
);

    localparam [31:0] ZERO32 = 32'h00000000;
    localparam [4:0]  ZERO5  = 5'h00;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_wb    <= ZERO32;
            mem_load_data_wb <= ZERO32;
            rd_wb            <= ZERO5;
            wb_reg_file_wb   <= 1'b0;
            memtoreg_wb      <= 1'b0;
            pc_plus4_wb      <= ZERO32;
            pc_to_reg_wb     <= 1'b0;
        end else if (!en) begin
            // hold
        end else begin
            if (flush) begin
                alu_result_wb    <= ZERO32;
                mem_load_data_wb <= ZERO32;
                rd_wb            <= ZERO5;
                wb_reg_file_wb   <= 1'b0;
                memtoreg_wb      <= 1'b0;
                pc_plus4_wb      <= ZERO32;
                pc_to_reg_wb     <= 1'b0;
            end else begin
                alu_result_wb    <= alu_result_for_wb;
                mem_load_data_wb <= load_wb_data;
                rd_wb            <= rd_for_wb;
                wb_reg_file_wb   <= wb_reg_file_in;
                memtoreg_wb      <= memtoreg_in;
                pc_plus4_wb      <= pc_plus4_in;
                pc_to_reg_wb     <= pc_to_reg_in;
            end
        end
    end

endmodule
*/


// -------------------------------------------------
// mem_wb_reg
// Captures MEM stage outputs for writeback and forwarding.
// -------------------------------------------------
module mem_wb_reg (
    input  wire         clk,
    input  wire         rst,
    input  wire         en,
    input  wire         flush,

    // inputs from MEM stage
    input  wire [31:0]  alu_result_for_wb,
    input  wire [31:0]  load_wb_data,
    input  wire [4:0]   rd_for_wb,
    input  wire         wb_reg_file_in,
    input  wire         memtoreg_in,

    // optional: incoming pc+4 and pc_to_reg (for jal/jalr)
    input  wire [31:0]  pc_plus4_in,
    input  wire         pc_to_reg_in,

    // outputs to WB stage
    output reg  [31:0]  alu_result_wb,
    output reg  [31:0]  mem_load_data_wb,
    output reg  [4:0]   rd_wb,
    output reg          wb_reg_file_wb,
    output reg          memtoreg_wb,

    // outputs for forwarding (final write enable / rd)
    output reg  [31:0]  pc_plus4_wb,
    output reg          pc_to_reg_wb
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_wb   <= 32'b0;
            mem_load_data_wb<= 32'b0;
            rd_wb           <= 5'b0;
            wb_reg_file_wb  <= 1'b0;
            memtoreg_wb     <= 1'b0;
            pc_plus4_wb     <= 32'b0;
            pc_to_reg_wb    <= 1'b0;
        end
        else if (en) begin
            if (flush) begin
                alu_result_wb   <= 32'b0;
                mem_load_data_wb<= 32'b0;
                rd_wb           <= 5'b0;
                wb_reg_file_wb  <= 1'b0;
                memtoreg_wb     <= 1'b0;
                pc_plus4_wb     <= 32'b0;
                pc_to_reg_wb    <= 1'b0;
            end
            else begin
                alu_result_wb   <= alu_result_for_wb;
                mem_load_data_wb<= load_wb_data;
                rd_wb           <= rd_for_wb;
                wb_reg_file_wb  <= wb_reg_file_in;
                memtoreg_wb     <= memtoreg_in;
                pc_plus4_wb     <= pc_plus4_in;
                pc_to_reg_wb    <= pc_to_reg_in;
            end
        end
    end
endmodule


