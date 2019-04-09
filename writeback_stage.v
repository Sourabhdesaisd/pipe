// writeback_mux.v
// Selects writeback data for the register file.
// Priority: PC (if pc_to_reg_wb) > Mem (if memtoreg_wb) > ALU
module writeback_mux (
    input  wire [31:0] alu_result_wb,     // ALU result captured in MEM/WB
    input  wire [31:0] mem_load_data_wb,  // Load result captured in MEM/WB
    input  wire [31:0] pc_plus4_wb,       // PC+4 value to write for JAL/JALR (if used)
    input  wire        memtoreg_wb,       // 1 -> select memory load
    input  wire        pc_to_reg_wb,      // 1 -> select PC+4 (JAL/JALR). Takes priority over memtoreg
    output reg  [31:0] wr_data            // output -> regfile.wr_data
);

    always @(*) begin
        if (pc_to_reg_wb) begin
            // PC writes have top priority (JAL / JALR)
            wr_data = pc_plus4_wb;
        end
        else if (memtoreg_wb) begin
            // Loads write data from memory
            wr_data = mem_load_data_wb;
        end
        else begin
            // Default: ALU result
            wr_data = alu_result_wb;
        end
    end

endmodule

