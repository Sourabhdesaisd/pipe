module inst_mem(
   // input clk,
   // input rst,
    input [31:0] pc,
    input read_en,
 //   input write_en,
  //  input flush,
//    input [7:0] write_addr,
 //   input [31:0] write_data,
    output  [31:0] instruction
);
   // reg [31:0] instruction;

    // Memory array to hold instructions
    reg [31:0] mem [0:255]; // 1KB memory

    // Initialize memory using file
    initial begin
        $readmemh("instructions.hex", mem);
    end

   
    assign instruction = mem[pc[11:2]]; 
endmodule
