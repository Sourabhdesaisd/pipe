module inst_mem(
    input [31:0] pc,
    input read_en,
    output [31:0] instruction
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("instructions.hex", mem);
    end

    assign instruction = mem[pc[11:2]]; 
endmodule

