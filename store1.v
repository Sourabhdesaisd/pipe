// store_datapath.v  (modified ports & store_type encoding)
//
// Ports changed per request:
//  - store_type [1:0] : 00=SB, 01=SH, 10=SW
//  - write_data [31:0] replaces rs2_data

module store_datapath (
    input  wire [1:0]   store_type,     // 00=SB, 01=SH, 10=SW
    input  wire [31:0]  write_data,     // data to be stored (renamed from rs2_data)
    input  wire [31:0]  addr,           // effective address from ALU
    output reg  [31:0]  mem_write_data, // data aligned for memory
    output reg  [3:0]   byte_enable     // active byte lanes
);

    always @(*) begin
        // Default values
        mem_write_data = 32'b0;
        byte_enable    = 4'b0000;

        case (store_type)
            // ---------------- SB ----------------
            2'b00: begin
                case (addr[1:0])
                    2'b00: begin mem_write_data = {24'b0, write_data[7:0]};  byte_enable = 4'b0001; end
                    2'b01: begin mem_write_data = {16'b0, write_data[7:0], 8'b0}; byte_enable = 4'b0010; end
                    2'b10: begin mem_write_data = {8'b0,  write_data[7:0], 16'b0}; byte_enable = 4'b0100; end
                    2'b11: begin mem_write_data = {write_data[7:0], 24'b0};      byte_enable = 4'b1000; end
                endcase
            end

            // ---------------- SH ----------------
            2'b01: begin
                case (addr[1])
                    1'b0: begin mem_write_data = {16'b0, write_data[15:0]}; byte_enable = 4'b0011; end
                    1'b1: begin mem_write_data = {write_data[15:0], 16'b0}; byte_enable = 4'b1100; end
                endcase
            end

            // ---------------- SW ----------------
            2'b10: begin
                mem_write_data = write_data;
                byte_enable    = 4'b1111;
            end

            default: begin
                // Safe default: no write
                mem_write_data = 32'b0;
                byte_enable    = 4'b0000;
            end
        endcase
    end

endmodule


/*
//`timescale 1ns/1ps

module tb_store_datapath_simple;

    reg  [1:0]  store_type;   // 00=SB, 01=SH, 10=SW
    reg  [31:0] write_data;
    reg  [31:0] addr;

    wire [31:0] mem_write_data;
    wire [3:0]  byte_enable;

    // DUT
    store_datapath dut (
        .store_type(store_type),
        .write_data(write_data),
        .addr(addr),
        .mem_write_data(mem_write_data),
        .byte_enable(byte_enable)
    );
initial begin
$shm_open("wave.shm");
$shm_probe("ACTMF");
end


    initial begin
        $display("\n===== SIMPLE STORE DATAPATH TEST =====\n");

        // -------------------------------------------------
        // Test SB (Store Byte)
        // -------------------------------------------------
        $display("---- SB TEST ----");
        store_type = 2'b00;    
        write_data = 32'h000000AA;

        addr = 32'h00000000; #1;
        $display("SB addr=0  -> data=%h enable=%b", mem_write_data, byte_enable);

        addr = 32'h00000001; #1;
        $display("SB addr=1  -> data=%h enable=%b", mem_write_data, byte_enable);

        addr = 32'h00000002; #1;
        $display("SB addr=2  -> data=%h enable=%b", mem_write_data, byte_enable);

        addr = 32'h00000003; #1;
        $display("SB addr=3  -> data=%h enable=%b", mem_write_data, byte_enable);


        // -------------------------------------------------
        // Test SH (Store Halfword)
        // -------------------------------------------------
        $display("\n---- SH TEST ----");
        store_type = 2'b01;    
        write_data = 32'h0000BEEF;

        addr = 32'h00000000; #1;   // lower half
        $display("SH addr=0  -> data=%h enable=%b", mem_write_data, byte_enable);

        addr = 32'h00000002; #1;   // upper half
        $display("SH addr=2  -> data=%h enable=%b", mem_write_data, byte_enable);


        // -------------------------------------------------
        // Test SW (Store Word)
        // -------------------------------------------------
        $display("\n---- SW TEST ----");
        store_type = 2'b10;    
        write_data = 32'hDEADBEEF;

        addr = 32'h00000000; #1;
        $display("SW addr=0  -> data=%h enable=%b", mem_write_data, byte_enable);


        $display("\n===== TEST FINISHED =====\n");
        $finish;
    end

endmodule
*/

