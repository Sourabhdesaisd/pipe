module load_datapath (
    input  wire [2:0]   load_type,    // 000=LB, 001=LH, 010=LW, 011=LBU, 100=LHU
    input  wire [31:0]  mem_data_in,  // 32-bit data from memory
    input  wire [31:0]  addr,         // byte address from ALU
    output wire [31:0]  read_data     // final data to register file
);

    wire [31:0] MDR = mem_data_in;

    // -------------------------------
    // Byte selection (addr[1:0])
    // -------------------------------
    wire [7:0] selected_byte =
        (addr[1:0] == 2'b00) ? MDR[7:0]   :
        (addr[1:0] == 2'b01) ? MDR[15:8]  :
        (addr[1:0] == 2'b10) ? MDR[23:16] :
                               MDR[31:24];

    // -------------------------------
    // Halfword selection (addr[1])
    // -------------------------------
    wire [15:0] selected_half =
        (addr[1]) ? MDR[31:16] : MDR[15:0];

    // -------------------------------
    // Byte extension
    // -------------------------------
    wire [31:0] ext_byte =
        (load_type == 3'b000) ? {{24{selected_byte[7]}}, selected_byte} : // LB (sign extend)
        (load_type == 3'b011) ? {24'b0, selected_byte}                  : // LBU (zero extend)
                                32'b0;

    // -------------------------------
    // Halfword extension
    // -------------------------------
    wire [31:0] ext_half =
        (load_type == 3'b001) ? {{16{selected_half[15]}}, selected_half} : // LH (sign extend)
        (load_type == 3'b100) ? {16'b0, selected_half}                  : // LHU (zero extend)
                                32'b0;

    // -------------------------------
    // Final output based on load_type
    // -------------------------------
    assign read_data =
        (load_type == 3'b010) ? MDR :        // LW
        (load_type == 3'b000 || load_type == 3'b011) ? ext_byte :
        (load_type == 3'b001 || load_type == 3'b100) ? ext_half :
        32'b0;

endmodule



/*
//`timescale 1ns/1ps

module tb_load_datapath_simple;

    reg  [2:0]  load_type;     // LB/LH/LW/LBU/LHU
    reg  [31:0] mem_data_in;   // data from memory
    reg  [31:0] addr;          // byte address
    wire [31:0] read_data;     // output of load datapath

    // Instantiate DUT
    load_datapath DUT (
        .load_type(load_type),
        .mem_data_in(mem_data_in),
        .addr(addr),
        .read_data(read_data)
    );

    initial begin
        // Open waveform file
        $shm_open("wave.shm");
	$shm_probe("ACTMF");
end

initial begin

        $display("\n===== SIMPLE LOAD DATAPATH TEST =====\n");

        //-----------------------------
        // MEMORY WORD FOR TEST
        //-----------------------------
        mem_data_in = 32'hA1B2C3D4;  
        // Byte positions:
        // addr=0 -> D4
        // addr=1 -> C3
        // addr=2 -> B2
        // addr=3 -> A1

        //-----------------------------
        // Test LB (sign extend)
        //-----------------------------
        $display("---- LB TEST ----");
        load_type = 3'b000;

        addr = 0; #1; $display("LB addr=0 -> %h", read_data);
        addr = 1; #1; $display("LB addr=1 -> %h", read_data);
        addr = 2; #1; $display("LB addr=2 -> %h", read_data);
        addr = 3; #1; $display("LB addr=3 -> %h", read_data);

        //-----------------------------
        // Test LBU (zero extend)
        //-----------------------------
        $display("\n---- LBU TEST ----");
        load_type = 3'b011;

        addr = 0; #1; $display("LBU addr=0 -> %h", read_data);
        addr = 1; #1; $display("LBU addr=1 -> %h", read_data);
        addr = 2; #1; $display("LBU addr=2 -> %h", read_data);
        addr = 3; #1; $display("LBU addr=3 -> %h", read_data);

        //-----------------------------
        // Test LH (sign extend)
        //-----------------------------
        $display("\n---- LH TEST ----");
        load_type = 3'b001;

        addr = 0; #1; $display("LH addr=0 -> %h", read_data); // lower half
        addr = 2; #1; $display("LH addr=2 -> %h", read_data); // upper half

        //-----------------------------
        // Test LHU (zero extend)
        //-----------------------------
        $display("\n---- LHU TEST ----");
        load_type = 3'b100;

        addr = 0; #1; $display("LHU addr=0 -> %h", read_data);
        addr = 2; #1; $display("LHU addr=2 -> %h", read_data);

        //-----------------------------
        // Test LW (full word)
        //-----------------------------
        $display("\n---- LW TEST ----");
        load_type = 3'b010;

        addr = 0; #1; $display("LW addr=0 -> %h", read_data);

        $display("\n===== TEST FINISHED =====\n");
        $finish;
    end

endmodule
*/

