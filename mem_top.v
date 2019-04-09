// Top-level data memory unit connecting store, memory and load units
module data_memory_unit (
  input  wire        clk,         // clock for memory writes
  input  wire        mem_read,    // memory read enable (from control)
  input  wire        mem_write,   // memory write enable (from control)
  input  wire [1:0]  store_type,  // 00=SB,01=SH,10=SW (from control)
  input  wire [2:0]  load_type,   // 000=LB,001=LH,010=LW,011=LBU,100=LHU
  input  wire [31:0] alu_result,  // effective address (from ALU)
  input  wire [31:0] rs2,         // data to store (from register file)
  input  wire        memtoreg,  // choose memory result or ALU for WB
  output wire [31:0] wb_data      // data to write back to register file
);

  // Internal wires between submodules
  wire [3:0]  byte_enable;
  wire [31:0] mem_write_data;   // aligned write word prepared by store unit
  wire [31:0] mem_rdata;        // raw 32-bit word read from memory
  wire [31:0] load_result;      // final load result after extract & extend

  // ---------------------------
  // STORE submodule
  // ---------------------------
  // store_datapath: prepares mem_write_data and byte_enable from rs2 & alu_result
  store_datapath u_store (
    .store_type    (store_type),
    .write_data    (rs2),
    .addr          (alu_result),
    .mem_write_data(mem_write_data),
    .byte_enable   (byte_enable)
  );

  // ---------------------------
  // MEMORY submodule
  // ---------------------------
  // memory_1kb: accepts byte-enable & aligned write-data, returns 32-bit word
  memory_1kb u_mem (
    .clk        (clk),
    .mem_read   (mem_read),
    .mem_write  (mem_write),
    .addr       (alu_result),
    .write_data (mem_write_data),
    .byte_enable(byte_enable),
    .read_data  (mem_rdata)
  );

  // ---------------------------
  // LOAD submodule
  // ---------------------------
  // load_datapath: extracts requested byte/half/word and sign/zero extends
  load_datapath u_load (
    .load_type  (load_type),
    .mem_data_in(mem_rdata),
    .addr       (alu_result),
    .read_data  (load_result)
  );

  // ---------------------------
  // WRITEBACK SELECT
  // ---------------------------
  // If memtoreg asserted, WB gets value from load unit; otherwise ALU result
  assign wb_data = memtoreg ? load_result : alu_result;

endmodule



//`timescale 1ns/1ps

module tb_data_memory_unit;

  reg         clk;
  reg         mem_read;
  reg         mem_write;
  reg  [1:0]  store_type;
  reg  [2:0]  load_type;
  reg  [31:0] alu_result;
  reg  [31:0] rs2;
  reg         memtoreg;
  wire [31:0] wb_data;

  data_memory_unit dut (
    .clk        (clk),
    .mem_read   (mem_read),
    .mem_write  (mem_write),
    .store_type (store_type),
    .load_type  (load_type),
    .alu_result (alu_result),
    .rs2        (rs2),
    .memtoreg (memtoreg),
    .wb_data    (wb_data)
  );

initial begin
$shm_open("wave.shm");
$shm_probe("ACTMF");
end

  // Clock generation (10 ns)
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  task check_equal(input [31:0] expected, input [31:0] actual, input [8*64:1] testname);
    begin
      if (expected === actual)
        $display("%0t: PASS - %s : expected=0x%08h actual=0x%08h", $time, testname, expected, actual);
      else
        $display("%0t: FAIL - %s : expected=0x%08h actual=0x%08h", $time, testname, expected, actual);
    end
  endtask


  initial begin
    $dumpfile("tb_data_memory_unit.vcd");
    $dumpvars(0, tb_data_memory_unit);

    mem_read = 0;
    mem_write = 0;
    memtoreg = 0;
    load_type = 3'b010;
    store_type = 0;
    alu_result = 0;
    rs2 = 0;

    #20;

    // -----------------------------
    // TEST 1 — SW then LW
    // -----------------------------
    $display("\n----- TEST1: SW -> LW -----");

    alu_result = 32'h4;
    rs2 = 32'hAABBCCDD;
    store_type = 2'b10;

    // WRITE CYCLE
    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // READ CYCLE (full cycle)
    mem_read = 1;
    memtoreg = 1;
    load_type = 3'b010; // LW
    @(posedge clk); @(negedge clk);
    check_equal(32'hAABBCCDD, wb_data, "SW->LW");
    mem_read = 0; memtoreg = 0;


    // -----------------------------
    // TEST 2 — SB then LB/LBU
    // -----------------------------
    $display("\n----- TEST2: SB -> LB/LBU -----");

    alu_result = 32'h8;
    rs2 = 32'h9A;
    store_type = 2'b00;

    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // LB (sign extend)
    load_type = 3'b000;
    mem_read = 1; memtoreg = 1;
    @(posedge clk); @(negedge clk);
    check_equal(32'hFFFFFF9A, wb_data, "SB->LB");
    mem_read = 0;

    // LBU (zero extend)
    load_type = 3'b011;
    mem_read = 1; memtoreg = 1;
    @(posedge clk); @(negedge clk);
    check_equal(32'h0000009A, wb_data, "SB->LBU");
    mem_read = 0; memtoreg = 0;


    // -----------------------------
    // TEST 3 — SH then LH/LHU
    // -----------------------------
    $display("\n----- TEST3: SH -> LH/LHU -----");

    alu_result = 32'hC;
    rs2 = 32'h8001;
    store_type = 2'b01;

    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // LH
    load_type = 3'b001; mem_read = 1; memtoreg = 1;
    @(posedge clk); @(negedge clk);
    check_equal(32'hFFFF8001, wb_data, "SH->LH");
    mem_read = 0;

    // LHU
    load_type = 3'b100; mem_read = 1; memtoreg = 1;
    @(posedge clk); @(negedge clk);
    check_equal(32'h00008001, wb_data, "SH->LHU");
    mem_read = 0; memtoreg = 0;


    // -----------------------------
    // TEST 4 — Mixed stores inside same word
    // -----------------------------
    $display("\n----- TEST4: Mixed stores (byte+half) -----");

    // Clear word
    alu_result = 32'h10;
    rs2 = 0;
    store_type = 2'b10;
    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // SB at 0x10
    alu_result = 32'h10;
    rs2 = 8'h11;
    store_type = 2'b00;
    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // SH at 0x12
    alu_result = 32'h12;
    rs2 = 16'hA5A6;
    store_type = 2'b01;
    mem_write = 1;
    @(posedge clk); @(negedge clk);
    mem_write = 0;

    // read final word
    load_type = 3'b010;   // LW
    memtoreg = 1;
    mem_read = 1;
    @(posedge clk); @(negedge clk);
    check_equal(32'hA5A60011, wb_data, "Mixed writes word");
    mem_read = 0; memtoreg = 0;

    #20;
    $display("All tests done.");
    $finish;
  end

endmodule

