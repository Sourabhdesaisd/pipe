module ripple_carry_adder32 (
    input  [31:0] a,
    input  [31:0] b,
    input         cin,             // carry-in (0=ADD, 1=SUB)
    output [31:0] sum,
    output        carry_flag,      // CARRY FLAG (unsigned overflow)
    output        zero_flag,       // Result is zero
    output        negative_flag,   // MSB of result
    output        overflow_flag    // Signed overflow flag
);

    reg [32:0] c;      // Carry chain (c[0] to c[32])
    reg [31:0] s;      // Sum bits
    integer i;

    always @(*) begin
        c[0] = cin;    // starting carry

        // RIPPLE CARRY ADDER
        for (i = 0; i < 32; i = i + 1) begin
            // SUM
            s[i]   = a[i] ^ b[i] ^ c[i];

            // CARRY
            c[i+1] = (a[i] & b[i]) |
                     (a[i] & c[i]) |
                     (b[i] & c[i]);
        end
    end

    // Final SUM
    assign sum = s;

    // CARRY FLAG
    assign carry_flag = c[32];

    // ZERO FLAG
    assign zero_flag = (sum == 32'd0);

    // NEGATIVE FLAG
    assign negative_flag = sum[31];

    // SIGNED OVERFLOW FLAG
    assign overflow_flag = (a[31] == b[31]) && (sum[31] != a[31]);

endmodule

