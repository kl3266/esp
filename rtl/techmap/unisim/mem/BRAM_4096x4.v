`timescale 1 ps / 1 ps
// Copyright (c) 2014-2021, Columbia University
module BRAM_4096x4 (
    CLK0,
    A0,
    D0,
    Q0,
    WE0,
    WEM0,
    CE0,
    CLK1,
    A1,
    D1,
    Q1,
    WE1,
    WEM1,
    CE1
);
    input CLK0;
    input [11:0] A0;
    input [3:0] D0;
    output [3:0] Q0;
    input WE0;
    input [3:0] WEM0;
    input CE0;
    input CLK1;
    input [11:0] A1;
    input [3:0] D1;
    output [3:0] Q1;
    input WE1;
    input [3:0] WEM1;
    input CE1;

    reg        CE0_tmp;
    reg        CE1_tmp;
    reg [11:0] A0_tmp;
    reg [11:0] A1_tmp;
    reg        WE0_tmp;
    reg        WE1_tmp;
    reg [ 3:0] D0_tmp;
    reg [ 3:0] D1_tmp;

    always @(*) begin
        #5 A0_tmp = A0;
        A1_tmp  = A1;
        CE0_tmp = CE0;
        CE1_tmp = CE1;
        WE0_tmp = WE0;
        WE1_tmp = WE1;
        D0_tmp  = D0;
        D1_tmp  = D1;
    end

    RAMB16_S4_S4 bram (
        .DOA  (Q0),
        .DOB  (Q1),
        .ADDRA(A0_tmp),
        .ADDRB(A1_tmp),
        .CLKA (CLK0),
        .CLKB (CLK1),
        .DIA  (D0_tmp),
        .DIB  (D1_tmp),
        .ENA  (CE0_tmp),
        .ENB  (CE1_tmp),
        .SSRA (1'b0),
        .SSRB (1'b0),
        .WEA  (WE0_tmp),
        .WEB  (WE1_tmp)
    );
endmodule
