//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.11.03 Education
//Part Number: GW1NSR-LV4CQN48PC6/I5
//Device: GW1NSR-4C
//Created Time: Sun Oct 11 17:03:51 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_MULTADDALU your_instance_name(
        .dout(dout), //output [32:0] dout
        .caso(caso), //output [54:0] caso
        .a0(a0), //input [15:0] a0
        .b0(b0), //input [15:0] b0
        .a1(a1), //input [15:0] a1
        .b1(b1), //input [15:0] b1
        .ce(ce), //input ce
        .clk(clk), //input clk
        .reset(reset) //input reset
    );

//--------Copy end-------------------
