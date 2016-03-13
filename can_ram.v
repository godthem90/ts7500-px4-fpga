/* Verilog netlist generated by SCUBA Diamond_1.1_Production (517) */
/* Module Version: 6.1 */
/* C:\lscc\diamond\1.1\ispfpga\bin\nt\scuba.exe -w -lang verilog -synth synplify -bus_exp 7 -bb -arch mg5a00 -type bram -wp 10 -rp 0011 -rdata_width 8 -data_width 8 -num_rows 2048 -resetmode SYNC -cascade 11 -e  */
/* Thu Dec 23 15:34:35 2010 */


`timescale 1 ns / 1 ps
module can_ram (WrAddress, RdAddress, Data, WE, RdClock, RdClockEn, 
    Reset, WrClock, WrClockEn, Q);
    input wire [10:0] WrAddress;
    input wire [10:0] RdAddress;
    input wire [7:0] Data;
    input wire WE;
    input wire RdClock;
    input wire RdClockEn;
    input wire Reset;
    input wire WrClock;
    input wire WrClockEn;
    output wire [7:0] Q;

    wire scuba_vhi;
    wire scuba_vlo;

    VHI scuba_vhi_inst (.Z(scuba_vhi));

    VLO scuba_vlo_inst (.Z(scuba_vlo));

    // synopsys translate_off
    defparam can_ram_0_0_0_0.CSDECODE_B =  3'b000 ;
    defparam can_ram_0_0_0_0.CSDECODE_A =  3'b000 ;
    defparam can_ram_0_0_0_0.WRITEMODE_B = "NORMAL" ;
    defparam can_ram_0_0_0_0.WRITEMODE_A = "NORMAL" ;
    defparam can_ram_0_0_0_0.GSR = "DISABLED" ;
    defparam can_ram_0_0_0_0.RESETMODE = "SYNC" ;
    defparam can_ram_0_0_0_0.REGMODE_B = "NOREG" ;
    defparam can_ram_0_0_0_0.REGMODE_A = "NOREG" ;
    defparam can_ram_0_0_0_0.DATA_WIDTH_B = 9 ;
    defparam can_ram_0_0_0_0.DATA_WIDTH_A = 9 ;
    // synopsys translate_on
    DP16KB can_ram_0_0_0_0 (.DIA0(Data[0]), .DIA1(Data[1]), .DIA2(Data[2]), 
        .DIA3(Data[3]), .DIA4(Data[4]), .DIA5(Data[5]), .DIA6(Data[6]), 
        .DIA7(Data[7]), .DIA8(scuba_vlo), .DIA9(scuba_vlo), .DIA10(scuba_vlo), 
        .DIA11(scuba_vlo), .DIA12(scuba_vlo), .DIA13(scuba_vlo), .DIA14(scuba_vlo), 
        .DIA15(scuba_vlo), .DIA16(scuba_vlo), .DIA17(scuba_vlo), .ADA0(scuba_vlo), 
        .ADA1(scuba_vlo), .ADA2(scuba_vlo), .ADA3(WrAddress[0]), .ADA4(WrAddress[1]), 
        .ADA5(WrAddress[2]), .ADA6(WrAddress[3]), .ADA7(WrAddress[4]), .ADA8(WrAddress[5]), 
        .ADA9(WrAddress[6]), .ADA10(WrAddress[7]), .ADA11(WrAddress[8]), 
        .ADA12(WrAddress[9]), .ADA13(WrAddress[10]), .CEA(WrClockEn), .CLKA(WrClock), 
        .WEA(WE), .CSA0(scuba_vlo), .CSA1(scuba_vlo), .CSA2(scuba_vlo), 
        .RSTA(Reset), .DIB0(scuba_vlo), .DIB1(scuba_vlo), .DIB2(scuba_vlo), 
        .DIB3(scuba_vlo), .DIB4(scuba_vlo), .DIB5(scuba_vlo), .DIB6(scuba_vlo), 
        .DIB7(scuba_vlo), .DIB8(scuba_vlo), .DIB9(scuba_vlo), .DIB10(scuba_vlo), 
        .DIB11(scuba_vlo), .DIB12(scuba_vlo), .DIB13(scuba_vlo), .DIB14(scuba_vlo), 
        .DIB15(scuba_vlo), .DIB16(scuba_vlo), .DIB17(scuba_vlo), .ADB0(scuba_vlo), 
        .ADB1(scuba_vlo), .ADB2(scuba_vlo), .ADB3(RdAddress[0]), .ADB4(RdAddress[1]), 
        .ADB5(RdAddress[2]), .ADB6(RdAddress[3]), .ADB7(RdAddress[4]), .ADB8(RdAddress[5]), 
        .ADB9(RdAddress[6]), .ADB10(RdAddress[7]), .ADB11(RdAddress[8]), 
        .ADB12(RdAddress[9]), .ADB13(RdAddress[10]), .CEB(RdClockEn), .CLKB(RdClock), 
        .WEB(scuba_vlo), .CSB0(scuba_vlo), .CSB1(scuba_vlo), .CSB2(scuba_vlo), 
        .RSTB(Reset), .DOA0(), .DOA1(), .DOA2(), .DOA3(), .DOA4(), .DOA5(), 
        .DOA6(), .DOA7(), .DOA8(), .DOA9(), .DOA10(), .DOA11(), .DOA12(), 
        .DOA13(), .DOA14(), .DOA15(), .DOA16(), .DOA17(), .DOB0(Q[0]), .DOB1(Q[1]), 
        .DOB2(Q[2]), .DOB3(Q[3]), .DOB4(Q[4]), .DOB5(Q[5]), .DOB6(Q[6]), 
        .DOB7(Q[7]), .DOB8(), .DOB9(), .DOB10(), .DOB11(), .DOB12(), .DOB13(), 
        .DOB14(), .DOB15(), .DOB16(), .DOB17())
             /* synthesis MEM_LPC_FILE="can_ram.lpc" */
             /* synthesis MEM_INIT_FILE="" */
             /* synthesis CSDECODE_B="0b000" */
             /* synthesis CSDECODE_A="0b000" */
             /* synthesis WRITEMODE_B="NORMAL" */
             /* synthesis WRITEMODE_A="NORMAL" */
             /* synthesis GSR="DISABLED" */
             /* synthesis RESETMODE="SYNC" */
             /* synthesis REGMODE_B="NOREG" */
             /* synthesis REGMODE_A="NOREG" */
             /* synthesis DATA_WIDTH_B="9" */
             /* synthesis DATA_WIDTH_A="9" */;



    // exemplar begin
    // exemplar attribute can_ram_0_0_0_0 MEM_LPC_FILE can_ram.lpc
    // exemplar attribute can_ram_0_0_0_0 MEM_INIT_FILE 
    // exemplar attribute can_ram_0_0_0_0 CSDECODE_B 0b000
    // exemplar attribute can_ram_0_0_0_0 CSDECODE_A 0b000
    // exemplar attribute can_ram_0_0_0_0 WRITEMODE_B NORMAL
    // exemplar attribute can_ram_0_0_0_0 WRITEMODE_A NORMAL
    // exemplar attribute can_ram_0_0_0_0 GSR DISABLED
    // exemplar attribute can_ram_0_0_0_0 RESETMODE SYNC
    // exemplar attribute can_ram_0_0_0_0 REGMODE_B NOREG
    // exemplar attribute can_ram_0_0_0_0 REGMODE_A NOREG
    // exemplar attribute can_ram_0_0_0_0 DATA_WIDTH_B 9
    // exemplar attribute can_ram_0_0_0_0 DATA_WIDTH_A 9
    // exemplar end

endmodule
