/* Copyright 2009, Unpublished Work of Technologic Systems
 * All Rights Reserved.
 *
 * THIS WORK IS AN UNPUBLISHED WORK AND CONTAINS CONFIDENTIAL,
 * PROPRIETARY AND TRADE SECRET INFORMATION OF TECHNOLOGIC SYSTEMS.
 * ACCESS TO THIS WORK IS RESTRICTED TO (I) TECHNOLOGIC SYSTEMS 
 * EMPLOYEES WHO HAVE A NEED TO KNOW TO PERFORM TASKS WITHIN THE SCOPE
 * OF THEIR ASSIGNMENTS AND (II) ENTITIES OTHER THAN TECHNOLOGIC
 * SYSTEMS WHO HAVE ENTERED INTO APPROPRIATE LICENSE AGREEMENTS.  NO
 * PART OF THIS WORK MAY BE USED, PRACTICED, PERFORMED, COPIED, 
 * DISTRIBUTED, REVISED, MODIFIED, TRANSLATED, ABRIDGED, CONDENSED, 
 * EXPANDED, COLLECTED, COMPILED, LINKED, RECAST, TRANSFORMED, ADAPTED
 * IN ANY FORM OR BY ANY MEANS, MANUAL, MECHANICAL, CHEMICAL, 
 * ELECTRICAL, ELECTRONIC, OPTICAL, BIOLOGICAL, OR OTHERWISE WITHOUT
 * THE PRIOR WRITTEN PERMISSION AND CONSENT OF TECHNOLOGIC SYSTEMS.
 * ANY USE OR EXPLOITATION OF THIS WORK WITHOUT THE PRIOR WRITTEN
 * CONSENT OF TECHNOLOGIC SYSTEMS COULD SUBJECT THE PERPETRATOR TO
 * CRIMINAL AND CIVIL LIABILITY.
 */

/* The SPI SBUS is a module that bridges a SPI protocol into 16-bit
 * wishbone bus cycles.  The SBUS protocol allows for 16 specific
 * 16 bit registers (32-bytes of register space) to be read or written
 * and has provision for the WISHBONE bus cycles to take any amount
 * of time to be ack'ed through the mechanism of SBUS retries.  SPI 
 * bandwidth efficient burst register reads/writes are also possible when
 * writing/reading continuously to the same address.
 *
 * The general protocol consists of a 24-bit SPI transaction framed by the
 * SPI CS#.  Bits are clocked in on the rising edge and the first MOSI (input)
 * bit signifies whether the bus cycle is a read or write.  This is followed
 * by the 4 address bits MSB first. The remaining 19 bits depend on whether
 * the cycle is a read or write.  For reads, the WISHBONE bus cycle
 * is started as soon as the last address bit is clocked in. 
 *
 * The 24-bit SPI SBUS data format - bit 23 (WE) is first on wire:
 *
 * 16-bit WISHBONE READ operation
 *    23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 *    --|--|--|--|--|--|--|--|--|--------------------------------------------|
 * SI: 0|A3|A2|A1|A0|  |  |  | B|<------------should be zero---------------->|
 * SO:              |X2|X1|X0|MSB<----------returned READ data----------->LSB|
 *
 * 16-bit WISHBONE WRITE operation:
 *    23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 *    --|--|--|--|--|-----------------------------------------------|--|--|--|
 * SI: 1|A3|A2|A1|A0|MSB<-----------WRITE data------------------>LSB|N2|N1|N0|
 * SO:                                                              |Y2|Y1|Y0|
 *
 * * A3-A0: Address bits (sent by CPU)
 *
 * * X2-X0: ack bits for reads (sent by FPGA). If any are 1, the following
 *   data is valid.  If they are all 0, the WISHBONE cycle did not complete in 
 *   time and must be immediately retried.
 *
 * * Y2-Y0: ack bits for write (sent by FPGA). If any are 1, the write
 *   cycle was completed.  If all are 0, the WISHBONE write did not complete
 *   and must be immediately retried.
 *
 * * B: Burst read.  Setting this to 1 starts another WISHBONE read cycle for
 *   the same address.  See "SBUS Burst mode" below.
 *
 * * N2-N0: For burst writes, this represents bits 15-13 of the next burst 
 *   write data, otherwise these bits are Dont-Cares.  See "SBUS Burst mode"
 *   below.  
 *
 * When the WISHBONE cycle does not assert the wb_ack_i signal in time and the
 * transaction must be retried, it is not necessary to deassert and reassert
 * SPI CS# in between each retry, though it is necessary to start a new bus
 * cycle. 
 *
 * SBUS Burst mode:
 * ================
 * 
 * After a normal 24-bit SBUS READ/WRITE cycle takes place, it is possible to 
 * continue reading or writing to the same WISHBONE register by keeping CS# 
 * asserted and continuing the SPI clock 16 bits at a time.  However, once a 
 * burst is started, there is no provision to allow for an occasional long 
 * WISHBONE cycle and all WISHBONE cycles must be able to complete in 14 SPI
 * clock cycles otherwise results are undefined.
 *
 * For burst WRITEs, one issues a normal WRITE cycle as above with possible 
 * retries making sure that the 3 LSB (last) bits sent represent the 3 MSB
 * bits (15-13) of the next WRITE data.  Subsequent writes take place 16 bits
 * at a time while CS# is held asserted.  The burst stops once CS# is 
 * deasserted.
 *
 * For burst READs, one issues a normal 24-bit READ cycle and 0 or more 
 * retries, but with the "B" bit set.  Once the burst is started, bit 15 
 * must always be set as long as the read burst is to continue.  On the very
 * last READ in the burst, the 16-bit data sent by the FPGA on SO should be all
 * zeroes.  
 *
 * Implementation notes:
 * =====================
 *
 * The spi_sbus Verilog module WISHBONE clock output is synchronous to the
 * actual SPI clock.  This also means when the SPI bus becomes quiet, there are
 * no clocks to the WISHBONE bus.
 *
 * The sel_i input is used as a byte lane select.  On the TS-7500, there are
 * 2 dedicated GPIO pins from the CPU to the FPGA that are used for this.  
 * Without the sel_i inputs, it is not possible to have 8-bit reads/writes.
 *
 * On the TS-7500 implementation, there are 2 extra address lines via GPIO
 * pins on the CPU.  This allows a total address space of 4 * 32 bytes == 128
 * bytes == 64 individual 16-bit registers.  For these GPIO lines (and the 
 * byte lane selects) it is important that they be setup in advance of the SPI
 * transaction and remain stable during it.
 *
 * The spi_sbus_resync Verilog module is similar to the spi_sbus module except
 * that it can use an asynchronous WISHBONE clock.  This is what is used in the
 * TS-7500 since the common WISHBONE clock is 75Mhz but the SPI clock can be 
 * any rate from 32.5Mhz to 50Mhz depending on CPU speed.  A side effect of
 * this though is that clock resynchronization overhead can make even the 
 * shortest of bus cycles miss the 3 SPI clock window for acknowledgement and
 * force the majority of bus cycles into at least one retry.  For this reason,
 * a 2nd WISHBONE master interface is provided for WISHBONE slaves that are
 * okay with using the SPI SCK as their clock -- to use this bus, spiwbm_en_i
 * has to be asserted before the next clock edge after spiwbm_cyc_o, 
 * spiwbm_stb_o, and spiwbm_adr_o are asserted and valid.
 */

`ifdef TESTBENCH
`timescale 1 ns / 1 ps
`endif
module spi_sbus(
  wbm_clk_o,
  wbm_adr_o,
  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_sel_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i,

  upper_adr_i,
  sck_i,
  so_o,
  si_i,
  csn_i,
  sel_i,
  rst_i
);
output wbm_clk_o;
output [31:0] wbm_adr_o;
input [15:0] wbm_dat_i;
output [15:0] wbm_dat_o;
output wbm_we_o, wbm_cyc_o, wbm_stb_o;
input wbm_ack_i;
output [1:0] wbm_sel_o;
input rst_i, sck_i, si_i, csn_i;
input [1:0] sel_i;
output so_o;
input [1:0] upper_adr_i;

assign wbm_clk_o = !sck_i;
reg [15:0] idat;
reg [15:0] wbm_dat;
reg [1:0] upper_adr /* synthesis syn_useioff=0 */;
reg [5:0] wbm_adr;
reg [4:0] pstate, nstate;
reg [1:0] sel /* synthesis syn_useioff=0 */;
reg stb, wbm_we, nshift;
reg sbus_err, sbus_done, sbus_burst, sbus_done2, sbus_done2p;
wire sbus_en = !csn_i || (sbus_err && !sbus_done2p);
initial sbus_err = 1'b0;
initial sbus_done2 = 1'b0;
wire wbm_stb = stb || (nshift && idat[0]);
assign wbm_cyc_o = wbm_stb;
assign wbm_stb_o = wbm_stb;
assign wbm_adr_o = {upper_adr, wbm_adr[3:0], 1'b0};
assign wbm_dat_o = wbm_dat;
assign wbm_we_o = wbm_we;
assign wbm_sel_o = sel;
always @(posedge sck_i) begin
  if (pstate == 5'd0) begin
    wbm_we <= si_i;
    sel <= sel_i;
  end
  if (pstate == 5'd4) begin
    wbm_adr <= {idat[2:0], si_i};
    upper_adr <= upper_adr_i;
  end
  if (pstate == 5'd20) wbm_dat <= {idat[14:0], si_i};
end
always @(posedge sck_i or negedge sbus_en) begin
  if (!sbus_en) begin
    idat <= 16'd0;
    pstate <= 5'd0;
    sbus_burst <= 1'b0;
    sbus_err <= 1'b0;
    sbus_done2p <= 1'b0;
  end else begin
    sbus_done2p <= sbus_done2;
    if (pstate != 5'd23) pstate <= pstate + 1'b1;
    idat <= {idat[14:0], si_i};

    if (wbm_we && sbus_done && nstate == 5'd23) begin
      sbus_err <= 1'b0;
      pstate <= 5'd8;
    end

    if (pstate == 5'd7 && !wbm_we && !sbus_done) sbus_err <= 1'b1;
    else if (pstate == 5'd23 && !sbus_done) sbus_err <= 1'b1;
    else if (pstate == 5'd23 && !sbus_err) pstate <= 5'd8;

    if (sbus_err && !wbm_we && nshift && sbus_done) sbus_err <= 1'b0;
  end
end


reg [15:0] odat, ndat;
reg so;
reg so_q;
assign so_o = so_q;
always @(*) begin
  if (nshift) so = odat[15];
  else so = wbm_ack_i | sbus_done;
end

always @(posedge sck_i) so_q <= so;

always @(negedge sck_i or negedge sbus_en) begin
  if (!sbus_en) begin
    nstate <= 5'd0;
    odat <= 16'd0;
    ndat <= 16'd0;
    stb <= 1'b0;
    nshift <= 1'b0;
    sbus_done <= 1'b0;
    sbus_done2 <= 1'b0;
  end else begin
    nstate <= nstate + 1'b1;
    if (nstate == 5'd3 && !sbus_err) begin
      stb <= !wbm_we;
    end

    if (nstate == 5'd19 && wbm_we && !sbus_err) begin
      stb <= 1'b1;
    end

    if (nstate == 5'd6 && !wbm_we && 
      (sbus_done || (wbm_stb && wbm_ack_i))) nshift <= 1'b1;

    if (nshift) odat <= {odat[14:0], 1'bx};

    if (nshift && idat[0]) begin
      stb <= 1'b1;
      // sbus_done2 <= 1'b0;
      // sbus_done <= 1'b0;
    end

    if (wbm_stb && wbm_ack_i) begin
      if (!nshift) odat <= wbm_dat_i;
      ndat <= wbm_dat_i;
      stb <= 1'b0;
      sbus_done <= 1'b1;
      if (wbm_we) sbus_done2 <= 1'b1;
    end

    if (nstate == 5'd23 && !sbus_err) begin
      nstate <= 5'd8;
    end

    if (nstate == 5'd22 && !sbus_err) begin
      odat <= ndat;
    end


    if (nstate == 5'd23 && sbus_err) nstate <= 5'd0;
    if (nstate == 5'd23 && sbus_done && nshift) sbus_done2 <= 1'b1;

  end
end
endmodule


module spi_sbus_resync(
  wbm_clk_i,
  wbm_adr_o,
  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_sel_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i,

  upper_adr_i,
  sck_i,
  so_o,
  si_i,
  csn_i,
  sel_i,
  rst_i,

  wbm2_en_i,
  wbm2_clk_o,
  wbm2_adr_o,
  wbm2_sel_o,
  wbm2_we_o,
  wbm2_cyc_o,
  wbm2_stb_o,
  wbm2_dat_o,
  wbm2_dat_i,
  wbm2_ack_i
);
input wbm_clk_i;
output [31:0] wbm_adr_o;
input [15:0] wbm_dat_i;
output [15:0] wbm_dat_o;
output wbm_we_o, wbm_cyc_o, wbm_stb_o;
input wbm_ack_i;
output [1:0] wbm_sel_o;
input rst_i, sck_i, si_i, csn_i;
input [1:0] sel_i, upper_adr_i;
output so_o;

output wbm2_clk_o, wbm2_we_o, wbm2_stb_o, wbm2_cyc_o;
input wbm2_ack_i;
output [31:0] wbm2_adr_o;
input [15:0] wbm2_dat_i;
output [15:0] wbm2_dat_o;
output [1:0] wbm2_sel_o;
input wbm2_en_i;

wire wb_clk, wb_ack, wb_we, wb_cyc, wb_stb;
wire [15:0] wb_idat, wb_odat;
wire [31:0] wb_adr; 
wire [1:0] wb_sel;
wire flushack;
assign wbm2_clk_o = wb_clk;
assign wbm2_adr_o = wb_adr;
assign wbm2_cyc_o = wb_cyc;
assign wbm2_stb_o = wb_stb;
assign wbm2_we_o = wb_we;
assign wbm2_dat_o = wb_odat;
assign wbm2_sel_o = wb_sel;
spi_sbus sbuscore(
  .upper_adr_i(upper_adr_i),
  .rst_i(rst_i),
  .sck_i(sck_i),
  .so_o(so_o),
  .si_i(si_i),
  .csn_i(csn_i),
  .sel_i(sel_i),

  .wbm_clk_o(wb_clk),
  .wbm_cyc_o(wb_cyc),
  .wbm_stb_o(wb_stb),
  .wbm_we_o(wb_we),
  .wbm_adr_o(wb_adr),
  .wbm_dat_o(wb_odat),
  .wbm_dat_i(wbm2_en_i ? wbm2_dat_i : wb_idat),
  .wbm_sel_o(wb_sel),
  .wbm_ack_i(wbm2_en_i ? wbm2_ack_i : wb_ack)
);

reg first_cyc;
always @(posedge wb_clk or posedge rst_i) begin
  if (rst_i) begin
    first_cyc <= 1'b1;
  end else begin
    if (wb_cyc && wb_stb && wb_ack) first_cyc <= 1'b0;
    else if (!wb_cyc) first_cyc <= 1'b1;
  end
end

wire wbf_ack, wbf_we, wbf_cyc, wbf_stb;
wire [15:0] wbf_idat, wbf_odat;
wire [31:0] wbf_adr; 
wire [1:0] wbf_sel;
wb_writebuffer wbufcore(
  .wb_clk_i(wb_clk),
  .wb_rst_i(rst_i),
  .wb_cyc_i(wb_cyc && !wbm2_en_i),
  .wb_stb_i(wb_stb && (!first_cyc || flushack)),
  .wb_we_i(wb_we),
  .wb_adr_i(wb_adr),
  .wb_dat_i(wb_odat),
  .wb_dat_o(wb_idat),
  .wb_sel_i(wb_sel),
  .wb_ack_o(wb_ack),

  .wbm_cyc_o(wbf_cyc),
  .wbm_stb_o(wbf_stb),
  .wbm_we_o(wbf_we),
  .wbm_sel_o(wbf_sel),
  .wbm_dat_o(wbf_odat),
  .wbm_dat_i(wbf_idat),
  .wbm_adr_o(wbf_adr),
  .wbm_ack_i(wbf_ack),

  .flushreq_i(first_cyc),
  .flushack_o(flushack)
);

wb_resync resynccore(
  .wb_clk_i(wb_clk),
  .wb_rst_i(rst_i),
/*
  .wbs1_cyc_i(wb_cyc),
  .wbs1_stb_i(wb_stb),
  .wbs1_we_i(wb_we),
  .wbs1_dat_i(wb_odat),
  .wbs1_dat_o(wb_idat),
  .wbs1_ack_o(wb_ack),
  .wbs1_adr_i(wb_adr),
  .wbs1_sel_i(wb_sel),
*/

  .wbs1_cyc_i(wbf_cyc),
  .wbs1_stb_i(wbf_stb),
  .wbs1_we_i(wbf_we),
  .wbs1_dat_i(wbf_odat),
  .wbs1_dat_o(wbf_idat),
  .wbs1_ack_o(wbf_ack),
  .wbs1_adr_i(wbf_adr),
  .wbs1_sel_i(wbf_sel),


  .wbm_clk_i(wbm_clk_i),
  .wbm_cyc_o(wbm_cyc_o),
  .wbm_stb_o(wbm_stb_o),
  .wbm_we_o(wbm_we_o),
  .wbm_adr_o(wbm_adr_o),
  .wbm_dat_o(wbm_dat_o),
  .wbm_dat_i(wbm_dat_i),
  .wbm_ack_i(wbm_ack_i),
  .wbm_sel_o(wbm_sel_o)
);

endmodule


`ifdef TESTBENCH
module test();

wire clk;
wire [15:0] adr, odat;
reg [15:0] idat;
wire cyc, stb, we, so;
reg ack, csn, si, sck;

spi_sbus uut(
  .wbm_adr_o(adr),
  .wbm_cyc_o(cyc),
  .wbm_stb_o(stb),
  .wbm_we_o(we),
  .wbm_dat_o(odat),
  .wbm_dat_i(idat),
  .wbm_ack_i(ack),
  .wbm_clk_o(clk),

  .sck_i(sck),
  .so_o(so),
  .si_i(si),
  .csn_i(csn)
);

reg [7:0] count, wblat;
always @(posedge clk) begin
  idat <= 16'hdead;
  ack <= 1'b0;
  if (cyc & stb && !ack && count >= wblat) begin
    if (we) $display("%t WB write cycle, address: %x, data: %x", $time, adr,
      odat); 
    else $display("%t WB read cycle, address: %x", $time, adr);
    ack <= 1'b1;
    idat <= adr;
    count <= 4'd0;
  end else if (cyc & stb) count <= count + 1'b1;
  else count <= 4'd0;
end

task sbus_write16;
  input [3:0] address_i;
  input [15:0] data_i;
  integer i;
  reg [7:0] j;
  reg [7:0] retries;
begin
  j = 8'd0;
  retries = 8'd0;
  while (j[0] != 1'b1) begin
    #10;
    csn = 1'b0;
    #10;
    j[7:0] = {1'b1, address_i, 3'd0};
    for (i = 0; i < 5; i = i + 1) begin
      si = j[7 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
    si = 1'b0;
    j = 0;
    for (i = 0; i < 16; i = i + 1) begin
      si = data_i[15 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    for (i = 0; i < 3; i = i + 1) begin
      #9;
      #1 j[2 - i] = so;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    #10;
    if (j[0] == 1'b1) 
      $display("3-bit spi data in: %x, address: %x, retries: %d",
      j[2:0], address_i, retries);
    csn = 1'b1;
    #10;
    retries = retries + 1'b1;
  end
end
endtask

task sbus_burstwrite16;
  input [3:0] address_i;
  input [15:0] data_i;
  input [7:0] count_i;
  integer h;
  integer i;
  reg [7:0] j;
  reg [7:0] retries;
begin
  j = 8'd0;
  retries = 8'd0;
  while (j[0] != 1'b1) begin
    #10;
    csn = 1'b0;
    #10;
    j[7:0] = {1'b1, address_i, 3'd0};
    for (i = 0; i < 5; i = i + 1) begin
      si = j[7 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
    si = 1'b0;
    j = 0;
    for (i = 0; i < 16; i = i + 1) begin
      si = data_i[15 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    for (i = 0; i < 3; i = i + 1) begin
      si = data_i[15 - i];
      #9;
      #1 j[2 - i] = so;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    #10;
    if (j[0]) 
      $display("3-bit spi data in: %x, address: %x, retries: %d",
      j[2:0], address_i, retries);
    retries = retries + 1'b1;
  end


  for (h = 0; h < count_i - 1; h = h + 1) begin
    for (i = 0; i < 16; i = i + 1) begin
      if (i < 13) si = data_i[12 - i];
      else si = data_i[15 - (i - 12)];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
  end

  #10;
  csn = 1'b1;
  #10;
end
endtask

task sbus_read16;
  input [3:0] address_i;
  integer i;
  reg [18:0] j;
  reg [7:0] retries;
begin
  j = 19'd0;
  retries = 8'd0;
  while (j[16] != 1'b1) begin
    #10;
    csn = 1'b0;
    #10;
    j[7:0] = {1'b0, address_i, 3'd0};
    for (i = 0; i < 5; i = i + 1) begin
      si = j[7 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
    si = 1'b0;
    j = 0;
    for (i = 0; i < 19; i = i + 1) begin
      #9;
      #1 j[18 - i] = so;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    #10;
    if (j[16] == 1'b1) 
      $display("19-bit spi data in: %x, address: %x, retries: %d",
      j, address_i, retries);
    csn = 1'b1;
    #10;
    retries = retries + 1'b1;
  end
end
endtask

task sbus_burstread16;
  input [3:0] address_i;
  input [7:0] count_i;
  integer h;
  integer i;
  reg [18:0] j;
  reg [7:0] retries;
begin
  j = 19'd0;
  retries = 8'd0;
  while (j[16] != 1'b1) begin
    #10;
    csn = 1'b0;
    #10;
    j[7:0] = {1'b0, address_i, 3'd0};
    for (i = 0; i < 5; i = i + 1) begin
      si = j[7 - i];
      #10;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
    si = 1'b0;
    j = 0;
    for (i = 0; i < 19; i = i + 1) begin
      if (i == 3) si = 1'b1; else si = 1'b0;
      #9;
      #1 j[18 - i] = so;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end  
    #10;
    if (j[16] == 1'b1) 
      $display("19-bit spi data in: %x, address: %x, retries: %d",
      j, address_i, retries);
    retries = retries + 1'b1;
  end


  for (h = 0; h < count_i - 1; h = h + 1) begin
    for (i = 0; i < 16; i = i + 1) begin
      if (i == 0 && h != (count_i - 2)) si = 1'b1; else si = 1'b0;
      #9;
      #1 j[15 - i] = so;
      sck = 1'b1;
      #10;
      sck = 1'b0;
    end
    $display("  next data: %x", j[15:0]);
  end

  csn = 1'b1;
  #10;
end
endtask

initial begin
  $dumpfile("dump.vcd");
  $dumpvars(2, test);
  sck = 1'b0;
  si = 1'b0;
  csn = 1'b1;
  #100;
  /* for (wblat = 8'd0; wblat < 8'd64; wblat = wblat + 1'b1)
       sbus_read16(4'h5);  */
  wblat = 5;
  /* for (wblat = 8'd0; wblat < 8'd64; wblat = wblat + 1'b1)
       sbus_write16(4'h5, 16'h1234);  */
  /* sbus_write16(4'h5, 16'hb00b); */
  /* sbus_burstwrite16(4'h5, 8'd4, 8'd4); */
  sbus_burstread16(4'h5, 8'd4); 
  /* sbus_read16(4'h7); */
  
  #10000;
  $finish;
end


endmodule
`endif
