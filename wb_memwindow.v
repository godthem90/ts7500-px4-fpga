/* Copyright 2009-2010, Unpublished Work of Technologic Systems
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

`ifdef TEST8
`define TESTBENCH
`endif

`ifdef TEST16
`define TESTBENCH
`endif

`ifdef TEST32
`define TESTBENCH
`endif

`ifdef TESTBENCH
`timescale 1 ns / 1 ps
`endif

/* This core is a simple core to allow access to an up to 64Kbyte WISHBONE
 * address space via a 2 16-bit register window.  On the TS-7500 this core
 * appears at 0x3c and is used to address the 8kbyte blockram used by
 * the XUART core for TX/RX buffers.
 * 
 * Register map:
 * base + 0x0: Address reg (RW)
 * base + 0x2: Data reg (RW)
 *
 * * When the data reg is read or written, the address is automatically
 *   incremented by 2.  In this way, contiguous reads/writes of address
 *   space is optimized.
 */

module wb_memwindow16to32(
  wb_clk_i,
  wb_rst_i,
  wb_cyc_i,
  wb_stb_i,
  wb_we_i,
  wb_adr_i,
  wb_sel_i,
  wb_dat_i,
  wb_dat_o,
  wb_ack_o,

  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_sel_o,
  wbm_adr_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i
);

input wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
output wb_ack_o;
output [15:0] wb_dat_o;
input [15:0] wb_dat_i, wb_adr_i;
input [1:0] wb_sel_i;

output wbm_cyc_o, wbm_stb_o, wbm_we_o;
output [31:0] wbm_dat_o, wbm_adr_o;
input [31:0] wbm_dat_i;
output [3:0] wbm_sel_o;
input wbm_ack_i;

reg [15:0] adr;
reg [15:0] wb_dat;
reg wbm_cyc, wbm_stb, wb_ack;
always @(posedge wb_clk_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    adr <= 16'd0;
  end else begin
    if (wb_cyc_i && wb_stb_i && wb_we_i && !wb_adr_i[1]) adr <= wb_dat_i;
    if (wbm_cyc && wbm_stb && wbm_ack_i) adr <= adr + 2'd2;
  end
end

assign wbm_cyc_o = wbm_cyc;
assign wbm_stb_o = wbm_stb;
assign wb_ack_o = wb_ack;
assign wbm_dat_o[31:0] = {wb_dat_i[15:0], wb_dat_i[15:0]};
assign wb_dat_o = wb_dat;
assign wbm_adr_o = {16'd0, adr[15:2], 2'b00};
assign wbm_we_o = wb_we_i;
assign wbm_sel_o = adr[1] ? 4'b1100 : 4'b0011;
always @(*) begin
  if (wb_adr_i[1]) begin
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i;
    wb_ack = wbm_ack_i;
    wb_dat = adr[1] ? wbm_dat_i[31:16] : wbm_dat_i[15:0];
  end else begin
    wbm_cyc = 1'b0;
    wbm_stb = 1'bx;
    wb_ack = wb_cyc_i && wb_stb_i;
    wb_dat = adr;
  end
end
endmodule


/* This core is a simple core to allow access to an up to 64Kbyte WISHBONE
 * address space via a 4 16-bit register window.  
 * 
 * Register map:
 * base + 0x0: Address reg (RW)
 * base + 0x2: 8-bit data reg with auto-increment (RW)
 *
 * * When the data reg is read or written, the address is automatically
 *   incremented by 1.  In this way, contiguous reads/writes of address
 *   space is optimized.
 *
 * base + 0x4: 16-bit data reg with auto-increment (RW)
 *
 * * When the 16-bit data reg is read or written, 2 read or write 8-bit
 *   bus cycles are performed and the address is incremented by 2.
 *
 * base + 0x6: Combo address/data write reg (WO)
 *   bits 15-8: address bits 7-0 of bus write
 *   bits 7-0: data bits of bus write cycle.
 *
 * * This register is only meaningful write-only.  It encodes the 8 LSBs
 *   of the address as the 8 MSBs of the written data word.
 */

module wb_memwindow16to8(
  wb_clk_i,
  wb_rst_i,
  wb_cyc_i,
  wb_stb_i,
  wb_we_i,
  wb_adr_i,
  wb_sel_i,
  wb_dat_i,
  wb_dat_o,
  wb_ack_o,

  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_adr_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i
);

input wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
output wb_ack_o;
output [15:0] wb_dat_o;
input [15:0] wb_dat_i, wb_adr_i;
input [1:0] wb_sel_i;

output wbm_cyc_o, wbm_stb_o, wbm_we_o;
output [7:0] wbm_dat_o;
output [15:0] wbm_adr_o;
input [7:0] wbm_dat_i;
input wbm_ack_i;

reg [15:0] adr;
reg [15:0] wb_dat;
reg holdack, holdstb;
reg [7:0] datlatch;
reg wbm_cyc, wbm_stb, wb_ack;
always @(posedge wb_clk_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    adr <= 16'd0;
    holdack <= 1'b1;
    datlatch <= 8'd0;
    holdstb <= 1'b0;
  end else begin
    case (wb_adr_i[2:1])
    2'd0: if (wb_cyc_i && wb_stb_i && wb_we_i) adr <= wb_dat_i;
    2'd1: if (wbm_cyc && wbm_stb && wbm_ack_i) adr <= adr + 1'b1;
    2'd2: if (wbm_cyc && wbm_stb && wbm_ack_i) begin
      adr <= adr + 1'b1;
      if (holdack) begin
        holdack <= 1'b0;
        datlatch <= wbm_dat_i[7:0];
      end else holdack <= 1'b1; 
    end
    2'd3: if (wbm_cyc && wbm_stb && wbm_ack_i) adr[7:0] <= wb_dat_i[15:8] + 1'b1;
    endcase
    holdstb <= 1'b0;
    if (wbm_cyc && wbm_stb && wbm_ack_i) holdstb <= 1'b1;
  end
end

assign wbm_cyc_o = wbm_cyc;
assign wbm_stb_o = wbm_stb;
assign wb_ack_o = wb_ack;
reg [7:0] wbm_dat;
assign wbm_dat_o = wbm_dat;
assign wb_dat_o = wb_dat;
reg [15:0] wbm_adr;
assign wbm_adr_o = wbm_adr;
assign wbm_we_o = wb_we_i;
always @(*) begin
  case (wb_adr_i[2:1]) 
  2'd0: begin
    wbm_cyc = 1'b0;
    wbm_stb = 1'bx;
    wbm_dat = 8'hxx;
    wbm_adr = 16'hxxxx;
    wb_ack = wb_cyc_i && wb_stb_i;
    wb_dat = adr;
  end
  2'd1: begin
    wbm_adr = adr;
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i && !holdstb;
    wbm_dat = wb_dat_i[7:0];
    wb_ack = wbm_ack_i && !holdstb;
    wb_dat = {8'hxx, wbm_dat_i[7:0]};
  end
  2'd2: begin
    wbm_adr = adr;
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i && !holdstb;
    wbm_dat = holdack ? wb_dat_i[7:0] : wb_dat_i[15:8];
    wb_ack = wbm_ack_i && !holdack && !holdstb;
    wb_dat = {wbm_dat_i[7:0], datlatch};
  end
  2'd3: begin
    wbm_adr = {adr[15:8], wb_dat_i[15:8]};
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i && !holdstb;
    wbm_dat = wb_dat_i[7:0];
    wb_ack = wbm_ack_i && !holdstb;
    wb_dat = 16'hxxxx;
  end
  endcase
end


endmodule


/* This core is a simple core to allow access to an up to 128Mbyte WISHBONE
 * address space via a 4 16-bit register window.  It allows arbitrary mixing/
 * matching of WISHBONE slaves bus width (8/16/32) and has some optimization 
 * features to minimize superfluous cycles typical to accessing via a window. 
 *
 * This core's first use was on the TS-4500 to access the much larger address
 * space of ISA for the TS-8100 base board.
 * 
 * Register map:
 * base + 0x0: Upper address reg, address bits 26-11 (RW)
 * base + 0x2: WISHBONE config reg (RW)
 *   bit 15-14: bus width
 *     0 - reserved
 *     1 - 32 bit
 *     2 - 16 bit
 *     3 - 8 bit
 *   bit 13-12: Data reg #2 config (RW)
 *     0 - normal
 *     1 - two-cycle split (16bit in -> 2x 8bit cycles out)
 *     2 - two-cycle combine (2x 16bit cycles -> 32bit cycle)
 *     3 - combo adr/data write (8 LSBs: data, 8 MSBs: override 8 LSBS of adr)
 *   bit 11: Data reg #2 auto-increment enable
 *   bit 10-0: lower address reg, address bits 10-0 
 *
 * * Two-cycle combine reads/writes should first read/write the lower (LSB) 
 *   address to properly be combined.  The address is always auto-incremented. 
 *
 * * After a combo adr/data write cycle, the address reg points to the address
 *   that was previously written with optional auto-increment by 1.
 *
 * base + 0x4: Data reg #1 - never auto-increments (RW)
 * base + 0x6: Data reg #2 - side-effects as configured in WB config reg (RW)
 *
 */

module wb_memwindow(
  wb_clk_i,
  wb_rst_i,
  wb_cyc_i,
  wb_stb_i,
  wb_we_i,
  wb_adr_i,
  wb_sel_i,
  wb_dat_i,
  wb_dat_o,
  wb_ack_o,

  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_adr_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i,
  wbm_sel_o,
  wbm_buswidth_i 
);

input wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
output wb_ack_o;
output [15:0] wb_dat_o;
input [15:0] wb_dat_i, wb_adr_i;
input [1:0] wb_sel_i;

output wbm_cyc_o, wbm_stb_o, wbm_we_o;
output [31:0] wbm_dat_o;
output [31:0] wbm_adr_o;
input [31:0] wbm_dat_i;
input wbm_ack_i;
output [3:0] wbm_sel_o;

/* If 0, means nothing otherwise overrides width with same val as config reg */
/* External slaves should combinationally compute this based on wbm_adr_o */
input [1:0] wbm_buswidth_i; 

parameter address_width = 18;

wire [31:0] address_mask = (1 << address_width) - 1;

reg [26:0] adr;
reg [4:0] wconfig;
wire [1:0] buswidth;
assign buswidth = (wbm_buswidth_i == 2'b00) ? wconfig[4:3] : wbm_buswidth_i;

wire [1:0] mode = wconfig[2:1];

reg wbm_cyc, wbm_stb, wb_ack;
//reg holdack, holdstb;
reg [15:0] datlatch_r, datlatch_w;
reg combine, ack;
reg [1:0] ss; // split-cycle state
always @(posedge wb_clk_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    adr <= 27'd0;
    //holdack <= 1'b0;
    datlatch_r <= 16'd0;
    datlatch_w <= 16'd0;
    //holdstb <= 1'b0;
    wconfig <= 5'd0;
    combine <= 1'b0;
    ss <= 2'd0;
    ack <= 1'b0;
  end else begin
    //holdstb <= 1'b0;
    ack <= 1'b0;
    case (wb_adr_i[2:1])
    2'b00: if (wb_cyc_i && wb_stb_i && !ack) begin 
      if (wb_we_i) adr[26:11] <= wb_dat_i & address_mask[26:11];
      ack <= 1'b1;
    end
    2'b01: if (wb_cyc_i && wb_stb_i && !ack) begin
      if (wb_we_i) {wconfig, adr[10:0]} <= wb_dat_i;
      ack <= 1'b1;
    end
    2'b11: begin // data reg #2
      if (mode == 2'b00) begin // normal mode
        if (wconfig[0] && wbm_cyc && wbm_stb && wbm_ack_i) begin
          if (buswidth == 2'b01) adr <= adr + 2'b10;
          else if (buswidth == 2'b10) adr <= adr + 2'b10;
          else adr <= adr + 1'b1;
        end
      end else if (mode == 2'b01) begin // 2 cycle split
        case (ss)
          2'd0: if (wbm_cyc && wbm_stb && wbm_ack_i) begin
            ss <= 2'd1;
            datlatch_r[7:0] <= wbm_dat_i;
          end
          2'd1: begin
            ss <= 2'd2; // hold strobe
            if (wconfig[0]) adr <= adr + 1'b1;
          end
          2'd2: if (wbm_cyc && wbm_stb && wbm_ack_i) begin
            ss <= 2'd0;
            if (wconfig[0]) adr <= adr + 1'b1;
          end
        endcase
      end else if ((mode == 2'b10) && wb_cyc_i && wb_stb_i && wb_ack_o) begin
        // 2 cycle combine:
        adr <= adr + 2'b10;
        combine <= !combine;
        if (combine ^ wb_we_i) begin
          datlatch_w <= wb_dat_i;
          datlatch_r <= wbm_dat_i[31:16];
        end
      end else if ((mode == 2'b11) && wbm_cyc && wbm_stb && wbm_ack_i) begin
        // combo adress - data:
        if (wconfig[0]) adr <= {adr[26:8], wb_dat_i[15:8]} + 1'b1;
        else adr <= {adr[26:8], wb_dat_i[15:8]};
      end
    end
    endcase
    if ({wb_adr_i[2:1], mode} != 4'b1101) ss <= 3'd0;
  end
end

assign wbm_we_o = wb_we_i;
assign wbm_cyc_o = wbm_cyc;
assign wbm_stb_o = wbm_stb;
assign wb_ack_o = wb_ack;
reg [31:0] wbm_dat;
assign wbm_dat_o = wbm_dat;
reg [15:0] wb_dat;
assign wb_dat_o = wb_dat;
reg [31:0] wbm_adr;
assign wbm_adr_o = wbm_adr;
reg [3:0] wbm_sel;
assign wbm_sel_o = wbm_sel;

always @(*) begin
  if (wb_adr_i[2:1] == 2'b00) begin // address reg
    wbm_cyc = 1'b0;
    wbm_stb = 1'b0;
    wbm_dat = 32'hxxxxxxxx;
    wbm_adr = 32'hxxxxxxxx;
    wbm_sel = 4'hx;
    wb_ack = ack;
    wb_dat = adr[26:11] & address_mask[26:11];
  end else if (wb_adr_i[2:1] == 2'b01) begin // config reg
    wbm_cyc = 1'b0;
    wbm_stb = 1'b0;
    wbm_dat = 32'hxxxxxxxx;
    wbm_adr = 32'hxxxxxxxx;
    wbm_sel = 4'hx;
    wb_ack = ack;
    wb_dat = {wconfig, adr[10:0]};
  end else if (mode == 2'b01 && wb_adr_i[2:1] == 2'b11) begin // 2 cycle split
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i && (ss != 3'd1);
    wbm_dat[31:8] = 24'hxxxxxx;
    wbm_dat[7:0] = (ss == 3'd2) ? wb_dat_i[15:8] : wb_dat_i[7:0];
    wbm_adr = {5'd0, adr} & address_mask;
    wbm_sel = 4'h1;
    wb_ack = wbm_ack_i && (ss == 3'd2);
    wb_dat = {wbm_dat_i[7:0], datlatch_r[7:0]};
  end else if (mode == 2'b10 && wb_adr_i[2:1] == 2'b11) begin
    // 2 cycle combine.  wbm cycle is immediate on a read, not on a write.
    if (combine ^ wb_we_i) begin
      wbm_cyc = wb_cyc_i;
      wbm_stb = wb_stb_i;
      wb_ack = wbm_ack_i;
    end else begin
      wbm_cyc = 1'b0;
      wbm_stb = 1'b0;
      wb_ack = wb_cyc_i && wb_stb_i;
    end
    wbm_dat = {wb_dat_i, datlatch_w};
    wbm_adr = {5'd0, adr[26:2], 2'b00} & address_mask;
    wbm_sel = 4'hf;
    wb_dat = combine ? datlatch_r : wbm_dat_i[15:0];
  end else if (mode == 2'b11 && wb_adr_i[2:1] == 2'b11) begin
    // combo adress - data:
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i;
    wbm_dat = {24'hxxxxxx, wb_dat_i[7:0]};
    wbm_adr = {5'd0, adr[26:8], wb_dat_i[15:8]} & address_mask;
    wbm_sel = 4'h1;
    wb_ack = wbm_ack_i;
    wb_dat = 16'hxxxx; // this mode is write-only
  end else if (buswidth == 2'b01) begin // 32 bit normal mode
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i;
    wbm_dat = {wb_dat_i, wb_dat_i};
    wbm_adr = {5'd0, adr[26:2], 2'b00} & address_mask;
    wbm_sel = adr[1] ? {wb_sel_i, 2'b00} : {2'b00, wb_sel_i};
    wb_ack = wbm_ack_i;
    wb_dat = adr[1] ? wbm_dat_i[31:16] : wbm_dat_i[15:0];
  end else if (buswidth == 2'b11) begin // 8 bit normal mode
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i;
    wbm_dat = {24'hxxxxxx, wb_dat_i[7:0]};
    wbm_adr = {5'd0, adr} & address_mask;
    wbm_sel = 4'h1;
    wb_ack = wbm_ack_i;
    wb_dat = {8'hxx, wbm_dat_i[7:0]};
  end else begin // 16 bit normal mode
    wbm_cyc = wb_cyc_i;
    wbm_stb = wb_stb_i;
    wbm_dat = {16'hxxxx, wb_dat_i[15:0]};
    wbm_adr = {5'd0, adr} & address_mask;
    wbm_sel = {2'd0, wb_sel_i};
    wb_ack = wbm_ack_i;
    wb_dat = wbm_dat_i[15:0];
  end
end

endmodule

`ifdef TESTBENCH
module test();


reg clk, rst, cyc, stb, we;
wire ack;
reg [1:0] sel;
reg [15:0] idat, adr;
wire [15:0] odat;
wire mcyc, mstb, mwe; 
wire [31:0] madr, modat;
wire [3:0] msel;
reg mack;

wb_memwindow uut(
  .wb_clk_i(clk),
  .wb_rst_i(rst),

  .wb_cyc_i(cyc),
  .wb_stb_i(stb),
  .wb_we_i(we),
  .wb_adr_i(adr),
  .wb_dat_i(idat),
  .wb_sel_i(sel),
  .wb_dat_o(odat),
  .wb_ack_o(ack),

  .wbm_cyc_o(mcyc),
  .wbm_stb_o(mstb),
  .wbm_we_o(mwe),
  .wbm_adr_o(madr),
  .wbm_dat_o(modat),
  .wbm_dat_i(madr), /* Loop address as return data */
  .wbm_ack_i(mack), /* Auto-ack WB cycle */
  .wbm_sel_o(msel),
  .wbm_buswidth_i(2'b00)
);

task wb_read8;
  input [15:0] address_i;
  integer i;
  reg [7:0] clocks;
  reg [7:0] data;
begin
  clocks = 8'd0;
  cyc = 1'b1;
  stb = 1'b1;
  we = 1'b0;
  adr = address_i;
  sel = 2'b01;
  idat = 16'hxxxx;
  @(posedge clk);
  while (!ack) begin
    @(posedge clk);
    clocks = clocks + 1'b1;
  end
  data = odat[7:0];
  cyc = 1'b0; stb = 1'b0;
  $display("wb_read8 @ adr %x, took %d clocks, got dat %x", 
    address_i, clocks, data);
end
endtask


task wb_read16;
  input [15:0] address_i;
  integer i;
  reg [7:0] clocks;
  reg [15:0] data;
begin
  clocks = 8'd0;
  cyc = 1'b1;
  stb = 1'b1;
  we = 1'b0;
  adr = address_i;
  sel = 2'b11;
  idat = 16'hxxxx;
  @(posedge clk);
  while (!ack) begin
    @(posedge clk);
    clocks = clocks + 1'b1;
  end
  data = odat[15:0];
  cyc = 1'b0; stb = 1'b0;
  $display("wb_read16 @ adr %x, took %d clocks, got dat %x", 
    address_i, clocks, data);
end
endtask


task wb_write16;
  input [15:0] address_i;
  input [15:0] data_i;
  integer i;
  reg [7:0] clocks;
begin
  clocks = 8'd0;
  cyc = 1'b1;
  stb = 1'b1;
  we = 1'b1;
  adr = address_i;
  sel = 2'b11;
  idat = data_i;
  @(posedge clk);
  while (!ack) begin
    @(posedge clk);
    clocks = clocks + 1'b1;
  end
  cyc = 1'b0; stb = 1'b0;
  $display("wb_write16 @ adr %x, took %d clocks, sent dat %x", 
    address_i, clocks, data_i);
end
endtask

task wb_write8;
  input [15:0] address_i;
  input [7:0] data_i;
  integer i;
  reg [7:0] clocks;
begin
  clocks = 8'd0;
  cyc = 1'b1;
  stb = 1'b1;
  we = 1'b1;
  adr = address_i;
  sel = 2'b01;
  idat = {8'd0, data_i};
  @(posedge clk);
  while (!ack) begin
    @(posedge clk);
    clocks = clocks + 1'b1;
  end
  cyc = 1'b0; stb = 1'b0;
  $display("wb_write8 @ adr %x, took %d clocks, sent dat %x", 
    address_i, clocks, data_i);
end
endtask

always #5 clk = !clk;

always @(posedge clk) begin
  mack <= 1'b0;
  if (mcyc && mstb && !mack) begin
    mack <= 1'b1;
    if (mwe) $display("  -- WBM write for adr %x, dat %x, sel %x", 
      madr, modat, msel);
    else $display("  -- WBM read for adr %x, sel %x", madr, msel);
  end
end

initial begin
  $dumpfile("dump.vcd");
  $dumpvars(2, test);

  rst = 1; clk = 0;
  cyc = 0; stb = 0; we = 0; adr = 0; sel = 0; idat = 0;
  #100;
  rst = 0;
  #10;
  @(posedge clk);

`ifdef TEST8

  wb_write16(16'd0, 16'haaaa); /* Write upper address */
  wb_write16(16'd2, 16'hc000); /* Set cfg to 8-bit mode, all 0's lower adr */
  wb_write16(16'd4, 16'habcd); /* Write 8-bit 0xcd to adr */
  wb_write16(16'd2, 16'hc800); /* Set cfg to enable auto-incr */
  wb_write16(16'd6, 16'h17); /* Write 8-bit and incr address ... */
  wb_write16(16'd6, 16'h22); /* Write 8-bit and incr address ... */

  wb_write16(16'd2, 16'h3000); /* Set cfg to address-data write */
  wb_write16(16'd6, 16'h77a5); /* Write 0xa5 to address 0x77 */
  wb_write16(16'd6, 16'h885a); /* Write 0x5a to address 0x88 */

  wb_write16(16'd2, 16'h1040); /* Set cfg to 2 cycle split */
  wb_write16(16'd6, 16'h6789); /* Write 16-bit 0x6789 */
  wb_write16(16'd6, 16'habcd); /* Write 16-bit 0xabcd */
  wb_read16(16'd6);
  wb_write16(16'd2, 16'h1800); /* Set cfg to 2 cycle split with auto-inc */
  wb_write16(16'd6, 16'h6789); /* Write 16-bit 0x6789 */
  wb_write16(16'd6, 16'habcd); /* Write 16-bit 0xabcd */
  wb_read16(16'd6);
  wb_read16(16'd6);

`endif

`ifdef TEST16

  wb_write16(16'd0, 16'haaaa); /* Write upper address */
  wb_write16(16'd2, 16'h8800); /* Set cfg to 16-bit */
  wb_write16(16'd4, 16'h6789); /* Write 16-bit 0x6789 */
  wb_read16(16'd4);
  wb_write16(16'd2, 16'h8802); /* Set cfg to 16-bit */
  wb_write16(16'd4, 16'h6789); /* Write 16-bit 0x6789 */
  wb_read16(16'd4);
  wb_write16(16'd6, 16'h1111);
  wb_write16(16'd6, 16'h1112);
  wb_write16(16'd6, 16'h1113);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);

`endif

`ifdef TEST32

  wb_write16(16'd0, 16'haaaa); /* Write upper address */
  wb_write16(16'd2, 16'h4000); /* Set cfg to 32-bit */
  wb_write16(16'd4, 16'h6789); 
  wb_write16(16'd4, 16'habcd); 
  wb_write16(16'd2, 16'h4800); /* Set cfg to 32-bit with increment */
  wb_write16(16'd6, 16'h6789); 
  wb_write16(16'd6, 16'habcd); 
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_write16(16'd2, 16'h6000); /* Set cfg to 32-bit 2 cycle combines */
  wb_write16(16'd6, 16'h1111);
  wb_write16(16'd6, 16'h1112);
  wb_write16(16'd6, 16'h1113);
  wb_write16(16'd6, 16'h1114);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_read16(16'd6);
  wb_write16(16'd6, 16'h1113);
  wb_write16(16'd6, 16'h1114);
  wb_write16(16'd6, 16'h1113);
  wb_write16(16'd6, 16'h1114);

`endif

  #1000;
  $finish;
end

endmodule


`endif
