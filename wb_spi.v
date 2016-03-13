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

/* Core for high speed SPI with auto-CS# and support for 4 CS# signals.
 * Starts at offset 0x40 on the TS-75xx.  LUN#0 goes to onboard SPI NOR
 * flash chip and LUN#1 goes to offboard SPI (TS-752 flash chip)
 *
 * Register map:
 * base + 0x0: LUN register (R/W)
 *   bit 15: spi miso state (RO)
 *   bit 14: spi clk state (RW)
 *   bits 13-10: speed[3:0] (0- highest, 1- 1/2 speed, 2- 1/4, etc..) (RW)
 *   bits 9-8: LUN (0-3 representing the 4 CS# signals) (RW)
 *   bit 7: CS (1 - CS# is asserted) (RW)
 *   bits 6-1: reserved
 *   bit 0: speed[4] (RW)
 *
 * * The SPI clk state register should be set when CS# is deasserted.  Value
 *   0 makes SPI rising edge (CPOL=0), 1 is falling edge (CPOL=1).  This only
 *   applies to speed >= 1.  For speed == 0, SPI clock polarity/skew must be 
 *   set from the PLL phase adjust registers in the syscon block.
 *
 * * For the TS-75xx where the base clock is 75Mhz, speed settings break down
 *   as follows:
 *     0 - 75Mhz (/1)
 *     1 - 37.5Mhz (/2)
 *     2 - 18.75Mhz (/4)
 *     3 - 12.5Mhz (/6)
 *     4 - 9.375Mhz (/8)
 *     5 - 7.5Mhz (/10)
 *     6 - 6.25Mhz (/12)
 *     7 - 5.36Mhz (/14)
 *     8 - 4.68Mhz (/16)
 *     9 - 4.17Mhz (/18)
 *     ...
 *     15 - 2.5Mhz (/30)
 *     ...
 *     19 - 1.97Mhz (/38)
 *     ...
 *     31 - 1.21Mhz (/62)
 *
 * base + 0x2: previous SPI read data from last write (RO)
 * base + 0x4: reserved
 * base + 0x6: reserved
 * base + 0x8: SPI read/write with CS# to stay asserted (RW)
 * base + 0xa: SPI pipelined read with CS# to stay asserted (RO)
 *
 * * The pipelined read register is for read bursts and will automatically
 *   start a subsequent SPI read upon completion of the requested SPI read.
 *   Reading from this register infers that another read will shortly follow
 *   and allows this SPI controller "a head start" on the next read for
 *   optimimum read performance. This register should be accessed as long
 *   as there will be at least one more SPI read with CS# asserted to take
 *   place.
 *
 * base + 0xc: SPI read/write with CS# to deassert post-op (RW)
 * base + 0xe: reserved
 * 
 */

`ifdef TESTBENCH
`timescale 1 ns / 1 ps
`endif
module wb_spi(
  wb_clk_i,
  wb_rst_i,

  wb_cyc_i,
  wb_stb_i,
  wb_we_i,
  wb_adr_i,
  wb_dat_i,
  wb_sel_i,
  wb_dat_o,
  wb_ack_o,

  sck_i,
  sck_o,
  scken_o,
  so_o,
  si_i,
  csn_o,
  hispeed_o
);

input wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
input [31:0] wb_adr_i;
input [15:0] wb_dat_i;
input [1:0] wb_sel_i;
output [15:0] wb_dat_o;
output wb_ack_o;

output sck_o, so_o, scken_o;
output [3:0] csn_o;
input [3:0] si_i;
input sck_i;
output hispeed_o;

parameter hispeed_opt = 1'b1;
parameter lospeed_opt = 1'b1;

reg sck, so, cs;
reg [1:0] lun;
assign csn_o[3] = lun == 2'd3 ? !cs : 1'b1;
assign csn_o[2] = lun == 2'd2 ? !cs : 1'b1;
assign csn_o[1] = lun == 2'd1 ? !cs : 1'b1;
assign csn_o[0] = lun == 2'd0 ? !cs : 1'b1;
reg [3:0] state;
reg [4:0] count;
reg [15:0] wb_dat;
assign wb_dat_o = wb_dat;
reg scken, acc16, acc16s, wb_ack;
assign scken_o = scken;
assign wb_ack_o = wb_ack;
reg [7:0] odat, idat;
reg eot, rd;
reg [4:0] speed, cntdown;
reg sck, hispeed;
assign hispeed_o = hispeed;
assign sck_o = hispeed ? (wb_clk_i | !scken) : sck;
assign so_o = odat[7];
always @(posedge wb_clk_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    state <= 4'd0;
    count <= 5'd0;
    cntdown <= 5'd0;
    scken <= 1'b0;
    sck <= 1'b0;
    idat <= 8'd0;
    odat <= 8'd0;
    acc16 <= 1'b0;
    acc16s <= 1'b0;
    wb_ack <= 1'b0;
    cs <= 1'b0;
    eot <= 1'b0;
    rd <= 1'b0;
    lun <= 2'd0;
    speed <= 5'd0;
    hispeed <= 1'b1;
    wb_dat <= 16'd0;
  end else begin
    wb_ack <= 1'b0;

    if (speed != 5'd0) hispeed <= 1'b0;
    else hispeed <= 1'b1;

    case (state)
    4'd0: begin
      if (wb_cyc_i && wb_stb_i && !wb_ack && !wb_adr_i[3]) begin
        wb_ack <= 1'b1;
        if (!wb_adr_i[1]) begin
          if (wb_we_i && wb_sel_i[1]) begin
            lun <= wb_dat_i[9:8];
            if (lospeed_opt) begin
              speed[3:0] <= wb_dat_i[13:10];
              sck <= wb_dat_i[14];
            end
          end
          if (wb_we_i && wb_sel_i[0]) begin
            if (lospeed_opt) speed[4] <= wb_dat_i[0];
            cs <= wb_dat_i[7];
          end
          wb_dat[0] <= speed[4];
          wb_dat[7] <= cs;
          wb_dat[9:8] <= lun;
          wb_dat[13:10] <= speed[3:0];
          wb_dat[14] <= sck;
          wb_dat[15] <= si_i[lun];
        end
      end if (wb_cyc_i && wb_stb_i && !wb_ack && wb_adr_i[3]) begin
        odat <= wb_dat_i[7:0];
        if (wb_sel_i[1]) begin
          acc16 <= 1'b1;
          odat <= wb_dat_i[15:8];
        end else begin
          acc16 <= 1'b0;
          odat <= wb_dat_i[7:0];
          if (wb_we_i) wb_ack <= 1'b1;
        end
        scken <= 1'b1;
        cntdown <= speed;
        if (cs) sck <= !sck;
        if (hispeed && hispeed_opt) state <= 4'd1;
        else state <= 4'd4;
        count <= 4'd7;
        cs <= 1'b1;
        eot <= wb_adr_i[2];
        rd <= !wb_we_i;
      end
    end


    4'd1: begin /* shift */
      odat <= {odat[6:0], 1'bx};
      idat <= {idat[6:0], si_i[lun]};
      count <= count - 1'b1;
      if (count == 5'd0) begin
        acc16 <= 1'b0;
        if (acc16) begin
          wb_dat[15:8] <= {idat[6:0], si_i[lun]};
          odat <= wb_dat_i[7:0];
          if (!rd) wb_ack <= 1'b1;
          count <= 4'd7;
        end else begin
          wb_dat[7:0] <= {idat[6:0], si_i[lun]};
          if (rd && wb_cyc_i && wb_stb_i) wb_ack <= 1'b1;
          if (rd && wb_adr_i[1]) begin
            acc16 <= 1'b1;
            count <= 5'd7;
          end else begin 
            scken <= 1'b0;
            if (eot | wb_adr_i[2]) cs <= 1'b0;
            state <= 4'd0;
          end
          if (!(wb_cyc_i && wb_stb_i) && rd) begin
            state <= 4'd2; /* holding pattern */
            scken <= 1'b0;
          end
        end
      end
    end
    4'd2: begin
      if (wb_cyc_i && wb_stb_i) begin
        wb_ack <= 1'b1;
        if (wb_adr_i[1]) begin
          acc16 <= 1'b1;
          count <= 5'd7;
          state <= 4'd1;
          scken <= 1'b1;
        end else begin
          state <= 4'd0;
          if (wb_adr_i[2]) cs <= 1'b0;
        end
      end
    end


    4'd3: begin
      cntdown <= cntdown - 1'b1;
      if (cntdown == 5'd1) begin
        odat <= {odat[6:0], 1'bx};
        sck <= !sck;
        acc16s <= 1'b0;
        if (acc16s) begin
          odat <= wb_dat_i[7:0];
          if (!rd) wb_ack <= 1'b1;
        end
        state <= 4'd4;
        cntdown <= speed;   
      end
    end
    4'd4: begin
      cntdown <= cntdown - 1'b1;
      if (cntdown == 5'd1) begin
        sck <= !sck;
        idat <= {idat[6:0], si_i[lun]};
        count <= count - 1'b1;
        cntdown <= speed;
        state <= 4'd3;
        if (count == 5'd0) begin
          acc16 <= 1'b0;
          if (acc16) begin
            wb_dat[15:8] <= {idat[6:0], si_i[lun]};
            acc16s <= 1'b1;
            count <= 5'd7;
          end else begin
            wb_dat[7:0] <= {idat[6:0], si_i[lun]};
            if (rd && wb_cyc_i && wb_stb_i) wb_ack <= 1'b1;
            if (rd && wb_adr_i[1]) begin
              acc16 <= 1'b1;
              count <= 5'd7;
            end else begin 
              scken <= 1'b0;
              state <= 4'd6;
            end
            if (!(wb_cyc_i && wb_stb_i) && rd) begin
              state <= 4'd5; /* holding pattern */
              scken <= 1'b0;
            end
          end
        end
      end
    end
    4'd5: begin
      if (wb_cyc_i && wb_stb_i) begin
        wb_ack <= 1'b1;
        if (wb_adr_i[1]) begin
          acc16 <= 1'b1;
          count <= 5'd7;
          state <= 4'd3;
          scken <= 1'b1;
        end else begin
          eot <= wb_adr_i[2];
          state <= 4'd6;
        end
      end
    end
    4'd6: begin
      cntdown <= cntdown - 1'b1;
      if (cntdown == 5'd1) begin
        cntdown <= speed;
        if (cs & eot) begin
          sck <= !sck;
          state <= 4'd7;
        end else state <= 4'd0;
      end
    end

    4'd7: begin
      cntdown <= cntdown - 1'b1;
      if (cntdown == 5'd1) begin
        cntdown <= speed;
        cs <= 1'b0;
        state <= 4'd6;
      end
    end
    endcase
  end
end


endmodule

`ifdef TESTBENCH
module test();

reg clk, rst, cyc, stb, we;
wire ack, so, sck, csn, scken;
reg si;
reg [1:0] sel;
reg [15:0] idat, adr;
wire [15:0] odat;
wb_spi uut(
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

  .sck_o(sck),
  .scken_o(scken),
  .so_o(so),
  .si_i(si),
  .csn_o(csn)
);

always @(posedge sckps) begin
  si <= !si;
end

task wb_read8;
  input [15:0] address_i;
  integer i;
  reg [7:0] clocks;
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
  cyc = 1'b0; stb = 1'b0;
  $display("clocks %d", clocks);
end
endtask

task wb_read16;
  input [15:0] address_i;
  integer i;
  reg [7:0] clocks;
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
  cyc = 1'b0; stb = 1'b0;
  $display("clocks %d", clocks);
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
  $display("clocks %d", clocks);
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
  $display("clocks %d", clocks);
end
endtask

always #5 clk = !clk;
reg clkps;
always @(clk) clkps = #2.5 !clk; /* 270 deg phase shift */
wire sckps = clkps | !scken;

initial begin
  $dumpfile("dump.vcd");
  $dumpvars(2, test);
  rst = 1; clk = 0; si = 0;
  cyc = 0; stb = 0; we = 0; adr = 0; sel = 0; idat = 0; 
  #100;
  rst = 0;
  #10;
  @(posedge clk);
  // wb_write16(16'd0, 16'haaaa);
  // wb_write16(16'd0, 16'h5555);
  wb_write8(16'd0, 16'hf9);
  #200;
  @(posedge clk);
  wb_read16(16'd0);
  @(posedge clk);
  wb_read8(16'd4);

  #1000;
  $finish;
end

endmodule
`endif
