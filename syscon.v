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

/* Miscellaneous registers for the TS-7500.  Features:
 *   - Watchdog
 *   - GPIO in/out/direction
 *   - LEDs
 *   - RTC I2C interface
 *   - Lattice tagmem access
 *   - Random number generator 
 *   - SPI PLL phase control for SPI flash interface
 *   - Reset switch (from TS-752) enable
 *   - Latched mode/jumper pins for bootup medium determination
 *   - scratch reg for communicating bootup device to OS from BOOTROM
 *   - Model reg (16 bit), submodel reg (4-bit), and FPGA revision (4-bit)
 *
 * In the 128-byte SBUS address space, this core appears starting at
 * address 0x60 on the TS-7500. 
 *
 * Register map:
 * base + 0x0: Model ID reg (RO) - reads 0x7500 on the TS-7500
 * base + 0x2: submodel, fpga revision, RTC and LED control - (R/W)
 *   bit 15: green LED (1 - on)
 *   bit 14: red LED (1 - on)
 *   bit 13: RTC SCL input
 *   bit 12: RTC SDA input
 *   bit 11: RTC SCL direction (1 - output)
 *   bit 10: RTC SDA direction (1 - output)
 *   bit 9: RTC SCL output
 *   bit 8: RTC SDA output
 *   bit 7-4: Board submodel (RO) - 0x0 on production TS-7500
 *   bit 3-0: FPGA revision
 * base + 0x4: 16-bits of random data changed every 1 second. (RO)
 * base + 0x6: DIO and tagmem control (RW)
 *   bit 15-12: DIO input for pins 40(MSB)-37(LSB) (RO)
 *   bit 11-8: DIO output for pins 40(MSB)-37(LSB) (RW)
 *   bit 7-4: DIO direction for pins 40(MSB)-37(LSB) (1 - output) (RW)
 *   bit 3: Lattice tagmem clock (RW)
 *   bit 2: Lattice tagmem serial-in (RW)
 *   bit 1: Lattice tagmem CSn (RW)
 *   bit 0: Lattice tagmem serial-out (RO)
 * base + 0x8: DIO input for pins 36(MSB)-21(LSB) (RO)  
 * base + 0xa: DIO output for pins 36(MSB)-21(LSB) (RW)
 * base + 0xc: DIO direction for pins 36(MSB)-21(LSB) (1 - output) (RW)
 * base + 0xe: DIO input for pins 20(MSB)-5(LSB) (RO)  
 * base + 0x10: DIO output for pins 20(MSB)-5(LSB) (RW)
 * base + 0x12: DIO direction for pins 20(MSB)-5(LSB) (1 - output) (RW)
 * base + 0x14: Watchdog feed register (write only)
 *   write value 0x0: feed watchdog for another .338s (approx)
 *   write value 0x1: feed watchdog for another 2.706s (approx)
 *   write value 0x2: feed watchdog for another 10.824 (approx)
 *   write value 0x3: disable watchdog
 * base + 0x16: SPI PLL phase, latched mode bits, scratch reg
 *   bit 15-11: reserved
 *   bit 10-6: PLL phase (set by TS-BOOTROM) (RW)
 *   bit 5: mode3 latched bootstrap bit (RO)
 *   bit 4: Reset switch enable (1 - auto reboot when dio_i[9] == 0) (RW)
 *   bit 3-2: scratch reg (RW)
 *   bit 1: mode2 latched bootstrap bit (RO)
 *   bit 0: mode1 latched bootstrap bit (RO)
 *
 * * Not all DIO pins come out to the TS-7500 header under FPGA control.
 *   Consult schematic for more details -- some are power/grounds or come
 *   from the CPU.  Attempting to write or read values from these pins
 *   will have no effect.
 *
 * * Watchdog by default comes out of reset armed for .338 seconds.  
 *   TS-BOOTROM firmwarm feeds for 10.824 and OS code has 10.824 seconds
 *   to take over.
 *
 * Revision history:
 * 0 - Initial release
 * 1 - Added 5-bit PLL phase shift register for SPI.  (Needed for +80C)
 * 2 - Improved GPIO performance by putting the syscon block on its own
 *     clock domain synchronous to the SPI clock.  Also updated SPI controller.
 * 3 - XUART channel #0 RX line was hooked up incorrectly.
 * 4 - Added improved SPI controller and extra CS2# and CS3#
 *     optional CAN controller added
 *     moved to 16Kbyte bootrom
 *     fixed IRQ output
 * 5 - Updated SPI controller with extra slower speeds (1.2Mhz) and CS#
 *     bit in control reg.  Fixed int28 to always be high (fixes USB 
 *     slave issue)
 * 6 - New CAN synchronizer and sbus synchronizer.
 *
 */
module syscon(
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

  dio_i,
  dio_oe_o,
  dio_o,

  rtc_sda_o,
  rtc_sda_i,
  rtc_sda_oe_o,
  rtc_scl_o,
  rtc_scl_i,
  rtc_scl_oe_o,

  cpu_uart_rxd_o,
  cpu_uart_txd_i,

  internal_osc_o,
  clk_100mhz_i,
  reboot_o,
  led_grn_o,
  led_red_o,
  mode1_i,
  mode2_i,
  mode3_i,
  pllphase_o,
  can_enable_o,
  can_wbaccess_i
);

input wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
input [31:0] wb_adr_i;
input [15:0] wb_dat_i;
input [1:0] wb_sel_i;
output [15:0] wb_dat_o;
output wb_ack_o;

input [40:0] dio_i;
output [40:0] dio_oe_o;
output [40:0] dio_o;

input rtc_sda_i, rtc_scl_i;
output rtc_sda_o, rtc_sda_oe_o, rtc_scl_o, rtc_scl_oe_o;
output led_grn_o, led_red_o;
output internal_osc_o, reboot_o;
input clk_100mhz_i, mode1_i, mode2_i, mode3_i;

input cpu_uart_txd_i;
output cpu_uart_rxd_o;
output [4:0] pllphase_o;
output can_enable_o;
input can_wbaccess_i;

parameter wdog_default = 3;
parameter can_opt = 0;

localparam model = 16'h7500;
localparam submodel = 4'h0;
localparam revision = 4'h6;
localparam dio_mask = 41'h1fffff87be0;

wire random_clk;
OSCE #(.NOM_FREQ("3.1")) rndosc (random_clk);
assign internal_osc_o = random_clk;
reg [15:0] rndcnt;
reg [1:0] wdogctl;
reg [9:0] wdogcnt;
reg feed_en, reboot, shift;
wire sed_err;
reg resetsw_en;
reg [1:0] resetsw;
wire feed_bsy, feed_rdreq;
wire [15:0] wdog_dat;
assign reboot_o = reboot | (resetsw_en && resetsw == 2'b00) /*| sed_err */;
always @(posedge random_clk or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    rndcnt <= 16'd0;
    shift <= 1'b0;
    wdogctl <= wdog_default;
    wdogcnt <= 10'd0;
    resetsw <= 2'b11;
  end else begin
    shift <= 1'b0;
    rndcnt <= rndcnt + 1'b1;
    if (rndcnt == 16'd0) begin
      shift <= 1'b1;
      resetsw[0] <= dio_i[9];
      resetsw[1] <= resetsw[0] | dio_i[9];
      wdogcnt <= wdogcnt + 1'b1;
    end
    if (feed_rdreq) begin
      wdogctl <= wdog_dat;
      wdogcnt <= 10'd0;
    end
  end
end
always @(*) begin
  case (wdogctl)
  2'd0: reboot = wdogcnt[4]; /* approx .338 seconds */
  2'd1: reboot = wdogcnt[7]; /* approx 2.706 seconds */
  2'd2: reboot = wdogcnt[9]; /* approx 10.824 seconds */
  2'd3: reboot = 1'b0;
  endcase
end

wire [15:0] random;
crc randnumgen(
  .dat_i(clk_100mhz_i),
  .clk_i(random_clk),
  .clken_i(shift),
  .rst_i(wb_rst_i),
  .shift_i(1'b0),
  .crc16_o(random)
);
/*
SEDBA sedcore (
  .SEDENABLE(1'b1),
  .SEDSTART(1'b1),
  .SEDFRCERRN(1'b1),
  .SEDERR(sed_err)
);
*/
resync_fifo watchdogfifo(
  .rst_i(wb_rst_i),
  .wrclk_i(wb_clk_i),
  .wrreq_i(feed_en),
  .data_i(wb_dat_i),
  .full_wrclk_o(feed_bsy),
  .rdclk_i(random_clk),
  .rdreq_i(feed_rdreq),
  .q_o(wdog_dat),
  .full_rdclk_o(feed_rdreq)
);

reg [6:0] count;
reg [31:0] usec;
always @(posedge clk_100mhz_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    count <= 7'd0;
    usec <= 32'd0;
  end else begin
    count <= count + 1'b1;
    if (count == 7'd99) begin
      count <= 7'd0;
      usec <= usec + 1'b1;
    end
  end
end

reg spi_csn, spi_clk, spi_si;
wire spi_so;
tagmem tagmemcore(
  .CLK(spi_clk),
  .SI(spi_si),
  .SO(spi_so),
  .CS(spi_csn)
);

reg rtc_sda, rtc_scl, rtc_sda_oe, rtc_scl_oe;
assign rtc_sda_o = rtc_sda;
assign rtc_scl_o = rtc_scl;
assign rtc_sda_oe_o = rtc_sda_oe;
assign rtc_scl_oe_o = rtc_scl_oe;
reg [40:0] dio_oe, dio;
assign dio_oe_o = {dio_oe[40:9], 2'b01, dio_oe[6:0]};
assign dio_o = {dio[40:8], cpu_uart_txd_i, dio[6:0]};
assign cpu_uart_rxd_o = dio_i[8];
reg [15:0] wb_dat;
assign wb_dat_o = wb_dat;
reg led_grn, led_red;
assign led_grn_o = led_grn;
assign led_red_o = led_red;
reg [1:0] scratch;
reg [4:0] pllphase;
assign pllphase_o = pllphase;
reg can_enable;
assign can_enable_o = can_enable;
always @(posedge wb_clk_i or posedge wb_rst_i) begin
  if (wb_rst_i) begin
    dio_oe <= 41'd0;
    dio <= 41'd0;
    rtc_sda <= 1'b0;
    rtc_scl <= 1'b0;
    rtc_sda_oe <= 1'b0;
    rtc_scl_oe <= 1'b0;
    spi_clk <= 1'b0;
    spi_si <= 1'b0;
    spi_csn <= 1'b1;
    led_grn <= 1'b1;
    led_red <= 1'b1;
    scratch <= 2'b00;
    resetsw_en <= 1'b0;
    pllphase <= 5'h13;
    can_enable <= 1'b0;
  end else begin
    if (wb_cyc_i && wb_stb_i && wb_we_i) case (wb_adr_i[4:0])
      5'h2: begin
        {rtc_scl_oe, rtc_sda_oe, rtc_scl, rtc_sda} <= wb_dat_i[11:8];
        {led_grn, led_red} <= wb_dat_i[15:14];
      end
      5'h6: begin
        {dio[40:37], dio_oe[40:37]} <= wb_dat_i[11:4] & {2{dio_mask[40:37]}};
        {spi_clk, spi_si, spi_csn} <= wb_dat_i[3:1];
      end
      5'ha: dio[36:21] <= wb_dat_i[15:0] & dio_mask[36:21];
      5'hc: dio_oe[36:21] <= wb_dat_i[15:0] & dio_mask[36:21];
      5'h10: dio[20:5] <= wb_dat_i[15:0] & dio_mask[20:5];
      5'h12: dio_oe[20:5] <= wb_dat_i[15:0] & dio_mask[20:5];
      5'h16: begin
        if (can_opt) can_enable <= wb_dat_i[11];
        pllphase <= wb_dat_i[10:6];
        scratch <= wb_dat_i[3:2];
        resetsw_en <= wb_dat_i[4];
      end
    endcase
    if (can_opt && can_wbaccess_i) can_enable <= 1'b1;
  end
end

reg wb_ack;
assign wb_ack_o = wb_ack;
always @(*) begin
  wb_ack = wb_cyc_i && wb_stb_i;
  wb_dat = 16'hxxxx;
  feed_en = 1'b0;

  case (wb_adr_i[4:0]) 
  5'h0: wb_dat = model[15:0]; /* Model ID */ 
  5'h2: wb_dat = {led_grn, led_red, rtc_scl_i, rtc_sda_i, rtc_scl_oe,
    rtc_sda_oe, rtc_scl, rtc_sda, submodel[3:0], revision[3:0]};
  5'h4: wb_dat = random[15:0];
  5'h6: wb_dat = {dio_i[40:37], dio[40:37], dio_oe[40:37], spi_clk, 
    spi_si, spi_csn, spi_so};
  5'h8: wb_dat = dio_i[36:21];
  5'ha: wb_dat = dio[36:21];
  5'hc: wb_dat = dio_oe[36:21];
  5'he: wb_dat = dio_i[20:5];
  5'h10: wb_dat = dio[20:5];
  5'h12: wb_dat = dio_oe[20:5];
  5'h14: begin
    feed_en = wb_cyc_i & wb_stb_i & wb_we_i & !feed_bsy;
    wb_ack = wb_cyc_i & wb_stb_i & ((wb_we_i & !feed_bsy) | !wb_we_i);
  end
  5'h16: wb_dat = {{4{1'bx}}, can_enable, pllphase, mode3_i, resetsw_en,
    scratch, mode2_i, mode1_i};
  endcase
end

endmodule

