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

/* The TS-7500 was released Jul. 2009 and is comprised of a Cavium
 * CNS2132 250Mhz ARM9 CPU, Lattice XP2 5k FPGA, 64MB DDR SDRAM, and
 * 4MByte SPI NOR flash chip.  
 *
 * FPGA functions include:
 *   - micro SD slot 
 *   - SPI controller for 4MByte SPI flash chips on TS-7500 and TS-752.
 *   - DIO pins on the 40 pin header
 *   - Processor bootstrap + TS-BOOTROM firmware store
 *   - I2C RTC interface
 *   - watchdog
 *   - 8 XUART channels
 *   - storage of board MAC address and CPU speed mode in Lattice tagmem.
 *   - miscellaneous glue logic
 *   - optional CAN controller
 *   
 * CPU bootstrap is accomplished by the FPGA emulating a SPI flash
 * chip.  TS-BOOTROM resides in a 16KByte initialized blockram in the
 * FPGA.
 *
 * WISHBONE bridge interface is accomplished with a custom SPI protocol
 * "SBUS" and has provision currently for 128 bytes of registers.
 *
 */

module ts7500_top( 
  cpu_clkout_pad,
  fpga_25mhz_pad,
  clk_32khz_pad,

  dio_pad,

  cpu_uart_txd_pad,
  cpu_uart_rxd_pad,

  /* spi interface */
  spi_clk_pad,
  spi_mosi_pad,
  spi_miso_pad,

  gpio_a0_pad,
  gpio_a1_pad,

  gpio_a13_pad,
  gpio_a14_pad,

  gpio_a3_pad,
  gpio_a15_pad,
  gpio_a16_pad,
  gpio_a17_pad,

  gpio_a28_pad,
  gpio_a29_pad,

  int28_pad,

  gpio_a22_pad,
  gpio_a23_pad,

  /* reset pins */
  un_reset_pad,
  wd_resetn_pad,

  /* sd interface */
  en_sd_power_pad,
  sd_d0_pad,
  sd_d1_pad,
  sd_d2_pad,
  sd_d3_pad,
  sd_cmd_pad,
  sd_clk_pad,

  /* led control pins */
  red_led_pad,
  green_led_pad,

  /* ethernet led control pins */
  eth_left_ledn_pad,
  eth_right_ledn_pad,

  /* rtc pins */
  rtc_sda_pad,
  rtc_scl_pad,
  rtc_int1_pad,

  /* serial flash pins */
  ser_flash_wp_padn,
  ser_flash_cs_padn,
  flash_clk_pad,
  flash_mosi_pad,
  flash_miso_pad

);

/* global signals */
input fpga_25mhz_pad;
input cpu_clkout_pad;

inout [40:0] dio_pad;

input cpu_uart_txd_pad;
output cpu_uart_rxd_pad;
output clk_32khz_pad;

/* spi interface */
input spi_clk_pad;
input spi_mosi_pad;
inout spi_miso_pad;

/* interrupts */
output gpio_a0_pad;
output gpio_a1_pad;

/* i2c */
inout gpio_a13_pad;
inout gpio_a14_pad;

/* i2s */
inout gpio_a3_pad;
inout gpio_a15_pad;
inout gpio_a16_pad;
inout gpio_a17_pad;

inout gpio_a28_pad;  //SPI_CS0
inout gpio_a29_pad;  //SPI_CS1

inout int28_pad;

inout gpio_a22_pad;
inout gpio_a23_pad;

/* reset pins */
output un_reset_pad;
inout wd_resetn_pad;

/* sd interface */
output en_sd_power_pad;
inout sd_d0_pad;
inout sd_d1_pad;
inout sd_d2_pad;
inout sd_d3_pad;
inout sd_cmd_pad;
output sd_clk_pad /* synthesis syn_maxfan=1 syn_useioff=1 */;

/* serial flash pins */
output ser_flash_wp_padn;
output ser_flash_cs_padn;
output flash_mosi_pad;
output flash_clk_pad;
input flash_miso_pad;

/* led control pins */
output red_led_pad;
output green_led_pad;

/* ethernet led control pins */
output eth_left_ledn_pad;
output eth_right_ledn_pad;

/* rtc pins */
inout rtc_sda_pad;
output rtc_scl_pad;
input rtc_int1_pad;

assign gpio_a13_pad = 1'bz;
assign gpio_a14_pad = 1'bz;
assign gpio_a3_pad = 1'bz;
assign gpio_a15_pad = 1'bz;
assign gpio_a16_pad = 1'bz;
assign gpio_a17_pad = 1'bz;
assign gpio_a28_pad = 1'bz;
assign gpio_a29_pad = 1'bz;
assign gpio_a22_pad = 1'bz;
assign gpio_a23_pad = 1'bz;
assign eth_left_ledn_pad = gpio_a22_pad;
assign eth_right_ledn_pad = gpio_a23_pad;

parameter sdcard_opt = 1'b1;
parameter can_opt = 1'b0;
parameter spi_opt = 1'b1;
/* software currently requires these to be enabled/disabled contiguously. */
parameter xuart0_opt = 1'b1;
parameter xuart1_opt = 1'b1;
parameter xuart2_opt = 1'b1;
parameter xuart3_opt = 1'b1;
parameter xuart4_opt = 1'b1;
parameter xuart5_opt = 1'b1;
parameter xuart6_opt = 1'b1;
parameter xuart7_opt = 1'b1;

/****************************************************************************
 * Boilerplate FPGA configuration (clocks, PLL, DLL, reset) 
 ****************************************************************************/
wire pll_75mhz, pll_75mhz_shifted, pll_100mhz;
wire lock, lock2;
wire [4:0] pllphase;
pll clkgen (
  .CLK(fpga_25mhz_pad),
  .CLKOP(pll_75mhz),
  .CLKOS(pll_75mhz_shifted),
  .LOCK(lock),
  .DPHASE3(pllphase[4]),
  .DPHASE2(pllphase[3]),
  .DPHASE1(pllphase[2]),
  .DPHASE0(pllphase[1]),
  .WRDEL(pllphase[0])
);
pll2 clkgen2 (
  .CLK(fpga_25mhz_pad),
  .CLKOP(pll_100mhz),
  .LOCK(lock2)
);
wire internal_osc;
reg [15:0] rst_counter = 16'd0;
reg rstn = 1'b0;
reg lock_q;
always @(posedge pll_75mhz) begin
  lock_q <= lock && lock2;
  if (lock_q) rst_counter <= rst_counter + 1'b1;
  else begin
    rst_counter <= 16'd0;
    rstn <= 1'b0;
  end
  if (rst_counter[15]) rstn <= 1'b1;
end
wire rst = !rstn;
wire reboot;
assign wd_resetn_pad = !reboot;
reg [31:0] timer;
reg unreset;
reg spi_boot_done;
wire sbus_enabled;
wire can_irqn, xuart_irq;
assign gpio_a0_pad = xuart_irq;
assign gpio_a1_pad = (!can_irqn && can_opt);
wire [4:0] gpio_a17to13 = {gpio_a17_pad, gpio_a16_pad, gpio_a15_pad,
  gpio_a14_pad, gpio_a13_pad};
assign un_reset_pad = unreset;
wire wb_clk;
reg mode1, mode2, mode3;
always @(posedge pll_75mhz or posedge rst) begin
  if (rst) begin
    timer <= 32'd0;
    unreset <= 1'b0;
    spi_boot_done <= 1'b0;
    mode1 <= 1'b1;
    mode2 <= 1'b1;
    mode3 <= 1'b1;
  end else begin
    timer <= timer + 1'b1;
    if (timer >= 32'd64) begin
      unreset <= 1'b1;
      if (!unreset) begin
        mode1 <= dio_pad[7];
        mode2 <= dio_pad[5];
        mode3 <= gpio_a22_pad;
      end
    end
    if (unreset && gpio_a17to13[4:2] == 3'b110)
      spi_boot_done <= 1'b1;
  end
end
assign ser_flash_wp_padn = unreset;
assign clk_32khz_pad = 1'b0;
reg [4:0] seq_24mhz;
reg clk_24mhz;
wire can_enable;
always @(posedge pll_100mhz or posedge rst) begin
  if (rst) begin
    clk_24mhz <= 1'b1;
    seq_24mhz <= 5'd0;
  end else if (can_enable) begin
    seq_24mhz <= seq_24mhz + 1'b1;
    case (seq_24mhz)
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21: clk_24mhz <= !clk_24mhz;
    24: begin
      clk_24mhz <= !clk_24mhz;
      seq_24mhz <= 5'd0;
    end
    endcase
  end
end
wire led_red;
assign red_led_pad = !led_red;
wire led_grn;
assign green_led_pad = !led_grn;
assign wb_rst = rst;
wire [7:0] xuart_disable = {!xuart7_opt[0], !xuart6_opt[0], !xuart5_opt[0],
  !xuart4_opt[0], !xuart3_opt[0], !xuart2_opt[0], !xuart1_opt[0],
  !xuart0_opt[0]};
wire xuart_opt = (xuart_disable[7:0] != 8'hff);


/****************************************************************************
 * XUART blockram 
 ****************************************************************************/
wire spiboot_so;
wire spi_cs0 = gpio_a28_pad;
wire ramwbs1_cyc_i, ramwbs1_stb_i, ramwbs1_we_i, ramwbs1_ack_o;
wire [31:0] ramwbs1_dat_i, ramwbs1_dat_o;
wire [31:0] ramwbs1_adr_i;
wire ramwbs2_cyc_i, ramwbs2_stb_i, ramwbs2_we_i, ramwbs2_ack_o;
wire [31:0] ramwbs2_dat_i, ramwbs2_dat_o;
wire [31:0] ramwbs2_adr_i;
wire [3:0] ramwbs2_sel_i, ramwbs1_sel_i;
assign sbus_enabled = 1'b1;
assign dcs_clk = pll_75mhz;
blockram_8kbyte sramcore(
  .wb_clk_i(dcs_clk && xuart_opt),
  .wb_rst_i(rst),

  /* To XUART */
  .wb1_cyc_i(ramwbs1_cyc_i),
  .wb1_stb_i(ramwbs1_stb_i),
  .wb1_we_i(ramwbs1_we_i),
  .wb1_dat_i(ramwbs1_dat_i),
  .wb1_dat_o(ramwbs1_dat_o),
  .wb1_adr_i(ramwbs1_adr_i),
  .wb1_ack_o(ramwbs1_ack_o),
  .wb1_sel_i(ramwbs1_sel_i),

  /* To wb_memwindow */
  .wb2_cyc_i(ramwbs2_cyc_i),
  .wb2_stb_i(ramwbs2_stb_i),
  .wb2_we_i(ramwbs2_we_i),
  .wb2_dat_i(ramwbs2_dat_i),
  .wb2_dat_o(ramwbs2_dat_o),
  .wb2_adr_i(ramwbs2_adr_i),
  .wb2_ack_o(ramwbs2_ack_o),
  .wb2_sel_i(ramwbs2_sel_i)
);


/****************************************************************************
 * SBUS protocol core (SPI -> WISHBONE bridge)
 ****************************************************************************/
wire sbus_so;
wire spiwbm_cyc_o, spiwbm_stb_o, spiwbm_we_o;
wire [6:0] spiwbm_adr_o;
wire [15:0] spiwbm_dat_o;
reg spiwbm_ack_i;
reg [15:0] spiwbm_dat_i;
wire [1:0] spiwbm_sel_o;
wire spiwbm2_cyc_o, spiwbm2_stb_o, spiwbm2_we_o;
wire [6:0] spiwbm2_adr_o;
wire [15:0] spiwbm2_dat_o;
reg spiwbm2_ack_i;
reg [15:0] spiwbm2_dat_i;
wire [1:0] spiwbm2_sel_o;
reg spiwbm2_en;
wire spiwbm2_clk;
wire [3:0] spiwbm32_sel_o = 
  spiwbm_adr_o[1] ? {spiwbm_sel_o, 2'b00} : {2'b00, spiwbm_sel_o};
assign wb_clk = dcs_clk;
/* Note: Cavium SPI interface will actually work at 62.5Mhz even
 * though its only spec'ed to 50Mhz! 
 */
spi_sbus_resync sbuscore(
  .wbm_clk_i(dcs_clk),
  .wbm_adr_o(spiwbm_adr_o),
  .wbm_dat_i(spiwbm_dat_i),
  .wbm_ack_i(spiwbm_ack_i),
  .wbm_cyc_o(spiwbm_cyc_o),
  .wbm_stb_o(spiwbm_stb_o),
  .wbm_we_o(spiwbm_we_o),
  .wbm_dat_o(spiwbm_dat_o),
  .wbm_sel_o(spiwbm_sel_o),

  .wbm2_clk_o(spiwbm2_clk),
  .wbm2_en_i(spiwbm2_en),
  .wbm2_adr_o(spiwbm2_adr_o),
  .wbm2_dat_i(spiwbm2_dat_i),
  .wbm2_ack_i(spiwbm2_ack_i),
  .wbm2_cyc_o(spiwbm2_cyc_o),
  .wbm2_stb_o(spiwbm2_stb_o),
  .wbm2_we_o(spiwbm2_we_o),
  .wbm2_dat_o(spiwbm2_dat_o),
  .wbm2_sel_o(spiwbm2_sel_o),

  .upper_adr_i({gpio_a16_pad, gpio_a15_pad}),
  .rst_i(wb_rst),
  .csn_i(spi_cs0 || !sbus_enabled),
  .sck_i(spi_clk_pad),
  .si_i(spi_mosi_pad),
  .so_o(sbus_so),
  .sel_i({gpio_a17_pad, gpio_a3_pad})

);
assign spi_miso_pad = sbus_so;


/****************************************************************************
 * SD card logic
 ****************************************************************************/
reg sdwbs1_en;
wire [31:0] sdwbs1_dat;
wire [15:0] sdwbs1_dat_o = 
  spiwbm_adr_o[1] ? sdwbs1_dat[31:16] : sdwbs1_dat[15:0];
wire sdwbs1_ack_o;
wire sd_cmd, sd_cmd_oe;
wire [3:0] sd_dat, sd_dat_oe;
assign sd_cmd_pad = sd_cmd_oe ? sd_cmd : 1'bz;
assign sd_d0_pad = sd_dat_oe[0] ? sd_dat[0] : 1'bz;
assign sd_d1_pad = sd_dat_oe[1] ? sd_dat[1] : 1'bz;
assign sd_d2_pad = sd_dat_oe[2] ? sd_dat[2] : 1'bz;
assign sd_d3_pad = sd_dat_oe[3] ? sd_dat[3] : 1'bz;
wire [7:0] sd_clk;
wire sd_busy, sd_power;
assign en_sd_power_pad = unreset ? sd_power : 1'b1;
ts_sdcore sdcardcore(
  .wb_rst_i(wb_rst),
  .wb_clk_i(wb_clk & sdcard_opt),

  .wb1_adr_i(spiwbm_adr_o),
  .wb1_cyc_i(spiwbm_cyc_o && sdwbs1_en),
  .wb1_stb_i(spiwbm_stb_o && sdwbs1_en),
  .wb1_dat_o(sdwbs1_dat),
  .wb1_we_i(spiwbm_we_o),
  .wb1_dat_i({spiwbm_dat_o, spiwbm_dat_o}),
  .wb1_ack_o(sdwbs1_ack_o),
  .wb1_sel_i(spiwbm32_sel_o),
  .wb1_bte_i(2'b00),
  .wb1_cti_i(3'b000),

  .sd_cmd_i(sd_cmd_pad),
  .sd_cmd_o(sd_cmd),
  .sd_cmd_oe_o(sd_cmd_oe),
  .sd_dat_i({sd_d3_pad, sd_d2_pad, sd_d1_pad, sd_d0_pad}),
  .sd_dat_o(sd_dat),
  .sd_dat_oe_o(sd_dat_oe),
  .sd_clk_o(sd_clk),
 
  .sd_wprot_i(8'b00000000),
  .sd_detect_i(8'b00000001),
  .sd_power_o(sd_power),
  .sd_busy_o(sd_busy)
);
assign sd_clk_pad = sd_clk[0];


/****************************************************************************
 * SPI controller logic
 ****************************************************************************/
reg spiwbs_en;
wire [15:0] spiwbs_dat_o;
wire spiwbs_ack_o;
wire scken;
wire [3:0] csn;
reg [3:0] csn_q;
always @(posedge wb_clk) csn_q <= csn;
assign ser_flash_cs_padn = csn[0];
wire [3:0] si = {{3{dio_pad[12]}}, flash_miso_pad};
wire sck, hispeed;
assign flash_clk_pad = hispeed ? (pll_75mhz_shifted | !scken) : sck;
wire so;
assign flash_mosi_pad = so;
wb_spi spicore(
  .wb_rst_i(wb_rst),
  .wb_clk_i(wb_clk & spi_opt),
  .wb_cyc_i(spiwbm_cyc_o && spiwbs_en),
  .wb_stb_i(spiwbm_stb_o && spiwbs_en),
  .wb_dat_o(spiwbs_dat_o),
  .wb_dat_i(spiwbm_dat_o),
  .wb_adr_i(spiwbm_adr_o),
  .wb_we_i(spiwbm_we_o),
  .wb_ack_o(spiwbs_ack_o),
  .wb_sel_i(spiwbm_sel_o),

  .scken_o(scken),
  .so_o(so),
  .si_i(si),
  .csn_o(csn),
  .sck_o(sck),
  .hispeed_o(hispeed)
);


/****************************************************************************
 * TS-XUART super uarts
 ****************************************************************************/
wire xuartwbs_ack_o, xuartwbs_ack;
reg xuwbs_en;
wire [31:0] uartwbm_adr_o;
wire [15:0] uartwbm_dat_o, xuartwbs_dat_o;
wire [15:0] uartwbm_dat_i;
reg [7:0] xuart_rx;
wire [7:0] xuart_active;
wire [7:0] xuart_cts = 8'hff;
wire [7:0] xuart_txen, xuart_tx;
reg [7:0] xuart_rx_q, xuart_cts_q;
always @(posedge wb_clk) begin
  xuart_rx_q <= xuart_rx;
  xuart_cts_q <= xuart_cts;
end
assign ramwbs1_dat_i[31:0] = {uartwbm_dat_o[15:0], uartwbm_dat_o[15:0]}; 
assign ramwbs1_adr_i = {uartwbm_adr_o[15:2], 2'b00};
assign uartwbm_dat_i[15:0] = 
  uartwbm_adr_o[1] ? ramwbs1_dat_o[31:16] : ramwbs1_dat_o[15:0];
assign ramwbs1_sel_i = uartwbm_adr_o[1] ? 4'b1100 : 4'b0011;
assign xuartwbs_ack_o = xuartwbs_ack | ~xuart_opt;
ts_xuart tsxuartcore(
  .wb_clk_i(wb_clk & xuart_opt),
  .wb_rst_i(wb_rst),

  .wbm_cyc_o(ramwbs1_cyc_i),
  .wbm_stb_o(ramwbs1_stb_i),
  .wbm_we_o(ramwbs1_we_i),
  .wbm_ack_i(ramwbs1_ack_o),
  .wbm_adr_o(uartwbm_adr_o),
  .wbm_dat_i(uartwbm_dat_i),
  .wbm_dat_o(uartwbm_dat_o),


  .wbs_cyc_i(spiwbm_cyc_o && xuwbs_en),
  .wbs_stb_i(spiwbm_stb_o && xuwbs_en),
  .wbs_we_i(spiwbm_we_o),
  .wbs_adr_i(spiwbm_adr_o),
  .wbs_dat_i(spiwbm_dat_o),
  .wbs_dat_o(xuartwbs_dat_o),
  .wbs_ack_o(xuartwbs_ack_o),

  .baudclk_i(pll_100mhz),
  .cts_i(xuart_cts_q),
  .rx_i(xuart_rx_q),
  .tx_o(xuart_tx),
  .txen_o(xuart_txen),
  .active_o(xuart_active),
  .irq_o(xuart_irq),
  .uart_disable_i(xuart_disable)
);


/****************************************************************************
 * Syscon (misc control registers)
 ****************************************************************************/
reg scwbs_en;
wire [15:0] scwbs_dat_o;
wire scwbs_ack_o;
wire rtc_sda, rtc_scl, rtc_sda_oe, rtc_scl_oe;
assign rtc_sda_pad = rtc_sda_oe ? rtc_sda : 1'bz;
assign rtc_scl_pad = rtc_scl_oe ? rtc_scl : 1'bz;
wire [40:0] dio, dio_oe;
integer i;
wire can_tx, can_wbaccess;
reg [40:0] dio_reg;
assign dio_pad = dio_reg;
always @(*) begin
  for (i = 0; i <= 40; i = i + 1) begin
    dio_reg[i] = dio_oe[i] ? dio[i] : 1'bz;
  end

  /* DIO#7 is one of our latched bootstrap pins */
  if (!unreset) dio_reg[7] = 1'bz;

  /* XUARTS take over DIO pins when they are activiated */
  xuart_rx[0] = dio_pad[6];
  if (xuart_active[0]) begin
    dio_reg[5] = xuart_tx[0];
    dio_reg[6] = 1'bz;
  end
  xuart_rx[1] = dio_pad[20];
  if (xuart_active[1]) begin
    dio_reg[19] = xuart_tx[1];
    dio_reg[20] = 1'bz;
    dio_reg[27] = xuart_txen[1];
  end
  xuart_rx[2] = dio_pad[22];
  if (xuart_active[2]) begin
    dio_reg[21] = xuart_tx[2];
    dio_reg[22] = 1'bz;
    dio_reg[28] = xuart_txen[2];
  end
  xuart_rx[3] = dio_pad[24];
  if (can_opt && can_enable) begin
    dio_reg[23] = can_tx;
    dio_reg[24] = 1'bz;
  end else if (xuart_active[3]) begin
    dio_reg[23] = xuart_tx[3];
    dio_reg[24] = 1'bz;
  end
  xuart_rx[4] = dio_pad[26];
  if (xuart_active[4]) begin
    dio_reg[25] = xuart_tx[4];
    dio_reg[26] = 1'bz;
  end
  xuart_rx[5] = dio_pad[32];
  if (xuart_active[5]) begin
    dio_reg[31] = xuart_tx[5];
    dio_reg[32] = 1'bz;
    dio_reg[29] = xuart_txen[5];
  end
  xuart_rx[6] = dio_pad[34];
  if (xuart_active[6]) begin
    dio_reg[33] = xuart_tx[6];
    dio_reg[34] = 1'bz;
    dio_reg[30] = xuart_txen[6];
  end
  /*xuart_rx[7] = dio_pad[36];
  if (xuart_active[7]) begin
    dio_reg[35] = xuart_tx[7];
    dio_reg[36] = 1'bz;
  end*/

  /* SPI controller hijacks some pins when LUN#1, #2, or #3 is active */
  if (spi_opt) begin
    if (csn_q[1] == 1'b0) dio_reg[11] = 1'b1;
    if (csn_q[2] == 1'b0) dio_reg[37] = 1'b1;
    if (csn_q[3] == 1'b0) dio_reg[39] = 1'b1;
    if (csn[1] == 1'b0) dio_reg[11] = csn[1];
    if (csn[2] == 1'b0) dio_reg[37] = csn[2];
    if (csn[3] == 1'b0) dio_reg[39] = csn[3];
    if (csn[3:1] != 3'b111) begin
      dio_reg[14] = hispeed ? (pll_75mhz_shifted | !scken) : sck;
      dio_reg[13] = so;
      dio_reg[12] = 1'bz;
    end
  end
  
  dio_reg[36] = tmp_o;
  dio_reg[37] = pwm_gen_out1;
  dio_reg[38] = pwm_gen_out2;
  dio_reg[39] = pwm_gen_out3;
  dio_reg[40] = pwm_gen_out4;

end

syscon #(
  .wdog_default(3),
  .can_opt(can_opt)
) sysconcore(
  .wb_clk_i(spiwbm2_clk),
  .wb_rst_i(wb_rst),

  .wb_cyc_i(spiwbm2_cyc_o && scwbs_en),
  .wb_stb_i(spiwbm2_stb_o && scwbs_en),
  .wb_we_i(spiwbm2_we_o),
  .wb_adr_i(spiwbm2_adr_o),
  .wb_dat_i(spiwbm2_dat_o),
  .wb_sel_i(spiwbm2_sel_o),
  .wb_dat_o(scwbs_dat_o),
  .wb_ack_o(scwbs_ack_o),

  .dio_i(dio_pad),
  .dio_oe_o(dio_oe),
  .dio_o(dio),

  .rtc_sda_o(rtc_sda),
  .rtc_sda_i(rtc_sda_pad),
  .rtc_sda_oe_o(rtc_sda_oe),
  .rtc_scl_o(rtc_scl),
  .rtc_scl_i(rtc_scl_pad),
  .rtc_scl_oe_o(rtc_scl_oe),

  .cpu_uart_txd_i(cpu_uart_txd_pad),
  .cpu_uart_rxd_o(cpu_uart_rxd_pad),

  .clk_100mhz_i(pll_100mhz),
  .reboot_o(reboot),
  .led_grn_o(led_red),
  .led_red_o(led_grn),
  .mode1_i(mode1),
  .mode2_i(mode2),
  .mode3_i(mode3),
  .pllphase_o(pllphase),
  .internal_osc_o(internal_osc),
  .can_wbaccess_i(can_wbaccess),
  .can_enable_o(can_enable)
);


/****************************************************************************
 * SPI SBUS blockram window (for XUART 8kbyte memory access)
 ****************************************************************************/
reg mwinwbs_en;
wire [15:0] mwinwbs_dat_o;
wire mwinwbs_ack_o;
wb_memwindow16to32 mwincore(
  .wb_clk_i(wb_clk && xuart_opt),
  .wb_rst_i(wb_rst),

  .wb_cyc_i(spiwbm_cyc_o && mwinwbs_en),
  .wb_stb_i(spiwbm_stb_o && mwinwbs_en),
  .wb_we_i(spiwbm_we_o),
  .wb_adr_i(spiwbm_adr_o),
  .wb_dat_i(spiwbm_dat_o),
  .wb_dat_o(mwinwbs_dat_o),
  .wb_ack_o(mwinwbs_ack_o),
  
  .wbm_cyc_o(ramwbs2_cyc_i),
  .wbm_stb_o(ramwbs2_stb_i),
  .wbm_we_o(ramwbs2_we_i),
  .wbm_adr_o(ramwbs2_adr_i),
  .wbm_dat_o(ramwbs2_dat_i),
  .wbm_dat_i(ramwbs2_dat_o),
  .wbm_ack_i(ramwbs2_ack_o),
  .wbm_sel_o(ramwbs2_sel_i)
);

/****************************************************************************
 * px4 uav pwm generator
 ****************************************************************************/

reg px4wbs_en;
wire [15:0] px4wbs_dat_o;
wire px4wbs_ack_o;

wire pwm_gen_out1;
wire pwm_gen_out2;
wire pwm_gen_out3;
wire pwm_gen_out4;

wire tmp_o;

px4_logic px4_logic_core(
  .wb_clk_i(wb_clk),                                        
  .wb_rst_i(wb_rst),                                        
                                                                     
  .wb_cyc_i(spiwbm_cyc_o && px4wbs_en),                     
  .wb_stb_i(spiwbm_stb_o && px4wbs_en),                     
  .wb_adr_i(spiwbm_adr_o),                                  
  .wb_dat_i(spiwbm_dat_o), 
  .wb_dat_o(px4wbs_dat_o),
  .wb_ack_o(px4wbs_ack_o), 

  .clk_in(fpga_25mhz_pad),
  .ppm_in(dio_pad[35]),
  .temp_out(tmp_o),
  .pwm_out1(pwm_gen_out1),
  .pwm_out2(pwm_gen_out2),
  .pwm_out3(pwm_gen_out3),
  .pwm_out4(pwm_gen_out4)
);
/*
always @(*) begin
  dio_reg[37] = pwm_gen_out1;
  dio_reg[38] = pwm_gen_out2;
  dio_reg[39] = pwm_gen_out3;
  dio_reg[40] = pwm_gen_out4;
end
*/

/****************************************************************************
 * DEPRECATED - SJA1000C compatible CAN controller
 ****************************************************************************/
/*wire canwbs_cyc_i, canwbs_stb_i, canwbs_we_i, canwbs_ack;
wire [7:0] canwbs_dat_i, canwbs_dat_o;
wire [15:0] canwbs_adr_i;
assign can_wbaccess = canwbs_cyc_i && canwbs_stb_i;
wire canwbs_ack_o = canwbs_ack | !can_opt;
can_top cancore(
  .wb_clk_i(wb_clk && can_opt),
  .wb_rst_i(wb_rst),
  .wb_cyc_i(canwbs_cyc_i),
  .wb_stb_i(canwbs_stb_i),
  .wb_we_i(canwbs_we_i),
  .wb_dat_i(canwbs_dat_i),
  .wb_dat_o(canwbs_dat_o),
  .wb_adr_i(canwbs_adr_i[7:0]),
  .wb_ack_o(canwbs_ack),

  .clk_i(clk_24mhz && can_opt),
  .rx_i(dio_pad[24]),
  .tx_o(can_tx),
  .irq_on(can_irqn)
);*/


/****************************************************************************
 * DEPRECATED - SPI SBUS CAN window (for 256 by 8-bit SJA1000C CAN address space)
 ****************************************************************************/
/*reg mwinwbs2_en;
wire [15:0] mwinwbs2_dat_o;
wire mwinwbs2_ack_o;
wb_memwindow16to8 mwincore2(
  .wb_clk_i(wb_clk && can_opt),
  .wb_rst_i(wb_rst),

  .wb_cyc_i(spiwbm_cyc_o && mwinwbs2_en),
  .wb_stb_i(spiwbm_stb_o && mwinwbs2_en),
  .wb_we_i(spiwbm_we_o),
  .wb_adr_i(spiwbm_adr_o),
  .wb_dat_i(spiwbm_dat_o),
  .wb_dat_o(mwinwbs2_dat_o),
  .wb_ack_o(mwinwbs2_ack_o),
  
  .wbm_cyc_o(canwbs_cyc_i),
  .wbm_stb_o(canwbs_stb_i),
  .wbm_we_o(canwbs_we_i),
  .wbm_adr_o(canwbs_adr_i),
  .wbm_dat_o(canwbs_dat_i),
  .wbm_dat_i(canwbs_dat_o),
  .wbm_ack_i(canwbs_ack_o)
);*/


/****************************************************************************
 * SPI SBUS address decode
 ****************************************************************************/
assign gpio_a23_pad = 1'bz;
assign gpio_a22_pad = 1'bz;
always @(*) begin
  sdwbs1_en = 1'b0;
  xuwbs_en = 1'b0;
  mwinwbs_en = 1'b0;
  spiwbs_en = 1'b0;
  scwbs_en = 1'b0;
  spiwbm2_en = 1'b0;
  px4wbs_en = 1'b0;

  spiwbm_dat_i = 16'hxxxx;
  spiwbm_ack_i = 1'b1;
  spiwbm2_dat_i = 16'hxxxx;
  spiwbm2_ack_i = 1'b1;

  /*
   * Top level address decode:
   *
   * 0x0 - SD card
   * 0x10 - px4 (NAND controller (not in TS-7500) deprecated)
   * 0x20 - XUART/XUART memwindow
   * 0x40 - SPI interface (for NOR flash)
   * 0x50 - px4 (CAN memwindow deprecated)
   * 0x60 - syscon
   *
   * 4 bits of address come from the SBUS interface, 2 bits of address come
   * from GPIO A16 and A15 and each address represents 16-bit registers.  Total
   * 128 bytes of address space (64 16-bit regs)
   *
   * It is possible to 8-bit writes via GPIO A17 and A3 which act as active high
   * byte lane selects.
   */
  case (spiwbm_adr_o[6:5])
  2'd0: begin
    if (spiwbm_adr_o[4:0] >= 5'h10) begin
	  px4wbs_en = 1'b1;
      spiwbm_dat_i = px4wbs_dat_o;
      spiwbm_ack_i = px4wbs_ack_o;
	end else begin
      sdwbs1_en = 1'b1;
      spiwbm_dat_i = sdwbs1_dat_o;
      spiwbm_ack_i = sdcard_opt ? sdwbs1_ack_o : 1'b1;
	end
  end
  2'd1: begin
	if (spiwbm_adr_o[4:0] >= 5'h1c) begin
      mwinwbs_en = 1'b1;
      spiwbm_dat_i = mwinwbs_dat_o;
      spiwbm_ack_i = xuart_opt ? mwinwbs_ack_o : 1'b1;
    end else begin
      xuwbs_en = 1'b1;
      spiwbm_dat_i = xuartwbs_dat_o;
      spiwbm_ack_i = xuart_opt ? xuartwbs_ack_o : 1'b1;
    end
  end
  2'd2: begin
    if (spiwbm_adr_o[4:0] >= 5'h10) begin
	  px4wbs_en = 1'b1;
      spiwbm_dat_i = px4wbs_dat_o;
      spiwbm_ack_i = px4wbs_ack_o;
    end else begin
      spiwbs_en = 1'b1;
      spiwbm_dat_i = spiwbs_dat_o;
      spiwbm_ack_i = spi_opt ? spiwbs_ack_o : 1'b1;
    end
  end
  default: begin
    spiwbm_dat_i = 16'hxxxx;
    spiwbm_ack_i = 1'bx;
  end
  endcase

  if (spiwbm2_adr_o[6:5] == 2'd3) begin
    scwbs_en = 1'b1;
    spiwbm2_en = 1'b1;
    spiwbm2_dat_i = scwbs_dat_o;
    spiwbm2_ack_i = scwbs_ack_o;
  end
end


endmodule
