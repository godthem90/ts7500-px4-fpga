/* Copyright 2007-2009, Unpublished Work of Technologic Systems
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
/* Timing requirements @50Mhz (20ns bus cycle) SD speed (100Mhz wishbone)
 *
 * Output: (SD has 2ns hold and 6ns setup requirement)
 * * min tCO data - min tCO clk + 10ns > 2ns (SD hold margin)
 * * max tCO data - max tCO clk < 4ns (SD setup margin)
 *
 * Input: (SD has 14ns output delay and guarantees 2.5ns hold)
 * * max tCO clock + 14ns + tSU data < 20ns (FPGA setup margin)
 * * tHOLD data - min tCO clock < 2.5ns (FPGA hold margin)
 *
 * Timing requirements are hard to properly constrain because of
 * the relationship between tCO and tSU and the skew from the clock
 * source internally to the FPGA.  Best results are had by making the
 * clock outputs and data inputs as fast as possible and data outputs
 * as slow as possible to ensure adequate hold time.  PCB trace skew
 * helps FPGA hold margin, hurts FPGA setup margin.  SD hold margin can
 * be gained by adding combinational delays before the data outputs--
 * output registers in the I/O ring should not be used for data out.
 *
 */

module ts_sdcore(
  wb_clk_i,
  wb_rst_i,

  wb1_cyc_i,
  wb1_cti_i,
  wb1_bte_i,
  wb1_stb_i,
  wb1_we_i,
  wb1_adr_i,
  wb1_dat_i,
  wb1_sel_i,
  wb1_dat_o,
  wb1_ack_o,

  wb2_cyc_i,
  wb2_stb_i,
  wb2_cti_i,
  wb2_bte_i,
  wb2_we_i,
  wb2_adr_i,
  wb2_dat_i,
  wb2_sel_i,
  wb2_dat_o,
  wb2_ack_o,

  wb3_cyc_i,
  wb3_stb_i,
  wb3_cti_i,
  wb3_bte_i,
  wb3_we_i,
  wb3_adr_i,
  wb3_dat_i,
  wb3_sel_i,
  wb3_dat_o,
  wb3_ack_o,

  wb4_cyc_i,
  wb4_stb_i,
  wb4_cti_i,
  wb4_bte_i,
  wb4_we_i,
  wb4_adr_i,
  wb4_dat_i,
  wb4_sel_i,
  wb4_dat_o,
  wb4_ack_o,

  sd_clk_o,
  sd_cmd_o,
  sd_cmd_i,
  sd_cmd_oe_o,
  sd_dat_o,
  sd_dat_i,
  sd_dat_oe_o,
  sd_power_o,
  sd_detect_i,
  sd_wprot_i,
  sd_busy_o

);

input wb_clk_i, wb_rst_i;

input wb1_cyc_i, wb1_stb_i, wb1_we_i;
input [3:0] wb1_sel_i;
input [31:0] wb1_adr_i, wb1_dat_i;
output [31:0] wb1_dat_o;
output wb1_ack_o;
input [2:0] wb1_cti_i;
input [1:0] wb1_bte_i;

input wb2_cyc_i, wb2_stb_i, wb2_we_i;
input [3:0] wb2_sel_i;
input [31:0] wb2_adr_i, wb2_dat_i;
output [31:0] wb2_dat_o;
output wb2_ack_o;
input [2:0] wb2_cti_i;
input [1:0] wb2_bte_i;

input wb3_cyc_i, wb3_stb_i, wb3_we_i;
input [3:0] wb3_sel_i;
input [31:0] wb3_adr_i, wb3_dat_i;
output [31:0] wb3_dat_o;
output wb3_ack_o;
input [2:0] wb3_cti_i;
input [1:0] wb3_bte_i;

input wb4_cyc_i, wb4_stb_i, wb4_we_i;
input [3:0] wb4_sel_i;
input [31:0] wb4_adr_i, wb4_dat_i;
output [31:0] wb4_dat_o;
output wb4_ack_o;
input [2:0] wb4_cti_i;
input [1:0] wb4_bte_i;

output [7:0] sd_clk_o;
output sd_cmd_o, sd_cmd_oe_o, sd_power_o;
output [3:0] sd_dat_o;
output [3:0] sd_dat_oe_o;
input sd_cmd_i;
input [7:0] sd_wprot_i, sd_detect_i;
input [3:0] sd_dat_i;
output sd_busy_o;

endmodule

