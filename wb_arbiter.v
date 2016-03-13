/* Copyright 2005-2006, Unpublished Work of Technologic Systems
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

module wb_arbiter(
  wb_clk_i,

  wb1_adr_i,
  wb1_dat_i,
  wb1_dat_o,
  wb1_cyc_i,
  wb1_stb_i,
  wb1_cti_i,
  wb1_bte_i,
  wb1_we_i,
  wb1_sel_i,
  wb1_ack_o,

  wb2_adr_i,
  wb2_dat_i,
  wb2_dat_o,
  wb2_cyc_i,
  wb2_stb_i,
  wb2_cti_i,
  wb2_bte_i,
  wb2_we_i,
  wb2_sel_i,
  wb2_ack_o,

  wb3_adr_i,
  wb3_dat_i,
  wb3_dat_o,
  wb3_cyc_i,
  wb3_stb_i,
  wb3_cti_i,
  wb3_bte_i,
  wb3_we_i,
  wb3_sel_i,
  wb3_ack_o,

  wb4_adr_i,
  wb4_dat_i,
  wb4_dat_o,
  wb4_cyc_i,
  wb4_stb_i,
  wb4_cti_i,
  wb4_bte_i,
  wb4_we_i,
  wb4_sel_i,
  wb4_ack_o,

  wbowner_adr_o,
  wbowner_dat_i,
  wbowner_dat_o,
  wbowner_cyc_o,
  wbowner_stb_o,
  wbowner_cti_o,
  wbowner_bte_o,
  wbowner_we_o,
  wbowner_sel_o,
  wbowner_ack_i,
  wbowner_o
);
input wb_clk_i;

input [31:0] wb1_adr_i;
input [31:0] wb1_dat_i;
output [31:0] wb1_dat_o;
input wb1_cyc_i, wb1_stb_i, wb1_we_i;
output reg wb1_ack_o;
input [2:0] wb1_cti_i;
input [1:0] wb1_bte_i;
input [3:0] wb1_sel_i;

input [31:0] wb2_adr_i;
input [31:0] wb2_dat_i;
output [31:0] wb2_dat_o;
input wb2_cyc_i, wb2_stb_i, wb2_we_i;
output reg wb2_ack_o;
input [2:0] wb2_cti_i;
input [1:0] wb2_bte_i;
input [3:0] wb2_sel_i;

input [31:0] wb3_adr_i;
input [31:0] wb3_dat_i;
output [31:0] wb3_dat_o;
input wb3_cyc_i, wb3_stb_i, wb3_we_i;
output reg wb3_ack_o;
input [2:0] wb3_cti_i;
input [1:0] wb3_bte_i;
input [3:0] wb3_sel_i;

input [31:0] wb4_adr_i;
input [31:0] wb4_dat_i;
output [31:0] wb4_dat_o;
input wb4_cyc_i, wb4_stb_i, wb4_we_i;
output reg wb4_ack_o;
input [2:0] wb4_cti_i;
input [1:0] wb4_bte_i;
input [3:0] wb4_sel_i;

output reg [31:0] wbowner_adr_o;
output reg [31:0] wbowner_dat_o;
input [31:0] wbowner_dat_i;
output reg wbowner_cyc_o, wbowner_stb_o, wbowner_we_o;
input wbowner_ack_i;
output reg [2:0] wbowner_cti_o;
output reg [1:0] wbowner_bte_o;
output reg [3:0] wbowner_sel_o;
output [1:0] wbowner_o;

reg [1:0] owner = 2'd0;
assign wbowner_o = owner;
reg [1:0] nextowner = 2'd0;
wire [3:0] req = {wb4_cyc_i, wb3_cyc_i, wb2_cyc_i, wb1_cyc_i};

wire [75:0] wb1_ibus = {wb1_sel_i, wb1_cti_i, wb1_bte_i, wb1_cyc_i, wb1_stb_i,
  wb1_we_i, wb1_adr_i, wb1_dat_i};
wire [75:0] wb2_ibus = {wb2_sel_i, wb2_cti_i, wb2_bte_i, wb2_cyc_i, wb2_stb_i,
  wb2_we_i, wb2_adr_i, wb2_dat_i};
wire [75:0] wb3_ibus = {wb3_sel_i, wb3_cti_i, wb3_bte_i, wb3_cyc_i, wb3_stb_i,
  wb3_we_i, wb3_adr_i, wb3_dat_i};
wire [75:0] wb4_ibus = {wb4_sel_i, wb4_cti_i, wb4_bte_i, wb4_cyc_i, wb4_stb_i,
  wb4_we_i, wb4_adr_i, wb4_dat_i};

reg [75:0] obus;

assign wb1_dat_o = wbowner_dat_i;
assign wb2_dat_o = wbowner_dat_i;
assign wb3_dat_o = wbowner_dat_i;
assign wb4_dat_o = wbowner_dat_i;

always @(wbowner_ack_i or wb1_ibus or wb2_ibus or wb3_ibus or wb4_ibus or owner or
  obus) begin
  case (owner) /* synthesis full_case, parallel_case */
  2'd0: begin
    obus = wb1_ibus;
    wb1_ack_o = wbowner_ack_i;
    wb2_ack_o = 1'b0;
    wb3_ack_o = 1'b0;
    wb4_ack_o = 1'b0;
  end
  2'd1: begin
    obus = wb2_ibus;
    wb1_ack_o = 1'b0;
    wb2_ack_o = wbowner_ack_i;
    wb3_ack_o = 1'b0;
    wb4_ack_o = 1'b0;
  end
  2'd2: begin
    obus = wb3_ibus;
    wb1_ack_o = 1'b0;
    wb2_ack_o = 1'b0;
    wb3_ack_o = wbowner_ack_i;
    wb4_ack_o = 1'b0;
  end
  2'd3: begin
    obus = wb4_ibus;
    wb1_ack_o = 1'b0;
    wb2_ack_o = 1'b0;
    wb3_ack_o = 1'b0;
    wb4_ack_o = wbowner_ack_i;
  end
  endcase

  {wbowner_sel_o, wbowner_cti_o, wbowner_bte_o, wbowner_cyc_o, wbowner_stb_o,
    wbowner_we_o, wbowner_adr_o, wbowner_dat_o} = obus;
end

always @(req or owner) begin
  nextowner = owner;

  case (owner) /* synthesis full_case, parallel_case */
  2'd0: if (!req[0]) begin
    if (req[1]) nextowner = 2'd1;
    else if (req[2]) nextowner = 2'd2;
    else if (req[3]) nextowner = 2'd3;
  end

  2'd1: if (!req[1]) begin
    if (req[2]) nextowner = 2'd2;
    else if (req[3]) nextowner = 2'd3;
    else if (req[0]) nextowner = 2'd0;
  end

  2'd2: if (!req[2]) begin
    if (req[3]) nextowner = 2'd3;
    else if (req[0]) nextowner = 2'd0;
    else if (req[1]) nextowner = 2'd1;
  end

  2'd3: if (!req[3]) begin
    if (req[0]) nextowner = 2'd0;
    else if (req[1]) nextowner = 2'd1;
    else if (req[2]) nextowner = 2'd2;
  end
  endcase
end

always @(posedge wb_clk_i) begin
  owner <= nextowner;
end

endmodule
