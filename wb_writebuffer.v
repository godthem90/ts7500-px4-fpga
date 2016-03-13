/* Copyright 2005-2009, Unpublished Work of Technologic Systems
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

module wb_writebuffer(
  wb_clk_i,
  wb_rst_i,
  wb_adr_i,
  wb_dat_i,
  wb_dat_o,
  wb_cyc_i,
  wb_stb_i,
  wb_we_i,
  wb_ack_o,
  wb_sel_i,

  wbm_adr_o,
  wbm_dat_i,
  wbm_dat_o,
  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_ack_i,
  wbm_sel_o,

  flushreq_i,
  flushack_o
);
input wb_clk_i;
input wb_rst_i;
input [31:0] wb_adr_i, wb_dat_i, wbm_dat_i;
output reg [31:0] wb_dat_o, wbm_dat_o, wbm_adr_o;
input [3:0] wb_sel_i;
input wb_cyc_i, wb_stb_i, wb_we_i;
output reg wbm_cyc_o, wbm_stb_o, wbm_we_o;
input wbm_ack_i;
output reg wb_ack_o;
output reg [3:0] wbm_sel_o;
input flushreq_i;
output flushack_o;

wire [67:0] writebuffer_q;
reg wrreq, rdreq;
wire empty, full;
assign flushack_o = empty;
wb_writebuffer_fifo writebuffer(
 .clock(wb_clk_i),
 .data({wb_sel_i, wb_adr_i, wb_dat_i}),
 .q(writebuffer_q),
 .wrreq(wrreq),
 .rdreq(rdreq),
 .empty(empty),
 .full(full),
 .sclr(wb_rst_i)
);

always @(wb_cyc_i or wb_stb_i or wb_we_i or full or empty or writebuffer_q or
  wb_adr_i or wb_dat_i or flushreq_i or wbm_ack_i or wbm_dat_i) begin
  wb_ack_o = 1'b0;
  wrreq = 1'b0;
  wb_dat_o = wbm_dat_i;
  wbm_adr_o = wb_adr_i;
  wbm_dat_o = wb_dat_i;
  wbm_we_o = wb_we_i;
  wbm_sel_o = wb_sel_i;
  wbm_cyc_o = 1'b0;
  wbm_stb_o = 1'b0;
  rdreq = 1'b0;

  if ((flushreq_i && empty) || !flushreq_i) begin
    if (wb_cyc_i && wb_stb_i) begin
      if (wb_we_i && !full) begin
        wrreq = 1'b1;
        wb_ack_o = 1'b1;
      end

      if (!wb_we_i && empty) begin
        wbm_cyc_o = 1'b1;
        wbm_stb_o = 1'b1;
        wb_ack_o = wbm_ack_i;
      end
    end
  end

  if (!empty) begin
    {wbm_sel_o, wbm_adr_o, wbm_dat_o} = writebuffer_q;
    wbm_we_o = 1'b1;
    wbm_cyc_o = 1'b1;
    wbm_stb_o = 1'b1;
    rdreq = wbm_ack_i;
  end 

end

endmodule

module wb_writebuffer_fifo(
  clock,
  data,
  q,
  wrreq,
  rdreq,
  empty,
  full,
  sclr
);

input clock, rdreq, wrreq, sclr;
input [67:0] data;
output [67:0] q;
output full, empty;

(* syn_ramstyle="distributed" *) reg [67:0] fifomem [0:15];
reg [3:0] head, tail;
reg goingfull;
assign q = fifomem[head];
assign full = (head == tail) && goingfull;
assign empty = (head == tail) && !goingfull;
always @(posedge clock or posedge sclr) begin
  if (sclr) begin
    head <= 4'd0;
    tail <= 4'd0;
    goingfull <= 1'b0;
  end else begin
    if (wrreq && !rdreq) goingfull <= 1'b1;
    else if (!wrreq && rdreq) goingfull <= 1'b0;

    if (wrreq) tail <= tail + 1'b1;
    if (rdreq) head <= head + 1'b1;
  end
end

always @(posedge clock) begin
  if (wrreq) fifomem[tail] <= data;
end

endmodule
