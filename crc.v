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

module crc(
  dat_i,
  clk_i,
  clken_i,
  rst_i,
  shift_i,
  crc16_o,
  crc7_o
) /* synthesis syn_hier="hard" */;

input dat_i;
input clk_i;
input clken_i;
input rst_i;
input shift_i;
output [15:0] crc16_o;
output [6:0] crc7_o;

reg [15:0] crc16;
reg [6:0] crc7;
assign crc16_o = crc16;
assign crc7_o = crc7;
always @(posedge clk_i or posedge rst_i) begin
  if (rst_i) begin
    crc16 <= 16'd0;
    crc7 <= 7'd0;
  end else if (clken_i) begin
    crc16[0] <= dat_i ^ crc16[15];
    crc16[1] <= crc16[0];
    crc16[2] <= crc16[1];
    crc16[3] <= crc16[2];
    crc16[4] <= crc16[3];
    crc16[5] <= dat_i ^ crc16[15] ^ crc16[4];
    crc16[6] <= crc16[5];
    crc16[7] <= crc16[6];
    crc16[8] <= crc16[7];
    crc16[9] <= crc16[8];
    crc16[10] <= crc16[9];
    crc16[11] <= crc16[10];
    crc16[12] <= dat_i ^ crc16[15] ^ crc16[11];
    crc16[13] <= crc16[12];
    crc16[14] <= crc16[13];
    crc16[15] <= crc16[14];

    crc7[0] <= dat_i ^ crc7[6];
    crc7[1] <= crc7[0];
    crc7[2] <= crc7[1];
    crc7[3] <= dat_i ^ crc7[6] ^ crc7[2];
    crc7[4] <= crc7[3];
    crc7[5] <= crc7[4];
    crc7[6] <= crc7[5];
  end else if (shift_i) begin
    crc16 <= {crc16[14:0], 1'b0};
    crc7 <= {crc7[5:0], 1'b1};
  end
end

endmodule

