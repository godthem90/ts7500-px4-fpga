//////////////////////////////////////////////////////////////////////
////                                                              ////
////  can_register.v                                              ////
////                                                              ////
////                                                              ////
////  This file is part of the CAN Protocol Controller            ////
////  http://www.opencores.org/projects/can/                      ////
////                                                              ////
////                                                              ////
////  Author(s):                                                  ////
////       Igor Mohor                                             ////
////       igorm@opencores.org                                    ////
////                                                              ////
////                                                              ////
////  All additional information is available in the README.txt   ////
////  file.                                                       ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2002, 2003, 2004 Authors                       ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//// The CAN protocol is developed by Robert Bosch GmbH and       ////
//// protected by patents. Anybody who wants to implement this    ////
//// CAN IP core on silicon has to obtain a CAN protocol license  ////
//// from Bosch.                                                  ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: can_register.v,v $
// Revision 1.1  2010/02/02 00:11:07  joff
// Can core from TS-7500
//
// Revision 1.1  2010/01/29 22:28:12  joff
// CAN bus core -- from TS-7552, but with fix for RX buf count
//
// Revision 1.2  2010/01/26 23:54:11  joff
// Fix more opencore CAN bugs:
//   * Potential simultaneous read/write from FIFO buffer
//   * All registers now have global reset state
//
// Revision 1.1  2010/01/07 16:07:09  joff
// Add fixed CAN opencore with upgraded 2kbyte RX FIFO.
//
// Revision 1.1  2009/05/29 17:14:34  eddie
// Current shipping TS-7300 CAN core.
//
// Revision 1.7  2004/02/08 14:32:31  mohor
// Header changed.
//
// Revision 1.6  2003/03/20 16:58:50  mohor
// unix.
//
// Revision 1.4  2003/03/11 16:32:34  mohor
// timescale.v is used for simulation only.
//
// Revision 1.3  2003/02/09 02:24:33  mohor
// Bosch license warning added. Error counters finished. Overload frames
// still need to be fixed.
//
// Revision 1.2  2002/12/27 00:12:52  mohor
// Header changed, testbench improved to send a frame (crc still missing).
//
// Revision 1.1.1.1  2002/12/20 16:39:21  mohor
// Initial
//
//
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on


module can_register
( data_in,
  data_out,
  we,
  clk,
  rst
);

parameter WIDTH = 8; // default parameter of the register width

input [WIDTH-1:0] data_in;
input             we;
input             clk;
input             rst;

output [WIDTH-1:0] data_out;
reg    [WIDTH-1:0] data_out;



always @ (posedge clk or posedge rst)
begin
  if (rst) data_out <= 8'd0;
  else if (we)                        // write
    data_out<=#1 data_in;
end



endmodule