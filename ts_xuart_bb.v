/* Copyright 2008-2009, Unpublished Work of Technologic Systems
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
/*
 * TS-XUART - 8 asynchronous serial RX/TX with extended features
 * 
 * There are 2 wishbone interfaces, one master and one slave.  The
 * master should be connected to a 8 kbyte, 16-bit wide dual port
 * memory.  The slave is the control interface and contains 11 16-bit
 * registers.  In the below, 'iobase' refers to the address of the
 * processor the slave interface is mapped to, while 'membase' refers
 * to the address the dual ported RAM is decoded to.
 * 
 * Register map:
 * iobase + 0x0: TX/RX control channel #0
 *   bits 15-8: RCNT (read) / RIRQ (write)
 *   bits 7-0: TXPOS (read) / TXSTOP (write)
 * 
 * RCNT signifies the current count of entries written to the RX fifo
 * for the particular UART.  This counter will increment every time
 * an entry is written into the RX ring buffer on behalf of this UART
 * and wrap back to 0 on overflow.  An interrupt will be flagged on
 * a new received character when RCNT == RIRQ.  By this mechanism,
 * protocol request/response latency less than 1 bit time can be
 * acheived by precisely coordinating an interrupt to be generated at
 * end of packet should the packet size be known ahead of time.  Due
 * to possible internal synchronization issues, the RCNT cannot be
 * written from the CPU, so even though its reset value is 0, that
 * fact should not be counted on in driver software.
 * 
 * TXPOS is the position in the TX ring buffer of the next character
 * to be sent.  When TXPOS == TXSTOP the transmission is complete and
 * the UART will be idle.  To transmit characters, breaks, or timed
 * idle periods the UART TX ring should be loaded with data starting
 * at the current TXPOS and TXSTOP should be updated to point one past
 * the last entry.  Similar to RCNT, TXPOS is not writeable.
 * 
 * iobase + 0x2: TX/RX control channel #1
 * iobase + 0x4: TX/RX control channel #2
 * iobase + 0x6: TX/RX control channel #3
 * iobase + 0x8: TX/RX control channel #4
 * iobase + 0xa: TX/RX control channel #5
 * iobase + 0xc: TX/RX control channel #6
 * iobase + 0xe: TX/RX control channel #7
 * iobase + 0x10: IRQ status and acknowledgement
 *   bits 15-8: RXIRQ (1 - IRQ pending, 0 - IRQ not pending)
 *   bits 7-0: TXIRQ (1 - IRQ pending, 0 - IRQ not pending)
 * 
 * The IRQ signal output is set whenever any of the above bits are
 * set.  IRQ will continue to be asserted until all set bits are cleared.
 * Writes will have no effect.  To deassert TX IRQs, respective 
 * TX/RX control channel register must be read.  Bit 0 corresponds
 * to UART channel #0.
 * 
 * A TX IRQ is set whenever a TX op is loaded in the TX shift register
 * with the most-significant bit set (bit 15).  By specifically picking
 * which TX bytes will generate IRQs, IRQ load can be low while still
 * ensuring the transmitter is kept busy and at 100% utilization with
 * no inter-character idle gaps.
 * 
 * A RX IRQ is set either when RCNT == RIRQ as set in registers 0x0
 * - 0xe, or when a per-UART programmable idle threshold is exceeded.
 * This threshold can be set to generate an IRQ after 1, 2, 4, or 8
 * bit times of idle allowing an IRQ to be generated only after a
 * string of back-to-back characters has been received.  Details on
 * how this idle threshold is set is described below.  All RX IRQ bits
 * will be cleared on read of RX address (iobase + 0x14).
 * 
 * iobase + 0x12: TX/RX status
 *   bits 15-8: RX enabled (1 - RX is enabled, 0 - RX is disabled) (RW)
 *   bits 7-0: TX pending (1 - TX is running, 0 - TX completed) (RO)
 * 
 * When RX is first enabled, the first RX entry may be an idle or
 * break. The time as recorded will not have started from the moment
 * the UART was enabled and should probably be thrown away.
 * 
 * iobase + 0x14: RX address  (read-only)
 *   bits 15-0: current RX address (next to be written)
 * iobase + 0x16: TX config (write-only)
 *   bits 15-0: TX config (2-bits per channel)
 *     0 - TX suspended (TX idle)
 *     1 - TX suspended (TX break instead)
 *     2 - TX enabled
 *     3 - TX enabled with hardware CTS flow control
 * iobase + 0x16: TX flow control status (read-only)
 *   bits 15-0: TX flow status (2-bits per channel)
 *     0 - TX idle, CTS input deasserted
 *     1 - TX busy, CTS input deasserted
 *     2 - TX idle, CTS input asserted
 *     3 - TX busy, CTS input asserted
 *   
 * Memory map:
 * membase + 0x0: RX ring buffer
 * membase + 0x1000: TX ring buffer #0
 * membase + 0x1200: TX ring buffer #1
 * membase + 0x1400: TX ring buffer #2
 * membase + 0x1600: TX ring buffer #3
 * membase + 0x1800: TX ring buffer #4
 * membase + 0x1a00: TX ring buffer #5
 * membase + 0x1c00: TX ring buffer #6
 * membase + 0x1e00: TX ring buffer #7
 * 
 * TX opcodes:
 * ===========
 * 
 * Each transmitter has a dedicated 256 entry ring buffer in an external
 * memory.  Each entry is 16 bits and contains not only bytes to be
 * transmitted, but also opcodes for fixed length break/idle and
 * commands to change UART parameters such as baud rate, idle threshold,
 * and mode.  TX memory starts at offset 0x1000 of the external UART
 * memory and each ring buffer is laid out contigously.
 * 
 * The format of TX opcodes follow:
 * 
 *   bits 15-14: reserved, should be zero.
 *   bit 13: sets TX IRQ at start of TX processing this entry
 *   bit 12-11: opcode type
 *     00 - transmit data character
 *     01 - transmit timed break
 *     10 - transmit timed idle
 *     11 - control opcode (updates internal UART regs)
 *   bits 10-0: opcode data
 * 
 * Opcode '00' data bits 10-0 are right justified character data bits.
 * Number of data bits can be 7-10 as configured below.
 * 
 * Opcode '01' and '10' data bits 9-0 represent the number of bit
 * times the chosen line condition is set for (idle or break) until
 * the next character.  If bit 10 is set, the 10 bit number is left
 * shifted by 6 to allow for longer periods than could be represented
 * in a 10 bit counter.  Two back to back break opcodes will have at
 * least 2 bit times of idle between them.
 * 
 * Opcode '11' is for updating internal 16-bit UART registers.  The
 * 11 bit opcode data field is broken down into a 3 bit sub-op and a
 * 8 bit data payload.  The 3 bit sub-op represents the address of
 * the internal register to update and is the most-significant 3 bits
 * of the opcode data (bits 10-8).  Since each register is 16 bits
 * and there is only room for 8 bits of data, the upper 8 bits must
 * be latched in a previous control opcode by writing to the sub-op
 * field the reserved address of 7 (111).  The internal registers are
 * as follows:
 * 
 *   0: baud prescalar - 16 bit counter to generate 8x baud freq
 *   1: control reg
 *      bits 15-4: unused (don't cares)
 *      bits 3-2: idle threshold - sets RX irq after set idle time
 *        00 - 1 bit time
 *        01 - 8 bit times
 *        10 - 16 bit times
 *        11 - 32 bit times
 *      bits 1-0: character size
 *        00 - 7 bits
 *        01 - 8 bits
 *        10 - 9 bits
 *        11 - 10 bits
 *    2-6: reserved
 * 
 * All traditional UART modes are possible with 7-10 bit character
 * sizes.  The UART does not have special provision for automatically
 * appending/checking parity bits or multiple stop bits--  instead
 * these must be handled with software. A single start bit and single
 * stop bit is assumed with each transmit word.  Parity and multiple
 * stop bits can be added easily in software with precomputed lookup
 * tables.
 * 
 * RX processing:
 * ==============
 * 
 * The receiver uses 4kbytes arranged as a 2048 x 16-bit ring buffer.
 * When a character, break, or period of idle exceeding the set idle
 * threshold is received, an entry is written to the current address
 * in the ring buffer and the current address is incremented.  When
 * the address reaches the end of the ring buffer, it rolls over back
 * to 0.  The format of each entry into the ring buffer is as follows:
 * 
 *   bits 15-13: UART# character received from
 *   bits 12-11: RX type
 *     00 - character
 *     01 - break
 *     10 - idle
 *     11 - Invalid -- will never be written
 *   bits 10-0: RX data
 * 
 * When RX type is '00', the received character is present in data
 * bits 10-0 left justified.  The remaining LSB bits are random and
 * should not be used.
 * 
 * When the RX type is '01' or '10' -- indicating a break condition
 * or an extended period of idle was detected -- the remaining 11
 * least significant bits represent the length of the condition in
 * bit times.  However, if bit 10 is set, the actual 10 bit result
 * should be left shifted by 6.  A break of any length will be reported,
 * but an idle RX entry will only be inserted into the ring buffer if
 * it exceeds the programmed idle threshold from above.  A break must
 * first exceed the character time before being considered a break and
 * the counter only starts once that happens, i.e. for a 8N1 (8 data
 * bits, 1 start bit, 1 stop bit) 10 should be added to the counted
 * result.
 * 
 * Since an RX type of '11' will never be written, software may choose
 * to write this value to all data bytes as a way to know how much of
 * the RX ring buffer has been updated since the last RX processing
 * event.  It is expected that RX processing be not only run in response
 * to an RX interrupt, but also as part of a low speed timer interrupt.
 * By knowing which UARTS are activated and at what baud rates and
 * idle thresholds, worst case ring buffer overflow time can be
 * computed.
 * 
 */

module ts_xuart(
  wb_clk_i,
  wb_rst_i,

  wbm_cyc_o,
  wbm_stb_o,
  wbm_we_o,
  wbm_adr_o,
  wbm_dat_o,
  wbm_dat_i,
  wbm_ack_i,

  wbs_cyc_i,
  wbs_stb_i,
  wbs_we_i,
  wbs_adr_i,
  wbs_dat_i,
  wbs_dat_o,
  wbs_ack_o,

  uart_disable_i,
  baudclk_i,
  cts_i,
  rx_i,
  tx_o,
  txen_o,
  active_o,
  irq_o
);
input wb_clk_i, wb_rst_i, wbm_ack_i, wbs_cyc_i, wbs_stb_i, wbs_we_i;
output wbm_cyc_o, wbm_stb_o, wbm_we_o, wbs_ack_o;
input [15:0] wbm_dat_i, wbs_dat_i;
output [15:0] wbm_adr_o;
input [31:0] wbs_adr_i;
output [15:0] wbm_dat_o, wbs_dat_o;
input [7:0] rx_i;
input [7:0] cts_i;
output [7:0] tx_o, txen_o, active_o;
output irq_o;
input baudclk_i;
input [7:0] uart_disable_i;

endmodule
