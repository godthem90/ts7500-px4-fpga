/*
  New CAN FIFO Module
  author: Michael Schmidt, Technologic Systems
  Dec 9, 2009
*/
module can_fifo_new
( 
  clk,
  rst,

  wr,

  data_in,
  addr,
  data_out,
  fifo_selected,

  reset_mode,
  release_buffer,
  extended_mode,
  overrun,
  info_empty,
  info_cnt,
  data_out_valid

);

input         clk;
input         rst;
input         wr;
input   [7:0] data_in;
input   [5:0] addr;
input         reset_mode;
input         release_buffer;
input         extended_mode;
input         fifo_selected;

output  [7:0] data_out;
output        overrun;
output        info_empty;
output  [7:0] info_cnt;
output        data_out_valid;

reg wr_q;
reg [3:0] w_ptr;
assign write_done = (~wr) & wr_q & (w_ptr[3:0] != 4'd0);

// Delayed write signal
always @ (posedge clk or posedge rst)
begin
  if (rst) begin
    wr_q <= 1'b0;
	
  end else if (reset_mode) begin
    wr_q <= 1'b0;
	
  end else begin
    wr_q <= wr;
    
  end
end

reg overrun;
always @ (posedge clk or posedge rst) begin
  if (rst) begin
    w_ptr <= 4'd0;
  end else if (wr) begin
    w_ptr <= w_ptr + 1;
  end else if (reset_mode | write_done) begin
    w_ptr <= 0;
  end
end

/*
	We reserve 16 bytes for each packet
	Each packet therefore starts on a multiple of 16 bytes.
	With 256 bytes we can thus store 16 packets-- 2048 bytes: 128 packets
*/
//reg     [7:0] buffer [0:255];
reg		[6:0] rpkt;
reg		[6:0] wpkt;
reg		goingfull;

wire buf_empty = (rpkt[6:0] == wpkt[6:0]) && !goingfull;
wire buf_full = (rpkt[6:0] == wpkt[6:0]) && goingfull;
assign info_empty = buf_empty;
assign info_cnt[7:0] = buf_full ? 8'd128 : ((wpkt[6:0] - rpkt[6:0]) & 8'h7f);

always @ (posedge clk or posedge rst) begin
  if (rst) begin
    overrun <= 0;
  end else if (~buf_full & ~wr) begin
    overrun <= 0;
  end else if (buf_full & wr) begin
    overrun <= 1;
  end
end

always @ (posedge clk or posedge rst) begin
	if (rst) begin
		rpkt <= 0;
		wpkt <= 0;
        goingfull <= 0;
	end else begin
		if (reset_mode) begin
			rpkt <= 0;
			wpkt <= 0;
            goingfull <= 0;
		end else begin
			if (release_buffer && ~buf_empty) begin
				rpkt <= rpkt + 4'd1;
			end
			if (write_done && !buf_full) begin
				wpkt <= wpkt + 1;
			end
            if (!(release_buffer && !buf_empty) && 
              (write_done && !buf_full)) goingfull <= 1'b1;
            else if (!(write_done && !buf_full) &&
              (release_buffer && !buf_empty)) goingfull <= 1'b0; 
		end
	end
end

reg     [10:0] read_address;

//assign data_out = buffer[read_address];

// Selecting which address will be used for reading data from rx fifo
always @ (rpkt) begin
	read_address[10:4] = rpkt[6:0];
end

always @ (extended_mode or addr) begin
  if (extended_mode) begin // extended mode
    read_address[3:0] = (addr - 6'd16);
  end else begin           // normal mode
    read_address[3:0] = (addr - 6'd20);
  end
end

wire		[10:0] wr_pointer;
assign	wr_pointer[10:4] = wpkt[6:0];
assign  wr_pointer[3:0] = w_ptr[3:0];
wire	we = wr && !overrun && !buf_full;
reg     we_q;

always @(posedge clk or posedge rst) if (rst) we_q <= 1'b0; else we_q <= we;
assign data_out_valid = !we_q && !we;

can_ram buffer
(
.Q         (data_out),
.RdClock   (clk),
.WrClock   (clk),
.Data      (data_in),
.WrClockEn (we),
.WE        (1'b1),
.RdClockEn (!we),
.WrAddress (wr_pointer),
.RdAddress (read_address),
.Reset     (rst)
);

/*
always @ (posedge clk) begin
  if (wr & ~overrun) begin
    buffer[wr_pointer] <= data_in;
  end
end
*/
endmodule

