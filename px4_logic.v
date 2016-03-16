module pwm(
	wb_rst_i,
	clk_in,
	width,
	pwm_out
);

input wb_rst_i;
input clk_in;
input [15:0] width;

output reg pwm_out;

reg [15:0] counter;

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		pwm_out <= 0;
		counter <= 0;
	end else begin
		if( counter < width ) begin
			pwm_out <= 1;
		end else begin
			pwm_out <= 0;
		end
		counter <= counter + 1;	
	end
end

endmodule

module ppm(
	wb_rst_i,
	clk_in,
	ppm_in,
	channel1,
	channel2,
	channel3,
	channel4,
	channel5,
	channel6,
	channel7,
	channel8
);

input wb_rst_i;
input clk_in;
input ppm_in;

output [15:0] channel1;
output [15:0] channel2;
output [15:0] channel3;
output [15:0] channel4;
output [15:0] channel5;
output [15:0] channel6;
output [15:0] channel7;
output [15:0] channel8;

assign channel1 = channel_val[0];
assign channel2 = channel_val[1];
assign channel3 = channel_val[2];
assign channel4 = channel_val[3];
assign channel5 = channel_val[4];
assign channel6 = channel_val[5];
assign channel7 = channel_val[6];
assign channel8 = channel_val[7];

reg [15:0] channel_val [0:8];
reg [4:0] us_counter;
reg [15:0] ppm_in_cnt;
reg [2:0] channel_cnt;
reg prev_ppm_in;
reg rc_found;

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		us_counter <= 0;
		ppm_in_cnt <= 0;
		channel_cnt <= 0;
		prev_ppm_in <= 1;
		channel_val[0] <= 0;
		channel_val[1] <= 0;
		channel_val[2] <= 0;
		channel_val[3] <= 0;
		channel_val[4] <= 0;
		channel_val[5] <= 0;
		channel_val[6] <= 0;
		channel_val[7] <= 0;
	end else begin
		if(us_counter == 5'd24) begin
			us_counter <= 0;
			ppm_in_cnt <= ppm_in_cnt + 1;
			if(ppm_in == 1 && prev_ppm_in == 0) begin
				if(ppm_in_cnt > 16'd3000) begin
					channel_cnt <= 0;
				end else begin
					channel_val[channel_cnt] <= ppm_in_cnt;
					channel_cnt <= channel_cnt + 1;
				end
				ppm_in_cnt <= 0;
				prev_ppm_in <= 1;
			end else if(ppm_in == 0 && prev_ppm_in == 1) begin
				prev_ppm_in <= 0;
			end
		end else begin
			us_counter <= us_counter + 1;
		end
	end
end

endmodule

module px4_logic(
	wb_clk_i,
	wb_rst_i,

	wb_cyc_i,
	wb_stb_i,
	wb_we_i,
	wb_adr_i,
	wb_dat_i,
	wb_dat_o,
	wb_ack_o,

	clk_in,
	ppm_in,
	temp_out,
	pwm_out1,
	pwm_out2,
	pwm_out3,
	pwm_out4
);

output reg temp_out;

input         wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
input [6:0]   wb_adr_i;
input [15:0]  wb_dat_i;
output [15:0] wb_dat_o;
output        wb_ack_o;

input clk_in;
input ppm_in;
output pwm_out1;
output pwm_out2;
output pwm_out3;
output pwm_out4;

reg [15:0] width1;
reg [15:0] width2;
reg [15:0] width3;
reg [15:0] width4;

reg [15:0] counter1;
reg [15:0] counter2;
reg [15:0] counter3;
reg [15:0] counter4;

always @(posedge wb_clk_i or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		width1 <= 0;
		width2 <= 0;
		width3 <= 0;
		width4 <= 0;
	end else begin
		if (wb_cyc_i && wb_stb_i) begin
			case (wb_adr_i[6:0])
				7'h50: width1 <= wb_dat_i[15:0];
				7'h52: width2 <= wb_dat_i[15:0];
				7'h54: width3 <= wb_dat_i[15:0];
				7'h56: width4 <= wb_dat_i[15:0];
			endcase
		end
	end
end

pwm pwm_core1(
	.wb_rst_i(wb_rst_i),
	.clk_in(clk_in),
	.width(width1),
	.pwm_out(pwm_out1)
);

pwm pwm_core2(
	.wb_rst_i(wb_rst_i),
	.clk_in(clk_in),
	.width(width2),
	.pwm_out(pwm_out2)
);

pwm pwm_core3(
	.wb_rst_i(wb_rst_i),
	.clk_in(clk_in),
	.width(width3),
	.pwm_out(pwm_out3)
);

pwm pwm_core4(
	.wb_rst_i(wb_rst_i),
	.clk_in(clk_in),
	.width(width4),
	.pwm_out(pwm_out4)
);

wire [15:0] channel_val1;
wire [15:0] channel_val2;
wire [15:0] channel_val3;
wire [15:0] channel_val4;
wire [15:0] channel_val5;
wire [15:0] channel_val6;
wire [15:0] channel_val7;
wire [15:0] channel_val8;

ppm ppm_core(
	.wb_rst_i(wb_rst_i),
	.clk_in(clk_in),
	.ppm_in(ppm_in),
	.channel1(channel_val1),
	.channel2(channel_val2),
	.channel3(channel_val3),
	.channel4(channel_val4),
	.channel5(channel_val5),
	.channel6(channel_val6),
	.channel7(channel_val7),
	.channel8(channel_val8)
);


reg [15:0] wb_dat;
reg wb_ack;

assign wb_dat_o = wb_dat;
assign wb_ack_o = wb_ack;

always @(*) begin
	wb_dat <= 16'hxxxx;
	wb_ack <= wb_cyc_i && wb_stb_i;
	
	case (wb_adr_i[6:0])
		7'h10 : wb_dat <= channel_val1;
		7'h12 : wb_dat <= channel_val2;
		7'h14 : wb_dat <= channel_val3;
		7'h16 : wb_dat <= channel_val4;
		7'h18 : wb_dat <= channel_val5;
		7'h1a : wb_dat <= channel_val6;
		7'h1c : wb_dat <= channel_val7;
		7'h1e : wb_dat <= channel_val8;
	endcase
end

endmodule