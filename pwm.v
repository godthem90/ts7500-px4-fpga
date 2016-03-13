/*module pwm_core(
	clk_in,
	width,
	pwm_out
);

input clk_in;
input [7:0] width;

output reg pwm_out;

reg [7:0] counter;

always @(posedge clk_in) begin
	if( counter < width ) begin
		pwm_out <= 1;
	end else begin
		pwm_out <= 0;
	end
	counter <= counter + 1;	
end

endmodule*/

module pwm(
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
	pwm_out1,
	pwm_out2,
	pwm_out3,
	pwm_out4
);

input         wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i;
input [6:0]   wb_adr_i;
input [15:0]  wb_dat_i;
output [15:0] wb_dat_o;
output        wb_ack_o;

input clk_in;
output reg pwm_out1;
output reg pwm_out2;
output reg pwm_out3;
output reg pwm_out4;

reg [15:0] width1 = 0;
reg [15:0] width2 = 0;
reg [15:0] width3 = 0;
reg [15:0] width4 = 0;

reg [15:0] counter1 = 0;
reg [15:0] counter2 = 0;
reg [15:0] counter3 = 0;
reg [15:0] counter4 = 0;

always @(posedge wb_clk_i or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		width1 <= 0;
		width2 <= 0;
		width3 <= 0;
		width4 <= 0;
	end else begin
		if (wb_cyc_i && wb_stb_i) begin
			case (wb_adr_i[4:0])
				5'h10: width1 <= wb_dat_i[15:0];
				5'h12: width2 <= wb_dat_i[15:0];
				5'h14: width3 <= wb_dat_i[15:0];
				5'h16: width4 <= wb_dat_i[15:0];
			endcase
		end
	end
end

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		counter1 <= 0;
		pwm_out1 <= 0;
	end else begin
		if( counter1 < width1 ) begin
			pwm_out1 <= 1;
		end else begin
			pwm_out1 <= 0;
		end
		counter1 <= counter1 + 1;
	end
end

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		counter2 <= 0;
		pwm_out2 <= 0;
	end else begin
		if( counter2 < width2 ) begin
			pwm_out2 <= 1;
		end else begin
			pwm_out2 <= 0;
		end
		counter2 <= counter2 + 1;
	end
end

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		counter3 <= 0;
		pwm_out3 <= 0;
	end else begin
		if( counter3 < width3 ) begin
			pwm_out3 <= 1;
		end else begin
			pwm_out3 <= 0;
		end
		counter3 <= counter3 + 1;
	end
end

always @(posedge clk_in or posedge wb_rst_i) begin
	if(wb_rst_i) begin
		counter4 <= 0;
		pwm_out4 <= 0;
	end else begin
		if( counter4 < width4 ) begin
			pwm_out4 <= 1;
		end else begin
			pwm_out4 <= 0;
		end
		counter4 <= counter4 + 1;
	end	
end

reg [15:0] wb_dat;
reg wb_ack;

assign wb_dat_o = wb_dat;
assign wb_ack_o = wb_ack;

always @(*) begin
	wb_dat <= 16'hxxxx;
	wb_ack <= wb_cyc_i && wb_stb_i;
end

/*pwm_core pwm_core1(
	.clk_in(clk_in),
	.width(width1),
	.pwm_out(pwm_out1)
);

pwm_core pwm_core2(
	.clk_in(clk_in),
	.width(width2),
	.pwm_out(pwm_out2)
);

pwm_core pwm_core3(
	.clk_in(clk_in),
	.width(width3),
	.pwm_out(pwm_out3)
);

pwm_core pwm_core4(
	.clk_in(clk_in),
	.width(width4),
	.pwm_out(pwm_out4)
);*/

endmodule