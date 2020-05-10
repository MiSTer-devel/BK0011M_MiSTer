//
//
// Copyright (c) 2017 Sorgelig
//
// This program is GPL Licensed. See COPYING for the full license.
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module video
(
	input         reset,

	input         clk_sys,
	input         ce_12mp,
	input         ce_12mn,
	output        ce_pix,

	// Misc. signals
	input         bk0010,
	input         color_switch,
	input         monochome,
	input   [1:0] scale,
	input         forced_scandoubler,
	inout  [21:0] gamma_bus,

	// Video signals
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_VS,
	output        VGA_HS,
	output        VGA_DE,

	input         clk_vid,
	output [13:0] vram_addr,
	input  [15:0] vram_data,

	// CPU bus
	input  [15:0] bus_din,
	output [15:0] bus_dout,
	input  [15:0] bus_addr,
	input         bus_sync,
	input         bus_we,
	input   [1:0] bus_wtbt,
	input         bus_stb,
	output        bus_ack,
	output        irq2
);

assign irq2 = irq & irq_en;
reg irq = 1'b0;

assign vram_addr = {screen_bank, vcr, hc[8:4]};

reg  [9:0] hc;
reg  [8:0] vc;
reg  [7:0] vcr;
reg  [7:0] hbr;

reg  [2:0] blank_mask;
reg  HSync;
reg  VSync;
reg  VBlank;
wire HBlank = ~hbr[0];

reg ce_12mp2, ce_12mn2;
always @(posedge clk_sys) begin
	reg [1:0] ce_12mpd, ce_12mnd;
	ce_12mpd <= {ce_12mpd[0], ce_12mp};
	ce_12mnd <= {ce_12mnd[0], ce_12mn};
	
	ce_12mp2 <= |ce_12mpd;
	ce_12mn2 <= |ce_12mnd;
end

reg ce_12mph, ce_12mnh;
always @(posedge clk_vid) {ce_12mph, ce_12mnh} <= {ce_12mp2, ce_12mn2};

reg  mode512;
always @(posedge clk_vid) begin
	reg  col_mod;

	if(ce_12mph) begin
		hc <= hc + 1'd1;
		if(hc == 767) begin 
			hc <=0;

			vcr <= vcr + 1'd1;
			if(vc == 279) vcr <= scroll;

			vc <= vc + 1'd1;
			if (vc == 319) vc <= 9'd0;
		end

		if(hc == 593) begin
			HSync <= 1;
			if(vc == 276) begin
				VSync <= 1;
				mode512 <= ~color;
			end
			if(vc == 280) VSync <= 0;
		end
		
		if(hc == 649) begin
			HSync <= 0;
			if(vc == 256) irq <= 1;
			if(vc == 000) irq <= 0;
		end
	end

	if(ce_12mnh) begin
		VBlank <= vc[8];
		dotm <= hc[0];
		if(!hc[0]) begin
			dots <= {2'b00, dots[15:2]};
			hbr <= {1'b0, hbr[7:1]};
			if(!hc[9] && !(vc[8:6] & {1'b1, {2{~full_screen}}}) && !hc[3:1]) dots <= vram_data;
			if(!hc[9] && !hc[3:1]) hbr <= 8'hFF;
		end
	end
end

wire  [1:0] dotc = dots[1:0];
reg  [15:0] dots;
reg         dotm;

wire [15:0] palettes[16] = '{
	16'h9420, 16'h9BD0, 16'hD640, 16'hB260,
	16'hFD60, 16'hFFF0, 16'h9810, 16'hBA30,
	16'hDC50, 16'h1350, 16'h8AC0, 16'h96B0,
	16'h6920, 16'hF6B0, 16'hFB20, 16'hF620
};

wire [1:0] R;
wire G, B;
assign {R[1], B, G, R[0]} = color ? pal[{dotc[0],dotc[1], 2'b00} +:4] : {4{dotc[dotm]}};

wire hq2x = (scale == 1);

video_mixer #(.LINE_LENGTH(520), .HALF_DEPTH(1), .GAMMA(1)) video_mixer
(
	.*,
	.ce_pix(ce_12mph & (mode512 | dotm)),
	.ce_pix_out(ce_pix),

	.scanlines(0),
	.scandoubler(scale || forced_scandoubler),

	.mono(0),

	.R({R, R}),
	.G({4{G}}),
	.B({4{B}})
);

///////////////////////////////////////////////////////////////////////////////////////

reg  [15:0] reg664      = 'o001330;
reg  [15:0] reg662      = 'o045400;
reg  [15:0] pal;
wire        color       = ~monochome;
wire        screen_bank = ~bk0010 &  reg662[15];
wire        irq_en      = ~bk0010 & ~reg662[14];
wire        full_screen = reg664[9];
wire  [7:0] scroll      = reg664[7:0];

assign bus_dout = sel664 ? reg664 : 16'd0;
assign bus_ack  = bus_stb & (sel664 | sel662);

wire sel662  = bus_sync && (bus_addr[15:1] == (16'o177662 >> 1)) && bus_we && !bk0010;
wire sel664  = bus_sync && (bus_addr[15:1] == (16'o177664 >> 1));

wire [15:0] def_reg662 = bk0010 ? 16'o045400 : 16'o047400;

always @(posedge clk_sys) begin
	reg old_stb, old_cswitch;
	{old_stb, old_cswitch} <= {bus_stb, color_switch};

	if(reset) begin
		reg662 <= def_reg662;
		pal <= palettes[def_reg662[11:8]];
	end
	else
	if(~old_stb & bus_stb) begin
		if(sel664 & bus_we) {reg664[9], reg664[7:0]} <= {bus_din[9], bus_din[7:0]};
		if(sel662) begin
			reg662[15:8] <= bus_din[15:8];
			pal <= palettes[bus_din[11:8]];
		end
	end
	else
	if(~old_cswitch & color_switch & color) begin
		reg662[11:8] <= reg662[11:8] + 1'd1;
		pal <= palettes[reg662[11:8] + 1'd1];
	end
end

endmodule
