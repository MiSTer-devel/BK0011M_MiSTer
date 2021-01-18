////////////////////////////////////////////////////////////////////////////////
//
//
//
// BK0011M for MiSTer
// (C) 2017-2019 Sorgelig
//
// This source file and all other files in this project is free software: 
// you can redistribute it and/or modify it under the terms of the 
// GNU General Public License version 2 unless explicitly specified in particular file.
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//
//
//
////////////////////////////////////////////////////////////////////////////////

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output [11:0] VIDEO_ARX,
	output [11:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler

`ifdef USE_FB
	// Use framebuffer in DDRAM (USE_FB=1 in qsf)
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

`ifdef USE_DDRAM
	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,
`endif

`ifdef USE_SDRAM
	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,
`endif

`ifdef DUAL_SDRAM
	//Secondary SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign LED_USER  = dsk_copy;
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;

wire [1:0] ar = status[14:13];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

/////////////////////////////   CLOCKS   //////////////////////////////
wire plock;
wire clk_sys;
wire clk_vid;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_vid),
	.locked(plock)
);

reg  ce_cpu_p;
reg  ce_cpu_n;
wire ce_bus = ce_cpu_p;
reg  ce_psg;
reg  ce_12mp;
reg  ce_12mn;
reg  ce_6mp;
reg  ce_6mn;
reg  turbo;
reg  ce_bus_2;

always @(posedge clk_sys) begin
	reg  [3:0] div = 0;
	reg  [4:0] cpu_div = 0;
	reg  [5:0] psg_div = 0;

	cpu_div <= cpu_div + 1'd1;
	if(cpu_div == ((5'd23 + {bk0010, 3'b000})>>turbo)) begin
		cpu_div <= 0;
		if(!bus_sync) turbo <= status[1];
	end
	ce_cpu_p <= (cpu_div == 0);
	ce_cpu_n <= (cpu_div == (5'd12 + {bk0010, 2'b00})>>turbo);
	ce_bus_2 <= ce_cpu_p;

	div <= div + 1'd1;
	ce_12mp <= !div[2] & !div[1:0];
	ce_12mn <=  div[2] & !div[1:0];
	ce_6mp  <= !div[3] & !div[2:0];
	ce_6mn  <=  div[3] & !div[2:0];

	psg_div <= psg_div + 1'd1;
	if(psg_div == 55) psg_div <= 0;

	ce_psg <= !psg_div;
end


///////////////////////////  MIST ARM I/O  ////////////////////////////
wire [10:0] ps2_key;
wire        ps2_caps_led;
wire [24:0] ps2_mouse;

wire [15:0] joystick_0;
wire [15:0] joystick_1;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [31:0] status;

wire [31:0] sd_lba;
wire  [2:0] sd_rd;
wire  [2:0] sd_wr;
wire        sd_ack;
wire  [7:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire [15:0] sd_buff_din;
wire        sd_buff_wr;

wire  [2:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;

wire        ioctl_download;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire  [7:0] ioctl_index;

wire [21:0] gamma_bus;

`include "build_id.v"
localparam CONF_STR = 
{
	"BK0011M;;",
	"F,BIN;",
	"D2S1,DSK,Mount FDD(A);",
	"D2S2,DSK,Mount FDD(B);",
	"D2S0,VHD,Mount HDD;",
	"-;",
	"O3,Monochrome,No,Yes;",
	"ODE,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O78,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"OAB,Stereo mix,none,25%,50%,100%;",
	"OC,Sound mode,PSG,Covox;",
	"-;",
	"O5,Model,BK0011M,BK0010;",
	"H0O1,CPU Speed,4MHz,8MHz;",
	"H1O1,CPU Speed,3MHz,6MHz;",
	"-;",
	"O6,FDD/HDD,Enable,Disable;",
	"H0O9,SMK512,No,Yes;",
	"H1O9,A16M,No,Yes;",
	"-;",
	"R2,Reset & Unload Disks;",
	"J,A,B,C,D;",
	"jn,A,B,X,Y;",
	"V,v",`BUILD_DATE
};

hps_io #(.STRLEN($size(CONF_STR)>>3), .WIDE(1), .VDNUM(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.ps2_key(ps2_key),
	.ps2_kbd_led_use(3'b001),
	.ps2_kbd_led_status({2'b00, ps2_caps_led}),

	.ps2_mouse(ps2_mouse),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.status(status),
	.status_menumask({status[6], ~status[5], status[5]}), 

	.gamma_bus(gamma_bus),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index)
);



//////////////////////////////   CPU   ////////////////////////////////
wire        cpu_dclo;
wire        cpu_aclo;
wire  [3:1]	cpu_irq = {1'b0, irq2, (key_stop && !key_stop_block)};
wire        cpu_virq;
wire        cpu_iacko;
wire [15:0] cpu_dout;
wire        cpu_din_out;
wire        cpu_dout_out;
reg         cpu_ack;
wire  [2:1]	cpu_psel;
wire        bus_reset;
wire [15:0] bus_din = cpu_dout;
wire [15:0]	bus_addr;
wire        bus_sync;
wire        bus_we;
wire  [1:0]	bus_wtbt;
wire        bus_stb = cpu_dout_in | cpu_din_out;

vm1_reset reset
(
	.clk(CLK_50M),
	.reset(~sys_ready | reset_req | buttons[1] | status[2] | key_reset),
	.dclo(cpu_dclo),
	.aclo(cpu_aclo)
);

// Wait for bk0011m.rom or initial reset
reg sys_ready = 0;
always @(posedge clk_sys) begin
	reg old_rst = 0;
	old_rst <= status[0];
	if(old_rst & ~status[0]) sys_ready <= 1;
end

vm1_se cpu
(
	.pin_clk(clk_sys),
	.pin_ce_p(ce_cpu_p),
	.pin_ce_n(ce_cpu_n),
	.pin_ce_timer(ce_cpu_p),

	.pin_init(bus_reset),
	.pin_dclo(cpu_dclo),
	.pin_aclo(cpu_aclo),

	.pin_irq(cpu_irq),
	.pin_virq(cpu_virq),
	.pin_iako(cpu_iacko),

	.pin_addr(bus_addr),
	.pin_dout(cpu_dout),
	.pin_din(cpu_din),
	.pin_din_stb_out(cpu_din_out),
	.pin_dout_stb_out(cpu_dout_out),
	.pin_din_stb_in(cpu_din_out),
	.pin_dout_stb_in(cpu_dout_in),
	.pin_we(bus_we),
	.pin_wtbt(bus_wtbt),
	.pin_sync(bus_sync),
	.pin_rply(cpu_ack),

	.pin_dmr(dsk_copy),
	.pin_sack(0),

	.pin_sel(cpu_psel)
);

wire        cpu_dout_in  = dout_delay[{~bk0010,1'b0}] & cpu_dout_out;
wire        sysreg_sel   = cpu_psel[1];
wire        port_sel     = cpu_psel[2];
wire [15:0]	cpureg_data  = (bus_sync & !cpu_psel & (bus_addr[15:4] == (16'o177700 >> 4))) ? cpu_dout : 16'd0;
wire [15:0]	sysreg_data  = sysreg_sel ? {start_addr, 1'b1, ~key_down, 3'b000, super_flg, 2'b00} : 16'd0;
wire        sysreg_write = bus_stb & sysreg_sel & bus_we;
wire        port_write   = bus_stb & port_sel   & bus_we;

reg   [2:0] dout_delay;
always @(posedge clk_sys) if(ce_bus_2) dout_delay <= {dout_delay[1:0], cpu_dout_out};
always @(posedge clk_sys) if(ce_bus_2) cpu_ack <= keyboard_ack | scrreg_ack | ram_ack | disk_ack | ivec_ack;

reg  [15:0] cpu_din;
always @(posedge clk_sys) cpu_din <= cpureg_data | keyboard_data | scrreg_data | ram_data | sysreg_data | port_data | ivec_data;

reg  super_flg  = 1'b0;
wire sysreg_acc = bus_stb & sysreg_sel;
always @(posedge clk_sys) begin
	reg old_acc;
	old_acc <= sysreg_acc;
	if(~old_acc & sysreg_acc) super_flg <= bus_we;
end


/////////////////////////////   MEMORY   //////////////////////////////
wire [15:0]	ram_data;
wire        ram_ack;
wire  [1:0] screen_write;
reg         bk0010     = 1'bZ;
reg         disk_rom   = 1'bZ;
wire  [7:0] start_addr;
wire [15:0] ext_mode;
reg         cold_start = 1;
reg         mode_start = 1;
wire        bk0010_stub;

memory memory
(
	.*,

	.init(!plock),
	.sysreg_sel(sysreg_sel),
	.extension(status[9]),

	.bus_dout(ram_data),
	.bus_ack(ram_ack),

	.mem_copy(dsk_copy),
	.mem_copy_virt(dsk_copy_virt),
	.mem_copy_addr(dsk_copy_addr),
	.mem_copy_din(dsk_copy_dout),
	.mem_copy_dout(dsk_copy_din),
	.mem_copy_we(dsk_copy_we),
	.mem_copy_rd(dsk_copy_rd)
);

always @(posedge clk_sys) begin
	integer reset_time;
	reg old_dclo, old_sel, old_bk0010, old_disk_rom;

	old_dclo <= cpu_dclo;
	old_sel  <= sysreg_sel;

	if(!old_dclo & cpu_dclo) begin 
		reset_time   <= 10000000;
		old_bk0010   <= bk0010;
		old_disk_rom <= disk_rom;
	end else if(cpu_dclo) begin 
		if(ce_12mp && reset_time) reset_time <= reset_time - 1;
		bk0010     <= status[5];
		disk_rom   <= ~status[6];
		cold_start <= (old_bk0010 != bk0010) | (old_disk_rom != disk_rom) | !reset_time;
		mode_start <= 1;
	end

	if(old_sel && !sysreg_sel) mode_start <= 0;
end


///////////////////////////   INTERRUPTS   ////////////////////////////
wire [15:0]	ivec_o;
wire [15:0] ivec_data = ivec_sel ? ivec_o : 16'd0;

wire ivec_sel = cpu_iacko & !bus_we;
wire ivec_ack;

wire virq_req60, virq_req274;
wire virq_ack60, virq_ack274;

vic_wb #(2) vic 
(
	.clk_sys(clk_sys),
	.ce(ce_bus),
	.wb_rst_i(bus_reset),
	.wb_irq_o(cpu_virq),	
	.wb_dat_o(ivec_o),
	.wb_stb_i(ivec_sel & bus_stb),
	.wb_ack_o(ivec_ack),
	.ivec({16'o000060,  16'o000274}),
	.ireq({virq_req60, virq_req274}),
	.iack({virq_ack60, virq_ack274})
);


///////////////////   KEYBOARD, MOUSE, JOYSTICK   /////////////////////
reg key_stop_block;
always @(posedge clk_sys) begin
	reg old_write;
	old_write <= sysreg_write;
	if(~old_write & sysreg_write & ~cpu_dout[11] & bus_wtbt[1]) key_stop_block <= cpu_dout[12];
end

wire        key_down;
wire        key_stop;
wire        key_reset;
wire        key_color;
wire        key_bw;
wire [15:0]	keyboard_data;
wire        keyboard_ack;

keyboard keyboard
(
	.*,
	.bus_dout(keyboard_data),
	.bus_ack(keyboard_ack)
);

reg         joystick_or_mouse = 0;
wire [15:0] port_data = port_sel ? (~covox_enable & joystick_or_mouse ? mouse_state : joystick) : 16'd0;
wire [15:0] joystick =  joystick_0 | joystick_1;

wire  [8:0] pointer_dx = {ps2_mouse[4],ps2_mouse[15:8]};
wire  [8:0] pointer_dy = {ps2_mouse[5],ps2_mouse[23:16]};

reg   [6:0] mouse_state = 0;
wire        mouse_write = ~covox_enable & bus_wtbt[0] & port_write;
always @(posedge clk_sys) begin
	reg mouse_enable = 0;
	reg old_write;
	reg old_status;

	if(|joystick) joystick_or_mouse <= 0;

	old_write <= mouse_write;
	if(~old_write & mouse_write) begin 
		mouse_enable <= cpu_dout[3];
		if(!cpu_dout[3]) mouse_state[3:0] <= 0;
	end

	old_status <= ps2_mouse[24];
	if(old_status != ps2_mouse[24]) begin
		mouse_state[6] <= ps2_mouse[1];
		mouse_state[5] <= ps2_mouse[0];
		joystick_or_mouse <= 1;
		if(mouse_enable) begin
			if(!mouse_state[0] && !mouse_state[2]) begin
				if(!pointer_dy[8] && ( pointer_dy > 3)) mouse_state[0] <= 1;
				if( pointer_dy[8] && (~pointer_dy > 2)) mouse_state[2] <= 1;
			end
			if(!mouse_state[1] && !mouse_state[3]) begin
				if(!pointer_dx[8] && ( pointer_dx > 3)) mouse_state[1] <= 1;
				if( pointer_dx[8] && (~pointer_dx > 2)) mouse_state[3] <= 1;
			end
		end
	end
end


/////////////////////////////   AUDIO   ///////////////////////////////
reg [2:0] spk_out;
always @(posedge clk_sys) begin
	reg old_write;
	old_write <= sysreg_write;
	if(~old_write & sysreg_write) begin
		if((!bus_wtbt[1] || (!cpu_dout[11] && bus_wtbt[1])) && bus_wtbt[0]) spk_out <= {cpu_dout[6],cpu_dout[5],cpu_dout[2] & !bk0010};
	end
end

wire [7:0] channel_a;
wire [7:0] channel_b;
wire [7:0] channel_c;
wire [5:0] psg_active;

YM2149 psg
(
	.CLK(clk_sys),
	.CE(ce_psg),
	.RESET(bus_reset),
	.BDIR(~covox_enable & port_write),
	.BC(~covox_enable & bus_wtbt[1]),
	.DI(~bus_din[7:0]),
	.CHANNEL_A(channel_a),
	.CHANNEL_B(channel_b),
	.CHANNEL_C(channel_c),
	.ACTIVE(psg_active),
	.SEL(0),
	.MODE(0)
);

// COVOX
wire covox_enable = status[12];
reg [15:0] out_port_data;
wire [9:0] def_left_ch, def_right_ch;

always @(posedge clk_sys) begin
	reg old_write;
	old_write <= port_write;
	if (~old_write & port_write) begin
		if (bus_wtbt[0])
			out_port_data[7:0]  <= cpu_dout[7:0];
		if (bus_wtbt[1])
			out_port_data[15:8] <= cpu_dout[15:8];
	end
end

assign def_left_ch  = psg_active ? {1'b0, channel_a, 1'b0} + {2'b00, channel_b} + {2'b00, spk_out, 5'b00000} : {spk_out, 7'b0000000};
assign def_right_ch = psg_active ? {1'b0, channel_c, 1'b0} + {2'b00, channel_b} + {2'b00, spk_out, 5'b00000} : {spk_out, 7'b0000000};

assign AUDIO_S = 0;
assign AUDIO_L = {covox_enable ? {1'b0, out_port_data[7:0],  1'b0} + {2'b00, spk_out, 5'b00000} : def_left_ch,  6'd0};
assign AUDIO_R = {covox_enable ? {1'b0, out_port_data[15:8], 1'b0} + {2'b00, spk_out, 5'b00000} : def_right_ch, 6'd0};
assign AUDIO_MIX = status[11:10];


/////////////////////////////   VIDEO   ///////////////////////////////
wire [15:0]	scrreg_data;
wire        scrreg_ack;
wire        irq2;
wire [13:0] vram_addr;
wire [15:0] vram_data;

wire  [1:0] scale = status[8:7];
assign VGA_SL = {scale == 3, scale == 2};
assign VGA_F1 = 0;

video video
(
	.*,
	.reset(cpu_dclo),
	.ce_pix(CE_PIXEL),
	.color_switch(key_color),
	.monochome(status[3]),

	.bus_dout(scrreg_data),
	.bus_ack(scrreg_ack)
);

assign CLK_VIDEO = clk_vid;


//////////////////////////   DISK, TAPE   /////////////////////////////
wire        disk_ack;
wire        reset_req;
wire        dsk_copy;
wire        dsk_copy_virt;
wire [24:0] dsk_copy_addr;
wire [15:0] dsk_copy_din;
wire [15:0] dsk_copy_dout;
wire        dsk_copy_we;
wire        dsk_copy_rd;

disk disk(.*, .img_size(img_size[40:9]), .reset(cpu_dclo), .reset_full(status[2]), .bus_ack(disk_ack));

endmodule
