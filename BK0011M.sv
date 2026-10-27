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
	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign LED_USER  = dsk_copy;
assign LED_DISK  = {1'b0, fdd_motor | hdd_active};
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire [1:0] ar = status[14:13];

// Disk controller: standard KNGMD (4 drives), A16M or SMK512 (AltPro, 2 drives),
// standard KNGMD + Samara HDD board (2 drives), or none
wire  [2:0] disk_ctrl  = status[20:18];
wire        std_fdd    = (disk_ctrl == 3'd0);
wire        a16m       = (disk_ctrl == 3'd1);
wire        smk512     = (disk_ctrl == 3'd2);
wire        samara     = (disk_ctrl == 3'd3);
wire        no_disk    = disk_ctrl[2];
wire        extension  = a16m | smk512;

video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),
	.ARX((!ar) ? 12'd4 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd3 : 12'd0),
	.CROP_SIZE(0),
	.CROP_OFF(0),
	.SCALE(status[16:15])
);

/////////////////////////////   CLOCKS   //////////////////////////////
wire plock;
wire clk_sys;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(CLK_VIDEO),
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

// image slots: 0 the hard disk, 1-4 the floppy drives A-D (C and D only
// with the standard controller; Main auto-mounts boot0.vhd..boot3.vhd)
localparam DRIVES = 4;
localparam HDD    = 0;
localparam FDD0   = 1;
localparam SLOTS  = 5;

wire [31:0] fdd_lba;
wire [31:0] hdd_lba;
wire [SLOTS-1:0] sd_rd;
wire [SLOTS-1:0] sd_wr;
wire [SLOTS-1:0] sd_ack;
wire  [7:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire [15:0] fdd_buff_din;
wire [15:0] hdd_buff_din;
wire        sd_buff_wr;

wire [SLOTS-1:0] img_mounted;
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
	"D2S1,DSKIMGBKD,Mount FDD A;",
	"D2S2,DSKIMGBKD,Mount FDD B;",
	"D3S3,DSKIMGBKD,Mount FDD C;",
	"D3S4,DSKIMGBKD,Mount FDD D;",
	"D4SC0,HDIHDS,Mount HDD;",
	"-;",
	"OIK,Disk controller,Standard,A16M,SMK512,Samara,None;",
	"OH,FDD write protect,No,Yes;",
	"-;",
	"O3,Monochrome,No,Yes;",
	"ODE,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O78,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"OFG,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"-;",
	"OAB,Stereo mix,none,25%,50%,100%;",
	"OC,Sound mode,PSG,Covox;",
	"-;",
	"O5,Model,BK0011M,BK0010;",
	"H0O1,CPU Speed,4MHz,8MHz;",
	"H1O1,CPU Speed,3MHz,6MHz;",
	"-;",
	"R2,Reset;",
	"J,A,B,C,D;",
	"jn,A,B,X,Y;",
	"V,v",`BUILD_DATE
};

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(SLOTS)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.ps2_key(ps2_key),
	.ps2_kbd_led_use(3'b001),
	.ps2_kbd_led_status({2'b00, ps2_caps_led}),

	.ps2_mouse(ps2_mouse),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.status(status),
	.status_menumask({~(smk512 | samara), ~std_fdd, no_disk, ~status[5], status[5]}), 

	.gamma_bus(gamma_bus),

	.sd_lba('{hdd_lba,fdd_lba,fdd_lba,fdd_lba,fdd_lba}),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din('{hdd_buff_din,fdd_buff_din,fdd_buff_din,fdd_buff_din,fdd_buff_din}),
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
always @(posedge clk_sys) if(ce_bus_2) cpu_ack <= keyboard_ack | scrreg_ack | ram_ack | disk_ack | fdc_ack | ide_ack | ivec_ack;

reg  [15:0] cpu_din;
always @(posedge clk_sys) cpu_din <= cpureg_data | keyboard_data | scrreg_data | ram_data | sysreg_data | port_data | fdc_data | ide_data | ivec_data;

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

	.clk_vid(CLK_VIDEO),

	.init(!plock),
	.sysreg_sel(sysreg_sel),
	.extension(extension),

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
		disk_rom   <= ~no_disk;
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
// Covox x3: its full scale equals the PSG's (three channels at maximum), as in BKEmu
wire [9:0] covox_l = {1'b0, out_port_data[7:0],  1'b0} + {2'b00, out_port_data[7:0]};
wire [9:0] covox_r = {1'b0, out_port_data[15:8], 1'b0} + {2'b00, out_port_data[15:8]};

assign AUDIO_L = {covox_enable ? covox_l + {2'b00, spk_out, 5'b00000} : def_left_ch,  6'd0};
assign AUDIO_R = {covox_enable ? covox_r + {2'b00, spk_out, 5'b00000} : def_right_ch, 6'd0};
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
	.color_switch(key_color),
	.monochome(status[3]),

	.bus_dout(scrreg_data),
	.bus_ack(scrreg_ack)
);


//////////////////////////   TAPE (BIN), EXT MODE   ///////////////////
wire        disk_ack;
wire        reset_req;
wire        dsk_copy;
wire        dsk_copy_virt;
wire [24:0] dsk_copy_addr;
wire [15:0] dsk_copy_din;
wire [15:0] dsk_copy_dout;
wire        dsk_copy_we;
wire        dsk_copy_rd;

disk disk(.*, .reset(cpu_dclo), .bus_ack(disk_ack));


//////////////////////////   FLOPPY DISKS   ///////////////////////////
// The KNGMD: a KR1801VP1-128 at 177130/177132 and four drives.  A drive
// has a disk when a non-empty image is mounted in it; one mounted
// read-only is write protected, and the OSD can protect them all.
reg [DRIVES-1:0] mount_dsk = 0;
reg [DRIVES-1:0] img_ro    = 0;

always @(posedge clk_sys)
	for(int i = 0; i < DRIVES; i++)
		if(img_mounted[FDD0 + i]) begin
			mount_dsk[i] <= |img_size;
			img_ro[i]    <= img_readonly;
		end

wire [DRIVES-1:0] fdd_wprot = img_ro | {DRIVES{status[17]}};

wire [15:0] fdc_data;
wire        fdc_ack;
wire [15:0] fdd_data_in;      // controller -> drive
wire [15:0] fdd_data_out;     // drive -> controller
wire        fdd_write;
wire  [1:0] fdd_drive;
wire        fdd_motor;
wire        fdd_step;
wire        fdd_dir;
wire        fdd_head;
wire        fdd_valid;
wire        fdd_sync;
wire        fdd_crc_ok;
wire        fdd_rdy;
wire        fdd_tr0;
wire        fdd_ind;

// SMK512 / A16M: bit 2 of the mode register hides the controller from reads
wire        fdc_rd_block = extension & ext_mode[2];

vp1_128 fdc
(
	.clk_sys(clk_sys),
	.reset(cpu_dclo),
	.enable(disk_rom),
	.rd_block(fdc_rd_block),

	.bus_din(bus_din),
	.bus_dout(fdc_data),
	.bus_addr(bus_addr),
	.bus_sync(bus_sync),
	.bus_we(bus_we),
	.bus_stb(bus_stb),
	.bus_ack(fdc_ack),

	.data_in(fdd_data_out),
	.data_out(fdd_data_in),
	.write(fdd_write),
	.drive(fdd_drive),
	.motor(fdd_motor),
	.step(fdd_step),
	.dir(fdd_dir),
	.head(fdd_head),
	.valid(fdd_valid),
	.sync(fdd_sync),
	.crc_ok(fdd_crc_ok),
	.rdy(fdd_rdy),
	.tr0(fdd_tr0),
	.ind(fdd_ind),
	.wprot_dsk(fdd_wprot)
);

fdd4 #(.CLK_HZ(96000000)) fdd
(
	.clk(clk_sys),
	.rst(cpu_dclo),

	.data_in(fdd_data_in),
	.data_out(fdd_data_out),
	.write(fdd_write),

	.drive(fdd_drive),
	.motor(fdd_motor),
	.step(fdd_step),
	.dir(fdd_dir),
	.head(fdd_head),
	.valid(fdd_valid),
	.sync(fdd_sync),
	.crc_ok(fdd_crc_ok),
	.rdy(fdd_rdy),
	.tr0(fdd_tr0),
	.ind(fdd_ind),

	.mount_dsk(mount_dsk),
	.wprot_dsk(fdd_wprot),

	.sd_lba(fdd_lba),
	.sd_rd(sd_rd[FDD0 +: DRIVES]),
	.sd_wr(sd_wr[FDD0 +: DRIVES]),
	.sd_ack(sd_ack[FDD0 +: DRIVES]),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(fdd_buff_din),
	.sd_buff_wr(sd_buff_wr)
);


//////////////////////////   HARD DISK   //////////////////////////////
// The SMK512's IDE controller at 177740-177757 or the Samara one at
// 177620-177657.
wire [15:0] ide_data;
wire        ide_ack;
wire        hdd_active;

smk_ide hdd
(
	.clk_sys(clk_sys),
	.reset(cpu_dclo),
	.enable(disk_rom & (smk512 | samara)),
	.samara(samara),

	.bus_din(bus_din),
	.bus_dout(ide_data),
	.bus_addr(bus_addr),
	.bus_sync(bus_sync),
	.bus_we(bus_we),
	.bus_wtbt(bus_wtbt),
	.bus_stb(bus_stb),
	.bus_ack(ide_ack),

	.img_mounted(img_mounted[HDD]),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(hdd_lba),
	.sd_rd(sd_rd[HDD]),
	.sd_wr(sd_wr[HDD]),
	.sd_ack(sd_ack[HDD]),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(hdd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.active(hdd_active)
);

endmodule
