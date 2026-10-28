//
// SMK-512 IDE hard disk controller at 177740-177757, as the SMK BIOS v2.05 and
// BKEmu / bkemu-android have it:
//
//   177740 status (r) / command (w)   177750 sector     177741 drive address (r)
//   177742 drive & head               177752 count      177743 alt. status (r) /
//   177744 cylinder high              177754 error (r) /        device control (w)
//   177746 cylinder low                      features (w)
//   177756 data (16-bit words)
//
// ATA register index = inverted address bits 3:1; the odd addresses of the
// first two words are the control block. Every bit on the bus is INVERTED,
// sector data included, so an image holds the complement of what the software
// stores - the form of a raw SMK disk dump, of BKEmu's HDD images and of the
// .hdi files (512-byte header: word 1 cylinders, 3 heads, 6 sectors, byte 510
// A5, byte 511 the checksum), found by probing sector 0 at mount time. CHS with
// the header's or the default 16 x 63 geometry, INITIALIZE DRIVE PARAMETERS as
// the BIOS sets it (SETPRM), or LBA28 with bit 6 of the head register.
// Multi-sector reads are read ahead into a second buffer bank.
//
// With samara set it is the Samara controller instead: the same registers at
// 177640-177656, the control block as two words, 177620 drive address and
// 177622 alt. status / device control (BKEmu's map).
//
// A BKHD image (the old core's .vhd: "BKHD", version 1, from byte 16 a table
// of 32-bit block numbers where the partitions begin, the last one where the
// disk ends; data not inverted) is shown to the Samara controller as the
// disk its SRV utility would lay out: block 1 the partition table, the
// partitions from block 3 on, each one a descriptor block followed by its
// data. Those blocks are made up here; writes to them go nowhere.
//
// Transfer engine from UKNC_MiSTer's ide.v on the BK bus and the SMK map.
//
module smk_ide
(
	input             clk_sys,
	input             reset,        // power-on reset
	input             enable,       // the controller is in the machine
	input             samara,       // the Samara register map instead of the SMK's

	input      [15:0] bus_din,
	output     [15:0] bus_dout,
	input      [15:0] bus_addr,
	input             bus_sync,
	input             bus_we,
	input       [1:0] bus_wtbt,
	input             bus_stb,
	output            bus_ack,

	// hps_io, this drive's slot
	input             img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output reg [31:0] sd_lba,
	output            sd_rd,
	output            sd_wr,
	input             sd_ack,
	input       [7:0] sd_buff_addr,
	input      [15:0] sd_buff_dout,
	output reg [15:0] sd_buff_din,
	input             sd_buff_wr,

	output            active        // a transfer is going on, for the LED
);

//------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------
localparam [7:0] ST_ERROR = 8'h01, ST_DRQ = 8'h08, ST_SEEK = 8'h10,
				 ST_READY = 8'h40, ST_BUSY = 8'h80;
localparam [7:0] ER_NONE = 8'h00, ER_ABRT = 8'h04, ER_IDNF = 8'h10, ER_DIAG_OK = 8'h01;
localparam [2:0] REG_DATA = 3'd0, REG_ERROR = 3'd1, REG_COUNT = 3'd2,
				 REG_SECTOR = 3'd3, REG_CYL_LO = 3'd4, REG_CYL_HI = 3'd5,
				 REG_HEAD = 3'd6, REG_STATUS = 3'd7;
localparam       HEAD_LBA = 6;        // head register bit: LBA28 addressing
localparam       CTL_SRST = 2;        // device control bit: software reset
localparam [7:0] RESET_CLOCKS = 8'd64;
localparam [7:0] LAST_WORD = 8'd255;
localparam       SECTOR_SHIFT = 9;    // bytes to sectors
localparam [15:0] MAX_CYL = 16'hFFFF;
localparam [7:0] DEF_HEADS = 8'd16, DEF_SPT = 8'd63;
localparam       BK_PARTS = 8;        // BKHD partitions shown, C: to J:

//------------------------------------------------------------------------
// The bus
//------------------------------------------------------------------------
wire        smk_sel = bus_addr[15:4] == 12'o7776;                        // 177740..177757
wire        sam_sel = bus_addr[15:4] == 12'o7772;                        // 177640..177657
wire        sam_ctl = bus_addr[15:2] == 14'o37744;                       // 177620..177623
wire        sel   = enable & bus_sync & (samara ? (sam_sel | sam_ctl) : smk_sel);
// the control block: REG_HEAD = alt. status / device control, REG_STATUS =
// drive address; at the odd addresses of the SMK, at 177622/177620 of the Samara
wire        ctl   = samara ? sam_ctl : bus_addr[0];
wire  [2:0] ridx  = (samara & sam_ctl) ? (bus_addr[1] ? REG_HEAD : REG_STATUS) : ~bus_addr[3:1];
// the byte a write carries: the low one, or the high one of a byte write
// to an odd address
wire  [7:0] wbyte = ~((bus_addr[0] & ~bus_wtbt[0]) ? bus_din[15:8] : bus_din[7:0]);
wire [15:0] wword = ~bus_din;

assign bus_ack = bus_stb & sel;

reg  [15:0] dout_r = 16'hFFFF;
assign bus_dout = (sel & ~bus_we) ? dout_r : 16'd0;

reg old_stb = 1'b0;
wire stb_rise = bus_stb & ~old_stb;
wire rd_strobe = stb_rise & sel & ~bus_we;
wire wr_strobe = stb_rise & sel &  bus_we;

//------------------------------------------------------------------------
// Sector buffers: two banks the engine fills and the bus drains, one the
// bus fills and hps_io takes.
//------------------------------------------------------------------------
reg  [7:0]  bufoff  = 8'd0;          // word index 0..255
reg         bwr     = 1'b0;
reg  [7:0]  bwaddr  = 8'd0;
reg  [15:0] bwdata  = 16'd0;
reg         cur     = 1'b0;          // the bank the bus reads
reg         fbank   = 1'b0;          // the bank a read fills
reg  [15:0] bdout;

reg  [15:0] rbuf[0:511];
reg  [15:0] wbuf[0:255];

always @(posedge clk_sys) if(sd_buff_wr && sd_ack) rbuf[{fbank, sd_buff_addr}] <= sd_buff_dout;
always @(posedge clk_sys) bdout <= rbuf[{cur, bufoff}];

always @(posedge clk_sys) if(bwr) wbuf[bwaddr] <= bwdata;
always @(posedge clk_sys) sd_buff_din <= wbuf[sd_buff_addr];

//------------------------------------------------------------------------
// The image, as hps_io mounts it
//------------------------------------------------------------------------
reg         mnt_tgl  = 1'b0;
reg         mnt_has  = 1'b0;         // non-empty
reg         mnt_ro   = 1'b0;
reg  [31:0] mnt_secs = 32'd0;

always @(posedge clk_sys) if(img_mounted) begin
	mnt_tgl  <= ~mnt_tgl;
	mnt_has  <= |img_size;
	mnt_ro   <= img_readonly;
	mnt_secs <= |img_size[63:32+SECTOR_SHIFT] ? 32'hFFFFFFFF : img_size[32+SECTOR_SHIFT-1:SECTOR_SHIFT];
end

reg  [2:0]  mnt_s = 3'b000;
reg  [1:0]  ack_s = 2'b00;
always @(posedge clk_sys) begin
	mnt_s <= {mnt_s[1:0], mnt_tgl};
	ack_s <= {ack_s[0], sd_ack};
end
wire mnt_new = mnt_s[2] ^ mnt_s[1];
wire acked   = ack_s[1];

//------------------------------------------------------------------------
// The geometry: sector 0 is read at mount time and checked for the .hdi
// header; without one the image is raw and gets the default geometry.
// The words are picked off hps_io's stream.
//------------------------------------------------------------------------
localparam [7:0] W_CYL = 8'd1, W_HEADS = 8'd3, W_SPT = 8'd6, W_LAST = 8'd255;
localparam [7:0] HDI_SIG = 8'hA5;
localparam [7:0] W_BK_TAB = 8'd8;     // BKHD: the table's first word
localparam [7:0] W_BK_END = W_BK_TAB + 8'd2 * (BK_PARTS + 1);

reg         probing = 1'b0;
reg  [15:0] p_cyl, p_heads, p_spt;
reg  [7:0]  p_sum;                   // the header's checksum: bytes 0..510 sum to -byte 511
reg         p_sig;
reg  [15:0] p_bk0, p_bk1;
reg  [7:0]  p_bkver;
reg  [31:0] bk_tab[0:BK_PARTS];      // BKHD: the partition boundaries

always @(posedge clk_sys) if(probing && sd_buff_wr && sd_ack) begin
	case(sd_buff_addr)
	W_CYL:   p_cyl   <= sd_buff_dout;
	W_HEADS: p_heads <= sd_buff_dout;
	W_SPT:   p_spt   <= sd_buff_dout;
	default: ;
	endcase
	case(sd_buff_addr)
	8'd0: p_bk0   <= sd_buff_dout;
	8'd1: p_bk1   <= sd_buff_dout;
	8'd2: p_bkver <= sd_buff_dout[7:0];
	default: ;
	endcase
	if(sd_buff_addr >= W_BK_TAB && sd_buff_addr < W_BK_END) begin
		if(sd_buff_addr[0]) bk_tab[(sd_buff_addr - W_BK_TAB) >> 1][31:16] <= sd_buff_dout;
		else                bk_tab[(sd_buff_addr - W_BK_TAB) >> 1][15:0]  <= sd_buff_dout;
	end
	if(sd_buff_addr == 8'd0) p_sum <= sd_buff_dout[7:0] + sd_buff_dout[15:8];
	else if(sd_buff_addr != W_LAST) p_sum <= p_sum + sd_buff_dout[7:0] + sd_buff_dout[15:8];
	else p_sig <= (sd_buff_dout[7:0] == HDI_SIG) && (sd_buff_dout[15:8] == (8'd0 - (p_sum + sd_buff_dout[7:0])));
end

// A divider: cyl = sectors / (spt * heads)
reg  [31:0] dv_n = 32'd0;            // the dividend, shifting into the quotient
reg  [16:0] dv_r = 17'd0;
reg  [15:0] dv_d = 16'd0;
reg  [5:0]  dv_cnt = 6'd0;
wire [16:0] dv_sh = {dv_r[15:0], dv_n[31]};
wire        dv_ge = dv_sh >= {1'b0, dv_d};
localparam [5:0] DIV_STEPS = 6'd32;

reg  [7:0]  geo_spt = DEF_SPT, geo_heads = DEF_HEADS;
reg  [15:0] geo_cyl = 16'd0;
reg         geo_hdr = 1'b0;          // the image starts with the .hdi header sector

localparam [2:0] P_IDLE = 3'd0, P_REQ = 3'd1, P_WAIT = 3'd2, P_GEO = 3'd3, P_DIV = 3'd4, P_CYL = 3'd5;
reg  [2:0]  pst = P_IDLE;
reg         p_rd = 1'b0;
reg         mounted = 1'b0;          // probed, with a geometry
reg         bkhd = 1'b0;             // a BKHD image
reg  [3:0]  bk_np = 4'd0;            // its partitions

wire        p_bkhd = p_bk0 == "KB" && p_bk1 == "DH" && p_bkver == 8'd1;

// the partitions there are: each one ends after it begins
function [3:0] bk_count;
	input dummy;
	integer i;
	reg ok;
	begin
		bk_count = 4'd0;
		ok = 1'b1;
		for (i = 0; i < BK_PARTS; i = i + 1)
			if (ok && bk_tab[i] != 32'd0 && bk_tab[i+1] > bk_tab[i]) bk_count = i[3:0] + 4'd1;
			else ok = 1'b0;
	end
endfunction

always @(posedge clk_sys) begin
	if(dv_cnt != 6'd0) begin
		dv_cnt <= dv_cnt - 6'd1;
		dv_r   <= dv_ge ? dv_sh - {1'b0, dv_d} : dv_sh;
		dv_n   <= {dv_n[30:0], dv_ge};
	end

	case(pst)
	P_IDLE: ;
	// a request goes up only once a transfer still running from before
	// the mount is over, so its ack is not taken for this one's
	P_REQ: begin
		if(!acked) begin p_rd <= 1'b1; probing <= 1'b1; end
		if(p_rd && acked) begin p_rd <= 1'b0; pst <= P_WAIT; end
	end
	P_WAIT: if(!acked) begin probing <= 1'b0; pst <= P_GEO; end
	P_GEO: begin
		bkhd  <= p_bkhd && bk_count(1'b0) != 4'd0;
		bk_np <= bk_count(1'b0);
		if(p_sig && p_heads[7:0] != 8'd0 && p_heads[7:0] <= 8'd16 && p_spt[7:0] != 8'd0 && p_spt[15:8] == 8'd0) begin
			geo_hdr   <= 1'b1;
			geo_spt   <= p_spt[7:0];
			geo_heads <= p_heads[7:0];
			geo_cyl   <= p_cyl;
			mounted   <= 1'b1;
			pst       <= P_IDLE;
		end else begin
			geo_hdr   <= 1'b0;
			geo_spt   <= DEF_SPT;
			geo_heads <= DEF_HEADS;
			dv_n      <= mnt_secs;
			dv_d      <= {8'd0, DEF_SPT} * {8'd0, DEF_HEADS};
			dv_r      <= 17'd0;
			dv_cnt    <= DIV_STEPS;
			pst       <= P_DIV;
		end
	end
	P_DIV: if(dv_cnt == 6'd0) pst <= P_CYL;
	P_CYL: begin
		geo_cyl <= |dv_n[31:16] ? MAX_CYL : dv_n[15:0];
		mounted <= 1'b1;
		pst     <= P_IDLE;
	end
	default: pst <= P_IDLE;
	endcase

	// a new image: drop the drive and probe it
	if(mnt_new) begin
		mounted <= 1'b0;
		bkhd    <= 1'b0;
		p_rd    <= 1'b0;
		probing <= 1'b0;
		pst     <= mnt_has ? P_REQ : P_IDLE;
	end
end

//------------------------------------------------------------------------
// Drive state
//------------------------------------------------------------------------
reg  [7:0]  status = ST_BUSY, error = ER_NONE;
reg  [8:0]  sectorcount = 9'd0;      // 1..256
reg  [7:0]  cursector = 8'd0, curheadreg = 8'd0, /*features = 8'd0,*/ control = 8'd0;
reg  [3:0]  curhead = 4'd0;
reg  [15:0] curcyl = 16'd0;
reg  [7:0]  spt_r = DEF_SPT, heads_r = DEF_HEADS;
reg  [7:0]  resetcnt = 8'd0;

// Transfer engine states
localparam [3:0] S_IDLE = 4'd0, S_RD_LBA1 = 4'd1, S_RD_LBA2 = 4'd2, S_RD_REQ = 4'd3,
				 S_RD_WAIT = 4'd4, S_WR_LBA1 = 4'd5, S_WR_LBA2 = 4'd6, S_WR_REQ = 4'd7,
				 S_WR_WAIT = 4'd8, S_RD_MAP = 4'd9, S_WR_MAP = 4'd10, S_RD_MAP2 = 4'd11,
				 S_WR_MAP2 = 4'd12;
reg  [3:0]  st = S_IDLE;
reg         id_mode = 1'b0;      // the data register reads the IDENTIFY sector
reg         fmt_mode = 1'b0;     // FORMAT TRACK: the data written goes nowhere
reg  [31:0] lba_c = 32'd0;          // CHS: the cylinder's first sector
reg  [15:0] lba_h = 16'd0;          // the head's first sector in it
reg  [15:0] hs_r  = 16'd0;          // sectors per cylinder
reg  [8:0]  sec0  = 9'd0;           // the sector register as an offset in the image
always @(posedge clk_sys) begin
	hs_r <= spt_r * heads_r;
	sec0 <= {1'b0, cursector} - 9'd1 + {8'd0, geo_hdr};
end
reg  [31:0] eng_lba = 32'd0;
reg         eng_rd = 1'b0, eng_wr = 1'b0;
// read ahead: the bus wants the next sector (drained its own, or the
// command just came) / the other bank already holds it / a READ came
// while a fetch was in flight, so its result is thrown away / a WRITE's
// buffer filled while a fetch was in flight
reg         rd_pending = 1'b0, pf_valid = 1'b0, restart = 1'b0, wr_pending = 1'b0;
reg         pf_bank = 1'b0;      // the bank the fetched sector sits in
wire        sd_writing = (st == S_WR_REQ) || (st == S_WR_WAIT);
assign active = (st == S_RD_REQ) || (st == S_RD_WAIT) || sd_writing || probing;

always @(posedge clk_sys) sd_lba <= probing ? 32'd0 : eng_lba;
assign sd_rd = eng_rd | p_rd;
assign sd_wr = eng_wr;

// The IDENTIFY DEVICE sector, word by word.  Strings have the first
// character in the high byte of each word.
function [15:0] ident;
	input [7:0] w;
	input [15:0] cyl; input [7:0] hd; input [7:0] sp; input [31:0] total;
	begin
		case (w)
		8'd0:  ident = 16'h045a;
		8'd1:  ident = cyl;
		8'd3:  ident = {8'd0, hd};
		8'd4:  ident = {sp[6:0], 1'b0, 8'd0};               // bytes per track = spt * 512
		8'd5:  ident = 16'd512;
		8'd6:  ident = {8'd0, sp};
		8'd10, 8'd11, 8'd12, 8'd13, 8'd14, 8'd15, 8'd16, 8'd17, 8'd18, 8'd19:
			   ident = "00";                     // serial "00000000000000000000"
		8'd20: ident = 16'd3;                    // buffer type
		8'd21: ident = 16'd1;                    // buffer size, 512-byte units
		8'd22: ident = 16'd4;                    // ECC bytes
		8'd23: ident = "2.";                     // firmware "2.05    "
		8'd24: ident = "05";
		8'd25, 8'd26: ident = "  ";
		8'd27: ident = "BK";                     // model "BK0011M MiSTer SMK512 Hard Disk"
		8'd28: ident = "00";
		8'd29: ident = "11";
		8'd30: ident = "M ";
		8'd31: ident = "Mi";
		8'd32: ident = "ST";
		8'd33: ident = "er";
		8'd34: ident = " S";
		8'd35: ident = "MK";
		8'd36: ident = "51";
		8'd37: ident = "2 ";
		8'd38: ident = "Ha";
		8'd39: ident = "rd";
		8'd40: ident = " D";
		8'd41: ident = "is";
		8'd42: ident = "k ";
		8'd43, 8'd44, 8'd45, 8'd46:
			   ident = "  ";
		8'd47: ident = 16'h8001;
		8'd49: ident = 16'h0200;                 // LBA supported
		8'd53: ident = 16'd1;
		8'd54: ident = cyl;
		8'd55: ident = {8'd0, hd};
		8'd56: ident = {8'd0, sp};
		8'd57: ident = total[15:0];
		8'd58: ident = total[31:16];
		8'd60: ident = total[15:0];
		8'd61: ident = total[31:16];
		default: ident = 16'd0;
		endcase
	end
endfunction

wire [31:0] data_secs = mnt_secs - {31'd0, geo_hdr};
wire        wprot     = mnt_ro;

// LBA28 mode: the sector number is the register bytes themselves
wire        lba_mode = curheadreg[HEAD_LBA];
wire [27:0] lba_cur  = {curheadreg[3:0], curcyl, cursector};
wire [27:0] lba_nxt  = lba_cur + 28'd1;

// A BKHD image under the Samara controller: partition i (unit 2+i) has its
// descriptor at block s_i = start_i + 2 + i and its data right after, the
// image's start_i on.  Block 1 is the table, blocks 0 and 2 empty.
wire        bk_on = bkhd & samara;
localparam [3:0] BK_TABLE = 4'd8, BK_EMPTY = 4'd9;   // made-up sector kinds; 0..7 a descriptor
localparam [15:0] BK_PARAM = 16'o2000, BK_LOAD = 16'o1000, BK_REG16 = 16'o14400;   // as SRV writes them

// worked out once the table is there; registered, they settle long before
// the first access
reg  [31:0] bk_s[0:BK_PARTS-1];      // the descriptor block
reg  [31:0] bk_e[0:BK_PARTS-1];      // the block after the partition
reg  [15:0] bk_vol[0:BK_PARTS-1];    // the data blocks
genvar gi;
generate for (gi = 0; gi < BK_PARTS; gi = gi + 1) begin : bkp
	wire [31:0] len = bk_tab[gi+1] - bk_tab[gi];
	always @(posedge clk_sys) begin
		bk_s[gi]   <= bk_tab[gi] + 32'd2 + gi;
		bk_e[gi]   <= bk_tab[gi+1] + 32'd3 + gi;
		bk_vol[gi] <= |len[31:16] ? 16'hFFFF : len[15:0];
	end
end endgenerate

reg         syn_b[0:1];              // the bank holds a made-up sector
reg  [3:0]  syn_k[0:1];              // of this kind

// a made-up sector's words, as the software has them
function [15:0] bk_word;
	input [3:0] k;
	input [7:0] w;
	reg   [7:0] u;
	begin
		bk_word = 16'd0;
		if (k == BK_TABLE) begin
			u = (w - 8'd3) >> 1;
			case (w)
			8'd0: bk_word = 16'd2;                         // boot from C:
			8'd1: bk_word = hs_r;
			8'd2: bk_word = {heads_r - 8'd1, spt_r};
			default: if (w >= 8'd3 && u < {4'd0, bk_np})
				bk_word = w[0] ? bk_s[u[2:0]][15:0] : bk_s[u[2:0]][31:16];
			endcase
		end else if (k < BK_TABLE)
			case (w)
			8'd0: bk_word = 16'd2 + k;
			8'd1: bk_word = bk_vol[k[2:0]];
			8'd3: bk_word = BK_PARAM;
			8'd4: bk_word = BK_LOAD;
			8'd5: bk_word = BK_REG16;
			8'd6: bk_word = bk_s[k[2:0]][15:0];
			8'd7: bk_word = bk_s[k[2:0]][31:16];
			8'd8: bk_word = bk_e[k[2:0]][15:0];
			8'd9: bk_word = bk_e[k[2:0]][31:16];
			default: ;
			endcase
	end
endfunction

// next sector: CHS, or the LBA counted up
task next_sector;
	begin
	  if (lba_mode) begin
		cursector       <= lba_nxt[7:0];
		curcyl          <= lba_nxt[23:8];
		curheadreg[3:0] <= lba_nxt[27:24];
		curhead         <= lba_nxt[27:24];
	  end else begin
		// sectors are 1-based, heads compared in full width
		if ({1'b0, cursector} + 9'd1 > {1'b0, spt_r}) begin
			cursector <= 8'd1;
			if ({4'd0, curhead} + 8'd1 >= heads_r) begin
				curhead <= 4'd0;
				curcyl  <= curcyl + 16'd1;
			end else
				curhead <= curhead + 4'd1;
		end else
			cursector <= cursector + 8'd1;
	  end
	end
endtask

// the drive's power-on / software reset state
task drive_reset;
	begin
		status      <= ST_BUSY;
		error       <= ER_DIAG_OK;
		resetcnt    <= RESET_CLOCKS;
		st          <= S_IDLE;
		eng_rd      <= 1'b0;
		eng_wr      <= 1'b0;
		bufoff      <= 8'd0;
		sectorcount <= 9'd1;
		cursector   <= 8'd1;
		curcyl      <= 16'd0;
		curheadreg  <= 8'd0;
		curhead     <= 4'd0;
		cur         <= 1'b0;
		fbank       <= 1'b0;
		rd_pending  <= 1'b0;
		pf_valid    <= 1'b0;
		restart     <= 1'b0;
		wr_pending  <= 1'b0;
		pf_bank     <= 1'b0;
		id_mode     <= 1'b0;
		fmt_mode    <= 1'b0;
		spt_r       <= geo_spt;
		heads_r     <= geo_heads;
	end
endtask

// the data word the bus reads, every source registered like the buffer
// (a BKHD image holds the data as the software has it, a Samara one inverted)
reg  [15:0] idw, bkw;
always @(posedge clk_sys) begin
	idw <= ident(bufoff, geo_cyl, heads_r, spt_r, data_secs);
	bkw <= bk_word(syn_k[cur], bufoff);
end
wire [15:0] rd_word = id_mode ? idw : !bk_on ? bdout : syn_b[cur] ? ~bkw : ~bdout;
// the drive address register: drive 0 selected, head bits, write gate off
wire  [7:0] dar = {1'b1, 1'b1, ~curhead, 1'b1, 1'b0};
// the head register's DEV bit: the slave, which is not there
wire        dev1 = curheadreg[4];

reg  [BK_PARTS-1:0] bk_ge, bk_eq;   // the block is at / past partition i's start
reg         bk_low;                  // before the first partition
reg  [2:0]  bk_part;
integer     bi;

always @(posedge clk_sys) begin
	old_stb <= bus_stb;
	bwr     <= 1'b0;

	if (reset) begin
		drive_reset;
		control <= 8'd0;
		dout_r  <= 16'hFFFF;
	end else if (!mounted) begin
		// no drive on the cable: the bus reads all ones (inverted zeros)
		drive_reset;
		dout_r  <= 16'hFFFF;
	end else begin
		// power-on / reset timing
		if (resetcnt != 8'd0) begin
			resetcnt <= resetcnt - 8'd1;
			if (resetcnt == 8'd1)
				status <= (status & ~ST_BUSY) | ST_READY | ST_SEEK;
		end

		// handing a fetched sector to the bus: it is in a bank and the bus
		// wants it.  Then the bank just freed is filled with the next sector
		// of the command while this one drains.
		if (rd_pending && pf_valid) begin
			rd_pending <= 1'b0;
			pf_valid   <= 1'b0;
			id_mode    <= 1'b0;
			cur        <= pf_bank;
			bufoff     <= 8'd0;
			status     <= (status & ~ST_BUSY & ~ST_ERROR) | ST_DRQ | ST_SEEK;
			if (sectorcount != 9'd0 && st == S_IDLE) begin
				fbank <= ~pf_bank;
				st    <= S_RD_LBA1;
			end
		end

		//----------------------------------------------------------------
		// The bus: a read latches the word on the strobe's first clock, a
		// write takes the value there.  Everything inverted.
		//----------------------------------------------------------------
		if (rd_strobe) begin
			if (dev1) dout_r <= 16'hFFFF;                     // the slave: nobody there, status 0
			else case (ridx)
			REG_DATA: begin
				dout_r <= ~rd_word;
				if (status & ST_DRQ) begin
					if (bufoff == LAST_WORD) begin           // sector consumed: ContinueRead
						bufoff <= 8'd0;
						status <= status & ~ST_DRQ & ~ST_BUSY;
						if (!id_mode && (pf_valid || sectorcount != 9'd0 || st != S_IDLE)) begin
							rd_pending <= 1'b1;              // the next one: in the other bank, or on its way
							status     <= (status & ~ST_DRQ) | ST_BUSY;
							if (!pf_valid && st == S_IDLE) begin fbank <= ~cur; st <= S_RD_LBA1; end
						end
						id_mode <= 1'b0;
					end else
						bufoff <= bufoff + 8'd1;
				end
			end
			REG_ERROR:  dout_r <= ~{8'h00, error};
			REG_COUNT:  dout_r <= ~{8'h00, sectorcount[7:0]};
			REG_SECTOR: dout_r <= ~{8'h00, cursector};
			REG_CYL_LO: dout_r <= ~{8'h00, curcyl[7:0]};
			REG_CYL_HI: dout_r <= ~{8'h00, curcyl[15:8]};
			REG_HEAD:   dout_r <= (samara & ctl) ? ~{8'h00, status} : ~{status, curheadreg};   // 177743 / 177622: alternate status
			REG_STATUS: dout_r <= (samara & ctl) ? ~{8'h00, dar}    : ~{dar, status};         // 177741 / 177620: drive address
			endcase
		end

		if (wr_strobe) begin
			if (ctl) begin
				// 177743 / 177622: device control.  SRST set: BUSY; released:
				// the drive comes up again.  177741 / 177620 takes nothing.
				if (ridx == REG_HEAD) begin
					control <= wbyte;
					if (!control[CTL_SRST] && wbyte[CTL_SRST]) begin
						status <= ST_BUSY;
						st     <= S_IDLE;
						eng_rd <= 1'b0;
						eng_wr <= 1'b0;
					end
					if (control[CTL_SRST] && !wbyte[CTL_SRST]) drive_reset;
				end
			end else if (!dev1 || ridx == REG_HEAD || (ridx == REG_STATUS && wbyte == 8'h90)) begin
				// the slave selected: only the head register (to come back) and
				// EXECUTE DIAGNOSTICS reach the drive
				case (ridx)
				REG_DATA: if (status & ST_DRQ) begin            // DATA write
					bwr    <= 1'b1;
					bwaddr <= bufoff;
					bwdata <= bk_on ? ~wword : wword;
					if (bufoff == LAST_WORD) begin               // ContinueWrite
						bufoff <= 8'd0;
						if (fmt_mode) begin                      // FORMAT TRACK: done with the table
							fmt_mode <= 1'b0;
							status   <= (status & ~ST_DRQ & ~ST_BUSY & ~ST_ERROR) | ST_READY | ST_SEEK;
						end else begin
							status <= (status & ~ST_DRQ) | ST_BUSY;
							if (st == S_IDLE) st <= S_WR_LBA1;
							else              wr_pending <= 1'b1;    // a read-ahead is still in flight
						end
					end else
						bufoff <= bufoff + 8'd1;
				end
				//REG_ERROR:  features <= wbyte;
				REG_COUNT:  sectorcount <= (wbyte == 8'd0) ? 9'd256 : {1'b0, wbyte};
				REG_SECTOR: cursector <= wbyte;
				REG_CYL_LO: curcyl[7:0] <= wbyte;
				REG_CYL_HI: curcyl[15:8]<= wbyte;
				REG_HEAD:   begin curheadreg <= wbyte; curhead <= wbyte[3:0]; end
				REG_STATUS: if (!(status & ST_BUSY)) begin       // COMMAND
					casez (wbyte)
					8'b0010_00??: begin                          // READ SECTORS (20-23)
						status     <= (status | ST_BUSY) & ~ST_DRQ & ~ST_ERROR;
						rd_pending <= 1'b1;
						pf_valid   <= 1'b0;
						id_mode    <= 1'b0;
						fmt_mode   <= 1'b0;
						fbank      <= cur;
						if (st == S_IDLE) st <= S_RD_LBA1;
						else              restart <= 1'b1;       // a read-ahead of the last command is in flight
					end
					8'b0011_00??: begin                          // WRITE SECTORS (30-33)
						bufoff   <= 8'd0;
						pf_valid <= 1'b0;
						id_mode  <= 1'b0;
						fmt_mode <= 1'b0;
						status   <= (status | ST_DRQ) & ~ST_ERROR;
					end
					8'b0100_00??: begin                          // VERIFY SECTORS (40-43)
						error  <= ER_NONE;
						status <= (status & ~ST_BUSY & ~ST_DRQ & ~ST_ERROR) | ST_READY | ST_SEEK;
					end
					8'b0101_0???: begin                          // FORMAT TRACK (50): takes the table, writes nothing
						bufoff   <= 8'd0;
						pf_valid <= 1'b0;
						id_mode  <= 1'b0;
						fmt_mode <= 1'b1;
						error    <= ER_NONE;
						status   <= (status | ST_DRQ) & ~ST_ERROR;
					end
					8'b0001_????, 8'b0111_????: begin            // RECALIBRATE (1x), SEEK (7x)
						error  <= ER_NONE;
						status <= (status & ~ST_BUSY & ~ST_DRQ & ~ST_ERROR) | ST_READY | ST_SEEK;
					end
					8'h90: begin                                 // EXECUTE DIAGNOSTICS
						error  <= ER_DIAG_OK;
						status <= (status & ~ST_BUSY & ~ST_DRQ & ~ST_ERROR) | ST_READY | ST_SEEK;
					end
					8'h91: begin                                 // INITIALIZE DRIVE PARAMETERS
						spt_r   <= sectorcount[7:0];
						heads_r <= {4'd0, curhead} + 8'd1;
						error   <= ER_NONE;
						status  <= (status & ~ST_BUSY & ~ST_DRQ & ~ST_ERROR) | ST_READY | ST_SEEK;
					end
					8'hEC: begin                                 // IDENTIFY DEVICE
						id_mode     <= 1'b1;
						fmt_mode    <= 1'b0;
						bufoff      <= 8'd0;
						pf_valid    <= 1'b0;
						sectorcount <= 9'd1;
						error       <= ER_NONE;
						status      <= (status | ST_DRQ | ST_SEEK | ST_READY) & ~ST_BUSY & ~ST_ERROR;
					end
					default: begin                               // anything else: aborted
						error  <= ER_ABRT;
						status <= (status & ~ST_BUSY & ~ST_DRQ) | ST_READY | ST_ERROR;
					end
					endcase
				end
				endcase
			end
		end

		//----------------------------------------------------------------
		// The transfer engine
		//----------------------------------------------------------------
		case (st)
		S_IDLE: ;
		// the sector number in two multiply stages, then the request
		S_RD_LBA1, S_WR_LBA1: begin
			if (lba_mode) begin
				eng_lba <= {4'd0, lba_cur} + {31'd0, geo_hdr};
				st      <= (st == S_RD_LBA1) ? S_RD_MAP : S_WR_MAP;
			end else begin
				lba_c <= curcyl * hs_r;
				lba_h <= curhead * spt_r;
				st    <= (st == S_RD_LBA1) ? S_RD_LBA2 : S_WR_LBA2;
			end
		end
		S_RD_LBA2, S_WR_LBA2: begin
			eng_lba <= lba_c + {16'd0, lba_h} + {{23{sec0[8]}}, sec0};
			st      <= (st == S_RD_LBA2) ? S_RD_MAP : S_WR_MAP;
		end
		// a BKHD image: the block in the image, or a made-up one, which a
		// read takes from bk_word and a write drops.  First where the block
		// falls, then what it is.
		S_RD_MAP, S_WR_MAP: begin
			if (st == S_RD_MAP) syn_b[fbank] <= 1'b0;
			for (bi = 0; bi < BK_PARTS; bi = bi + 1) begin
				bk_ge[bi] <= bi < bk_np && eng_lba >= bk_s[bi];
				bk_eq[bi] <= eng_lba == bk_s[bi];
			end
			bk_low <= eng_lba < bk_s[0];
			if (!bk_on) st <= (st == S_RD_MAP) ? S_RD_REQ : S_WR_REQ;
			else        st <= (st == S_RD_MAP) ? S_RD_MAP2 : S_WR_MAP2;
		end
		S_RD_MAP2, S_WR_MAP2: begin
			bk_part = 3'd0;
			for (bi = 1; bi < BK_PARTS; bi = bi + 1)
				if (bk_ge[bi]) bk_part = bi[2:0];
			if (bk_low || bk_eq[bk_part]) begin
				if (st == S_RD_MAP2) syn_b[fbank] <= 1'b1;
				syn_k[fbank] <= (eng_lba == 32'd1) ? BK_TABLE : bk_low ? BK_EMPTY : {1'b0, bk_part};
				st           <= (st == S_RD_MAP2) ? S_RD_WAIT : S_WR_WAIT;
			end else begin
				eng_lba <= eng_lba - 32'd3 - bk_part;
				st      <= (st == S_RD_MAP2) ? S_RD_REQ : S_WR_REQ;
			end
		end
		// hps_io takes the request, raises sd_ack for the transfer and
		// drops it after; a request goes up only with no ack left over
		// from a transfer the reset cut short
		S_RD_REQ: begin
			if (!acked) eng_rd <= 1'b1;
			if (eng_rd && acked) begin eng_rd <= 1'b0; st <= S_RD_WAIT; end
		end
		S_RD_WAIT: if (!acked) begin                              // ReadSectorDone
			if (restart) begin                                    // a newer READ: fetch its sector instead
				restart <= 1'b0;
				fbank   <= cur;
				st      <= S_RD_LBA1;
			end else begin
				error <= ER_NONE;
				if (sectorcount != 9'd0) sectorcount <= sectorcount - 9'd1;
				if (sectorcount > 9'd1) next_sector;
				// the sector is in fbank; the hand-over above gives it to
				// the bus when it asks, and starts the read ahead of the
				// next one into the bank that frees
				pf_valid <= 1'b1;
				pf_bank  <= fbank;
				if (wr_pending) begin wr_pending <= 1'b0; st <= S_WR_LBA1; end
				else            st <= S_IDLE;
			end
		end
		S_WR_REQ: begin
			if (wprot) begin
				// a read-only image: no write, ERROR with ABORTED, the
				// buffer left as it is
				status <= (status & ~ST_BUSY & ~ST_DRQ) | ST_ERROR | ST_READY | ST_SEEK;
				error  <= ER_ABRT;
				bufoff <= 8'd0;
				st     <= S_IDLE;
			end else begin
				if (!acked) eng_wr <= 1'b1;
				if (eng_wr && acked) begin eng_wr <= 1'b0; st <= S_WR_WAIT; end
			end
		end
		S_WR_WAIT: if (!acked) begin                              // WriteSectorDone
			error  <= ER_NONE;
			bufoff <= 8'd0;
			if (sectorcount != 9'd0) sectorcount <= sectorcount - 9'd1;
			if (sectorcount > 9'd1) begin
				next_sector;
				status <= (status & ~ST_BUSY & ~ST_ERROR) | ST_DRQ | ST_SEEK;
			end else
				status <= (status & ~ST_BUSY & ~ST_DRQ & ~ST_ERROR) | ST_READY | ST_SEEK;
			st <= S_IDLE;
		end
		default: st <= S_IDLE;
		endcase
	end
end

endmodule
