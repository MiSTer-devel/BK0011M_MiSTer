//
// Four floppy drives behind the KR1801VP1-128 (vp1_128.sv).
//
// From UKNC_MiSTer / tang-uknc fdd4.v (MIT): a spinning track of ten 512-byte
// sectors - gaps, marks and headers from the template in rawtrk.hex, the data
// from the image through hps_io's sector interface. Words written by the CPU
// are parsed: the 256 words after a data mark are a sector, sent to the image
// when complete; its number comes from a header written just before (format)
// or from the header passing under the head (sector write).
//
// Image layout: (track * 2 + head) * 10 + sector, 512 bytes each, 819200 bytes
// for 80 tracks. An empty drive holds IND up (the ROM reports error 6); a
// write-protected one takes no data.
//
module fdd4 #(parameter CLK_HZ = 96000000)
(
	input             clk,
	input             rst,

	input      [15:0] data_in,      // the controller's data register, written by the CPU
	output reg [15:0] data_out,
	input             write,        // the PPU writes data_in

	input      [ 1:0] drive,
	input             motor,
	input             step,
	input             dir,
	input             head,
	output            valid,
	output            sync,
	output            crc_ok,
	output            rdy,
	output            tr0,
	output            ind,

	input      [ 3:0] mount_dsk,    // an image is in the drive
	input      [ 3:0] wprot_dsk,    // and is write protected

	// hps_io sector interface, on clk

	output     [31:0] sd_lba,
	output     [ 3:0] sd_rd,
	output     [ 3:0] sd_wr,
	input      [ 3:0] sd_ack,
	input      [ 7:0] sd_buff_addr,
	input      [15:0] sd_buff_dout,
	output     [15:0] sd_buff_din,
	input             sd_buff_wr
);
//---------------------------------------------------------------------------------
// Size dsk image in sectors
//---------------------------------------------------------------------------------
// 1600 = 819200 / 512
//---------------------------------------------------------------------------------
// Spin disk emulation
//---------------------------------------------------------------------------------
// 3125 16-bit words per revolution at 300 rpm: one word in 64 us (250 kbit/s
// MFM), CLK_HZ / 15625 clocks.
//---------------------------------------------------------------------------------
localparam WORD_CYCLES   = CLK_HZ / 15625;   // clocks per word, 64 us
localparam CNT_W         = $clog2(WORD_CYCLES);
localparam VALID_FIRST   = WORD_CYCLES / 160;       // TR window inside a word:
localparam VALID_LAST    = (WORD_CYCLES * 9) / 16;  // 0.4 us .. 36 us, as at 25 MHz
localparam SECTORS       = 10;        // per track side
localparam SECTOR_WORDS  = 256;

// Word positions in a sector, as in the track template (rawtrk.hex)
localparam W_SYNC_HDR    = 9'h18;     // A1A1 before the header mark
localparam W_TRK         = 9'h1A;     // header: track, head
localparam W_SEK         = 9'h1B;     // header: sector, size
localparam W_CRC_HDR     = 9'h1C;     // header CRC; the sector is fetched here
localparam W_SYNC_DATA   = 9'h2F;     // A1A1 before the data mark
localparam W_DATA_FIRST  = 9'h31;
localparam W_DATA_LAST   = 9'h130;
localparam W_CRC_DATA    = 9'h131;
localparam W_SECTOR_LAST = 9'h132;    // the next sector starts after this
localparam W_TRACK_LAST  = 9'h168;    // the gap after the tenth sector ends here
localparam W_INDEX_END   = 9'd75;     // index hole, see ind below

// The controller's data register has its bytes the other way round for
// writing: a word read is (first byte << 8) | second byte, a word written
// goes onto the disk low byte first (as UKNCBTL's Floppy.cpp has it).  So
// the marks the PPU writes are byte-swapped against the template's, and the
// data words it writes are the little-endian words of the image.
localparam WR_MARK_HEADER = 16'hFEA1;   // A1 FE on the disk
localparam WR_MARK_DATA   = 16'hFBA1;   // A1 FB
localparam GAP_WORD      = 16'h4E4E;

reg  [CNT_W-1:0] count_clk = 0;
reg  [ 8:0] word_sectr =      0;
reg  [ 3:0] no_sec     =   4'd8;
reg  [ 7:0] no_trk     =   8'd5;
reg         clk_dsk    =      0;   // one clock high per word - an ENABLE, not a clock
reg         clk_dsk_n  =      0;   // the same, one cycle later
//---------------------------------------------------------------------------------
// Everything is on clk, the sector interface included.
//---------------------------------------------------------------------------------
reg  [1:0]  step_s     = 2'b00;
always @(posedge clk) step_s <= {step_s[0], step};
wire        step_r     = step_s[0];              // step, synchronised
wire        step_rise  = step_s[0] & ~step_s[1]; // its rising edge
//---------------------------------------------------------------------------------
wire        disk_insert = mount_dsk[drive];
wire        spinning    = motor && disk_insert;

assign valid    = count_clk > VALID_FIRST && count_clk < VALID_LAST && spinning;
assign sync     = (word_sectr == W_SYNC_HDR || word_sectr == W_SYNC_DATA) && spinning;
assign crc_ok   = (word_sectr == W_CRC_HDR  || word_sectr == W_CRC_DATA ) && spinning;
// The index hole: once a revolution, the first 75 words of the track (150
// bytes, BKBTL's FLOPPY_INDEXLENGTH).  An empty drive holds the line up:
// the ROM waits for it to drop and reports error 6, as with no disk.
assign ind      = !disk_insert || (no_sec == 4'd0 && word_sectr < W_INDEX_END && motor);
assign tr0      = !no_trk;
assign rdy      = spinning;

wire [15:0] data_raw;
wire [15:0] data_sec;
reg         _trk  = 1'b0;
reg         _sek  = 1'b0;
reg         _data = 1'b0;

always @(*)
	case({disk_insert,_data,_sek,_trk})
	'b1000 : data_out = data_raw;
	'b1001 : data_out = {no_trk,7'd0,head};
	'b1010 : data_out = {3'd0,no_sec+1'b1,8'd2};
	'b1100 : data_out = {data_sec[7:0],data_sec[15:8]};
	default: data_out = GAP_WORD;
	endcase
//---------------------------------------------------------------------------------
reg         head_old = 1'b1;
reg  [ 1:0] drive_old= 1'b0;

always @(posedge clk)
	if(step_rise)no_trk <= dir ? (no_trk == 8'd82 ? no_trk : no_trk + 1'b1) : (no_trk ? no_trk - 1'b1 : 8'd0);

always @(posedge clk)
	if(!motor || step_r || head_old!=head || drive_old!=drive)begin
		word_sectr <=  9'h0;
		clk_dsk    <=  1'b0;
		clk_dsk_n  <=  1'b0;
		count_clk  <=  0;
		head_old   <=  head;
		drive_old  <= drive;
	end else begin
//------ disk rotate immitation ---------------------------------------------------
		clk_dsk    <=       1'b0;
		clk_dsk_n  <=    clk_dsk;

		count_clk <= count_clk==WORD_CYCLES-1 ? 13'd0 : count_clk + 1'b1;
		if(!count_clk)begin
			clk_dsk <= 1'b1;
			if(word_sectr==W_TRACK_LAST)begin no_sec <= 4'h0; word_sectr <= 9'h0; end
			else
				if(word_sectr==W_SECTOR_LAST && no_sec < SECTORS-1)begin no_sec <= no_sec + 1'b1; word_sectr <= 9'h0; end
				else word_sectr <= word_sectr + 1'b1;
		end
//---------------------------------------------------------------------------------
	end
//---------------------------------------------------------------------------------
// raw sectr, emul raw trk
// kolesiko krutit'sa - lave mutit'sa
//---------------------------------------------------------------------------------
reg  [15:0] rawtrk[0:511];
reg  [15:0] raw_q = 16'h0000;
initial $readmemh("rtl/fdd/rawtrk.hex", rawtrk);

// read once a word, as tang-uknc's ROM did
always @(posedge clk) if(clk_dsk_n && motor) raw_q <= rawtrk[word_sectr];
assign data_raw = raw_q;
//---------------------------------------------------------------------------------
// Which part of the sector passes the head
//---------------------------------------------------------------------------------
function [31:0] lba(input [7:0] trk, input hd, input [3:0] sec);
	lba = ({trk,hd} * SECTORS) + sec;
endfunction

reg  sd_rd_word = 1'b0;   // the header CRC is passing: time to fetch the sector

always @(posedge clk)
	if(!motor)begin
		sd_rd_word <= 1'b0;
		_trk  <= 1'b0;
		_sek  <= 1'b0;
		_data <= 1'b0;
	end else if(clk_dsk_n)begin
		sd_rd_word <= word_sectr == W_CRC_HDR;
		_trk       <= word_sectr == W_TRK;
		_sek       <= word_sectr == W_SEK;
		_data      <= word_sectr >= W_DATA_FIRST && word_sectr <= W_DATA_LAST;
	end
//---------------------------------------------------------------------------------
// Sector requests
//
// hps_io takes a request (a bit of sd_rd or sd_wr up), raises that drive's
// sd_ack for the transfer and drops it after.  One request at a time: a
// read is not asked for while a write waits, and a write waits until no
// read is outstanding, so an ack always belongs to the one request up.
//---------------------------------------------------------------------------------
reg  [ 1:0] ack_s    = 2'b00;
always @(posedge clk) ack_s <= {ack_s[0], |sd_ack};
wire        ack      = ack_s[0];
wire        ack_rise = ack_s[0] & ~ack_s[1];

reg         read_sd    = 1'b0;
reg         sd_rd_d    = 1'b0;
reg         wr_pending = 1'b0;    // a written sector is in wbuf
reg         wr_go      = 1'b0;    // and is asked for
reg  [31:0] wr_lba     = 0;
reg  [ 1:0] wr_drive   = 0;
wire        cap_done;
wire [31:0] cap_lba;
wire [ 1:0] cap_drive;

always @(posedge clk)begin
	sd_rd_d <= sd_rd_word;
	if(!motor || ack || wr_pending) read_sd <= 1'b0;
	else if(sd_rd_word && !sd_rd_d) read_sd <= disk_insert;

	if(cap_done)begin
		wr_pending <= 1'b1;
		wr_lba     <= cap_lba;
		wr_drive   <= cap_drive;
	end
	if(wr_pending && !wr_go && !ack && !read_sd) wr_go <= 1'b1;
	if(wr_go && ack_rise)begin
		wr_go      <= 1'b0;
		wr_pending <= 1'b0;
	end
end

assign sd_lba = wr_go ? wr_lba : lba(no_trk, head, no_sec);
assign sd_rd  = {3'b000, read_sd} << drive;
assign sd_wr  = {3'b000, wr_go}   << wr_drive;
//---------------------------------------------------------------------------------
// Read buffer: filled by the HPS, read a word at a time as the data field
// passes.  Little-endian words, byte 2n in bits 7:0.
//---------------------------------------------------------------------------------
reg  [ 7:0] count_word = 8'd0;

always @(posedge clk)
	if(clk_dsk)count_word <= _data ? count_word + 1'b1 : 8'd0;

reg  [15:0] rbuf[0:SECTOR_WORDS-1];
reg  [15:0] rbuf_q;

always @(posedge clk) if(sd_buff_wr && |sd_ack) rbuf[sd_buff_addr] <= sd_buff_dout;
always @(posedge clk) rbuf_q <= rbuf[count_word];
assign data_sec = rbuf_q;
//---------------------------------------------------------------------------------
// Writing: parse the words the CPU writes.  A write-protected disk takes
// nothing (the ROM checks the WPR bit first anyway).
//---------------------------------------------------------------------------------
localparam CAP_IDLE = 2'd0;
localparam CAP_TRK  = 2'd1;   // after a header mark: the track word
localparam CAP_SEK  = 2'd2;   // then the sector word
localparam CAP_DATA = 2'd3;   // after a data mark: the sector's words

reg  [ 1:0] cap_state = CAP_IDLE;
reg  [ 7:0] cap_idx   = 8'd0;
reg  [ 3:0] hdr_sec   = 4'd0;
reg         hdr_valid = 1'b0;
reg  [31:0] cap_lba_r = 0;
reg  [ 1:0] cap_drv_r = 0;
reg         cap_done_r= 1'b0;
reg         write_d   = 1'b0;
wire        wr_word   = write & ~write_d;
wire [ 7:0] wr_sec    = data_in[7:0];    // the header's sector byte; numbers count from 1

reg  [15:0] wbuf[0:SECTOR_WORDS-1];
reg  [15:0] wbuf_q;

always @(posedge clk)begin
	write_d    <= write;
	cap_done_r <= 1'b0;
	if(!spinning || wprot_dsk[drive] || step_r || head_old!=head || drive_old!=drive)begin
		cap_state <= CAP_IDLE;
		hdr_valid <= 1'b0;
	end else if(wr_word)
		case(cap_state)
		CAP_IDLE:
			if(data_in == WR_MARK_HEADER) cap_state <= CAP_TRK;
			else if(data_in == WR_MARK_DATA)begin
				cap_state <= CAP_DATA;
				cap_idx   <= 8'd0;
				cap_lba_r <= lba(no_trk, head, hdr_valid ? hdr_sec : no_sec);
				cap_drv_r <= drive;
				hdr_valid <= 1'b0;
			end
		CAP_TRK:
			cap_state <= CAP_SEK;
		CAP_SEK:begin
			cap_state <= CAP_IDLE;
			hdr_valid <= wr_sec >= 1 && wr_sec <= SECTORS;
			hdr_sec   <= wr_sec[3:0] - 1'd1;
		end
		CAP_DATA:begin
			wbuf[cap_idx] <= data_in;
			cap_idx       <= cap_idx + 1'd1;
			if(cap_idx == SECTOR_WORDS-1)begin
				cap_state  <= CAP_IDLE;
				cap_done_r <= 1'b1;
			end
		end
		endcase
end

assign cap_done  = cap_done_r;
assign cap_lba   = cap_lba_r;
assign cap_drive = cap_drv_r;

always @(posedge clk) wbuf_q <= wbuf[sd_buff_addr];
assign sd_buff_din = wbuf_q;
//---------------------------------------------------------------------------------
endmodule
