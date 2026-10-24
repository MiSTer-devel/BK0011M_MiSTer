//
// KR1801VP1-128 floppy controller (KNGMD) on the CPU bus: 177130 control/status,
// 177132 data. The drives are in fdd4.v. Modelled after BKBTL's Floppy.cpp.
//
// 177130 write: bits 0-3 drive select (lowest set bit wins), 4 motor, 5 head,
//               6 step direction, 7 step, 8 start reading (the mark hunt begins
//               when it goes back down), 9 write mark, 10 precompensation.
// 177130 read:  0 TR0, 1 RDY, 2 WPR, 7 TR, 14 CRC, 15 IND; zero with no drive.
// 177132: the data word; any access drops TR, a read leaves write mode.
// SMK/A16M hide both registers from reads while bit 2 of the mode register is
// set (rd_block): the access is not answered then.
//
module vp1_128
(
	input         clk_sys,
	input         reset,
	input         enable,        // the controller is in the machine
	input         rd_block,      // SMK/A16M: reads of 177130/177132 disabled

	input  [15:0] bus_din,
	output [15:0] bus_dout,
	input  [15:0] bus_addr,
	input         bus_sync,
	input         bus_we,
	input         bus_stb,
	output        bus_ack,

	// the drives, fdd4.v
	input  [15:0] data_in,       // the word under the head
	output reg [15:0] data_out,  // the word to write
	output reg    write,         // one clock: data_out is to be written
	output reg [1:0] drive,
	output reg    motor,
	output reg    step,
	output reg    dir,
	output reg    head,
	input         valid,         // a word is ready (rises once a word)
	input         sync,          // a mark is passing
	input         crc_ok,
	input         rdy,
	input         tr0,
	input         ind,
	input   [3:0] wprot_dsk
);

wire sel    = enable & bus_sync & (bus_addr[15:2] == 14'o37626);   // 177130..177133
wire sel132 = sel &  bus_addr[1];
wire sel130 = sel & ~bus_addr[1];

assign bus_ack = bus_stb & sel & (bus_we | ~rd_block);

reg        drive_sel = 1'b0;    // some drive is selected
reg        tr = 1'b0;
reg        gdr = 1'b0;          // bit 8 as last written
reg        find_sync = 1'b0;
reg [15:0] dout = 16'd0;
wire       wpr = wprot_dsk[drive];

wire [15:0] status = drive_sel ? {ind, crc_ok, 6'd0, tr & ~find_sync & motor, 4'd0, wpr, rdy, tr0} : 16'd0;

assign bus_dout = (sel & ~bus_we & ~rd_block) ? (bus_addr[1] ? dout : status) : 16'd0;

reg old_stb = 1'b0, old_valid = 1'b0;
wire stb_rise = bus_stb & ~old_stb;

always @(posedge clk_sys) begin
	old_stb   <= bus_stb;
	old_valid <= valid;
	write     <= 1'b0;
	step      <= 1'b0;

	if(reset) begin
		drive_sel <= 1'b0;
		drive     <= 2'd0;
		motor     <= 1'b0;
		head      <= 1'b0;
		dir       <= 1'b0;
		tr        <= 1'b0;
		gdr       <= 1'b0;
		find_sync <= 1'b0;
		data_out  <= 16'd0;
	end else begin
		if(~old_valid & valid) tr <= 1'b1;
		if(sync) find_sync <= 1'b0;

		if(stb_rise & sel130 & bus_we) begin
			drive_sel <= |bus_din[3:0];
			drive     <= bus_din[0] ? 2'd0 : bus_din[1] ? 2'd1 : bus_din[2] ? 2'd2 : 2'd3;
			motor     <= bus_din[4];
			head      <= bus_din[5];
			dir       <= bus_din[6];
			step      <= bus_din[7];
			gdr       <= bus_din[8];
			// the hunt for a mark starts when bit 8 goes back down
			if(gdr & ~bus_din[8]) find_sync <= ~sync;
		end

		if(stb_rise & sel132 & bus_we) begin
			data_out  <= bus_din;
			write     <= 1'b1;
			tr        <= 1'b0;
			find_sync <= 1'b0;
		end

		if(stb_rise & sel132 & ~bus_we & ~rd_block) begin
			dout      <= data_in;
			tr        <= 1'b0;
			find_sync <= 1'b0;
		end
	end
end

endmodule
