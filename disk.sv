
module disk
(
	input         clk_sys,
	input         ce_bus,

	input         reset,
	input         reset_full,
	input         disk_rom,
	input         bk0010,
	output [15:0] ext_mode,
	output reg    reset_req,
	output reg    bk0010_stub,

	input  [15:0] bus_din,
	input  [15:0] bus_addr,
	input         bus_sync,
	input         bus_we,
	input   [1:0] bus_wtbt,
	input         bus_stb,
	output        bus_ack,

	output        dsk_copy,
	output        dsk_copy_virt,
	output [24:0] dsk_copy_addr,
	input  [15:0] dsk_copy_din,
	output [15:0] dsk_copy_dout,
	output        dsk_copy_we,
	output        dsk_copy_rd,

	output [31:0] sd_lba,
	output reg [2:0] sd_rd,
	output reg [2:0] sd_wr,

	input         sd_ack,
	input         sd_ack_conf,
	input   [7:0] sd_buff_addr,
	input  [15:0] sd_buff_dout,
	output [15:0] sd_buff_din,
	input         sd_buff_wr,

	output        sd_conf,

	input   [2:0] img_mounted,
	input  [31:0] img_size,

	input         ioctl_download,
	input         ioctl_wr,
	input  [24:0] ioctl_addr,
	input  [15:0] ioctl_dout,
	input   [7:0] ioctl_index
);

assign sd_conf = 1'b0;
assign sd_lba  = conf ? 0 : lba;

reg [31:0] hdd_sig;
reg  [7:0] hdd_ver;

wire [31:0] hdr_out;
reg   [6:0] hdr_addr;
sector_w2d sector_hdr
(
	.clock(clk_sys),
	.data(sd_buff_dout),
	.wraddress(sd_buff_addr),
	.wren(conf & sd_ack & sd_buff_wr),
	.rdaddress(hdr_addr),
	.q(hdr_out)
);

wire [15:0] bk_ram_out;
sector_w2w sector_rd
(
	.clock(clk_sys),
	.data(sd_buff_dout),
	.wraddress(sd_buff_addr),
	.wren(!conf & sd_ack & sd_buff_wr),
	.rdaddress(bk_addr),
	.q(bk_ram_out)
);

reg  [15:0] bk_data_wr;
reg         bk_wr;
wire  [7:0] sd_ram_out;
sector_w2w sector_wr
(
	.clock(clk_sys),
	.data(bk_data_wr),
	.wraddress(bk_addr),
	.wren(bk_wr),
	.rdaddress(sd_buff_addr),
	.q(sd_buff_din)
);

always @(posedge clk_sys) begin
	reg old_wr;
	old_wr <= sd_buff_wr;
	if(sd_ack & ~old_wr & sd_buff_wr & conf) begin
		case(sd_buff_addr)
			0: hdd_sig[31:16] <= {sd_buff_dout[7:0],sd_buff_dout[15:8]};
			1: hdd_sig[15:0]  <= {sd_buff_dout[7:0],sd_buff_dout[15:8]};
			2: hdd_ver        <=  sd_buff_dout[7:0];
		endcase
	end
end

assign dsk_copy        = ioctl_download | processing;
assign dsk_copy_we     = ioctl_download ? ioctl_wr   : copy_we;
assign dsk_copy_rd     = ioctl_download ? 1'b0       : copy_rd;
assign dsk_copy_addr   = ioctl_download ? ioctl_addr : copy_addr;
assign dsk_copy_dout   = ioctl_download ? ioctl_dout : copy_dout;
assign dsk_copy_virt   = ioctl_download ? 1'b0       : copy_virt;

reg         fdd_ready = 0;
reg  [24:0] fdd_size = 0;

reg  [15:0] tape_addr;
reg  [15:0] tape_len;

always @(posedge clk_sys) begin
	reg old_we;
	old_we <= ioctl_wr;
	if(~old_we & ioctl_wr) begin
		if(ioctl_addr == 25'h100000) tape_addr <= {ioctl_dout[15:1], 1'b0};
		if(ioctl_addr == 25'h100002) tape_len  <= (ioctl_dout+1'd1) & ~16'd1;
	end
end

always @(posedge clk_sys) begin
	reg old_download;
	reg in_range;
	
	old_download <= ioctl_download;
	if(!old_download & ioctl_download & bk0010 & (ioctl_index == 1)) reset_req <=1;

	if(old_download & !ioctl_download) begin

		reset_req <= 0;

		case(ioctl_index)
			1: begin 
					in_range  <= 0;
					bk0010_stub <= bk0010;
				end

			'h41: begin 
					fdd_ready <= 1;
					fdd_size  <= ioctl_addr - 25'h120000;
				end
		endcase
	end

	if(reset_full) fdd_ready <= 0;
	if(bus_addr[15:13] == 3'b101) in_range <=1;
	if(in_range & (bus_addr[15:13] < 3'b101) & bk0010_stub) bk0010_stub <=0; 
end

//Allow write for stop/start disk motor and extended memory mode.
wire       sel130  = bus_sync && (bus_addr[15:1] == (16'o177130 >> 1)) && bus_wtbt[0];
wire       sel130w = sel130 && bus_we;
wire       sel130r = sel130 && !bus_we && !(bk0010 && mode130[2]);
assign     ext_mode = mode130;
reg [15:0] mode130;
reg        mode130_strobe = 1'b0;

always @(posedge clk_sys) begin
	reg old_stb;
	old_stb <= bus_stb;

	if(reset) begin
		mode130_strobe <= 1'b0;
		mode130 <= bk0010 ? 16'o160 : 16'o140;

	end else if(!old_stb & bus_stb & sel130w) begin
		mode130[3:2] <= bus_din[3:2];
		if(mode130_strobe) begin 
			mode130[6:4]   <= bus_din[6:4];
			mode130[11:8]  <= {bus_din[0], bus_din[3], bus_din[2], bus_din[10]};
			mode130_strobe <= 1'b0;
		end else begin 
			mode130_strobe <= (bus_din == 16'o6);
		end
	end;
end

//LBA access. Main access for Floppy and HDD read/write.
wire sel132 = bus_we && bus_sync && (bus_addr[15:1] == (16'o177132 >> 1));

//CHS access. Currently not supported and always returns error.
//Paramaters can be recalculated for LBA call, but none of apps 
//used CHS access with exception of specific floppy utilities.
wire sel134 = bus_we && bus_sync && (bus_addr[15:1] == (16'o177134 >> 1));

//BIN loader
wire sel670 = bus_we && bus_sync && bk0010_stub && (bus_addr[15:1] == (16'o177670 >> 1));

wire stb132 = bus_stb && sel132;
wire stb134 = bus_stb && sel134;
wire stb670 = bus_stb && sel670;
wire valid  = (disk_rom & (sel130w | sel130r | sel132 | sel134)) | sel670;

assign bus_ack = bus_sync & bus_stb & valid;

wire        reg_access = (disk_rom & (stb132 | stb134)) | stb670;

reg  [24:0] copy_addr;
reg  [15:0] copy_dout;
wire [15:0] copy_din = dsk_copy_din;
reg         copy_virt;
reg         copy_we;
reg         copy_rd;
reg   [7:0] bk_addr;
reg  [31:0] lba;
reg         conf       = 0;
reg         processing = 0;

typedef enum 
{
	ST_R, ST_R2, ST_R3, ST_R4,
	ST_W, ST_W2, ST_W3, ST_W4,
	
	ST_CP_R2V, ST_CP_V2R,
	
	ST_CP, ST_CP2,
	
	ST_PAR, ST_PAR2, ST_PAR3, ST_PAR4, ST_PAR5, ST_PAR6, ST_PAR7, ST_PAR8,

	ST_HR, ST_HR2, ST_HR3, ST_HR4, ST_HR5,
	ST_HW, ST_HW2, ST_HW3, ST_HW4, ST_HW5, ST_HW6, ST_HW7, ST_HW8, ST_HW9,
	
	ST_FR,
	
	ST_BIN, ST_BIN2, ST_BIN3, ST_BIN4, ST_BIN5,

	ST_RES, ST_RES2, ST_RES3,
	ST_RES_OK

} io_state_t;

always @(posedge clk_sys) begin
	reg  old_access, old_reset;
	reg [2:0] old_mounted;

	io_state_t io_state, io_cp_ret, io_rw_ret;

	reg  [5:0] ack;
	reg        io_busy = 0;
	reg        mounted = 0;
	reg  [1:0] cp_virt;
	reg [24:0] addr_r, addr_w, cp_len;
	reg [15:0] SP, PSW, vaddr, error, total_size, part_size;
	reg [31:0] hdd_start, hdd_end;
	reg  [7:0] disk;
	reg        write;

	if(ce_bus) begin
		old_access <= reg_access;
		if(!old_access && reg_access) begin 
			processing <= 1;
			io_state   <= stb670 ? ST_BIN : ST_PAR;
			SP         <= bus_din;
			copy_rd    <= 0;
			copy_we    <= 0;
			bk_wr      <= 0;
			sd_wr      <= 0;
			sd_rd      <= 0;
		end

		old_reset <= reset;
		if(!old_reset && reset) begin
			processing <= 0;
			io_busy    <= 0;
			sd_wr      <= 0;
			sd_rd      <= 0;
			copy_rd    <= 0;
			copy_we    <= 0;
		end

		ack <= {sd_ack, ack[5:1]};
		if(ack[0] && !ack[1]) begin
			if(conf) begin
				mounted <= ((hdd_sig == "BKHD") && (hdd_ver == 1));
				conf    <= 1'b0;
			end
			io_busy <= 0;
		end

		if(!ack[0] && ack[1]) begin
			sd_wr <= 0;
			sd_rd <= 0;
		end

		if(processing) begin
			case(io_state)
				ST_R:
					begin
						copy_addr <= addr_r;
						copy_rd   <= 0;
						io_state  <= io_state.next();
					end
				ST_R2,
				ST_R3:
					begin
						copy_rd   <= 1;
						io_state  <= io_state.next();
					end
				ST_R4:
					begin
						copy_rd   <= 0;
						addr_r    <= addr_r + 2'd2;
						io_state  <= io_rw_ret;
						io_rw_ret <= io_rw_ret.next();
					end

				ST_W:
					begin
						copy_addr <= addr_w;
						copy_we   <= 0;
						io_state  <= io_state.next();
					end
				ST_W2,
				ST_W3:
					begin
						copy_we   <= 1;
						io_state  <= io_state.next();
					end
				ST_W4:
					begin
						copy_we   <= 0;
						addr_w    <= addr_w + 2'd2;
						io_state  <= io_rw_ret;
						io_rw_ret <= io_rw_ret.next();
					end

				ST_CP_R2V:
					begin
						cp_virt   <= 2'b01;
						io_state  <= ST_CP;
					end
				ST_CP_V2R:
					begin
						cp_virt   <= 2'b10;
						io_state  <= ST_CP;
					end

				ST_CP:
					begin
						if(!cp_len) io_state <= io_cp_ret;
						else begin
							io_state <= ST_R;
							io_rw_ret<= io_state.next();
						end
						cp_len    <= cp_len - 1'd1;
						copy_virt <= cp_virt[1];
					end
				ST_CP2:
					begin
						copy_virt <= cp_virt[0];
						copy_dout <= copy_din;
						io_state  <= ST_W;
						io_rw_ret <= ST_CP;
					end

				ST_PAR:
					begin
						copy_virt <= 1;
						addr_r    <= SP+16'd6;
						io_state  <= ST_R;
						io_rw_ret <= io_state.next();
					end
				ST_PAR2:
					begin
						// R3 - address of paramters
						addr_r    <= copy_din + 16'o34;
						io_state  <= ST_R;
					end
				ST_PAR3:
					begin
						// 34(R3) - Disk number
						disk      <= copy_din[7:0];
						hdr_addr  <= 7'd2 + copy_din[6:0];
						addr_r    <= SP;
						io_state  <= ST_R;
					end
				ST_PAR4:
					begin
						// R0 - start block
						lba       <= (disk>=2) ? hdr_out + copy_din : copy_din;
						hdd_start <= hdr_out;
						io_state  <= ST_R;
					end
				ST_PAR5:
					begin
						// R1 - length
						write     <= copy_din[15];
						total_size<= copy_din[15] ? (~copy_din[15:0])+1'd1 : copy_din[15:0];
						hdr_addr  <= hdr_addr + 1'd1;
						io_state  <= ST_R;
					end
				ST_PAR6:
					begin
						// R2 - address of buffer
						vaddr     <= copy_din;
						hdd_end   <= hdr_out;
						addr_r    <= SP+16'd8;
						io_state  <= ST_R;
					end
				ST_PAR7:
					begin
						// PSW to return the status
						PSW       <= copy_din;
						addr_r    <= 16'o52;
						io_state  <= ST_R;
					end
				ST_PAR8:
					begin
						error     <= copy_din;
						addr_w    <= vaddr;
						addr_r    <= vaddr;

						if(disk>=2) begin
							//VHD access
							if((bus_addr != 16'o177132) || !mounted || !hdd_end || !hdd_start || (disk >= 125)) begin
								error[7:0] <= 6;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else if(!total_size || vaddr[0]) begin
								error[7:0] <= 10;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else if((lba+((total_size+255) >> 8)) > hdd_end) begin
								error[7:0] <= 5;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else 
								if(write) io_state <= ST_HW; // write
									else   io_state <= ST_HR; // read

						end else begin
							//DSK access
							if((bus_addr != 16'o177132) || write || !fdd_ready || !fdd_size) begin
								error[7:0] <= 6;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else if(!total_size || vaddr[0]) begin
								error[7:0] <= 10;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else if((lba+((total_size+255) >> 8)) > (fdd_size >> 9)) begin
								error[7:0] <= 5;
								PSW[0]     <= 1;
								io_state   <= ST_RES;
							end else begin 
								lba        <= (lba << 9) + 32'h120000;
								io_state   <= ST_FR; // read
							end
						end
					end

				// Floppy read
				ST_FR:
					begin
						addr_r       <= lba[24:0];
						cp_len       <= total_size;
						io_state     <= ST_CP_R2V;
						io_cp_ret    <= ST_RES_OK;
					end

				// HDD read
				ST_HR:
					begin
						if(!io_busy) begin
							bk_wr     <= 0;
							sd_rd     <= 1;
							io_busy   <= 1;
							part_size <= (total_size < 16'd256) ? total_size : 16'd256;
							io_state  <= io_state.next();
						end
					end
				ST_HR2:
					begin
						if(!io_busy) begin
							bk_addr   <= 0;
							total_size<= total_size - part_size;
							io_state  <= io_state.next();
						end
					end
				ST_HR3:
					begin
						copy_virt    <= 1;
						copy_dout    <= bk_ram_out;
						io_state     <= ST_W;
						io_rw_ret    <= io_state.next();
					end
				ST_HR4:
					begin
						bk_addr      <= bk_addr + 2'd1;
						part_size    <= part_size - 2'd1;
						io_state     <= io_state.next();
					end
				ST_HR5:
					begin
						if(part_size != 0) io_state <= ST_HR3;
						else if(total_size != 0) begin 
							lba       <= lba + 1;
							io_state  <= ST_HR;
						end else begin
							io_state  <= ST_RES_OK;
						end
					end

				// HDD write
				ST_HW:
					begin
						if(!io_busy) begin
							bk_wr     <= 0;
							bk_addr   <= 0;
							part_size <= (total_size < 16'd256) ? total_size : 16'd256;
							io_state  <= io_state.next();
						end
					end
				ST_HW2:
					begin
						total_size   <= total_size - part_size;
						io_state     <= io_state.next();
					end
				ST_HW3:
					begin
						copy_virt    <= 1;
						bk_wr        <= 0;
						io_state     <= ST_R;
						io_rw_ret    <= io_state.next();
					end
				ST_HW4:
					begin
						bk_data_wr   <= copy_din;
						part_size    <= part_size - 2'd1;
						io_state     <= io_state.next();
					end
				ST_HW5: begin
						bk_wr        <= 1;
						io_state     <= io_state.next();
					end
				ST_HW6: begin
						bk_wr        <= 0;
						io_state     <= io_state.next();
					end
				ST_HW7: begin
						bk_addr      <= bk_addr + 2'd1;
						io_state     <= io_state.next();
						if(part_size != 0) io_state <= ST_HW3;
					end
				ST_HW8:
					begin
						if(!io_busy) begin
							sd_wr     <= 1;
							io_busy   <= 1;
							io_state  <= io_state.next();
						end
					end
				ST_HW9:
					begin
						if(!io_busy) begin
							if(total_size != 0) begin 
								lba    <= lba + 1;
								io_state <= ST_HW;
							end else begin
								io_state <= ST_RES_OK;
							end
						end
					end

				// Successful result.
				ST_RES_OK:
					begin
						error[7:0]   <= 0;
						PSW[0]       <= 0;
						io_state     <= ST_RES;
					end

				// Finish. Post the exit code.
				ST_RES:
					begin
						copy_virt    <= 1;
						copy_dout    <= error;
						addr_w       <= 16'o52;
						io_state     <= ST_W;
						io_rw_ret    <= io_state.next();
					end
				ST_RES2:
					begin
						copy_dout    <= PSW;
						addr_w       <= SP+16'd8;
						io_state     <= ST_W;
					end
				ST_RES3:
					begin
						processing   <= 0;
					end

				// BIN copy
				ST_BIN:
					begin
						// replace return address in stack
						copy_virt    <= 1;
						copy_dout    <= tape_addr;
						addr_w       <= SP;
						io_state     <= ST_W;
						io_rw_ret    <= io_state.next();
					end
				ST_BIN2:
					begin
						// start address after EMT 36
						copy_dout    <= tape_addr;
						addr_w       <= 16'o264;
						io_state     <= ST_W;
					end
				ST_BIN3:
					begin
						// length after EMT 36
						copy_dout    <= tape_len;
						io_state     <= ST_W;
					end
				ST_BIN4:
					begin
						addr_r       <= 25'h100004;
						addr_w       <= tape_addr;
						cp_len       <= tape_len[15:1];
						io_state     <= ST_CP_R2V;
						io_cp_ret    <= io_state.next();
					end
				ST_BIN5:
					begin
						processing   <= 0;
					end
			endcase
		end
	end

	if(!io_busy) begin
		if(!old_mounted[0] && img_mounted[0]) begin
			mounted    <= 0;
			conf       <= 1;
			sd_rd      <= 1;
			io_busy    <= 1;
			
			processing <= 0; // can brake on-going IO, but nothing can be done.
			sd_wr      <= 0;
			copy_rd    <= 0;
			copy_we    <= 0;
		end
		old_mounted <= img_mounted;
	end
end

endmodule

module sector_w2d
(
	input	        clock,

	input	  [7:0] wraddress,
	input	 [15:0] data,
	input	        wren,

	input	  [6:0] rdaddress,
	output [31:0] q
);

mixed_width_dpram #(.WORDS(128), .WW(16), .RW(32)) ram(.*);

endmodule


module sector_w2w
(
	input	        clock,

	input	  [7:0] wraddress,
	input	 [15:0] data,
	input	        wren,

	input	  [7:0] rdaddress,
	output [15:0] q
);

mixed_width_dpram #(.WORDS(256), .WW(16), .RW(16)) ram(.*);

endmodule


module mixed_width_dpram
#(
	parameter WORDS = 256,
	parameter RW    = 8,
	parameter WW    = 32
)
(
	input clock,
	input wren, 

	input [$clog2((RW < WW) ? WORDS : (WORDS * RW)/WW) - 1 : 0] wraddress, 
	input [$clog2((RW < WW) ? (WORDS * WW)/RW : WORDS) - 1 : 0] rdaddress, 

	input        [WW-1:0] data, 
	output logic [RW-1:0] q
);
   
localparam int R = (RW < WW) ? WW/RW : RW/WW;
localparam int B = (RW < WW) ? RW: WW;

logic [R-1:0][B-1:0] ram[0:WORDS-1];

generate if(RW < WW) begin
	// Smaller read?
	always_ff@(posedge clock)
	begin
		if(wren) ram[wraddress] <= data;
		q <= ram[rdaddress / R][rdaddress % R];
	end
end
else begin 
	// Smaller write?
	always_ff@(posedge clock)
	begin
		if(wren) ram[wraddress / R][wraddress % R] <= data;
		q <= ram[rdaddress];
	end
end 
endgenerate
   
endmodule
