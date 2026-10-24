//
// disk.sv
//
// The SMK512 / A16M mode register behind writes to 177130 (ext_mode, for
// memory.sv) and the BIN loader of BK0010 mode, which feeds a downloaded
// .BIN through the tape-read stub. The floppies are in fdd/.
//
module disk
(
	input         clk_sys,
	input         ce_bus,

	input         reset,
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

	input         ioctl_download,
	input         ioctl_wr,
	input  [24:0] ioctl_addr,
	input  [15:0] ioctl_dout,
	input   [7:0] ioctl_index
);

wire [24:0] down_addr = ioctl_addr + ((ioctl_index) ? 25'h100000 : 25'h0E0000);

assign dsk_copy        = ioctl_download | processing;
assign dsk_copy_we     = ioctl_download ? ioctl_wr   : copy_we;
assign dsk_copy_rd     = ioctl_download ? 1'b0       : copy_rd;
assign dsk_copy_addr   = ioctl_download ? down_addr  : copy_addr;
assign dsk_copy_dout   = ioctl_download ? ioctl_dout : copy_dout;
assign dsk_copy_virt   = ioctl_download ? 1'b0       : copy_virt;

reg  [15:0] tape_addr;
reg  [15:0] tape_len;

always @(posedge clk_sys) begin
	reg old_we;
	old_we <= ioctl_wr;
	if(~old_we & ioctl_wr) begin
		if(down_addr == 25'h100000) tape_addr <= {ioctl_dout[15:1], 1'b0};
		if(down_addr == 25'h100002) tape_len  <= (ioctl_dout+1'd1) & ~16'd1;
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
		endcase
	end

	if(bus_addr[15:13] == 3'b101) in_range <=1;
	if(in_range & (bus_addr[15:13] < 3'b101) & bk0010_stub) bk0010_stub <=0; 
end

// The SMK512 / A16M mode register: a write of 6 to 177130 opens it, the
// next write sets the mode and page.  The floppy controller answers the
// bus; this only watches.
wire       sel130w = bus_sync && (bus_addr[15:1] == (16'o177130 >> 1)) && bus_wtbt[0] && bus_we;
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

//BIN loader: the tape-read stub of BK0010 mode traps here
wire sel670 = bus_we && bus_sync && bk0010_stub && (bus_addr[15:1] == (16'o177670 >> 1));
wire stb670 = bus_stb && sel670;

assign bus_ack = bus_sync & bus_stb & sel670;

reg  [24:0] copy_addr;
reg  [15:0] copy_dout;
wire [15:0] copy_din = dsk_copy_din;
reg         copy_virt;
reg         copy_we;
reg         copy_rd;
reg         processing = 0;

typedef enum 
{
	ST_R, ST_R2, ST_R3, ST_R4,
	ST_W, ST_W2, ST_W3, ST_W4,
	
	ST_CP_R2V, ST_CP_V2R,
	
	ST_CP, ST_CP2,
	
	ST_BIN, ST_BIN2, ST_BIN3, ST_BIN4, ST_BIN5

} io_state_t;

io_state_t io_state, io_cp_ret, io_rw_ret, io_state_n, io_cp_ret_n, io_rw_ret_n;

always @(posedge clk_sys) begin
	io_state  <= io_state_n;
	io_cp_ret <= io_cp_ret_n;
	io_rw_ret <= io_rw_ret_n;
end

always @(posedge clk_sys) begin
	reg  old_access, old_reset;
	reg  [1:0] cp_virt;
	reg [24:0] addr_r, addr_w, cp_len;
	reg [15:0] SP;

	if(ce_bus) begin
		old_access <= stb670;
		if(!old_access && stb670) begin 
			processing <= 1;
			io_state_n <= ST_BIN;
			SP         <= bus_din;
			copy_rd    <= 0;
			copy_we    <= 0;
		end

		old_reset <= reset;
		if(!old_reset && reset) begin
			processing <= 0;
			copy_rd    <= 0;
			copy_we    <= 0;
		end

		if(processing) begin
			case(io_state)
				ST_R:
					begin
						copy_addr <= addr_r;
						copy_rd   <= 0;
						io_state_n<= io_state.next();
					end
				ST_R2,
				ST_R3:
					begin
						copy_rd   <= 1;
						io_state_n<= io_state.next();
					end
				ST_R4:
					begin
						copy_rd   <= 0;
						addr_r    <= addr_r + 2'd2;
						io_state_n<= io_rw_ret;
						io_rw_ret_n<= io_rw_ret.next();
					end

				ST_W:
					begin
						copy_addr <= addr_w;
						copy_we   <= 0;
						io_state_n<= io_state.next();
					end
				ST_W2,
				ST_W3:
					begin
						copy_we   <= 1;
						io_state_n<= io_state.next();
					end
				ST_W4:
					begin
						copy_we   <= 0;
						addr_w    <= addr_w + 2'd2;
						io_state_n<= io_rw_ret;
						io_rw_ret_n<= io_rw_ret.next();
					end

				ST_CP_R2V:
					begin
						cp_virt   <= 2'b01;
						io_state_n<= ST_CP;
					end
				ST_CP_V2R:
					begin
						cp_virt   <= 2'b10;
						io_state_n<= ST_CP;
					end

				ST_CP:
					begin
						if(!cp_len) io_state_n <= io_cp_ret;
						else begin
							io_state_n <= ST_R;
							io_rw_ret_n<= io_state.next();
						end
						cp_len    <= cp_len - 1'd1;
						copy_virt <= cp_virt[1];
					end
				ST_CP2:
					begin
						copy_virt <= cp_virt[0];
						copy_dout <= copy_din;
						io_state_n<= ST_W;
						io_rw_ret_n<= ST_CP;
					end

				// BIN copy
				ST_BIN:
					begin
						// replace return address in stack
						copy_virt    <= 1;
						copy_dout    <= tape_addr;
						addr_w       <= SP;
						io_state_n   <= ST_W;
						io_rw_ret_n  <= io_state.next();
					end
				ST_BIN2:
					begin
						// start address after EMT 36
						copy_dout    <= tape_addr;
						addr_w       <= 16'o264;
						io_state_n   <= ST_W;
					end
				ST_BIN3:
					begin
						// length after EMT 36
						copy_dout    <= tape_len;
						io_state_n   <= ST_W;
					end
				ST_BIN4:
					begin
						addr_r       <= 25'h100004;
						addr_w       <= tape_addr;
						cp_len       <= tape_len[15:1];
						io_state_n   <= ST_CP_R2V;
						io_cp_ret_n  <= io_state.next();
					end
				ST_BIN5:
					begin
						processing   <= 0;
					end
			endcase
		end
	end
end

endmodule
