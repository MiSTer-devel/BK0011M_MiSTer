# BK0011M for [MiSTer Board](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

This project of [popular USSR home computer](https://en.wikipedia.org/wiki/Electronika_BK) is based on precise [KR1801VM1](http://zx-pk.ru/showthread.php?t=23978) Verilog model by Vslav

### Features:
- Fully functional BK0011M and BK0010 with close to real CPU and Video timings (need furter corrections)
- Turbo mode (6MHz for BK0010, 8MHz for BK0011M)
- Real floppy controller: the KNGMD's KR1801VP1-128 at 177130/177132 with four drives (A-D), read, write and format, DSK/IMG/BKD images (80x2x10x512 = 819200 bytes), driven by the stock disk ROMs 326 (BK0011M) and 253 (BK0010)
- Real hard disk: the SMK-512's IDE controller at 177740-177757 with the stock SMK-512 v2.05 BIOS (BK0011M + SMK512 option), HDI/HDS/IMG images
- BIN file support for BK0010 mode (only single file apps are supported)
- A16M extension for BK0010 (support for short and long resets)
- SMK512 extension for BK0011M
- YM2149 with 1.71MHz clock
- Joystick
- Mouse

No contention model is used in turbo mode, thus speed is higher more than twice.

### Disks

The floppies and the hard disk are real controllers instead of the
block-device emulation behind patched disk ROMs: the unmodified ROMs
326/253/A16M/SMK-512, operating systems and copy/format utilities drive
them as on the hardware.

**Floppies.** Four drives behind a KR1801VP1-128 (`rtl/fdd/vp1_128.sv`),
each a spinning MFM track emulation fed from the image sector by sector
(`rtl/fdd/fdd4.v`, from UKNC_MiSTer / tang-uknc). Images are raw sector
dumps, 819200 bytes for 80 tracks x 2 sides x 10 sectors x 512 bytes
(`.dsk`, `.img`, `.bkd`); they are written through, so take copies. An
image mounted read-only, or all of them with the "FDD write protect"
option, reports write protection to the ROM. An empty drive behaves as a
drive without a disk (error 6).

**Hard disk.** The SMK-512 IDE controller (`rtl/ide/smk_ide.sv`): ATA
registers at 177740 (status/command) ... 177756 (data), the control block
at 177741/177743, every bit on the bus inverted as on the real board. The
image holds the drive's data as the SMK stores it, i.e. inverted against
the BK's memory - this is the format of a raw dump of a real SMK disk or
CF card, of BKEmu's HDD Image Maker and of the images that circulate for
the SMK. `.hdi` files (512-byte BKEmu header with the geometry) and raw
`.hds`/`.img` files (sector 0 = LBA 0, geometry from the BIOS's partition
table via INITIALIZE DRIVE PARAMETERS) are both accepted. Partitioning and
the MBR are the SMK BIOS's business (SETMBR, the AltPro service disks);
the old `bkhdutil` VHD format is not supported any more.

The BIOS boots from the hard disk when one is mounted and the SMK512
option is on (at the AltPro prompt, `T1` boots C:). Images named
`boot0.vhd` (the hard disk) and `boot1.vhd` .. `boot3.vhd` (drives A-C)
in `/media/fat/games/BK0011M/` are mounted automatically at core start.

### Installation:
Copy the *.rbf file at the root of the SD card.

There are couple disk images in [extra](https://github.com/MiSTer-devel/BK0011M_MiSTer/tree/master/extra) folder with CSIDOS OS with complete ecosystem (viewer,text editor, music editor, debugger, etc.) written by me more than 20 years ago. Image also incudes some music and utils written by different people.

*Most documents are in Russian languange.*
