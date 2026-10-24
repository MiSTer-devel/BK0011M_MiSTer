# BK0011M for [MiSTer Board](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

This project of [popular USSR home computer](https://en.wikipedia.org/wiki/Electronika_BK) is based on precise [KR1801VM1](http://zx-pk.ru/showthread.php?t=23978) Verilog model by Vslav

### Features:
- Fully functional BK0011M and BK0010 with close to real CPU and Video timings (need furter corrections)
- Turbo mode (6MHz for BK0010, 8MHz for BK0011M)
- Real floppy controller: the KNGMD's KR1801VP1-128 at 177130/177132 with four drives (A-D), read, write and format, DSK/IMG/BKD images (80x2x10x512 = 819200 bytes), driven by the stock disk ROMs 326 (BK0011M) and 253 (BK0010)
- BIN file support for BK0010 mode (only single file apps are supported)
- A16M extension for BK0010 (support for short and long resets)
- SMK512 extension for BK0011M
- YM2149 with 1.71MHz clock
- Joystick
- Mouse

No contention model is used in turbo mode, thus speed is higher more than twice.

### Disks

The floppy disks are a real controller now instead of the block-device
emulation behind patched disk ROMs: the unmodified ROMs 326/253 (and the
A16M / SMK-512 ones), operating systems and copy/format utilities drive it
as on the hardware.

**Floppies.** Four drives behind a KR1801VP1-128 (`rtl/fdd/vp1_128.sv`),
each a spinning MFM track emulation fed from the image sector by sector
(`rtl/fdd/fdd4.v`, from UKNC_MiSTer / tang-uknc). Images are raw sector
dumps, 819200 bytes for 80 tracks x 2 sides x 10 sectors x 512 bytes
(`.dsk`, `.img`, `.bkd`); they are written through, so take copies. An
image mounted read-only, or all of them with the "FDD write protect"
option, reports write protection to the ROM. An empty drive behaves as a
drive without a disk (error 6).

The SMK-512 hard disk controller is not emulated yet; the old `bkhdutil`
VHD format is not supported any more.

### Installation:
Copy the *.rbf file at the root of the SD card.

There are couple disk images in [extra](https://github.com/MiSTer-devel/BK0011M_MiSTer/tree/master/extra) folder with CSIDOS OS with complete ecosystem (viewer,text editor, music editor, debugger, etc.) written by me more than 20 years ago. Image also incudes some music and utils written by different people.

*Most documents are in Russian languange.*
