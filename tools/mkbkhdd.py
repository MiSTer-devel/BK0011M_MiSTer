#!/usr/bin/env python3
"""mkbkhdd.py - assemble an AltPro (SMK-512) hard disk image for the BK out of
floppy images.

    mkbkhdd.py -o disk.img -m mbr.bin [--boot N] [--label] image1.dsk image2.dsk ...

Each floppy image (819200 or 409600 bytes, raw sectors) becomes one logical
disk (C:, D:, ...) of 1638 blocks, 26 heads of the 16 x 63 geometry, so every
partition starts on a head boundary as the SMK BIOS lays them out.  The
drive's data is stored complemented (the SMK inverts the bus), as a raw dump
of a real SMK disk is.  Layout (BIOS v2.05, see the SMK documentation):

    sectors 0-6   the MBR, loaded to 20000 and run if it starts with NOP
                  (taken from an existing image: -m mbr.bin, 7 sectors,
                  stored form)
    sector 7      the copy of 176000-176777: geometry at 176772/4/6, the
                  number of logical disks and the boot unit at 176770, the
                  partition table growing down from 176766 (start = track*16
                  + head, length in blocks), then the table's checksum
    sector 63+    the partitions

The boot unit (--boot, default 2 = C:) is what the BIOS boots without a key.
"""
import argparse, os, struct, sys

HEADS, SPT = 16, 63
PART_HEADS = 26                     # 1638 blocks: a floppy (1600) rounded to heads
PART_BLOCKS = PART_HEADS * SPT
FLOPPY = 819200
CHECK = 0o12701                     # the table checksum constant

def inv(b): return bytes(x ^ 0xFF for x in b)

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--out", required=True)
ap.add_argument("-m", "--mbr", required=True, help="7 sectors of MBR in stored (inverted) form")
ap.add_argument("--boot", type=int, default=2, help="boot unit, 2 = C:")
ap.add_argument("--hdi", action="store_true", help="prepend the 512-byte BKEmu .hdi header")
ap.add_argument("images", nargs="+")
a = ap.parse_args()

if len(a.images) > 124: sys.exit("at most 124 logical disks")
mbr = open(a.mbr, "rb").read()
if len(mbr) < 7 * 512: sys.exit("the MBR must be 7 sectors")
mbr = mbr[:7 * 512]

# geometry: the first track holds the MBR and the config sector
total_heads = 1 + PART_HEADS * len(a.images)
cyl = -(-total_heads // HEADS)
total_sectors = cyl * HEADS * SPT

# the config sector, software view
cfg = bytearray(512)
def putw(off, v): struct.pack_into("<H", cfg, off, v & 0xFFFF)
putw(0o776, cyl); putw(0o774, HEADS); putw(0o772, SPT)
putw(0o770, (a.boot << 8) | len(a.images))
for i in range(len(a.images)):
    head_index = 1 + PART_HEADS * i
    trk, hd = divmod(head_index, HEADS)
    putw(0o766 - 4 * i, (trk << 4) | hd)
    putw(0o764 - 4 * i, PART_BLOCKS)
cs_off = 0o770 - 4 * len(a.images) - 2
s = CHECK
for off in range(cs_off + 2, 0o1000, 2):
    s += struct.unpack_from("<H", cfg, off)[0]
putw(cs_off, s)

with open(a.out, "wb") as f:
    if a.hdi:
        h = bytearray(512)
        struct.pack_into("<HHHHHHH", h, 0, 0x045A, cyl, 0xC837, HEADS, 0x7E00, 512, SPT)
        h[510] = 0xA5; h[511] = (-sum(h[:511])) & 0xFF
        f.write(h)
    f.write(mbr)
    f.write(inv(cfg))
    f.write(b"\xFF" * ((SPT - 8) * 512))            # the rest of track 0, head 0: empty
    for i, path in enumerate(a.images):
        d = open(path, "rb").read()
        if len(d) not in (FLOPPY, FLOPPY // 2): print("warning: %s is %d bytes" % (path, len(d)), file=sys.stderr)
        d = d[:PART_BLOCKS * 512].ljust(PART_BLOCKS * 512, b"\0")
        f.write(inv(d))
        print("%c: %s" % (ord("C") + i, os.path.basename(path)))
    written = 7 * 512 + 512 + (SPT - 8) * 512 + PART_BLOCKS * 512 * len(a.images)
    f.write(b"\xFF" * (total_sectors * 512 - written))
print("%s: %d logical disks, %d cylinders x %d x %d, %d bytes%s" % (a.out, len(a.images), cyl, HEADS, SPT, total_sectors * 512 + (512 if a.hdi else 0), " (.hdi)" if a.hdi else ""))
