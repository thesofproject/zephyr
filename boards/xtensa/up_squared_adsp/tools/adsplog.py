#!/usr/bin/python3
import sys
import fcntl
import struct
import mmap
import time

CMD_OPEN_DEVICE  = 0x47
CMD_OPEN_DEVICE_LEN = 40

MAP_SIZE = 8192
SLOT_SIZE = 64

# Location of the log output window within the mapping of the DSP
# HPSRAM.  These numbers cribbed from existing scripting, I don't know
# what they really mean.  The driver on the DSP just hard codes an
# address.
WIN_OFFSET = 0x80000
WIN_ID = 3
WIN_SIZE = 0x20000

LOG_OFFSET = WIN_OFFSET + WIN_ID * WIN_SIZE

fd = open("/dev/hda")

# The ioctl result is a diag_dev_handle_t:
#
# struct diag_hda_bar_t {
#         phys_addr_t base_physical;
#         void *base_virtual;
#         uint32_t size;
# } __packed;
# 
# struct diag_dev_handle_t {
#         struct diag_hda_bar_t hda_bar;
#         struct diag_hda_bar_t dsp_bar;
# } __packed;

buf = bytearray(CMD_OPEN_DEVICE_LEN)
fcntl.ioctl(fd, CMD_OPEN_DEVICE, buf)
dsp_addr = struct.unpack("=QQLQQL", buf)[3]

# Map the 8k of the DSP (the "physical" address) at 0xe0000.  That's
# defined in tooling as 0x80000+3*0x20000, where 0x80000 is "FW_SRAM",
# 3 is the "win_id", and 0x20000 is just a magic number.  The
# device-side driver doesn't seem to have docs on these either.
mem = mmap.mmap(fd.fileno(), MAP_SIZE, offset=dsp_addr + LOG_OFFSET,
                prot=mmap.PROT_READ)

# The mapping is an array of 64-byte "slots", each of which is
# prefixed by a magic number, which should be 0x55aa for log data,
# followed a 16 bit "ID" number, followed by a null-terminated string
# in the final 60 bytes.  The aDSP firmware will write sequential IDs
# into the buffer starting from an ID of zero in the first slot, and
# wrapping at the end.  So the algorithm here is to find the smallest
# valid slot, print its data, and then enter a polling loop waiting
# for the next slot to be valid and have the correct next ID before
# printing that too.

# NOTE: unfortunately there's no easy way to detect a warm reset of
# the device, it will just jump back to the beginning and start
# writing there, where we aren't looking.  Really that level of
# robustness needs to be handled in the kernel.

next_slot = 0
next_id = 0xffff

for slot in range(int(MAP_SIZE / SLOT_SIZE)):
    off = slot * SLOT_SIZE
    (magic, sid) = struct.unpack("HH", mem[off:off+4])
    if magic == 0x55aa:
        if sid < next_id:
            next_slot = slot
            next_id = sid

while True:
    off = next_slot * SLOT_SIZE
    (magic, sid) = struct.unpack("HH", mem[off:off+4])
    if magic == 0x55aa and sid == next_id:
        # This dance because indexing large variable-length slices of
        # the mmap() array seems to produce garbage....
        msgbytes = []
        for i in range(4, SLOT_SIZE):
            b = mem[off+i]
            if b == 0:
                break;
            msgbytes.append(b)
        msg = bytearray(len(msgbytes))
        for i in range(len(msgbytes)):
            msg[i] = msgbytes[i]

        sys.stdout.write(msg.decode(encoding="utf-8", errors="ignore"))
        next_slot = int((next_slot + 1) % (MAP_SIZE / SLOT_SIZE))
        next_id += 1
    else:
        sys.stdout.flush()
        time.sleep(0.25)
