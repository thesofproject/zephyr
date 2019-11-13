#!/usr/bin/python3

"""Logging reader tool"""

import argparse
import os
import sys
from ctypes import string_at

QEMU_ETRACE = "/dev/shm/qemu-bridge-hp-sram-mem"
SOF_ETRACE = "/sys/kernel/debug/sof/etrace"

QEMU_OFFSET = 0x8000
SOF_OFFSET = 0x0

MAGIC = 0x55aa
SLOT_LEN = 64
SLOT_NUM = int(8192 / SLOT_LEN)


def read_bytes(buffer):
    return int.from_bytes(buffer, byteorder='little')


class Loglist:
    """Loglist class"""

    def __init__(self, argument):
        """Constructor for the loglist takes argument filename or buffer"""

        if isinstance(argument, str):
            f = open(argument, "rb")
            self.buffer = f.read(SLOT_NUM * SLOT_LEN)
        elif isinstance(argument, int):
            self.buffer = string_at(argument, SLOT_NUM * SLOT_LEN)
        else:
            return

        self.loglist = []
        self.parse()

    def parse_slot(self, slot):
        magic = read_bytes(slot[0:2])

        if magic == MAGIC:
            id_num = read_bytes(slot[2:4])
            logstr = slot[4:].decode(errors='replace').split('\r', 1)[0]
            self.loglist.append((id_num, logstr))

    def parse(self):
        for x in range(0, SLOT_NUM):
            slot = self.buffer[x * SLOT_LEN : (x + 1) * SLOT_LEN]
            self.parse_slot(slot)

    def print(self):
        for pair in sorted(self.loglist):
            print('{} : {}'.format(*pair))


def parse_args():
    """Parsing command line arguments"""

    parser = argparse.ArgumentParser(description='Logging tool')

    parser.add_argument('-e', '--etrace', choices=['sof', 'qemu'], default="sof")

    parser.add_argument('-f', '--file')

    args = parser.parse_args()

    return args

def main():
    """Main"""

    args = parse_args()

    if os.geteuid() != 0:
        sys.exit("Please run this program as root / sudo")

    if args.file != None:
        etrace = args.file
        offset = 0
    else:
        if args.etrace == 'sof':
            etrace = SOF_ETRACE
            offset = SOF_OFFSET
        else:
            etrace = QEMU_ETRACE
            offset = QEMU_OFFSET

    l = Loglist(etrace)
    l.print()

if __name__ == "__main__":

    main()
