#!/usr/bin/env python3
#
# Copyright (c) 2018 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

import os
import time
import argparse
import logging
import array

from colorama import Fore, Style
from utilities.device import Device
from utilities.etrace import Etrace
import utilities.registers as regs_def
import utilities.platforms as plat_def

def parse_args():

    arg_parser = argparse.ArgumentParser(description="Dump trace message")

    arg_parser.add_argument("-o", "--output-file", default=None,
                            help="Save to output file")
    arg_parser.add_argument("-d", "--debug", default=False, action='store_true',
                            help="Display debug information")

    args = arg_parser.parse_args()

    return args

def main():
    """ Main Entry Point """

    args = parse_args()

    log_level = logging.INFO
    if args.debug:
        log_level = logging.DEBUG

    logging.basicConfig(level=log_level, format="%(message)s")

    dev = Device()
    dev.open_device()

    etrace = Etrace(dev)
    etrace.print()

    if args.output_file:
        etrace.save(args.output_file)


if __name__ == "__main__":
    try:
        main()
        os._exit(0)
    except KeyboardInterrupt:  # Ctrl-C
        os._exit(14)
    except SystemExit:
        raise
    except BaseException:
        import traceback

        traceback.print_exc()
        os._exit(16)
