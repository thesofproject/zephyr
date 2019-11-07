# SOF-Zephyr Tools

## SOF Firmware loader

This script loads the SOF firmware file to Intel HDA controller and reads the firmware status and IPC message.

In order to run the script, the SOF diagnostic driver must be installed on the system.

### Install SOF Diagnostic Driver
SOF Diagnostic Driver can be found from [sof-diagnostic-driver](https://github.com/thesofproject/sof-diagnostic-driver) page.

```
$ git clone https://github.com/thesofproject/sof-diagnostic-driver
$ cd sof-diagnostic-driver
$ make
$ sudo insmod diag_driver.ko
```

### Using script
***Note that the script must run in sudo***

```
usage: fw_loader.py [-h] -f FIRMWARE [-d]

ADSP firmware loader

optional arguments:
  -h, --help            show this help message and exit
  -f FIRMWARE, --firmware FIRMWARE
                        ADSP firmware file to load
  -d, --debug           Display debug information

```

### Example output
```
 $ sudo ./fw_loader.py -f /home/tester/Downloads/sof-zephyr/sof-apl.ri
Start firmware downloading...
Open HDA device: /dev/hda
Reset DSP...
Firmware Status Register (0xFFFFFFFF)
   Boot:   0xFFFFFF (UNKNOWN)
   Wait:   0x0F (UNKNOWN)
   Module: 0x07
   Error:  0x01
   IPC CMD  : 0xFFFFFFFF
   IPC LEN  : 0xFFFFFFFF
Booting up DSP...
Firmware Status Register (0x05000001)
   Boot:   0x000001 (INIT_DONE)
   Wait:   0x05 (DMA_BUFFER_FULL)
   Module: 0x00
   Error:  0x00
Downloading firmware...
Start firmware downloading...
Checking firmware status...
Firmware Status Register (0x00000005)
   Boot:   0x000005 (FW_ENTERED)
   Wait:   0x00 (UNKNOWN)
   Module: 0x00
   Error:  0x00
Firmware download completed successfully
Reading IPC FwReady Message...
IPC Firmware Ready Message: (0x70000000) (0x0000006C)
   DSP box offset:     0 (0x0000)
   Host box offset:    0 (0x0000)
   DSP box size:       0 (0x0000)
   Host box size:      0 (0x0000)

Firmware Version:
   Major version:      1 (0x0001)
   Minor version:      1 (0x0001)
   Micro number:       0 (0x0000)
   Build number:       8 (0x0008)
   Date:             Aug 26 2019
   Time:                09:58:08
   Tag:                    9e2aa
   Abi version: 50368512 (0x3009000)

Flags:   0x00000000

IPC Firmware Ready Extended Message: (0x70000000) (0x000000B8)
Message Type:          1 (IPC_EXT_WINDOW)

Number of Windows:     7

Window type:      IPC_REGION_REGS (5)
Window id:             0
Window flags:          0 (0x0000)
Window size:        4096 (0x1000)
Window offset:         0 (0x0000)

Window type:     IPC_REGION_UPBOX (1)
Window id:             0
Window flags:          0 (0x0000)
Window size:        4096 (0x1000)
Window offset:      4096 (0x1000)

Window type:   IPC_REGION_DOWNBOX (0)
Window id:             1
Window flags:          0 (0x0000)
Window size:        8192 (0x2000)
Window offset:         0 (0x0000)

Window type:     IPC_REGION_DEBUG (3)
Window id:             2
Window flags:          0 (0x0000)
Window size:        4096 (0x1000)
Window offset:         0 (0x0000)

Window type: IPC_REGION_EXCEPTION (6)
Window id:             2
Window flags:          0 (0x0000)
Window size:        2048 (0x0800)
Window offset:      2048 (0x0800)

Window type:    IPC_REGION_STREAM (4)
Window id:             2
Window flags:          0 (0x0000)
Window size:        4096 (0x1000)
Window offset:      4096 (0x1000)

Window type:     IPC_REGION_TRACE (2)
Window id:             3
Window flags:          0 (0x0000)
Window size:        8192 (0x2000)
Window offset:         0 (0x0000)
```
