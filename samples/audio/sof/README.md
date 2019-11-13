# Zephyr on APL ADSP

# Build Zephyr image for xtensa DSP

## Build SDK for the board

To build the board we need to use yet another toolchain.  Get source here:
https://github.com/dcpleung/sdk-ng.git and check out branch `intel_apl_adsp`.

```bash
$ git clone https://github.com/dcpleung/sdk-ng.git
$ cd sdk-ng
$ git checkout intel_apl_adsp
```

Build crosstool-ng with command:

```bash
$ ./go.sh xtensa_intel_apl_adsp
```

After build we need to set Zephyr environment variable with:

```bash
export ZEPHYR_TOOLCHAIN_VARIANT=xtools
export XTOOLS_TOOLCHAIN_PATH=<...>/sdk-ng/build/output/
```

## Build Zephyr kernel

Get source from repository https://github.com/thesofproject/zephyr.git
and make checkout master branch.

Please refer to [Clone the Zephyr Repositories](https://docs.zephyrproject.org/latest/getting_started/index.html#clone-the-zephyr-repositories)
for more information.

```bash
$ west init zephyrproject -m https://github.com/thesofproject/zephyr.git
$ cd zephyrproject
$ west update
```

Although the BSP is based on `intel_s1000_crb` compiler and tools described for
the board does not produce working code. Instead use xtools described in the
chapter [Build SDK for the board](#build-sdk-for-the-board).

Build `samples/audio/sof` sample and we can use built zephyr.elf later.

Build for qemu with:
```bash
$ west build -p -b up_squared_adsp zephyr/samples/audio/sof -DOVERLAY_CONFIG=overlay-qemu.conf
```

Build for `up_squared` ADSP with:
```bash
$ west build -p -b up_squared_adsp zephyr/samples/audio/sof -DOVERLAY_CONFIG=overlay-up_squared.conf
```

We would be using zephyr/zephyr.elf later on.

## Build SOF image

From the SOF image we need ROM for booting with SOF Qemu and bootloader (startup code).

For building SOF we need to build special toolchain.

Follow instructions here: https://thesofproject.github.io/latest/getting_started/build-guide/build-from-scratch.html.

But it is enough to build toolchain only for Apollo Lake and skip others.

When it comes to building actual firmware we use the following configuration:

```bash
$ cd ~/work/sof/sof
$ ./scripts/xtensa-build-all.sh -d -r apl
```

Where `-d` for debug and `-r` for building ROM which is needed for Qemu to start with.

The built images of `ROM`, `bootloader` and SOF `kernel` are located in
`build_apl_gcc/src/arch/xtensa`.

### Build SOF image with Docker

As an alternative, SOF provides a docker image that contains all required toolchains.

Refer the following page for more details: https://thesofproject.github.io/latest/getting_started/build-guide/build-with-docker.html

At first, install docker image.

```bash
$ docker pull thesofproject/sof
$ docker tag thesofproject/sof sof
```

Run docker image with the script.

```bash
$ cd ~/work/sof/sof
$ ./scripts/docker-run.sh ./scripts/xtensa-build-all.sh -d -r apl
```

Or run docker with bash and run the command from there

```bash
$ ./scripts/docker-run.sh bash
```

Now in docker shell run

```bash
$ ./scripts/xtensa-build-all.sh -d -r apl
```

## Create rimage (*.ri)

The image is created with rimage tool (found inside SOF project). The sequence
is taken from `src/arch/xtensa/CMakeLists.txt`.

```bash
$ cd ~/work/sof/sof
$ xtensa-apl-elf-objcopy -O binary build_apl_gcc/src/platform/apollolake/base_module mod-apl.bin
$ xtensa-apl-elf-objcopy --add-section .module=mod-apl.bin --set-section-flags .module=load,readonly ../zephyr/build/zephyr/zephyr.elf
$ build_apl_gcc/rimage_ep/build/rimage -o sof-apl.ri -p sof-apl.ldc -m apl -k rimage/keys/otc_private_key.pem -i 3 build_apl_gcc/src/arch/xtensa/bootloader-apl  ../zephyr/build/zephyr/zephyr.elf
```

The image created `sof-apl.ri` is our final image we can use on Qemu or on `up_squared` ADSP.

### Create rimage with Docker

Same SOF docker image can be used to create the rimage.

Copy the `zephyr.elf`, built from [Build Zephyr Kernel](#build-zephyr-kernel)
section, to sof source directory, so it can be accessed in the docker
environment.

```bash
$ cd ~/work/sof/sof
$ mkdir zephyr
$ cp <zephyr dir>/build/zephyr/zephyr.elf zephyr/
```

Run SOF docker image and build the image

```bash
$ cd ~/work/sof/sof
$ ./scripts/docker-run.sh bash
#
# Now in the docker shell
#
$ xtensa-apl-elf-objcopy -O binary build_apl_gcc/src/platform/apollolake/base_module mod-apl.bin
$ xtensa-apl-elf-objcopy --add-section .module=mod-apl.bin --set-section-flags .module=load,readonly zephyr/zephyr.elf
$ build_apl_gcc/rimage_ep/build/rimage -o sof-apl.ri -p sof-apl.ldc -m apl -k rimage/keys/otc_private_key.pem -i 3 build_apl_gcc/src/arch/xtensa/bootloader-apl zephyr/zephyr.elf
```

# Using image on Qemu (SOF Qemu)

## Install SOF Qemu

This is Qemu with SOF patches for the APL xtensa. It is capable of running
exactly the same code as a real hardware (ADSP on `up_squared`).

### Get Qemu with docker

```bash
$ docker pull thesofproject/sofqemu
$ docker tag thesofproject/sofqemu sofqemu
```

### Build Qemu from local sources

Instructions are taken from docker build configuration in
`scripts/docker_build/sof_qemu/Dockerfile`

```bash
$ git clone https://github.com/thesofproject/qemu.git
$ cd qemu
$ git checkout sof-stable
$ ./configure --prefix=`pwd`/ --target-list=xtensa-softmmu --enable-coroutine-pool
$ make
```

## Running and debugging with Qemu

### Run image on Qemu

```bash
$ cd qemu
$ ./xtensa-softmmu/qemu-system-xtensa -cpu broxton -M adsp_bxt -nographic -kernel ../sof.git/sof-apl.ri -rom ../sof.git/build_apl_gcc/src/arch/xtensa/rom-apl.bin -s -S -semihosting
```

Note that `-s` is a _shorthand for -gdb tcp::1234_ and `-S` is to _freeze
CPU at startup and use ‘c’ to start execution_. Skip these options to run
without debugger.

Note that `-semihosting` enables logging to qemu console using simcall
instructions with xtensa simulator backend.

Use the `sof-apl.ri` created in the [Create rimage](#create-rimage-ri) section.

Using helper script:

```bash
$ ./xtensa-host.sh apl -d -k ../sof/sof-apl.ri -r ../sof/build_apl_gcc/src/arch/xtensa/rom-apl.bin
```

Where `sof-apl.ri` is an image created with rimage tool combining bootloader
and kernel, `rom-apl.bin` - is ROM image which should do the same boot
sequence as xtensa DSP. Since we can run exactly the same images on Qemu and on
a real `up_squared` hardware we can use this Qemu for early development.

### Run image with Docker Qemu

Use the following script to run the SOF Qemu docker image.

```bash
$ cd ~/work/sof/sof
$ ./scripts/docker-qemu.sh bash
```

Then follow the commands above.

### Qemu target - host communication

Communication between Qemu running Zephyr and host Linux can be done through
/dev/shm. For example if Zephyr code in the sample does something like

```cpp
mailbox_sw_reg_write(SRAM_REG_ROM_STATUS, 0xabbac0fe);
```

In the Linux host we can verify value written with:

```bash
$ hexdump -C /dev/shm/qemu-bridge-hp-sram-mem | grep e000
0000e000  fe c0 ba ab 00 00 00 00  50 01 00 00 00 00 00 00  |........P.......|
```

### Connect gdb debugger to the Qemu

Connect gdb with:

```bash
$ xtensa-apl-elf-gdb build_apl_gcc/sof
```

Connect GUI frontend **nemiver** with:

```bash
$ nemiver --gdb-binary=/home/niko/work/sof/xtensa-apl-elf/bin/xtensa-apl-elf-gdb --remote=localhost:1234 build_apl_gcc/sof
```

Connect **DDD** frontend with:

```bash
$ ddd --debugger xtensa-apl-elf-gdb build_apl_gcc/sof
```

To connect to the target you can use script:

```bash
$ cat script.gdb
target remote :1234
```

Run it with gdb script with:

```bash
$ gdb --command=script.gdb ....
```

# Using image on `up_squared` board

At the moment only `up_squared` board is supported.

## Prerequisites and Setup instructions

Please refer to https://thesofproject.github.io/latest/getting_started/setup/setup_up_2_board.html
for prerequitites and instructions.

## Run image on ADSP

After copying `sof-apl.ri` into `/lib/firmware/intel/sof` SOF kernel module
should initialize and start ADSP.

# Logging

## Using xtensa simulator backend

Xtensa simulator backend, already present in Zephyr, uses `simcall`
instructions to print to qemu console, so it works only in qemu. You should be
able to select this backend for this  board.

**_Note: Important to run qemu with the key “-semihosting”._**

## Using Experimental memory ring buffer log backend

There is adsp logging backend implemented writing to the same memory location
as SOF. Logging implemented using the memory as a ring buffer with slots of 256
bytes. Logs are prepended with magic number 2 bytes and log id 2 bytes. On a
host side logs might be read with python tool.
