# Using gem5 in an SST simulation

## Overview

This directory contains the library needed to use gem5 TimingSimpleCPU model in
an SST-driven simulation.

When compiled, the gem5 library for SST `libgem5.so` will be generated,
containing the `libgem5_*.so` as well as the gem5 Component and the
SST Responder SubComponent.

```text
                           On/Off-chip devs     TimingSimpleCPU
                                ^                 ^         ^
                                |                 |         |
                                v                 v         v
                            ==================================== [gem5::NonCoherentXBar]
                                                      ^ [OutgoingRequestBridge]
           gem5_system_port                           |
[OutgoingRequestBridge] ^                             |
                        |                             |
         [SSTResponder] v                             v [SSTResponder]
gem5 Component {SSTResponderSubComponent, SSTResponderSubComponent}
                         ^                        ^
                         |                        |
                         v                        v
                    ================================== [SST Bus]
                                        ^
                                        |
                                        v
                                     SST cache  <---->  SST memory
```

## Components and SubComponents

- gem5 Component has the following responsibilities,
  - initializing the gem5 Python environment
  - instantiating/setting-up the gem5 SimObjects as specified by the gem5
configuration
  - connect every SSTResponderSubComponent to the corresponding
OutgoingRequestBridge
  - handling a gem5 event queue (with all thread-synchronization barriers
removed)
  - handling executions of gem5 events when it has clockTick yielded by SST
**Note:** there should only be one gem5 Component per process.

- SSTResponderSubComponent has the responsibity of receiving requests from
gem5, translating requests to an SST Request and sending it to SSTResponder.
Upon receiving a response from the memory interface, SSTResponderSubComponent
will translate the response to a gem5 Packet and send it to the its
OutgoingRequestBridge.

- SSTResponder is owned by SSTResponderSubComponent. The responder will receive
the request from the SubComponent and send it to the SST memory hierarchy.

## Installation

See `INSTALL.md`.

## Running an example simulation (RISCV)

Downloading the built bootloader containing a Linux Kernel and a workload,

```sh
wget http://dist.gem5.org/dist/develop/misc/riscv/bbl-busybox-boot-exit
```

Running the simulation

```sh
sst --add-lib-path=./ sst/example.py
```

The example SST system configuration will instantiate the gem5 system
as specified in the gem5 system configuration located at
`gem5/configs/example/sst/riscv_fs.py`. This configuration will download
the `bbl-busybox-boot-exit` resource, which contains an m5 binary, and
`m5 exit` will be called upon the booting process reaching the early userspace.
More information about building a bootloader containing a Linux Kernel and a
customized workload is available at
[https://github.com/gem5/gem5-resources/tree/stable/src/riscv-boot-exit-nodisk].

## Running an example simulation (Arm)

Download the prebuilt bootloader and Linux Kernel with embedded initramfs and
extract them under the $M5_PATH directory (make sure M5_PATH points to a valid
directory):

```sh
wget http://dist.gem5.org/dist/develop/arm/aarch-sst-20211207.tar.bz2
tar -xf aarch-sst-20211207.tar.bz2

# copying bootloaders
cp binaries/boot* $M5_PATH/binaries/

# copying Linux Kernel
cp binaries/vmlinux_exit.arm64 $M5_PATH/binaries/
```

`vmlinux_exit.arm64` contains an m5 binary, and `m5 exit` will be called upon
the booting process reaching the early userspace.

Run the simulation:

```sh
sst sst/arm_example.py
```

## Notes

- SwapReq from gem5 requires reading from memory and writing to memory.
We handle the request in SST in a way that, when SST gets the response
from memory, SST will send that response to gem5, while SST will send
a write request with modified data to memory.
