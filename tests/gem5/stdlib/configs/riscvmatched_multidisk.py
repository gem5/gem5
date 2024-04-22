# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This is a gem5 configuration script to test the multi-disk image feature for
the RISCVMatchedBoard.

By running `lsblk` command after boot, we can see the disks mounted. If the two
disks in this example are mounted, the output should be:

```shell
NAME    MAJ:MIN RM  SIZE RO TYPE MOUNTPOINT
vda     254:0    0  7.5G  0 disk
├─vda1  254:1    0  7.4G  0 part /
├─vda12 254:12   0    4M  0 part
├─vda13 254:13   0    1M  0 part
├─vda14 254:14   0    4M  0 part
└─vda15 254:15   0  106M  0 part
vdb     254:16   0  7.5G  0 disk
├─vdb1  254:17   0  7.4G  0 part
├─vdb12 254:28   0    4M  0 part
├─vdb13 254:29   0    1M  0 part
├─vdb14 254:30   0    4M  0 part
└─vdb15 254:31   0  106M  0 part
```

Note that the same disk image has been mounted twice, but only once as the root
filesystem.
"""

import argparse

from gem5.isas import ISA
from gem5.prebuilt.riscvmatched.riscvmatched_board import RISCVMatchedBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(isa_required=ISA.RISCV)

argparser = argparse.ArgumentParser()

# Set the resource directory.
argparser.add_argument(
    "--resource-directory", type=str, required=False, default=None
)

args = argparser.parse_args()


board = RISCVMatchedBoard(
    is_fs=True,
)

bootloader = obtain_resource(
    "riscv-bootloader-opensbi-1.3.1",
    resource_directory=args.resource_directory,
)
boot_image = obtain_resource(
    "riscv-ubuntu-20.04-img", resource_directory=args.resource_directory
)
kernel = obtain_resource(
    "riscv-linux-6.5.5-kernel", resource_directory=args.resource_directory
)
additional_images = [
    obtain_resource(
        "riscv-ubuntu-20.04-img", resource_directory=args.resource_directory
    )
]

board.set_kernel_disk_workload(
    kernel=kernel,
    bootloader=bootloader,
    disk_image=boot_image,
    additional_disk_images=additional_images,
    readfile_contents="lsblk",
)

simulator = Simulator(board=board)
simulator.run()
