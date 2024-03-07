# Copyright (c) 2023 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import base64
import os
import sys
import tempfile

import runfs
from amd import AmdGPUOptions
from common import (
    GPUTLBOptions,
    Options,
)
from ruby import Ruby

import m5

demo_runscript_without_checkpoint = """\
export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH
export HSA_ENABLE_INTERRUPT=0
export HCC_AMDGPU_TARGET=gfx90a
free -m
dmesg -n8
dd if=/root/roms/mi200.rom of=/dev/mem bs=1k seek=768 count=128
if [ ! -f /lib/modules/`uname -r`/updates/dkms/amdgpu.ko ]; then
    echo "ERROR: Missing DKMS package for kernel `uname -r`. Exiting gem5."
    /sbin/m5 exit
fi
modprobe -v amdgpu ip_block_mask=0x6f ppfeaturemask=0 dpm=0 audio=0 ras_enable=0
echo "Running {} {}"
echo "{}" | base64 -d > myapp
chmod +x myapp
./myapp {}
/sbin/m5 exit
"""

demo_runscript_with_checkpoint = """\
export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH
export HSA_ENABLE_INTERRUPT=0
export HCC_AMDGPU_TARGET=gfx90a
dmesg -n8
dd if=/root/roms/mi200.rom of=/dev/mem bs=1k seek=768 count=128
if [ ! -f /lib/modules/`uname -r`/updates/dkms/amdgpu.ko ]; then
    echo "ERROR: Missing DKMS package for kernel `uname -r`. Exiting gem5."
    /sbin/m5 exit
fi
modprobe -v amdgpu ip_block_mask=0x6f ppfeaturemask=0 dpm=0 audio=0 ras_enable=0
echo "Running {} {}"
echo "{}" | base64 -d > myapp
chmod +x myapp
/sbin/m5 checkpoint
./myapp {}
/sbin/m5 exit
"""


def addDemoOptions(parser):
    parser.add_argument(
        "-a", "--app", default=None, help="GPU application to run"
    )
    parser.add_argument(
        "-o", "--opts", default="", help="GPU application arguments"
    )


def runMI200GPUFS(cpu_type):
    parser = argparse.ArgumentParser()
    runfs.addRunFSOptions(parser)
    Options.addCommonOptions(parser)
    AmdGPUOptions.addAmdGPUOptions(parser)
    Ruby.define_options(parser)
    GPUTLBOptions.tlb_options(parser)
    addDemoOptions(parser)

    # Parse now so we can override options
    args = parser.parse_args()
    demo_runscript = ""

    # Create temp script to run application
    if args.app is None:
        print(f"No application given. Use {sys.argv[0]} -a <app>")
        sys.exit(1)
    elif args.kernel is None:
        print(f"No kernel path given. Use {sys.argv[0]} --kernel <vmlinux>")
        sys.exit(1)
    elif args.disk_image is None:
        print(f"No disk path given. Use {sys.argv[0]} --disk-image <linux>")
        sys.exit(1)
    elif not os.path.isfile(args.app):
        print("Could not find applcation", args.app)
        sys.exit(1)

    # Choose runscript Based on whether any checkpointing args are set
    if args.checkpoint_dir is not None:
        demo_runscript = demo_runscript_with_checkpoint
    else:
        demo_runscript = demo_runscript_without_checkpoint

    with open(os.path.abspath(args.app), "rb") as binfile:
        encodedBin = base64.b64encode(binfile.read()).decode()

    _, tempRunscript = tempfile.mkstemp()
    with open(tempRunscript, "w") as b64file:
        runscriptStr = demo_runscript.format(
            args.app, args.opts, encodedBin, args.opts
        )
        b64file.write(runscriptStr)

    if args.second_disk == None:
        args.second_disk = args.disk_image

    # Defaults for MI200
    args.ruby = True
    args.cpu_type = "X86KvmCPU"
    args.mem_size = "8GB"  # CPU host memory
    args.dgpu = True
    args.dgpu_mem_size = "16GB"  # GPU device memory
    args.dgpu_start = "0GB"
    args.checkpoint_restore = 0
    args.disjoint = True
    args.timing_gpu = True
    args.script = tempRunscript
    args.dgpu_xor_low_bit = 0
    args.gpu_device = "MI200"

    # Run gem5
    runfs.runGpuFSSystem(args)


if __name__ == "__m5_main__":
    runMI200GPUFS("X86KvmCPU")
