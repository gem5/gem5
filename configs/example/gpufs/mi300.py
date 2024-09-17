# Copyright (c) 2024 Advanced Micro Devices, Inc.
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

""" This file creates an X86 system with a KVM CPU and GPU device capable of
running the MI300 ISA (gfx942). Most of this file sets up a runscript which
will load in a binary, shell script, or python file from the host and run that
within gem5. Jump to line 146 for list of system parameters to configure.
"""

import argparse
import base64
import os
import sys
import tempfile
from typing import Optional

import runfs
from amd import AmdGPUOptions
from common import (
    GPUTLBOptions,
    Options,
)
from ruby import Ruby

import m5

from gem5.resources.resource import AbstractResource

demo_runscript_without_checkpoint = """\
export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH
export HSA_ENABLE_INTERRUPT=0
export HCC_AMDGPU_TARGET=gfx942
export HSA_OVERRIDE_GFX_VERSION="9.4.2"
dmesg -n8
cat /proc/cpuinfo
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
export HCC_AMDGPU_TARGET=gfx942
export HSA_OVERRIDE_GFX_VERSION="9.4.2"
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


def runMI300GPUFS(
    cpu_type,
    disk: Optional[AbstractResource] = None,
    kernel: Optional[AbstractResource] = None,
    app: Optional[AbstractResource] = None,
):
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

    if disk != None:
        args.disk_image = disk.get_local_path()
    if kernel != None:
        args.kernel = kernel.get_local_path()
    if app != None:
        args.app = app.get_local_path()

    # Create temp script to run application
    if not os.path.isfile(args.app):
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

    args.script = tempRunscript

    # Defaults for CPU
    args.cpu_type = "X86KvmCPU"
    args.mem_size = "8GiB"

    # Defaults for MI300X
    args.gpu_device = "MI300X"
    args.dgpu_mem_size = "16GB"  # GPU memory size, must be 16GB currently.

    # See: https://rocm.docs.amd.com/en/latest/conceptual/gpu-arch/mi300.html
    # Topology for one XCD. Number of CUs is approximately 304 / 8, rounded
    # up to 40 due to gem5 restriction of 4 CUs per SQC / scalar cache.
    args.num_compute_units = 40
    args.gpu_topology = "Crossbar"

    # Run gem5
    runfs.runGpuFSSystem(args)


if __name__ == "__m5_main__":
    runMI300GPUFS("X86KvmCPU")
