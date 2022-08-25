# Copyright (c) 2022 Advanced Micro Devices, Inc.
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

import m5
import runfs
import tempfile
import argparse
import sys
import os

from amd import AmdGPUOptions
from common import Options
from common import GPUTLBOptions
from ruby import Ruby

samples_runscript = """\
export LD_LIBRARY_PATH=/opt/rocm/lib:$LD_LIBRARY_PATH
export HSA_ENABLE_INTERRUPT=0
dmesg -n3
dd if=/root/roms/vega10.rom of=/dev/mem bs=1k seek=768 count=128
if [ ! -f /lib/modules/`uname -r`/updates/dkms/amdgpu.ko ]; then
    echo "ERROR: Missing DKMS package for kernel `uname -r`. Exiting gem5."
    /sbin/m5 exit
fi
modprobe -v amdgpu ip_block_mask=0xff ppfeaturemask=0 dpm=0 audio=0
echo "Running {}"
cd /home/gem5/HIP-Examples/HIP-Examples-Applications/{}/
make clean
make
/sbin/m5 exit
"""


def addSamplesOptions(parser):
    parser.add_argument(
        "-a",
        "--app",
        default=None,
        choices=[
            "BinomialOption",
            "BitonicSort",
            "FastWalshTransform",
            "FloydWarshall",
            "HelloWorld",
            "Histogram",
            "MatrixMultiplication",
            "PrefixSum",
            "RecursiveGaussian",
            "SimpleConvolution",
            "dct",
            "dwtHaar1D",
        ],
        help="GPU application to run",
    )
    parser.add_argument(
        "-o", "--opts", default="", help="GPU application arguments"
    )


if __name__ == "__m5_main__":
    parser = argparse.ArgumentParser()
    runfs.addRunFSOptions(parser)
    Options.addCommonOptions(parser)
    AmdGPUOptions.addAmdGPUOptions(parser)
    Ruby.define_options(parser)
    GPUTLBOptions.tlb_options(parser)
    addSamplesOptions(parser)

    # Parse now so we can override options
    args = parser.parse_args()

    # Create temp script to run application
    if args.app is None:
        print("No application given. Use %s -a <app>" % sys.argv[0])
        sys.exit(1)
    elif args.kernel is None:
        print("No kernel path given. Use %s --kernel <vmlinux>" % sys.argv[0])
        sys.exit(1)
    elif args.disk_image is None:
        print("No disk path given. Use %s --disk-image <linux>" % sys.argv[0])
        sys.exit(1)
    elif args.gpu_mmio_trace is None:
        print(
            "No MMIO trace path. Use %s --gpu-mmio-trace <path>" % sys.argv[0]
        )
        sys.exit(1)

    _, tempRunscript = tempfile.mkstemp()
    with open(tempRunscript, "w") as b64file:
        runscriptStr = samples_runscript.format(args.app, args.app)
        b64file.write(runscriptStr)

    if args.second_disk == None:
        args.second_disk = args.disk_image

    # Defaults for Vega10
    args.ruby = True
    args.cpu_type = "X86KvmCPU"
    args.num_cpus = 1
    args.mem_size = "3GB"
    args.dgpu = True
    args.dgpu_mem_size = "16GB"
    args.dgpu_start = "0GB"
    args.checkpoint_restore = 0
    args.disjoint = True
    args.timing_gpu = True
    args.script = tempRunscript
    args.dgpu_xor_low_bit = 0

    print(args.disk_image)

    # Run gem5
    runfs.runGpuFSSystem(args)
