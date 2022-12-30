# Copyright (c) 2014-2016 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

from m5.params import *

from m5.objects.Device import BasicPioDevice
from m5.objects.Gic import *


class NoMaliGpuType(Enum):
    vals = ["T60x", "T62x", "T760"]


class NoMaliGpu(PioDevice):
    type = "NoMaliGpu"
    cxx_header = "dev/arm/gpu_nomali.hh"
    cxx_class = "gem5::NoMaliGpu"

    pio_addr = Param.Addr("Device base address")

    platform = Param.RealView(Parent.any, "Platform this device is part of.")

    gpu_type = Param.NoMaliGpuType("T760", "GPU type")
    ver_maj = Param.UInt32(0, "GPU Version (Major)")
    ver_min = Param.UInt32(0, "GPU Version (Minor)")
    ver_status = Param.UInt32(0, "GPU Version (Status)")

    int_gpu = Param.UInt32("Interrupt number for GPU interrupts")
    int_job = Param.UInt32("Interrupt number for JOB interrupts")
    int_mmu = Param.UInt32("Interrupt number for MMU interrupts")


class CustomNoMaliGpu(NoMaliGpu):
    """Base class for custom NoMali implementation that need to override
    configuration registers. See CustomNoMaliT760 for a usage example.

    """

    type = "CustomNoMaliGpu"
    cxx_header = "dev/arm/gpu_nomali.hh"
    cxx_class = "gem5::CustomNoMaliGpu"

    gpu_id = Param.UInt32("")
    l2_features = Param.UInt32("")
    tiler_features = Param.UInt32("")
    mem_features = Param.UInt32("")
    mmu_features = Param.UInt32("")
    as_present = Param.UInt32("")
    js_present = Param.UInt32("")

    thread_max_threads = Param.UInt32("")
    thread_max_workgroup_size = Param.UInt32("")
    thread_max_barrier_size = Param.UInt32("")
    thread_features = Param.UInt32("")

    texture_features = VectorParam.UInt32("")
    js_features = VectorParam.UInt32("")

    shader_present = Param.UInt64("")
    tiler_present = Param.UInt64("")
    l2_present = Param.UInt64("")


class CustomNoMaliT760(CustomNoMaliGpu):
    """Example NoMali T760 r0p0-0 configuration using the defaults from
    the NoMali library.

    """

    gpu_id = 0x07500000

    l2_features = 0x07130206
    tiler_features = 0x00000809
    mem_features = 0x00000001
    mmu_features = 0x00002830
    as_present = 0x000000FF
    js_present = 0x00000007

    thread_max_threads = 0x00000100
    thread_max_workgroup_size = 0x00000100
    thread_max_barrier_size = 0x00000100
    thread_features = 0x0A040400

    texture_features = [0x00FE001E, 0x0000FFFF, 0x9F81FFFF]
    js_features = [0x0000020E, 0x000001FE, 0x0000007E]

    shader_present = 0x0000000F
    tiler_present = 0x00000001
    l2_present = 0x00000001
