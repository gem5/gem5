# Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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
from m5.defines import buildEnv
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class X86GPUTLB(ClockedObject):
    type = "X86GPUTLB"
    cxx_class = "gem5::X86ISA::GpuTLB"
    cxx_header = "arch/amdgpu/common/tlb.hh"
    size = Param.Int(64, "TLB size (number of entries)")
    assoc = Param.Int(64, "TLB associativity")

    if buildEnv.get("FULL_SYSTEM", False):
        walker = Param.X86PagetableWalker(
            X86PagetableWalker(), "page table walker"
        )

    hitLatency = Param.Int(2, "Latency of a TLB hit")
    missLatency1 = Param.Int(5, "Latency #1 of a TLB miss")
    missLatency2 = Param.Int(100, "Latency #2 of a TLB miss")
    maxOutstandingReqs = Param.Int(64, "# of maximum outstanding requests")
    cpu_side_ports = VectorResponsePort("Ports on side closer to CPU/CU")
    slave = DeprecatedParam(
        cpu_side_ports, "`slave` is now called `cpu_side_ports`"
    )
    mem_side_ports = VectorRequestPort("Ports on side closer to memory")
    master = DeprecatedParam(
        mem_side_ports, "`master` is now called `mem_side_ports`"
    )
    allocationPolicy = Param.Bool(True, "Allocate on an access")
    accessDistance = Param.Bool(False, "print accessDistance stats")


class TLBCoalescer(ClockedObject):
    type = "TLBCoalescer"
    cxx_class = "gem5::TLBCoalescer"
    cxx_header = "arch/amdgpu/common/tlb_coalescer.hh"

    probesPerCycle = Param.Int(2, "Number of TLB probes per cycle")
    coalescingWindow = Param.Int(1, "Permit coalescing across that many ticks")
    cpu_side_ports = VectorResponsePort("Port on side closer to CPU/CU")
    slave = DeprecatedParam(
        cpu_side_ports, "`slave` is now called `cpu_side_ports`"
    )
    mem_side_ports = VectorRequestPort("Port on side closer to memory")
    master = DeprecatedParam(
        mem_side_ports, "`master` is now called `mem_side_ports`"
    )
    disableCoalescing = Param.Bool(False, "Dispable Coalescing")
