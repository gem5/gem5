#
#  Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
#  All rights reserved.
#
#  For use for simulation and test purposes only
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#  may be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Lisa Hsu
#

from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *

from m5.objects.MemObject import MemObject

if buildEnv['FULL_SYSTEM']:
    class X86PagetableWalker(MemObject):
        type = 'X86PagetableWalker'
        cxx_class = 'X86ISA::Walker'
        port = SlavePort("Port for the hardware table walker")
        system = Param.System(Parent.any, "system object")

class X86GPUTLB(MemObject):
    type = 'X86GPUTLB'
    cxx_class = 'X86ISA::GpuTLB'
    cxx_header = 'gpu-compute/gpu_tlb.hh'
    size = Param.Int(64, "TLB size (number of entries)")
    assoc = Param.Int(64, "TLB associativity")

    if buildEnv['FULL_SYSTEM']:
        walker = Param.X86PagetableWalker(X86PagetableWalker(),
                                          "page table walker")

    hitLatency = Param.Int(2, "Latency of a TLB hit")
    missLatency1 = Param.Int(5, "Latency #1 of a TLB miss")
    missLatency2 = Param.Int(100, "Latency #2 of a TLB miss")
    maxOutstandingReqs = Param.Int(64, "# of maximum outstanding requests")
    slave = VectorSlavePort("Port on side closer to CPU/CU")
    master = VectorMasterPort("Port on side closer to memory")
    allocationPolicy = Param.Bool(True, "Allocate on an access")
    accessDistance = Param.Bool(False, "print accessDistance stats")

class TLBCoalescer(MemObject):
    type = 'TLBCoalescer'
    cxx_class = 'TLBCoalescer'
    cxx_header = 'gpu-compute/tlb_coalescer.hh'
    probesPerCycle = Param.Int(2, "Number of TLB probes per cycle")
    coalescingWindow = Param.Int(1, "Permit coalescing across that many ticks")
    slave = VectorSlavePort("Port on side closer to CPU/CU")
    master = VectorMasterPort("Port on side closer to memory")
    disableCoalescing = Param.Bool(False,"Dispable Coalescing")
