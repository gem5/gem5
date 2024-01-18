# Copyright 2021 Google, Inc.
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

from m5.objects.BaseAtomicSimpleCPU import BaseAtomicSimpleCPU
from m5.objects.BaseMinorCPU import BaseMinorCPU
from m5.objects.BaseNonCachingSimpleCPU import BaseNonCachingSimpleCPU
from m5.objects.BaseO3CPU import BaseO3CPU
from m5.objects.BaseTimingSimpleCPU import BaseTimingSimpleCPU
from m5.objects.FuncUnit import *
from m5.objects.FUPool import *
from m5.objects.X86Decoder import X86Decoder
from m5.objects.X86ISA import X86ISA
from m5.objects.X86LocalApic import X86LocalApic
from m5.objects.X86MMU import X86MMU
from m5.proxy import Self


class X86CPU:
    ArchDecoder = X86Decoder
    ArchMMU = X86MMU
    ArchInterrupts = X86LocalApic
    ArchISA = X86ISA


class X86AtomicSimpleCPU(BaseAtomicSimpleCPU, X86CPU):
    mmu = X86MMU()


class X86NonCachingSimpleCPU(BaseNonCachingSimpleCPU, X86CPU):
    mmu = X86MMU()


class X86TimingSimpleCPU(BaseTimingSimpleCPU, X86CPU):
    mmu = X86MMU()


class X86IntMultDiv(IntMultDiv):
    # DIV and IDIV instructions in x86 are implemented using a loop which
    # issues division microops.  The latency of these microops should really be
    # one (or a small number) cycle each since each of these computes one bit
    # of the quotient.
    opList = [
        OpDesc(opClass="IntMult", opLat=3),
        OpDesc(opClass="IntDiv", opLat=1, pipelined=False),
    ]

    count = 2


class DefaultX86FUPool(FUPool):
    FUList = [
        IntALU(),
        X86IntMultDiv(),
        FP_ALU(),
        FP_MultDiv(),
        ReadPort(),
        SIMD_Unit(),
        PredALU(),
        WritePort(),
        RdWrPort(),
        IprPort(),
    ]


class X86O3CPU(BaseO3CPU, X86CPU):
    mmu = X86MMU()
    needsTSO = True

    # For x86, each CC reg is used to hold only a subset of the
    # flags, so we need 4-5 times the number of CC regs as
    # physical integer regs to be sure we don't run out.  In
    # typical real machines, CC regs are not explicitly renamed
    # (it's a side effect of int reg renaming), so they should
    # never be the bottleneck here.
    numPhysCCRegs = Self.numPhysIntRegs * 5

    # DIV and IDIV instructions in x86 are implemented using a loop which
    # issues division microops.  The latency of these microops should really be
    # one (or a small number) cycle each since each of these computes one bit
    # of the quotient.
    fuPool = DefaultX86FUPool()


class X86MinorCPU(BaseMinorCPU, X86CPU):
    mmu = X86MMU()
