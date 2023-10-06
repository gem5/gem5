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
from m5.objects.ArmDecoder import ArmDecoder
from m5.objects.ArmInterrupts import ArmInterrupts
from m5.objects.ArmISA import ArmISA
from m5.objects.ArmMMU import ArmMMU
from m5.objects.BaseAtomicSimpleCPU import BaseAtomicSimpleCPU
from m5.objects.BaseMinorCPU import BaseMinorCPU
from m5.objects.BaseNonCachingSimpleCPU import BaseNonCachingSimpleCPU
from m5.objects.BaseO3Checker import BaseO3Checker
from m5.objects.BaseO3CPU import BaseO3CPU
from m5.objects.BaseTimingSimpleCPU import BaseTimingSimpleCPU
from m5.proxy import Self


class ArmCPU:
    ArchDecoder = ArmDecoder
    ArchMMU = ArmMMU
    ArchInterrupts = ArmInterrupts
    ArchISA = ArmISA


class ArmAtomicSimpleCPU(BaseAtomicSimpleCPU, ArmCPU):
    mmu = ArmMMU()


class ArmNonCachingSimpleCPU(BaseNonCachingSimpleCPU, ArmCPU):
    mmu = ArmMMU()


class ArmTimingSimpleCPU(BaseTimingSimpleCPU, ArmCPU):
    mmu = ArmMMU()


class ArmO3Checker(BaseO3Checker, ArmCPU):
    mmu = ArmMMU()


class ArmO3CPU(BaseO3CPU, ArmCPU):
    mmu = ArmMMU()

    # For x86, each CC reg is used to hold only a subset of the
    # flags, so we need 4-5 times the number of CC regs as
    # physical integer regs to be sure we don't run out.  In
    # typical real machines, CC regs are not explicitly renamed
    # (it's a side effect of int reg renaming), so they should
    # never be the bottleneck here.
    numPhysCCRegs = Self.numPhysIntRegs * 5

    def addCheckerCpu(self):
        self.checker = ArmO3Checker(
            workload=self.workload,
            exitOnError=False,
            updateOnError=True,
            warnOnlyOnLoadError=True,
        )
        self.checker.mmu.itb.size = self.mmu.itb.size
        self.checker.mmu.dtb.size = self.mmu.dtb.size
        self.checker.cpu_id = self.cpu_id


class ArmMinorCPU(BaseMinorCPU, ArmCPU):
    mmu = ArmMMU()
