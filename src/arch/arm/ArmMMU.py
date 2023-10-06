# -*- mode:python -*-
# Copyright (c) 2020-2021 Arm Limited
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
from m5.objects.ArmSystem import ArmRelease
from m5.objects.ArmTLB import ArmStage2TLB
from m5.objects.ArmTLB import ArmTLB
from m5.objects.BaseMMU import BaseMMU
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


# Basic stage 1 translation objects
class ArmTableWalker(ClockedObject):
    type = "ArmTableWalker"
    cxx_class = "gem5::ArmISA::TableWalker"
    cxx_header = "arch/arm/table_walker.hh"
    is_stage2 = Param.Bool(False, "Is this object for stage 2 translation?")
    num_squash_per_cycle = Param.Unsigned(
        2,
        "Number of outstanding walks that can be squashed per cycle",
    )

    port = RequestPort("Table Walker port")

    sys = Param.System(Parent.any, "system object parameter")


# Stage 2 translation objects, only used when virtualisation is being used
class ArmStage2TableWalker(ArmTableWalker):
    is_stage2 = True


class ArmMMU(BaseMMU):
    type = "ArmMMU"
    cxx_class = "gem5::ArmISA::MMU"
    cxx_header = "arch/arm/mmu.hh"

    # L2 TLBs
    l2_shared = ArmTLB(entry_type="unified", size=1280, partial_levels=["L2"])

    # L1 TLBs
    itb = ArmTLB(entry_type="instruction", next_level=Parent.l2_shared)
    dtb = ArmTLB(entry_type="data", next_level=Parent.l2_shared)

    stage2_itb = Param.ArmTLB(
        ArmStage2TLB(entry_type="instruction"),
        "Stage 2 Instruction TLB",
    )
    stage2_dtb = Param.ArmTLB(
        ArmStage2TLB(entry_type="data"),
        "Stage 2 Data TLB",
    )

    itb_walker = Param.ArmTableWalker(ArmTableWalker(), "HW Table walker")
    dtb_walker = Param.ArmTableWalker(ArmTableWalker(), "HW Table walker")

    stage2_itb_walker = Param.ArmTableWalker(
        ArmStage2TableWalker(),
        "HW Table walker",
    )
    stage2_dtb_walker = Param.ArmTableWalker(
        ArmStage2TableWalker(),
        "HW Table walker",
    )

    sys = Param.System(Parent.any, "system object parameter")

    release_se = Param.ArmRelease(
        Parent.isa[0].release_se,
        "Set of features/extensions to use in SE mode",
    )

    @classmethod
    def walkerPorts(cls):
        return [
            "mmu.itb_walker.port",
            "mmu.dtb_walker.port",
            "mmu.stage2_itb_walker.port",
            "mmu.stage2_dtb_walker.port",
        ]

    def connectWalkerPorts(self, iport, dport):
        self.itb_walker.port = iport
        self.dtb_walker.port = dport
        self.stage2_itb_walker.port = iport
        self.stage2_dtb_walker.port = dport
