# -*- mode:python -*-

# Copyright (c) 2009, 2013, 2015 ARM Limited
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

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5.objects.BaseTLB import BaseTLB
from m5.objects.ClockedObject import ClockedObject

# Basic stage 1 translation objects
class ArmTableWalker(ClockedObject):
    type = 'ArmTableWalker'
    cxx_class = 'ArmISA::TableWalker'
    cxx_header = "arch/arm/table_walker.hh"
    is_stage2 =  Param.Bool(False, "Is this object for stage 2 translation?")
    num_squash_per_cycle = Param.Unsigned(2,
            "Number of outstanding walks that can be squashed per cycle")

    # The port to the memory system. This port is ultimately belonging
    # to the Stage2MMU, and shared by the two table walkers, but we
    # access it through the ITB and DTB walked objects in the CPU for
    # symmetry with the other ISAs.
    port = MasterPort("Port used by the two table walkers")

    sys = Param.System(Parent.any, "system object parameter")

class ArmTLB(BaseTLB):
    type = 'ArmTLB'
    cxx_class = 'ArmISA::TLB'
    cxx_header = "arch/arm/tlb.hh"
    sys = Param.System(Parent.any, "system object parameter")
    size = Param.Int(64, "TLB size")
    walker = Param.ArmTableWalker(ArmTableWalker(), "HW Table walker")
    is_stage2 = Param.Bool(False, "Is this a stage 2 TLB?")

# Stage 2 translation objects, only used when virtualisation is being used
class ArmStage2TableWalker(ArmTableWalker):
    is_stage2 = True

class ArmStage2TLB(ArmTLB):
    size = 32
    walker = ArmStage2TableWalker()
    is_stage2 = True

class ArmStage2MMU(SimObject):
    type = 'ArmStage2MMU'
    cxx_class = 'ArmISA::Stage2MMU'
    cxx_header = 'arch/arm/stage2_mmu.hh'
    tlb = Param.ArmTLB("Stage 1 TLB")
    stage2_tlb = Param.ArmTLB("Stage 2 TLB")

    sys = Param.System(Parent.any, "system object parameter")

class ArmStage2IMMU(ArmStage2MMU):
    # We rely on the itb being a parameter of the CPU, and get the
    # appropriate object that way
    tlb = Parent.any
    stage2_tlb = ArmStage2TLB()

class ArmStage2DMMU(ArmStage2MMU):
    # We rely on the dtb being a parameter of the CPU, and get the
    # appropriate object that way
    tlb = Parent.any
    stage2_tlb = ArmStage2TLB()

class ArmITB(ArmTLB):
    stage2_mmu = ArmStage2IMMU()

class ArmDTB(ArmTLB):
    stage2_mmu = ArmStage2DMMU()
