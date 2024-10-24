# -*- mode:python -*-

# Copyright (c) 2009, 2013, 2015, 2021, 2024 Arm Limited
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

from m5.objects.BaseTLB import BaseTLB
from m5.objects.ReplacementPolicies import LRURP
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class ArmLookupLevel(Enum):
    vals = ["L0", "L1", "L2", "L3"]


class TLBIndexingPolicy(SimObject):
    type = "TLBIndexingPolicy"
    abstract = True
    cxx_class = "gem5::IndexingPolicyTemplate<gem5::ArmISA::TLBTypes>"
    cxx_header = "arch/arm/pagetable.hh"
    cxx_template_params = ["class Types"]

    # Get the size from the parent (cache)
    num_entries = Param.Int(Parent.size, "number of TLB entries")

    # Get the associativity
    assoc = Param.Int(Parent.assoc, "associativity")


class TLBSetAssociative(TLBIndexingPolicy):
    type = "TLBSetAssociative"
    cxx_class = "gem5::ArmISA::TLBSetAssociative"
    cxx_header = "arch/arm/pagetable.hh"


class ArmTLB(BaseTLB):
    type = "ArmTLB"
    cxx_class = "gem5::ArmISA::TLB"
    cxx_header = "arch/arm/tlb.hh"
    sys = Param.System(Parent.any, "system object parameter")
    size = Param.Int(64, "TLB size")
    assoc = Param.Int(
        Self.size, "Associativity of the TLB. Fully Associative by default"
    )
    indexing_policy = Param.TLBIndexingPolicy(
        TLBSetAssociative(assoc=Parent.assoc, num_entries=Parent.size),
        "Indexing policy of the TLB",
    )
    replacement_policy = Param.BaseReplacementPolicy(
        LRURP(), "Replacement policy of the TLB"
    )
    is_stage2 = Param.Bool(False, "Is this a stage 2 TLB?")

    partial_levels = VectorParam.ArmLookupLevel(
        [],
        "List of intermediate lookup levels allowed to be cached in the TLB "
        "(=holding intermediate PAs obtained during a table walk",
    )


class ArmStage2TLB(ArmTLB):
    size = 32
    is_stage2 = True
