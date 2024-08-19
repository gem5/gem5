# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
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

from m5.objects import (
    ArmDefaultRelease,
    VExpress_GEM5_Foundation,
    VExpress_GEM5_V1,
)
from m5.util import warn

from gem5.components.boards.arm_board import ArmBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.utils.requires import requires


class ArmDemoBoard(ArmBoard):
    """
    This prebuilt ARM board is used for demonstration purposes. It simulates an
    ARM 3GHz dual-core system with a 2GB DDR4_2400 memory system. A
    PrivateL1PrivateL2 cache hierarchy is set with an l1 data and instruction
    cache, each 16kB, and a single bank l2 cache of 256kB.

    **DISCLAIMER**: This board is solely for demonstration purposes. This board
    is not known to be representative of any real-world system or produce
    reliable statistical results.
    """

    def __init__(self, use_kvm=False):
        """
        :param use_kvm: If True, the board will use a SimpleProcessor
            with cpy type of CPUTypes.KVM. If False, the board will use a SimpleProcessor with
            a cpu type of CPUTypes.TIMING.
        """
        requires(
            isa_required=ISA.ARM,
        )

        warn(
            "The ARMDemoBoard is solely for demonstration purposes. "
            "This board is not known to be be representative of any "
            "real-world system. Use with caution."
        )
        cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
            l1d_size="16kB", l1i_size="16kB", l2_size="256kB"
        )

        memory = DualChannelDDR4_2400(size="2GB")

        if use_kvm:
            processor = SimpleProcessor(
                cpu_type=CPUTypes.TIMING, num_cores=2, isa=ISA.ARM
            )
            release = ArmDefaultRelease.for_kvm()
            platform = VExpress_GEM5_V1()

        else:
            processor = SimpleProcessor(
                cpu_type=CPUTypes.TIMING, num_cores=2, isa=ISA.ARM
            )
            release = ArmDefaultRelease()
            platform = VExpress_GEM5_Foundation()

        super().__init__(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
            release=release,
            platform=platform,
        )
