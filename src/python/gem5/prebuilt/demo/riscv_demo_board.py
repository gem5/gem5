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

from m5.util import warn

from ...components.boards.riscv_board import RiscvBoard
from ...components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from ...components.memory import DualChannelDDR4_2400
from ...components.processors.cpu_types import CPUTypes
from ...components.processors.simple_processor import SimpleProcessor
from ...isas import ISA
from ...resources.resource import AbstractResource
from ...utils.override import overrides
from ...utils.requires import requires


class RiscvDemoBoard(RiscvBoard):
    """
    This board is based on the X86DemoBoard.

    This prebuilt RISCV board is used for demonstration purposes. It simulates
    an RISCV 1.4GHz dual-core system with a 4GiB DDR4_2400 memory system. A
    private L1, shared L2 cache hierarchy is set with a l1 data and instruction
    cache, each 64KiB with an associativity of 8, and a single bank l2 cache of
    1MiB with an associativity of 16.

    **DISCLAIMER**: This board is solely for demonstration purposes. This board
    is not known to be representative of any real-world system or produce
    reliable statistical results.

    """

    def __init__(self):
        requires(
            isa_required=ISA.RISCV,
        )

        warn(
            "The RiscvDemoBoard is solely for demonstration purposes. "
            "This board is not known to be be representative of any "
            "real-world system. Use with caution."
        )

        memory = DualChannelDDR4_2400(size="4GiB")

        processor = SimpleProcessor(
            cpu_type=CPUTypes.TIMING,
            isa=ISA.RISCV,
            num_cores=2,
        )

        # Here we setup the parameters of the l1 and l2 caches.
        cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
            l1d_size="64KiB", l1i_size="64KiB", l2_size="1MiB"
        )

        super().__init__(
            clk_freq="1.4GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )
