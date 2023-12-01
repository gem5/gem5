# Copyright (c) 2021 The Regents of the University of California
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

from ...coherence_protocol import CoherenceProtocol
from ...components.boards.x86_board import X86Board
from ...components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from ...components.memory.single_channel import SingleChannelDDR3_1600
from ...components.processors.cpu_types import CPUTypes
from ...components.processors.simple_processor import SimpleProcessor
from ...isas import ISA
from ...utils.requires import requires


class X86DemoBoard(X86Board):
    """
    This prebuilt X86 board is used for demonstration purposes. It simulates
    an X86 3GHz quad-core system with a 2GB DDR3_1600 memory system. A
    MESI_Two_Level cache hierarchy is set with an l1 data and instruction
    cache, each 32kB with an associativity of 8, and a single bank l2 cache of
    1MB with an associativity of 16.

    **DISCLAIMER**: This board is solely for demonstration purposes. This board
    is not known to be representative of any real-world system or produce
    reliable statistical results.

    Example
    -------

    An example of using the X86DemoBoard can be found in
    ``configs/example/gem5_library/x86-ubuntu-run.py``.

    To run:

    .. code-block::

        scons build/X86/gem5.opt -j`nproc`
        ./build/X86/gem5.opt configs/example/gem5_library/x86-ubuntu-run.py

    """

    def __init__(self):
        requires(
            isa_required=ISA.X86,
            coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
        )

        warn(
            "The X86DemoBoard is solely for demonstration purposes. "
            "This board is not known to be be representative of any "
            "real-world system. Use with caution."
        )

        memory = SingleChannelDDR3_1600(size="2GB")
        processor = SimpleProcessor(
            cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=4
        )
        cache_hierarchy = MESITwoLevelCacheHierarchy(
            l1d_size="32kB",
            l1d_assoc=8,
            l1i_size="32kB",
            l1i_assoc=8,
            l2_size="1MB",
            l2_assoc=16,
            num_l2_banks=1,
        )

        super().__init__(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )
