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

from m5.objects import (
    IOXBar,
    Pc,
    Port,
    X86FsLinux,
)
from m5.util import warn

from ...components.boards.se_binary_workload import SEBinaryWorkload
from ...components.boards.x86_board import X86Board
from ...components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from ...components.memory.multi_channel import DualChannelDDR4_2400
from ...components.processors.cpu_types import CPUTypes
from ...components.processors.simple_processor import SimpleProcessor
from ...isas import ISA
from ...utils.override import overrides
from ...utils.requires import requires


class X86DemoBoard(X86Board, SEBinaryWorkload):
    """
    This prebuilt X86 board is used for demonstration purposes. It simulates
    an X86 3GHz dual-core system with a 3GiB DDR4_2400 memory system. The
    cache hierarchy consists of per-core private L1 instruction and data
    caches (64KiB each) connected to a shared 8MiB L2 cache.

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
        )

        warn(
            "The X86DemoBoard is solely for demonstration purposes. "
            "This board is not known to be be representative of any "
            "real-world system. Use with caution."
        )

        # The other demo boards have 4 GiB of memory, but X86Board can only
        # support up to 3 GiB.
        memory = DualChannelDDR4_2400(size="3GiB")
        processor = SimpleProcessor(
            cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=2
        )

        cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
            l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
        )

        super().__init__(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

    @overrides(X86Board)
    def _setup_board(self) -> None:
        if self._is_fs:
            self.pc = Pc()

            self.workload = X86FsLinux()

            # North Bridge
            self.iobus = IOXBar()

            # Set up all of the I/O.
            self._setup_io_devices()

            self.m5ops_base = 0xFFFF0000

    @overrides(X86Board)
    def has_io_bus(self) -> bool:
        return self.is_fullsystem()

    @overrides(X86Board)
    def get_io_bus(self) -> IOXBar:
        if self.has_io_bus():
            return self.iobus
        else:
            raise NotImplementedError(
                "X86DemoBoard does not have an IO bus. "
                "Use `has_io_bus()` to check this."
            )

    @overrides(X86Board)
    def has_coherent_io(self) -> bool:
        return self.is_fullsystem()

    @overrides(X86Board)
    def get_mem_side_coherent_io_port(self) -> Port:
        if self.has_coherent_io():
            return self.iobus.mem_side_ports
        else:
            raise NotImplementedError(
                "x86DemoBoard does not have any I/O ports. Use has_coherent_io"
                " to check this."
            )
