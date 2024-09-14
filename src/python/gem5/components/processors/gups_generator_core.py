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


from typing import Optional

from m5.objects.ClockDomain import SrcClockDomain
from m5.objects.GUPSGen import GUPSGen
from m5.objects.VoltageDomain import VoltageDomain
from m5.params import (
    Addr,
    Port,
)

from ...utils.override import overrides
from .abstract_core import AbstractCore
from .abstract_generator_core import AbstractGeneratorCore


class GUPSGeneratorCore(AbstractGeneratorCore):
    def __init__(
        self,
        start_addr: Addr,
        mem_size: str,
        update_limit: int,
        clk_freq: Optional[str],
    ):
        """
        Create a GUPSGeneratorCore as the main generator.
        """
        super().__init__()
        self.generator = GUPSGen(
            start_addr=start_addr, mem_size=mem_size, update_limit=update_limit
        )
        if clk_freq:
            clock_domain = SrcClockDomain(
                clock=clk_freq, voltage_domain=VoltageDomain()
            )
            self.generator.clk_domain = clock_domain

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.generator.port = port
