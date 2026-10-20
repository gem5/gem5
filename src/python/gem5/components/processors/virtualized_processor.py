# Copyright (c) 2026 The Regents of The University of California
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

"""A low-configuration, host-resolved virtualized processor."""

from ...isas import ISA
from .cpu_types import CPUTypes
from .simple_processor import SimpleProcessor
from .simple_switchable_processor import SimpleSwitchableProcessor
from .virtualization import resolve_virtualized_cpu_type


class VirtualizedProcessor(SimpleProcessor):
    """A processor that selects KVM or AppleVirt for the current host."""

    def __init__(self, num_cores: int, isa: ISA, *, clk_freq: str) -> None:
        cpu_type = resolve_virtualized_cpu_type(isa=isa, num_cores=num_cores)
        super().__init__(
            cpu_type=cpu_type,
            num_cores=num_cores,
            isa=isa,
            clk_freq=clk_freq,
        )


class VirtualizedSwitchableProcessor(SimpleSwitchableProcessor):
    """A host-resolved virtualized CPU switched to a simulated CPU."""

    def __init__(
        self,
        switch_core_type: CPUTypes,
        num_cores: int,
        isa: ISA,
        *,
        clk_freq: str,
    ) -> None:
        if switch_core_type in (CPUTypes.KVM, CPUTypes.APPLE_VIRT):
            raise ValueError(
                "The switch core type must be a simulated CPU type"
            )
        starting_core_type = resolve_virtualized_cpu_type(
            isa=isa, num_cores=num_cores
        )
        super().__init__(
            starting_core_type=starting_core_type,
            switch_core_type=switch_core_type,
            num_cores=num_cores,
            isa=isa,
            clk_freq=clk_freq,
        )
