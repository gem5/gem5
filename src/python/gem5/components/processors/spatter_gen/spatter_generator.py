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

from typing import (
    List,
    Optional,
    Union,
)

from m5.objects import (
    SpatterProcessingMode,
    SrcClockDomain,
    VoltageDomain,
)
from m5.stats import dump as dump_stats
from m5.stats import reset as reset_stats
from m5.util import fatal

from ....utils.override import overrides
from ..abstract_generator import AbstractGenerator
from .spatter_generator_core import SpatterGeneratorCore
from .spatter_kernel import SpatterKernel


class SpatterGenerator(AbstractGenerator):
    def __init__(
        self,
        num_cores: int = 1,
        processing_mode: Union[SpatterProcessingMode, str] = "synchronous",
        int_regfile_size: int = 384,
        fp_regfile_size: int = 224,
        request_gen_latency: int = 2,
        request_gen_rate: int = 4,
        request_buffer_entries: int = 32,
        send_rate: int = 2,
        clk_freq: Optional[str] = None,
    ) -> None:
        super().__init__(
            cores=self._create_cores(
                num_cores,
                processing_mode,
                int_regfile_size,
                fp_regfile_size,
                request_gen_latency,
                request_gen_rate,
                request_buffer_entries,
                send_rate,
            )
        )
        # no need for else block since it will intialize generator.clk_domain
        # the clock domain of its closest ancestor in the SimObject tree.
        if not clk_freq is None:
            clock_domain = SrcClockDomain(
                clock=clk_freq, voltage_domain=VoltageDomain()
            )
            for generator in self.cores:
                generator.clk_domain = clock_domain

        self._num_kernels = 0
        self._sync = processing_mode == "synchronous"

    def _create_cores(
        self,
        num_cores: int,
        processing_mode: Union[SpatterProcessingMode, str],
        int_regfile_size: int,
        fp_regfile_size: int,
        request_gen_latency: int,
        request_gen_rate: int,
        request_buffer_entries: int,
        send_rate: int,
    ) -> List[SpatterGeneratorCore]:
        return [
            SpatterGeneratorCore(
                processing_mode,
                int_regfile_size,
                fp_regfile_size,
                request_gen_latency,
                request_gen_rate,
                request_buffer_entries,
                send_rate,
            )
            for _ in range(num_cores)
        ]

    def add_kernel(self, kernels: List[SpatterKernel]) -> None:
        assert len(kernels) == len(self.cores)
        for core, kernel in zip(self.cores, kernels):
            if kernel.empty():
                fatal(
                    f"Cannot add {kernel} since it's empty. "
                    "At the moment SpatterGenerator (or gem5::SpatterGen) "
                    "does not support adding empty kernels to cores. As a "
                    "temporary fix you can try adding 1 dummy element to the "
                    "trace. You can also set fix_empty_trace to True in the "
                    "constructor of the SpatterKernel which automatically "
                    "inserts a dummy element (0) to the trace."
                )
            core.add_kernel(kernel)
        self._num_kernels += 1

    @overrides(AbstractGenerator)
    def start_traffic(self) -> None:
        for core in self.cores:
            core.start_traffic()

    def _proceed_past_sync_point(self) -> None:
        if not self._sync:
            return
        for core in self.cores:
            core.generator.proceedPastSyncPoint()

    def handle_spatter_exit(self):
        spatter_exits_observed = 0
        sync_points_observed = 0
        sync_points_expected = self._num_kernels if self._sync else 1
        while True:
            spatter_exits_observed += 1
            if spatter_exits_observed % len(self.cores) == 0:
                sync_points_observed += 1
                dump_stats()
                reset_stats()
                self._proceed_past_sync_point()
            yield not (sync_points_observed < sync_points_expected)
