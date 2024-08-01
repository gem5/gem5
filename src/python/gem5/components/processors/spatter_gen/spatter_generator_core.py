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
    Union,
)

from m5.objects.SpatterGen import (
    SpatterGen,
    SpatterProcessingMode,
)
from m5.params import Port

from ....utils.override import overrides
from ..abstract_core import AbstractCore
from ..abstract_generator_core import AbstractGeneratorCore
from .spatter_kernel import SpatterKernel


class SpatterGeneratorCore(AbstractGeneratorCore):
    def __init__(
        self,
        processing_mode: Union[SpatterProcessingMode, str],
        int_regfile_size: int,
        fp_regfile_size: int,
        request_gen_latency: int,
        request_gen_rate: int,
        request_buffer_entries: int,
        send_rate: int,
    ):
        super().__init__()
        self.generator = SpatterGen(
            processing_mode=processing_mode,
            int_regfile_size=int_regfile_size,
            fp_regfile_size=fp_regfile_size,
            request_gen_latency=request_gen_latency,
            request_gen_rate=request_gen_rate,
            request_buffer_entries=request_buffer_entries,
            send_rate=send_rate,
        )
        # TODO: Type this properly
        self._kernels: List = []

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.generator.port = port

    def add_kernel(self, kernel: SpatterKernel) -> None:
        self._kernels.append(kernel)

    def start_traffic(self) -> None:
        for kernel in self._kernels:
            self.generator.addKernel(*kernel.cxx_call_args())
