# Copyright (c) 2024 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.objects import (
    MessageBuffer,
    RubyCache,
    SQC_Controller,
    TreePLRURP,
)


class SQCCache(SQC_Controller):
    def __init__(
        self,
        sqc_size: str,
        sqc_assoc: int,
        network,
        cache_line_size,
    ):
        """Creating SQC cache controller. This is the Icache for GPU devices."""

        super().__init__()

        self.L1cache = RubyCache(
            size=sqc_size,
            assoc=sqc_assoc,
            dataArrayBanks=8,
            tagArrayBanks=8,
            dataAccessLatency=1,
            tagAccessLatency=1,
            resourceStalls=True,
            replacement_policy=TreePLRURP(),
        )

        self.connectQueues(network)

    def connectQueues(self, network):
        self.requestFromSQC = MessageBuffer(ordered=True)
        self.requestFromSQC.out_port = network.in_port

        self.probeToSQC = MessageBuffer(ordered=True)
        self.probeToSQC.in_port = network.out_port

        self.responseToSQC = MessageBuffer(ordered=True)
        self.responseToSQC.in_port = network.out_port

        self.mandatoryQueue = MessageBuffer()
