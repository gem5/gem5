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
    TCC_Controller,
    TreePLRURP,
)


class TCCCache(TCC_Controller):
    def __init__(
        self,
        tcc_size: str,
        tcc_assoc: int,
        network,
        cache_line_size,
    ):
        """Creating TCC cache controller. This is the L2 cache for GPU devices."""

        super().__init__()

        self.L2cache = RubyCache(
            size=tcc_size,
            assoc=tcc_assoc,
            dataArrayBanks=256,
            tagArrayBanks=256,
            dataAccessLatency=8,
            tagAccessLatency=2,
            resourceStalls=True,
            replacement_policy=TreePLRURP(),
            atomicLatency=0,
            atomicALUs=64,
        )

        self.connectQueues(network)

    def connectQueues(self, network):
        self.requestFromTCP = MessageBuffer(ordered=True)
        self.requestFromTCP.in_port = network.out_port

        self.responseToCore = MessageBuffer(ordered=True)
        self.responseToCore.out_port = network.in_port

        self.probeFromNB = MessageBuffer()
        self.probeFromNB.in_port = network.out_port

        self.responseFromNB = MessageBuffer()
        self.responseFromNB.in_port = network.out_port

        self.requestToNB = MessageBuffer(ordered=True)
        self.requestToNB.out_port = network.in_port

        self.responseToNB = MessageBuffer()
        self.responseToNB.out_port = network.in_port

        self.unblockToNB = MessageBuffer()
        self.unblockToNB.out_port = network.in_port

        self.triggerQueue = MessageBuffer(ordered=True)
