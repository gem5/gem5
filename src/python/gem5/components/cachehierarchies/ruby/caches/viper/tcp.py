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
    TCP_Controller,
    TreePLRURP,
)


class TCPCache(TCP_Controller):
    def __init__(
        self,
        tcp_size: str,
        tcp_assoc: int,
        network,
        cache_line_size,
    ):
        """Creating TCP cache controller. This is the L1 cache for GPU devices."""

        super().__init__()

        self.L1cache = RubyCache(
            size=tcp_size,
            assoc=tcp_assoc,
            dataArrayBanks=16,
            tagArrayBanks=16,
            dataAccessLatency=4,
            tagAccessLatency=1,
            resourceStalls=True,
            replacement_policy=TreePLRURP(),
        )

        self.connectQueues(network)

    def connectQueues(self, network):
        self.requestFromTCP = MessageBuffer(ordered=True)
        self.requestFromTCP.out_port = network.in_port

        self.responseFromTCP = MessageBuffer(ordered=True)
        self.responseFromTCP.out_port = network.in_port

        self.unblockFromCore = MessageBuffer()
        self.unblockFromCore.out_port = network.in_port

        self.probeToTCP = MessageBuffer(ordered=True)
        self.probeToTCP.in_port = network.out_port

        self.responseToTCP = MessageBuffer(ordered=True)
        self.responseToTCP.in_port = network.out_port

        self.mandatoryQueue = MessageBuffer()
