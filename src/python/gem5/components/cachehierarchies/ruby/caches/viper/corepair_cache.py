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

import math

from m5.objects import (
    GPU_VIPER_CorePair_Controller,
    MessageBuffer,
    RubyCache,
    TreePLRURP,
)

from gem5.components.processors.abstract_core import AbstractCore


class CorePairCache(GPU_VIPER_CorePair_Controller):
    def __init__(
        self,
        l1i_size: str,
        l1i_assoc: int,
        l1d_size: str,
        l1d_assoc: int,
        l2_size: str,
        l2_assoc: int,
        network,
        cache_line_size,
        core: AbstractCore,
    ):
        """Creating CorePair cache controller. Consist of both instruction
        and data cache for a pair of L1s and a single L2 cache shared between
        them.
        """
        super().__init__()

        self.send_evictions = core.requires_send_evicts()

        self.L1Icache = RubyCache(
            size=l1i_size,
            assoc=l1i_assoc,
            replacement_policy=TreePLRURP(),
            resourceStalls=False,
            dataArrayBanks=2,
            tagArrayBanks=2,
            dataAccessLatency=1,
            tagAccessLatency=1,
        )

        self.L1D0cache = RubyCache(
            size=l1d_size,
            assoc=l1d_assoc,
            replacement_policy=TreePLRURP(),
            resourceStalls=False,
            dataArrayBanks=2,
            tagArrayBanks=2,
            dataAccessLatency=1,
            tagAccessLatency=1,
        )

        self.L1D1cache = RubyCache(
            size=l1d_size,
            assoc=l1d_assoc,
            replacement_policy=TreePLRURP(),
            resourceStalls=False,
            dataArrayBanks=2,
            tagArrayBanks=2,
            dataAccessLatency=1,
            tagAccessLatency=1,
        )

        self.L2cache = RubyCache(
            size=l2_size,
            assoc=l2_assoc,
            replacement_policy=TreePLRURP(),
            resourceStalls=False,
            dataArrayBanks=16,
            tagArrayBanks=16,
        )

        self.connectQueues(network)

    def connectQueues(self, network):
        self.requestFromCore = MessageBuffer()
        self.requestFromCore.out_port = network.in_port

        self.responseFromCore = MessageBuffer()
        self.responseFromCore.out_port = network.in_port

        self.unblockFromCore = MessageBuffer()
        self.unblockFromCore.out_port = network.in_port

        self.probeToCore = MessageBuffer()
        self.probeToCore.in_port = network.out_port

        self.responseToCore = MessageBuffer()
        self.responseToCore.in_port = network.out_port

        self.mandatoryQueue = MessageBuffer()
        self.triggerQueue = MessageBuffer(ordered=True)
