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
    GPU_VIPER_Directory_Controller,
    MessageBuffer,
    RubyDirectoryMemory,
)


class ViperDirectory(GPU_VIPER_Directory_Controller):
    def __init__(self, network, cache_line_size, mem_range, port):
        super().__init__()
        self._cache_line_size = cache_line_size

        self.addr_ranges = [mem_range]
        self.directory = RubyDirectoryMemory(
            block_size=cache_line_size,
            ruby_system=network.ruby_system,
        )
        # Connect this directory to the memory side.
        self.memory_out_port = port

        # Turn off TCC (GPU cache) related parameters
        self.noTCCdir = True
        self.TCC_select_num_bits = 0

        # Defaults which must be set
        self.CPUonly = False
        self.GPUonly = False
        self.useL3OnWT = False
        self.L2isWB = False

        self.connectQueues(network)

    def connectQueues(self, network):
        self.requestFromDMA = MessageBuffer(ordered=True)
        self.requestFromDMA.in_port = network.out_port

        self.responseToDMA = MessageBuffer()
        self.responseToDMA.out_port = network.in_port

        self.requestFromCores = MessageBuffer(ordered=True)
        self.requestFromCores.in_port = network.out_port

        self.responseFromCores = MessageBuffer()
        self.responseFromCores.in_port = network.out_port

        self.unblockFromCores = MessageBuffer()
        self.unblockFromCores.in_port = network.out_port

        self.probeToCore = MessageBuffer()
        self.probeToCore.out_port = network.in_port

        self.responseToCore = MessageBuffer()
        self.responseToCore.out_port = network.in_port

        self.triggerQueue = MessageBuffer(ordered=True)
        self.L3triggerQueue = MessageBuffer(ordered=True)

        self.requestToMemory = MessageBuffer()
        self.responseFromMemory = MessageBuffer()


# This is intended to be used on the CPU side
class ViperCPUDirectory(ViperDirectory):
    def __init__(self, network, cache_line_size, mem_range, port):
        super().__init__(network, cache_line_size, mem_range, port)

        self.CPUonly = True
        self.GPUonly = False


# This is intended to be used on the GPU side
class ViperGPUDirectory(ViperDirectory):
    def __init__(self, network, cache_line_size, mem_range, port):
        super().__init__(network, cache_line_size, mem_range, port)

        self.CPUonly = False
        self.GPUonly = True
