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


from m5.objects import MessageBuffer

from ......utils.override import overrides
from ..abstract_dma_controller import AbstractDMAController


# There is a controller for GPU and GPU to keep the "version" numbers
# incrementing seperately
class ViperCPUDMAController(AbstractDMAController):
    def __init__(self, network, cache_line_size):
        super().__init__(network, cache_line_size)

    @overrides(AbstractDMAController)
    def connectQueues(self, network):
        # A buffer size of 0 means it is an infinite queue. The VIPER
        # DMA controller has not been thoroughly tested with finite buffers.
        # Test
        self.mandatoryQueue = MessageBuffer(buffer_size=0)
        self.responseFromDir = MessageBuffer(buffer_size=0)
        self.responseFromDir.in_port = network.out_port
        self.requestToDir = MessageBuffer(buffer_size=0)
        self.requestToDir.out_port = network.in_port


class ViperGPUDMAController(AbstractDMAController):
    def __init__(self, network, cache_line_size):
        super().__init__(network, cache_line_size)

    @overrides(AbstractDMAController)
    def connectQueues(self, network):
        # A buffer size of 0 means it is an infinite queue. The VIPER
        # DMA controller has not been thoroughly tested with finite buffers.
        # Test
        self.mandatoryQueue = MessageBuffer(buffer_size=0)
        self.responseFromDir = MessageBuffer(buffer_size=0)
        self.responseFromDir.in_port = network.out_port
        self.requestToDir = MessageBuffer(buffer_size=0)
        self.requestToDir.out_port = network.in_port
