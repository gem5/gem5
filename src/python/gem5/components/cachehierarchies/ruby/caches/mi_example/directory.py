# Copyright (c) 2021 The Regents of the University of California
# All Rights Reserved.
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


from m5.objects.DirectoryMemory import RubyDirectoryMemory
from m5.objects.MessageBuffer import MessageBuffer

from ......utils.override import overrides
from ..abstract_directory import AbstractDirectory


class Directory(AbstractDirectory):
    """
    The directory controller for the MI_Example cache hierarchy.
    """

    def __init__(self, network, cache_line_size, mem_range, port):
        super().__init__(network, cache_line_size)
        self.addr_ranges = [mem_range]
        self.directory = RubyDirectoryMemory()
        # Connect this directory to the memory side.
        self.memory_out_port = port

    @overrides(AbstractDirectory)
    def connectQueues(self, network):
        self.requestToDir = MessageBuffer(ordered=True)
        self.requestToDir.in_port = network.out_port
        self.dmaRequestToDir = MessageBuffer(ordered=True)
        self.dmaRequestToDir.in_port = network.out_port

        self.responseFromDir = MessageBuffer()
        self.responseFromDir.out_port = network.in_port
        self.dmaResponseFromDir = MessageBuffer(ordered=True)
        self.dmaResponseFromDir.out_port = network.in_port
        self.forwardFromDir = MessageBuffer()
        self.forwardFromDir.out_port = network.in_port
        self.requestToMemory = MessageBuffer()
        self.responseFromMemory = MessageBuffer()
