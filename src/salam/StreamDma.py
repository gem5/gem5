# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

from m5.objects.Device import DmaDevice
from m5.params import *
from m5.proxy import *


class StreamDma(DmaDevice):
    type = "StreamDma"
    cxx_header = "salam/stream_dma.hh"
    devicename = Param.String("stream_dma", "Name of DMA device")
    pio_addr = Param.Addr("Device Address")
    pio_delay = Param.Latency("100ns", "PIO Latency")
    pio_size = Param.Addr(32, "MMR Size")
    stream_in = ResponsePort("Stream buffer access port for S2MM")
    stream_out = ResponsePort("Stream buffer access port for MM2S")
    stream_addr = Param.Addr("Stream interface address")
    stream_size = Param.Addr(8, "Stream width")
    status_in = ResponsePort("Stream buffer status port")
    status_out = ResponsePort("Stream buffer status port")
    status_addr = Param.Addr("Address for accessing buffer status")
    status_size = Param.Addr(4, "Size of the buffer status register")
    mem_delay = Param.Latency("1ns", "Buffer read time")
    read_buffer_size = Param.UInt64(1024, "Read buffer size")
    write_buffer_size = Param.UInt64(1024, "Write buffer size")
    max_pending = Param.Unsigned(8, "Maximum number of pending DMA reads")
    max_req_size = Param.Unsigned(
        Parent.cache_line_size, "Maximum size of a DMA request"
    )
    gic = Param.BaseGic(Parent.any, "Gic on which to trigger interrupts")
    rd_int = Param.UInt32(210, "Interrupt for read buffer")
    wr_int = Param.UInt32(211, "Interrupt for write buffer")

    bandwidth = Param.MemoryBandwidth(
        "12.6GiB/s", "Combined read and write bandwidth"
    )
