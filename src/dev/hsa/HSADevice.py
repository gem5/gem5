# Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
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

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5.objects.Device import DmaVirtDevice
from m5.objects.VegaGPUTLB import VegaPagetableWalker


class HSAPacketProcessor(DmaVirtDevice):
    type = "HSAPacketProcessor"
    cxx_header = "dev/hsa/hsa_packet_processor.hh"
    cxx_class = "gem5::HSAPacketProcessor"

    pioAddr = Param.Addr("doorbell physical address")
    numHWQueues = Param.Int("Number of HW queues")
    # See:
    # Sooraj Puthoor et al., Oversubscribed Command Queues in GPUs. In the
    # proceedings of the 11th Workshop on General Purpose GPUs (GPGPU). 2018.
    wakeupDelay = Param.Tick(100000000, "Scheduling quantum")
    # This value was obtained empirically on Kaveri hardware via the KPS
    # benchmark from ATMI.
    # See: https://github.com/RadeonOpenCompute/atmi/tree/master/examples/
    #      runtime/kps
    pktProcessDelay = Param.Tick(4400000, "Packet processing delay")
    walker = Param.VegaPagetableWalker(
        VegaPagetableWalker(), "Page table walker"
    )
