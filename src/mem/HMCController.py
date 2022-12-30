# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2015 The University of Bologna
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

from m5.params import *
from m5.objects.XBar import *

# References:
# [1] http://www.open-silicon.com/open-silicon-ips/hmc/
# [2] Ahn, J.; Yoo, S.; Choi, K., "Low-Power Hybrid Memory Cubes With Link
#   Power Management and Two-Level Prefetching," TVLSI 2015

# The HMCController class highlights the fact that a component is required
# between host and HMC to convert the host protocol (AXI for example) to the
# serial links protocol. Moreover, this component should have large internal
# queueing to hide the access latency of the HMC.
# Plus, this controller can implement more advanced global scheduling policies
# and can reorder and steer transactions if required. A good example of such
# component is available in [1].
# Also in [2] there is a similar component which is connected to all serial
# links, and it schedules the requests to the ones which are not busy.
# These two references clarify two things:
# 1. The serial links support the same address range and packets can travel
#  over any of them.
# 2. One host can be connected to more than 1 serial link simply to achieve
#  higher bandwidth, and not for any other reason.

# In this model, we have used a round-robin counter, because it is the
# simplest way to schedule packets over the non-busy serial links. However,
# more advanced scheduling algorithms are possible and even host can dedicate
# each serial link to a portion of the address space and interleave packets
# over them. Yet in this model, we have not made any such assumptions on the
# address space.


class HMCController(NoncoherentXBar):
    type = "HMCController"
    cxx_header = "mem/hmc_controller.hh"
    cxx_class = "gem5::HMCController"
