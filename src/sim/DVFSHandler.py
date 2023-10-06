# Copyright (c) 2013-2014 ARM Limited
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
from m5.proxy import *
from m5.SimObject import SimObject


# The handler in its current form is design to be centeralized, one per system
# and manages all the source clock domains (SrcClockDomain) it is configured to
# handle.  The specific voltage and frequency points are configured per clock
# and voltage domain.
class DVFSHandler(SimObject):
    type = "DVFSHandler"
    cxx_header = "sim/dvfs_handler.hh"
    cxx_class = "gem5::DVFSHandler"

    # List of controllable clock domains which in turn reference the appropriate
    # voltage domains
    domains = VectorParam.SrcClockDomain([], "list of domains")

    # System domain (its clock and voltage) is not controllable
    sys_clk_domain = Param.SrcClockDomain(
        Parent.clk_domain, "Clk domain in which the handler is instantiated"
    )

    enable = Param.Bool(False, "Enable/Disable the handler")

    # The transition latency depends on how much time the PLLs and voltage
    # regualators takes to migrate from current levels to the new level, is
    # usally variable and hardware implementation dependent. In order to
    # accomodate this effect with ease, we provide a fixed transition latency
    # associated with all migrations. Configure this to maximum latency that
    # the hardware will take to migratate between any two perforamnce levels.
    transition_latency = Param.Latency(
        "100us", "fixed latency for perf level migration"
    )
