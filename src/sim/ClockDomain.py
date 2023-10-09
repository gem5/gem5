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

# Abstract clock domain
class ClockDomain(SimObject):
    type = "ClockDomain"
    cxx_header = "sim/clock_domain.hh"
    cxx_class = "gem5::ClockDomain"
    abstract = True


# Source clock domain with an actual clock, and a list of voltage and frequency
# op points
class SrcClockDomain(ClockDomain):
    type = "SrcClockDomain"
    cxx_header = "sim/clock_domain.hh"
    cxx_class = "gem5::SrcClockDomain"

    # Single clock frequency value, or list of frequencies for DVFS
    # Frequencies must be ordered in descending order
    # Note: Matching voltages should be defined in the voltage domain
    clock = VectorParam.Clock("Clock period")

    # A source clock must be associated with a voltage domain
    voltage_domain = Param.VoltageDomain("Voltage domain")

    # Domain ID is an identifier for the DVFS domain as understood by the
    # necessary control logic (either software or hardware). For example, in
    # case of software control via cpufreq framework the IDs should correspond
    # to the neccessary identifier in the device tree blob which is interpretted
    # by the device driver to communicate to the domain controller in hardware.
    domain_id = Param.Int32(-1, "domain id")

    # Initial performance level from the list of available operation points
    # Defaults to maximum performance
    init_perf_level = Param.UInt32(0, "Initial performance level")


# Derived clock domain with a parent clock domain and a frequency
# divider
class DerivedClockDomain(ClockDomain):
    type = "DerivedClockDomain"
    cxx_header = "sim/clock_domain.hh"
    cxx_class = "gem5::DerivedClockDomain"

    clk_domain = Param.ClockDomain("Parent clock domain")
    clk_divider = Param.Unsigned(1, "Frequency divider")
