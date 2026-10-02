# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

from m5.objects.Device import BasicPioDevice
from m5.params import *
from m5.proxy import *


class CommInterface(BasicPioDevice):
    type = "CommInterface"
    cxx_header = "salam/comm_interface.hh"

    flags_size = Param.Addr(
        0x1, "Size of the address range dedicated to device flags"
    )
    config_size = Param.Addr(
        0x0, "Size of the addess range dedicated to device configuration"
    )
    pio_size = Param.Addr(
        0x8,
        "Size of MMRs. Should be large enough to support flags, config, "
        "and global var addresses",
    )
    devicename = Param.String(
        "comm_interface", "Name of comm_interface device"
    )
    local = VectorRequestPort(
        "Master points connected to the local cluster xbar"
    )
    acp = VectorRequestPort(
        "Master ports connected to the cluster coherency xbar"
    )
    stream = VectorRequestPort("Master ports connected to streaming devices")
    spm = VectorRequestPort(
        "Master ports connected to private scratchpad memory"
    )
    reg = VectorRequestPort("Master ports connected to private register banks")
    system = Param.System(Parent.any, "Parent system of the device")
    cache_line_size = Param.Unsigned(
        Parent.cache_line_size, "Cache line size in bytes"
    )
    gic = Param.BaseGic(Parent.any, "Gic on which to trigger interrupts")
    int_num = Param.Int32(-1, "Interrupt number that connects to GIC")
    clock_period = Param.Int(10, "Clock period in ns")
    premap_data = Param.Bool(
        False,
        "Whether or not the memory read/write locations for data predefined",
    )
    data_bases = VectorParam.Addr(
        [0x0], "Base addresses for data if they are predefined"
    )
    enable_debug_msgs = Param.Bool(
        False, "Whether or not this device will display debug messages"
    )
    reset_spm = Param.Bool(
        False,
        "Reset the ready state of any connected scratchpad memories when"
        " finished executing",
    )
