# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert

from m5.params import *
from m5.proxy import *
from MemObject import *

class PhysicalMemory(MemObject):
    type = 'PhysicalMemory'
    port = VectorPort("the access port")
    range = Param.AddrRange(AddrRange('128MB'), "Device Address")
    file = Param.String('', "memory mapped file")
    latency = Param.Latency('1t', "latency of an access")
    zero = Param.Bool(False, "zero initialize memory")

class DRAMMemory(PhysicalMemory):
    type = 'DRAMMemory'
    # Many of these should be observed from the configuration
    cpu_ratio = Param.Int(5,"ratio between CPU speed and memory bus speed")
    mem_type = Param.String("SDRAM", "Type of DRAM (DRDRAM, SDRAM)")
    mem_actpolicy = Param.String("open", "Open/Close policy")
    memctrladdr_type = Param.String("interleaved", "Mapping interleaved or direct")
    bus_width = Param.Int(16, "")
    act_lat = Param.Int(2, "RAS to CAS delay")
    cas_lat = Param.Int(1, "CAS delay")
    war_lat = Param.Int(2, "write after read delay")
    pre_lat = Param.Int(2, "precharge delay")
    dpl_lat = Param.Int(2, "data in to precharge delay")
    trc_lat = Param.Int(6, "row cycle delay")
    num_banks = Param.Int(4, "Number of Banks")
    num_cpus = Param.Int(4, "Number of CPUs connected to DRAM")

