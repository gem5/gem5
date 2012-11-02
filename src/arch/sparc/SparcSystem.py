# Copyright (c) 2007 The Regents of The University of Michigan
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

from SimpleMemory import SimpleMemory
from System import System

class SparcSystem(System):
    type = 'SparcSystem'
    cxx_header = 'arch/sparc/system.hh'
    _rom_base = 0xfff0000000
    _nvram_base = 0x1f11000000
    _hypervisor_desc_base = 0x1f12080000
    _partition_desc_base = 0x1f12000000
    # ROM for OBP/Reset/Hypervisor
    rom = Param.SimpleMemory(
        SimpleMemory(range=AddrRange(_rom_base, size='8MB')),
            "Memory to hold the ROM data")
    # nvram
    nvram = Param.SimpleMemory(
        SimpleMemory(range=AddrRange(_nvram_base, size='8kB')),
        "Memory to hold the nvram data")
    # hypervisor description
    hypervisor_desc = Param.SimpleMemory(
        SimpleMemory(range=AddrRange(_hypervisor_desc_base, size='8kB')),
        "Memory to hold the hypervisor description")
    # partition description
    partition_desc = Param.SimpleMemory(
        SimpleMemory(range=AddrRange(_partition_desc_base, size='8kB')),
        "Memory to hold the partition description")

    reset_addr = Param.Addr(_rom_base, "Address to load ROM at")
    hypervisor_addr = Param.Addr(Addr('64kB') + _rom_base,
                                 "Address to load hypervisor at")
    openboot_addr = Param.Addr(Addr('512kB') + _rom_base,
                               "Address to load openboot at")
    nvram_addr = Param.Addr(_nvram_base, "Address to put the nvram")
    hypervisor_desc_addr = Param.Addr(_hypervisor_desc_base,
            "Address for the hypervisor description")
    partition_desc_addr = Param.Addr(_partition_desc_base,
            "Address for the partition description")

    reset_bin = Param.String("file that contains the reset code")
    hypervisor_bin = Param.String("file that contains the hypervisor code")
    openboot_bin = Param.String("file that contains the openboot code")
    nvram_bin = Param.String("file that contains the contents of nvram")
    hypervisor_desc_bin = Param.String("file that contains the hypervisor description")
    partition_desc_bin = Param.String("file that contains the partition description")
    load_addr_mask = 0xffffffffff
