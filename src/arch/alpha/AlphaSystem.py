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
from m5.proxy import *
from System import System

class AlphaSystem(System):
    type = 'AlphaSystem'
    cxx_header = "arch/alpha/system.hh"
    console = Param.String("file that contains the console code")
    pal = Param.String("file that contains palcode")
    system_type = Param.UInt64("Type of system we are emulating")
    system_rev = Param.UInt64("Revision of system we are emulating")
    load_addr_mask = 0xffffffffff

class LinuxAlphaSystem(AlphaSystem):
    type = 'LinuxAlphaSystem'
    cxx_header = "arch/alpha/linux/system.hh"
    system_type = 34
    system_rev = 1 << 10

    boot_cpu_frequency = Param.Frequency(Self.cpu[0].clk_domain.clock[0]
                                         .frequency,
                                         "boot processor frequency")

class FreebsdAlphaSystem(AlphaSystem):
    type = 'FreebsdAlphaSystem'
    cxx_header = "arch/alpha/freebsd/system.hh"
    system_type = 34
    system_rev = 1 << 10
