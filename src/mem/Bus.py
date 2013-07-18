# Copyright (c) 2012 ARM Limited
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
# Copyright (c) 2005-2008 The Regents of The University of Michigan
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
#          Andreas Hansson

from MemObject import MemObject
from System import System
from m5.params import *
from m5.proxy import *

class BaseBus(MemObject):
    type = 'BaseBus'
    abstract = True
    cxx_header = "mem/bus.hh"
    slave = VectorSlavePort("vector port for connecting masters")
    master = VectorMasterPort("vector port for connecting slaves")
    header_cycles = Param.Cycles(1, "cycles of overhead per transaction")
    width = Param.Unsigned(8, "bus width (bytes)")

    # The default port can be left unconnected, or be used to connect
    # a default slave port
    default = MasterPort("Port for connecting an optional default slave")

    # The default port can be used unconditionally, or based on
    # address range, in which case it may overlap with other
    # ports. The default range is always checked first, thus creating
    # a two-level hierarchical lookup. This is useful e.g. for the PCI
    # bus configuration.
    use_default_range = Param.Bool(False, "Perform address mapping for " \
                                       "the default port")

class NoncoherentBus(BaseBus):
    type = 'NoncoherentBus'
    cxx_header = "mem/noncoherent_bus.hh"

class CoherentBus(BaseBus):
    type = 'CoherentBus'
    cxx_header = "mem/coherent_bus.hh"

    system = Param.System(Parent.any, "System that the bus belongs to.")
