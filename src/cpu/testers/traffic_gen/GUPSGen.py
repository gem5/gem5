# Copyright (c) 2021 The Regents of the University of California.
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
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class GUPSGen(ClockedObject):
    """
    This ClockedObject implements the RandomAccess benchmark specified by HPCC
    benchmarks in https://icl.utk.edu/projectsfiles/hpcc/RandomAccess.
    """

    type = "GUPSGen"
    cxx_header = "cpu/testers/traffic_gen/gups_gen.hh"
    cxx_class = "gem5::GUPSGen"

    system = Param.System(Parent.any, "System this generator is a part of")

    port = RequestPort("Port that should be connected to other components")

    start_addr = Param.Addr(
        0,
        "Start address for allocating update table,"
        " should be a multiple of block_size",
    )

    mem_size = Param.MemorySize(
        "Size for allocating update table, based on"
        " randomAccess benchmark specification, this"
        " should be equal to half of total system memory"
        " ,also should be a power of 2",
    )

    update_limit = Param.Int(
        0,
        "The number of updates to issue before the simulation is over",
    )

    request_queue_size = Param.Int(
        1024,
        "Maximum number of parallel outstanding requests",
    )

    init_memory = Param.Bool(
        False,
        "Whether or not to initialize the memory,"
        " it does not effect the performance",
    )
