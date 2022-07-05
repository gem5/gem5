# Copyright (c) 2005-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
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
from m5.params import *
from m5.proxy import *

from m5.objects.ClockedObject import ClockedObject


class RubyTester(ClockedObject):
    type = "RubyTester"
    cxx_header = "cpu/testers/rubytest/RubyTester.hh"
    cxx_class = "gem5::RubyTester"

    num_cpus = Param.Int("number of cpus / RubyPorts")
    cpuInstDataPort = VectorRequestPort(
        "cpu combo ports to inst & " "data caches"
    )
    cpuInstPort = VectorRequestPort("cpu ports to only inst caches")
    cpuDataPort = VectorRequestPort("cpu ports to only data caches")
    checks_to_complete = Param.Int(100, "checks to complete")
    deadlock_threshold = Param.Int(50000, "how often to check for deadlock")
    wakeup_frequency = Param.Int(10, "number of cycles between wakeups")
    check_flush = Param.Bool(False, "check cache flushing")
    system = Param.System(Parent.any, "System we belong to")
