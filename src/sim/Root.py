# Copyright (c) 2005-2007 The Regents of The University of Michigan
# Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
# Copyright (c) 2013 Mark D. Hill and David A. Wood
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
from m5.SimObject import SimObject
from m5.util import fatal


class Root(SimObject):
    _the_instance = None

    def __new__(cls, **kwargs):
        if Root._the_instance:
            fatal("Attempt to allocate multiple instances of Root.")
            return None

        # first call: allocate the unique instance
        #
        # If SimObject ever implements __new__, we may want to pass
        # kwargs here, but for now this goes straight to
        # object.__new__ which prints an ugly warning if you pass it
        # args.  Seems like a bad design but that's the way it is.
        Root._the_instance = SimObject.__new__(cls)
        return Root._the_instance

    @classmethod
    def getInstance(cls):
        return Root._the_instance

    def path(self):
        return "root"

    type = "Root"
    cxx_header = "sim/root.hh"
    cxx_class = "gem5::Root"

    # By default, root sim object and hence all other sim objects schedule
    # event on the eventq with index 0.
    eventq_index = 0

    # Simulation Quantum for multiple main event queue simulation.
    # Needs to be set explicitly for a multi-eventq simulation.
    sim_quantum = Param.Tick(0, "simulation quantum")

    full_system = Param.Bool("if this is a full system simulation")

    # Time syncing prevents the simulation from running faster than real time.
    time_sync_enable = Param.Bool(False, "whether time syncing is enabled")
    time_sync_period = Param.Clock("100ms", "how often to sync with real time")
    time_sync_spin_threshold = Param.Clock(
        "100us", "when less than this much time is left, spin"
    )
