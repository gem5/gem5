# -*- mode:python -*-
# Copyright (c) 2009-2014 ARM Limited
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
#
# Authors: Matt Horsnell
#          Andreas Sandberg

from m5.defines import buildEnv
from m5.SimObject import SimObject
from m5.params import *
from m5.params import isNullPointer
from m5.proxy import *

class ArmPMU(SimObject):
    type = 'ArmPMU'
    cxx_class = 'ArmISA::PMU'
    cxx_header = 'arch/arm/pmu.hh'

    @classmethod
    def export_methods(cls, code):
        code('''
      void addEventProbe(unsigned int id,
                        SimObject *obj, const char *name);
''')

    # To prevent cycles in the configuration hierarchy, we don't keep
    # a list of supported events as a configuration param. Instead, we
    # keep them in a local list and register them using the
    # addEventProbe interface when other SimObjects register their
    # probe listeners.
    _deferred_event_types = []
    # Override the normal SimObject::regProbeListeners method and
    # register deferred event handlers.
    def regProbeListeners(self):
        for event_id, obj, name in self._deferred_event_types:
            self.getCCObject().addEventProbe(event_id, obj.getCCObject(), name)

        self.getCCObject().regProbeListeners()

    def addEventProbe(self, event_id, obj, *args):
        """Add a probe-based event to the PMU if obj is not None."""

        if obj is None:
            return

        for name in args:
            self._deferred_event_types.append((event_id, obj, name))

    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    eventCounters = Param.Int(31, "Number of supported PMU counters")
    pmuInterrupt = Param.Int(68, "PMU GIC interrupt number")
