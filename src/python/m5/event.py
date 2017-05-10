# Copyright (c) 2017 ARM Limited
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
# Copyright (c) 2006 The Regents of The University of Michigan
# Copyright (c) 2013 Advanced Micro Devices, Inc.
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
#
# Authors: Nathan Binkert

import m5
import _m5.event

from _m5.event import GlobalSimLoopExitEvent as SimExit
from _m5.event import PyEvent as Event
from _m5.event import getEventQueue, setEventQueue

mainq = None

class EventWrapper(Event):
    """Helper class to wrap callable objects in an Event base class"""

    def __init__(self, func, **kwargs):
        super(EventWrapper, self).__init__(**kwargs)

        if not callable(func):
            raise RuntimeError("Can't wrap '%s', object is not callable" % \
                               str(func))

        self._func = func

    def __call__(self):
        self._func()

    def __str__(self):
        return "EventWrapper(%s)" % (str(self._func), )


class ProgressEvent(Event):
    def __init__(self, eventq, period):
        super(ProgressEvent, self).__init__()
        self.period = int(period)
        self.eventq = eventq
        self.eventq.schedule(self, m5.curTick() + self.period)

    def __call__(self):
        print "Progress! Time now %fs" % (m5.curTick()/1e12)
        self.eventq.schedule(self, m5.curTick() + self.period)


def create(func, priority=Event.Default_Pri):
    """Create an Event from a function"""

    return EventWrapper(func, priority=priority)

__all__ = [ 'Event', 'EventWrapper', 'ProgressEvent', 'SimExit',
            'mainq', 'create' ]
