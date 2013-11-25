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
import internal.event

from internal.event import PythonEvent, GlobalSimLoopExitEvent as SimExit

mainq = None

def create(obj, priority=None):
    if priority is None:
        priority = Event.Default_Pri
    return PythonEvent(obj, priority)


# As a reminder, priorities found in sim/eventq.hh are stuck into the
# Event class by swig
class Event(PythonEvent):
    def __init__(self, priority=None):
        if priority is None:
            priority = Event.Default_Pri
        super(Event, self).__init__(self, priority)

class ProgressEvent(Event):
    def __init__(self, eventq, period):
        super(ProgressEvent, self).__init__()
        self.period = int(period)
        self.eventq = eventq
        self.eventq.schedule(self, m5.curTick() + self.period)

    def __call__(self):
        print "Progress! Time now %fs" % (m5.curTick()/1e12)
        self.eventq.schedule(self, m5.curTick() + self.period)

def getEventQueue(index):
    return internal.event.getEventQueue(index)

def setEventQueue(eventq):
    internal.event.curEventQueue(eventq)

__all__ = [ 'create', 'Event', 'ProgressEvent', 'SimExit', 'mainq' ]
