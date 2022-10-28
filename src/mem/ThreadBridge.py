# Copyright 2022 Google, LLC
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

from m5.SimObject import SimObject
from m5.params import *


class ThreadBridge(SimObject):
    """Bridge for SimObjects from different threads (EventQueues)

    When two SimObjects running on two separate threads (EventQueues), an
    access from one side to the other side could easily cause event scheduled
    on the wrong event queue.

    ThreadBridge is used to migrate the EventQueue to the one used by
    ThreadBridge itself before sending transation to the other side to avoid
    the issue. The receiver side is expected to use the same EventQueue that
    the ThreadBridge is using.

    Given that this is only used for simulation speed accelerating, only the
    atomic and functional access are supported.

    Example:

    sys.initator = Initiator(eventq_index=0)
    sys.target = Target(eventq_index=1)
    sys.bridge = ThreadBridge(eventq_index=1)

    sys.initator.out_port = sys.bridge.in_port
    sys.bridge.out_port = sys.target.in_port
    """

    type = "ThreadBridge"
    cxx_header = "mem/thread_bridge.hh"
    cxx_class = "gem5::ThreadBridge"

    in_port = ResponsePort("Incoming port")
    out_port = RequestPort("Outgoing port")
