# Copyright (c) 2026 ZhangJian
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

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class TimingThreadBridge(ClockedObject):
    """Timing bridge between SimObjects on different threads (EventQueues).

    Unlike ThreadBridge, which only supports atomic and functional traffic,
    TimingThreadBridge supports timing requests and responses across an
    EventQueue boundary. The response side (in_port) is owned by this
    object's EventQueue, while the request side (out_port) is owned by
    peer_queue. Packets cross the boundary through asynchronous EventQueue
    insertions, never through a synchronous sendTiming call into the other
    thread.

    Example:

    sys.requestor = Requestor(eventq_index=0)
    sys.responder = Responder(eventq_index=1)
    sys.bridge = TimingThreadBridge(eventq_index=0, peer_queue=1)

    sys.requestor.out_port = sys.bridge.in_port
    sys.bridge.out_port = sys.responder.in_port
    """

    type = "TimingThreadBridge"
    cxx_header = "mem/timing_thread_bridge.hh"
    cxx_class = "gem5::TimingThreadBridge"

    in_port = ResponsePort(
        "Response side (requestor-facing); receives requests and sends "
        "responses"
    )
    out_port = RequestPort(
        "Request side (responder-facing); sends requests and receives "
        "responses"
    )

    delay = Param.Cycles(
        0,
        "Cross-queue delivery latency in cycles. Must be >= "
        "ceil(sim_quantum / clock_period) when peer_queue differs from "
        "this object's eventq_index; zero is only valid for a same-queue "
        "bridge. Enforced in init().",
    )
    req_size = Param.Unsigned(16, "Request FIFO depth on the request side")
    resp_size = Param.Unsigned(16, "Response FIFO depth on the response side")
    ranges = VectorParam.AddrRange(
        [AllMemory], "Address ranges advertised by the response side"
    )
    peer_queue = Param.Int(
        0, "Event queue index of the request (out_port) side"
    )
