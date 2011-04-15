/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2009-2010 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/testers/directedtest/DirectedGenerator.hh"
#include "cpu/testers/directedtest/RubyDirectedTester.hh"
#include "debug/DirectedTest.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "sim/sim_exit.hh"

RubyDirectedTester::RubyDirectedTester(const Params *p)
  : MemObject(p), directedStartEvent(this),
    m_requests_to_complete(p->requests_to_complete),
    generator(p->generator)
{
    m_requests_completed = 0;

    // add the check start event to the event queue
    schedule(directedStartEvent, 1);
}

RubyDirectedTester::~RubyDirectedTester()
{
    for (int i = 0; i < ports.size(); i++)
        delete ports[i];
}

void
RubyDirectedTester::init()
{
    assert(ports.size() > 0);
    generator->setDirectedTester(this);
}

Port *
RubyDirectedTester::getPort(const std::string &if_name, int idx)
{
    if (if_name != "cpuPort") {
        panic("RubyDirectedTester::getPort: unknown port %s requested", if_name);
    }

    if (idx >= (int)ports.size()) {
        ports.resize(idx + 1);
    }

    if (ports[idx] != NULL) {
        panic("RubyDirectedTester::getPort: port %d already assigned", idx);
    }

    CpuPort *port = new CpuPort(csprintf("%s-port%d", name(), idx), this, idx);

    ports[idx] = port;
    return port;
}

Tick
RubyDirectedTester::CpuPort::recvAtomic(PacketPtr pkt)
{
    panic("RubyDirectedTester::CpuPort::recvAtomic() not implemented!\n");
    return 0;
}

bool
RubyDirectedTester::CpuPort::recvTiming(PacketPtr pkt)
{
    tester->hitCallback(idx, pkt->getAddr());
    
    //
    // Now that the tester has completed, delete the packet, then return
    //
    delete pkt->req;
    delete pkt;
    return true;
}

Port*
RubyDirectedTester::getCpuPort(int idx)
{
    assert(idx >= 0 && idx < ports.size());

    return ports[idx];
}

void
RubyDirectedTester::hitCallback(NodeID proc, Addr addr)
{
    DPRINTF(DirectedTest,
            "completed request for proc: %d addr: 0x%x\n",
            proc,
            addr);

    generator->performCallback(proc, addr);    
    schedule(directedStartEvent, curTick());
}

void
RubyDirectedTester::wakeup()
{
    if (m_requests_completed < m_requests_to_complete) {
        if (!generator->initiate()) {
            schedule(directedStartEvent, curTick() + 1);
        }
    } else {
        exitSimLoop("Ruby DirectedTester completed");
    }
}

RubyDirectedTester *
RubyDirectedTesterParams::create()
{
    return new RubyDirectedTester(this);
}
