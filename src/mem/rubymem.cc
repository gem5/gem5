/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 *
 * Authors: Daniel Sanchez
 */

#include <iostream>
#include <fstream>

#include "arch/isa_traits.hh"
#include "base/output.hh"
#include "base/types.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/ruby/init.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/rubymem.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"

using namespace std;
using namespace TheISA;

RubyMemory::RubyMemory(const Params *p)
  : PhysicalMemory(p)
{
    config_file = p->config_file;
    config_options = p->config_options;
    stats_file = p->stats_file;
    num_cpus = p->num_cpus;
    ruby_clock = p->clock;
    ruby_phase = p->phase;

    debug = p->debug;
    debug_file = p->debug_file;
}

void
RubyMemory::init()
{
    init_variables();
    g_NUM_PROCESSORS = num_cpus;

    init_simulator(this);

    if (debug) {
        g_debug_ptr->setVerbosityString("high");
        g_debug_ptr->setDebugTime(1);
        if (debug_file != "") {
             g_debug_ptr->setDebugOutputFile("ruby.debug");
        }
    }

    //You may want to set some other options...
    //g_debug_ptr->setVerbosityString("med");
    //g_debug_ptr->setFilterString("lsNqST");
    //g_debug_ptr->setFilterString("lsNST");
    //g_debug_ptr->setDebugTime(1);
    //g_debug_ptr->setDebugOutputFile("ruby.debug");


    g_system_ptr->clearStats();

    if (ports.size() == 0) {
        fatal("RubyMemory object %s is unconnected!", name());
    }

    for (PortIterator pi = ports.begin(); pi != ports.end(); ++pi) {
        if (*pi)
            (*pi)->sendStatusChange(Port::RangeChange);
    }

    //Print stats at exit
    RubyExitCallback* rc = new RubyExitCallback(this);
    registerExitCallback(rc);

    //Sched RubyEvent, automatically reschedules to advance ruby cycles
    rubyTickEvent = new RubyEvent(this);
    schedule(rubyTickEvent, curTick + ruby_clock + ruby_phase);
}

//called by rubyTickEvent
void RubyMemory::tick() {
    g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 1);
    schedule(rubyTickEvent, curTick + ruby_clock); //dsm: clock_phase was added here. This is wrong, the phase is only added on the first tick
}


RubyMemory::~RubyMemory() {
    delete g_system_ptr;
}

void
RubyMemory::hitCallback(Packet* pkt)
{
    RubyMemoryPort* port = m_packet_to_port_map[pkt];
    assert(port != NULL);
    m_packet_to_port_map.erase(pkt);

    DPRINTF(MemoryAccess, "Hit callback\n");

    bool needsResponse = pkt->needsResponse();
    doAtomicAccess(pkt);

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());
        DPRINTF(MemoryAccess, "Sending packet back over port\n");
        port->sendTiming(pkt);
    } else {
        delete pkt;
    }
    DPRINTF(MemoryAccess, "Hit callback done!\n");
}

Port *
RubyMemory::getPort(const std::string &if_name, int idx)
{
    // Accept request for "functional" port for backwards compatibility
    // with places where this function is called from C++.  I'd prefer
    // to move all these into Python someday.
    if (if_name == "functional") {
        return new RubyMemoryPort(csprintf("%s-functional", name()), this);
    }

    if (if_name != "port") {
        panic("RubyMemory::getPort: unknown port %s requested", if_name);
    }

    if (idx >= ports.size()) {
        ports.resize(idx+1);
    }

    if (ports[idx] != NULL) {
        panic("RubyMemory::getPort: port %d already assigned", idx);
    }

    RubyMemoryPort *port =
        new RubyMemoryPort(csprintf("%s-port%d", name(), idx), this);

    ports[idx] = port;
    return port;
}

RubyMemory::RubyMemoryPort::RubyMemoryPort(const std::string &_name,
                                       RubyMemory *_memory)
    : PhysicalMemory::MemoryPort::MemoryPort(_name, _memory)
{
    ruby_mem = _memory;
}

bool
RubyMemory::RubyMemoryPort::recvTiming(PacketPtr pkt)
{
    DPRINTF(MemoryAccess, "Timing access caught\n");

    //dsm: based on SimpleTimingPort::recvTiming(pkt);

    // If the device is only a slave, it should only be sending
    // responses, which should never get nacked.  There used to be
    // code to hanldle nacks here, but I'm pretty sure it didn't work
    // correctly with the drain code, so that would need to be fixed
    // if we ever added it back.
    assert(pkt->isRequest());

    if (pkt->memInhibitAsserted()) {
        warn("memInhibitAsserted???");
        // snooper will supply based on copy of packet
        // still target's responsibility to delete packet
        delete pkt;
        return true;
    }

    ruby_mem->m_packet_to_port_map[pkt] = this;

    Sequencer* sequencer = g_system_ptr->getSequencer(pkt->req->contextId());

    if ( ! sequencer->isReady(pkt) ) {
      DPRINTF(MemoryAccess, "Sequencer isn't ready yet!!\n");
      return false;
    }

    DPRINTF(MemoryAccess, "Issuing makeRequest\n");

    sequencer->makeRequest(pkt);
    return true;
}

void
RubyMemory::RubyMemoryPort::sendTiming(PacketPtr pkt)
{
    schedSendTiming(pkt, curTick + 1); //minimum latency, must be > 0
}

void RubyMemory::printConfigStats()
{
    std::ostream *os = simout.create(stats_file);
    g_system_ptr->printConfig(*os);
    *os << endl;
    g_system_ptr->printStats(*os);
}


//Right now these functions seem to be called by RubySystem. If they do calls
// to RubySystem perform it intended actions, you'll get into an inf loop
//FIXME what's the purpose of these here?
void RubyMemory::printStats(std::ostream & out) const {
    //g_system_ptr->printConfig(out);
}

void RubyMemory::clearStats() {
    //g_system_ptr->clearStats();
}

void RubyMemory::printConfig(std::ostream & out) const {
    //g_system_ptr->printConfig(out);
}


//Python-interface code
RubyMemory *
RubyMemoryParams::create()
{
    return new RubyMemory(this);
}

