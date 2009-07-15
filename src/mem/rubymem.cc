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
#include "base/str.hh"
#include "base/types.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/ruby/libruby.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/rubymem.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"

using namespace std;
using namespace TheISA;

map<int64_t, PacketPtr> RubyMemory::pending_requests;

RubyMemory::RubyMemory(const Params *p)
  : PhysicalMemory(p)
{
    ruby_clock = p->clock;
    ruby_phase = p->phase;

    ifstream config(p->config_file.c_str());

    vector<RubyObjConf> sys_conf;
    while (!config.eof()) {
        char buffer[4096];
        config.getline(buffer, sizeof(buffer));
        string line = buffer;
        if (line.empty())
            continue;
        vector<string> tokens;
        tokenize(tokens, line, ' ');
        assert(tokens.size() >= 2);
        vector<string> argv;
        for (size_t i=2; i<tokens.size(); i++) {
            std::replace(tokens[i].begin(), tokens[i].end(), '%', ' ');
            std::replace(tokens[i].begin(), tokens[i].end(), '#', '\n');
            argv.push_back(tokens[i]);
        }
        sys_conf.push_back(RubyObjConf(tokens[0], tokens[1], argv));
        tokens.clear();
        argv.clear();
    }

    RubySystem::create(sys_conf);

    for (int i = 0; i < params()->num_cpus; i++) {
        RubyPort *p = RubySystem::getPort(csprintf("Sequencer_%d", i),
                                          ruby_hit_callback);
        ruby_ports.push_back(p);
    }
}

void
RubyMemory::init()
{
    if (params()->debug) {
        g_debug_ptr->setVerbosityString("high");
        g_debug_ptr->setDebugTime(1);
        if (!params()->debug_file.empty()) {
            g_debug_ptr->setDebugOutputFile(params()->debug_file.c_str());
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
void
RubyMemory::tick()
{
    RubyEventQueue *eq = RubySystem::getEventQueue();
    eq->triggerEvents(eq->getTime() + 1);
    schedule(rubyTickEvent, curTick + ruby_clock);
}

RubyMemory::~RubyMemory()
{
}

void
RubyMemory::hitCallback(PacketPtr pkt, Port *port)
{
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
        return new Port(csprintf("%s-functional", name()), this);
    }

    if (if_name != "port") {
        panic("RubyMemory::getPort: unknown port %s requested", if_name);
    }

    if (idx >= (int)ports.size()) {
        ports.resize(idx+1);
    }

    if (ports[idx] != NULL) {
        panic("RubyMemory::getPort: port %d already assigned", idx);
    }

    Port *port = new Port(csprintf("%s-port%d", name(), idx), this);

    ports[idx] = port;
    return port;
}

RubyMemory::Port::Port(const std::string &_name, RubyMemory *_memory)
    : PhysicalMemory::MemoryPort::MemoryPort(_name, _memory)
{
    ruby_mem = _memory;
}

bool
RubyMemory::Port::recvTiming(PacketPtr pkt)
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

    // Save the port in the sender state object
    pkt->senderState = new SenderState(this, pkt->senderState);

    RubyRequestType type = RubyRequestType_NULL;
    Addr pc = 0;
    if (pkt->isRead()) {
        if (pkt->req->isInstFetch()) {
            type = RubyRequestType_IFETCH;
            pc = pkt->req->getPC();
        } else {
            type = RubyRequestType_LD; 
        }
    } else if (pkt->isWrite()) {
        type = RubyRequestType_ST;
    } else if (pkt->isReadWrite()) {
      //        type = RubyRequestType_RMW;
    }

    RubyRequest ruby_request(pkt->getAddr(), pkt->getPtr<uint8_t>(),
                             pkt->getSize(), pc, type,
                             RubyAccessMode_Supervisor);

    // Submit the ruby request
    RubyPort *ruby_port = ruby_mem->ruby_ports[pkt->req->contextId()];
    int64_t req_id = ruby_port->makeRequest(ruby_request);
    if (req_id == -1) {
        RubyMemory::SenderState *senderState =
            safe_cast<RubyMemory::SenderState *>(pkt->senderState);

        // pop the sender state from the packet
        pkt->senderState = senderState->saved;
        delete senderState;
        return false;
    }

    // Save the request for the callback
    RubyMemory::pending_requests[req_id] = pkt;

    return true;
}

void
ruby_hit_callback(int64_t req_id)
{
    typedef map<int64_t, PacketPtr> map_t;
    map_t &prm = RubyMemory::pending_requests;

    map_t::iterator i = prm.find(req_id);
    if (i == prm.end())
        panic("could not find pending request %d\n", req_id);

    PacketPtr pkt = i->second;
    prm.erase(i);

    RubyMemory::SenderState *senderState =
        safe_cast<RubyMemory::SenderState *>(pkt->senderState);
    RubyMemory::Port *port = senderState->port;
    
    // pop the sender state from the packet
    pkt->senderState = senderState->saved;
    delete senderState;

    port->ruby_mem->hitCallback(pkt, port);
}

void
RubyMemory::Port::sendTiming(PacketPtr pkt)
{
    schedSendTiming(pkt, curTick + 1); //minimum latency, must be > 0
}

void RubyMemory::printConfigStats()
{
    std::ostream *os = simout.create(params()->stats_file);
    RubySystem::printConfig(*os);
    *os << endl;
    RubySystem::printStats(*os);
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

