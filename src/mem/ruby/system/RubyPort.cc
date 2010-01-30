
/*
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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

#include "mem/physical.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

uint16_t RubyPort::m_num_ports = 0;

RubyPort::RequestMap RubyPort::pending_cpu_requests;

RubyPort::RubyPort(const Params *p)
    : MemObject(p),
      funcMemPort(csprintf("%s-funcmem_port", name()), this)
{
    m_version = p->version;
    assert(m_version != -1);

    m_controller = NULL;
    m_mandatory_q_ptr = NULL;

    m_port_id = m_num_ports++;
    m_request_cnt = 0;
    m_hit_callback = ruby_hit_callback;
    pio_port = NULL;
    assert(m_num_ports <= 2048); // see below for reason
}

void RubyPort::init()
{
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
}

Port *
RubyPort::getPort(const std::string &if_name, int idx)
{
    if (if_name == "port") {
        return new M5Port(csprintf("%s-port%d", name(), idx), this);
    } else if (if_name == "pio_port") {
        //
        // ensure there is only one pio port
        //
        assert(pio_port == NULL);

        pio_port = new PioPort(csprintf("%s-pio-port%d", name(), idx), 
                                     this);

        return pio_port;
    } else if (if_name == "funcmem_port") {
        return &funcMemPort;
    }
    return NULL;
}

RubyPort::PioPort::PioPort(const std::string &_name, 
                           RubyPort *_port)
    : SimpleTimingPort(_name, _port)
{
    DPRINTF(Ruby, "creating port to ruby sequencer to cpu %s\n", _name);
    ruby_port = _port;
}

RubyPort::M5Port::M5Port(const std::string &_name, 
                         RubyPort *_port)
    : SimpleTimingPort(_name, _port)
{
    DPRINTF(Ruby, "creating port from ruby sequcner to cpu %s\n", _name);
    ruby_port = _port;
}

Tick
RubyPort::PioPort::recvAtomic(PacketPtr pkt)
{
    panic("RubyPort::PioPort::recvAtomic() not implemented!\n");
    return 0;
}


Tick
RubyPort::M5Port::recvAtomic(PacketPtr pkt)
{
    panic("RubyPort::M5Port::recvAtomic() not implemented!\n");
    return 0;
}


bool
RubyPort::PioPort::recvTiming(PacketPtr pkt)
{
    //
    // In FS mode, ruby memory will receive pio responses from devices and
    // it must forward these responses back to the particular CPU.
    //
    DPRINTF(MemoryAccess, 
            "Pio response for address %#x\n",
            pkt->getAddr());

    assert(pkt->isResponse());

    //
    // First we must retrieve the request port from the sender State
    //
    RubyPort::SenderState *senderState = 
      safe_cast<RubyPort::SenderState *>(pkt->senderState);
    M5Port *port = senderState->port;
    assert(port != NULL);
    
    // pop the sender state from the packet
    pkt->senderState = senderState->saved;
    delete senderState;
    
    port->sendTiming(pkt);
    
    return true;
}

bool
RubyPort::M5Port::recvTiming(PacketPtr pkt)
{
    DPRINTF(MemoryAccess, 
            "Timing access caught for address %#x\n",
            pkt->getAddr());

    //dsm: based on SimpleTimingPort::recvTiming(pkt);

    //
    // After checking for pio responses, the remainder of packets
    // received by ruby should only be M5 requests, which should never 
    // get nacked.  There used to be code to hanldle nacks here, but 
    // I'm pretty sure it didn't work correctly with the drain code, 
    // so that would need to be fixed if we ever added it back.
    //
    assert(pkt->isRequest());

    if (pkt->memInhibitAsserted()) {
        warn("memInhibitAsserted???");
        // snooper will supply based on copy of packet
        // still target's responsibility to delete packet
        delete pkt;
        return true;
    }

    //
    // Check for pio requests and directly send them to the dedicated
    // pio port.
    //
    if (!isPhysMemAddress(pkt->getAddr())) {
        assert(ruby_port->pio_port != NULL);

        //
        // Save the port in the sender state object to be used later to
        // route the response
        //
        pkt->senderState = new SenderState(this, pkt->senderState);

        return ruby_port->pio_port->sendTiming(pkt);
    }

    //
    // For DMA and CPU requests, translate them to ruby requests before
    // sending them to our assigned ruby port.
    //
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
        type = RubyRequestType_RMW_Write;
    }

    RubyRequest ruby_request(pkt->getAddr(), pkt->getPtr<uint8_t>(),
                             pkt->getSize(), pc, type,
                             RubyAccessMode_Supervisor);

    // Submit the ruby request
    int64_t req_id = ruby_port->makeRequest(ruby_request);
    if (req_id == -1) {
        return false;
    }

    // Save the request for the callback
    RubyPort::pending_cpu_requests[req_id] = new RequestCookie(pkt, this);

    return true;
}

void
RubyPort::ruby_hit_callback(int64_t req_id)
{
    //
    // Note: This single fuction can be called by cpu and dma ports,
    // as well as the functional port.  
    //
    RequestMap::iterator i = pending_cpu_requests.find(req_id);
    if (i == pending_cpu_requests.end())
        panic("could not find pending request %d\n", req_id);

    RequestCookie *cookie = i->second;
    pending_cpu_requests.erase(i);

    Packet *pkt = cookie->pkt;
    M5Port *port = cookie->m5Port;
    delete cookie;

    port->hitCallback(pkt);
}

void
RubyPort::M5Port::hitCallback(PacketPtr pkt)
{

    bool needsResponse = pkt->needsResponse();

    DPRINTF(MemoryAccess, "Hit callback needs response %d\n",
            needsResponse);

    ruby_port->funcMemPort.sendFunctional(pkt);

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());
        DPRINTF(MemoryAccess, "Sending packet back over port\n");
        sendTiming(pkt);
    } else {
        delete pkt;
    }
    DPRINTF(MemoryAccess, "Hit callback done!\n");
}

bool
RubyPort::M5Port::sendTiming(PacketPtr pkt)
{
    schedSendTiming(pkt, curTick + 1); //minimum latency, must be > 0
    return true;
}

bool
RubyPort::PioPort::sendTiming(PacketPtr pkt)
{
    schedSendTiming(pkt, curTick + 1); //minimum latency, must be > 0
    return true;
}

bool
RubyPort::M5Port::isPhysMemAddress(Addr addr)
{
    AddrRangeList physMemAddrList;
    bool snoop = false;
    ruby_port->funcMemPort.getPeerAddressRanges(physMemAddrList, snoop);
    for(AddrRangeIter iter = physMemAddrList.begin(); 
        iter != physMemAddrList.end(); 
        iter++) {
        if (addr >= iter->start && addr <= iter->end) {
            DPRINTF(MemoryAccess, "Request found in %#llx - %#llx range\n",
                    iter->start, iter->end);
            return true;
        }
    }
    assert(isPioAddress(addr));
    return false;
}

bool
RubyPort::M5Port::isPioAddress(Addr addr)
{
    AddrRangeList pioAddrList;
    bool snoop = false;
    if (ruby_port->pio_port == NULL) {
        return false;
    }
  
    ruby_port->pio_port->getPeerAddressRanges(pioAddrList, snoop);
    for(AddrRangeIter iter = pioAddrList.begin(); 
        iter != pioAddrList.end(); 
        iter++) {
        if (addr >= iter->start && addr <= iter->end) {
            DPRINTF(MemoryAccess, "Pio request found in %#llx - %#llx range\n",
                    iter->start, iter->end);
            return true;
        }
    }
    return false;
}

