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

#include "config/the_isa.hh"
#if THE_ISA == X86_ISA
#include "arch/x86/insts/microldstop.hh"
#endif // X86_ISA
#include "cpu/testers/rubytest/RubyTester.hh"
#include "mem/physical.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/RubyPort.hh"

RubyPort::RubyPort(const Params *p)
    : MemObject(p)
{
    m_version = p->version;
    assert(m_version != -1);

    physmem = p->physmem;

    m_controller = NULL;
    m_mandatory_q_ptr = NULL;

    m_request_cnt = 0;
    pio_port = NULL;
    physMemPort = NULL;

    m_usingRubyTester = p->using_ruby_tester;
    access_phys_mem = p->access_phys_mem;
}

void
RubyPort::init()
{
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
}

Port *
RubyPort::getPort(const std::string &if_name, int idx)
{
    if (if_name == "port") {
        return new M5Port(csprintf("%s-port%d", name(), idx), this,
                          access_phys_mem);
    }

    if (if_name == "pio_port") {
        // ensure there is only one pio port
        assert(pio_port == NULL);

        pio_port = new PioPort(csprintf("%s-pio-port%d", name(), idx), this);

        return pio_port;
    }

    if (if_name == "physMemPort") {
        // RubyPort should only have one port to physical memory
        assert (physMemPort == NULL);

        physMemPort = new M5Port(csprintf("%s-physMemPort", name()), this,
                                 access_phys_mem);

        return physMemPort;
    }

    if (if_name == "functional") {
        // Calls for the functional port only want to access
        // functional memory.  Therefore, directly pass these calls
        // ports to physmem.
        assert(physmem != NULL);
        return physmem->getPort(if_name, idx);
    }

    return NULL;
}

RubyPort::PioPort::PioPort(const std::string &_name,
                           RubyPort *_port)
    : SimpleTimingPort(_name, _port)
{
    DPRINTF(RubyPort, "creating port to ruby sequencer to cpu %s\n", _name);
    ruby_port = _port;
}

RubyPort::M5Port::M5Port(const std::string &_name,
                         RubyPort *_port, bool _access_phys_mem)
    : SimpleTimingPort(_name, _port)
{
    DPRINTF(RubyPort, "creating port from ruby sequcner to cpu %s\n", _name);
    ruby_port = _port;
    _onRetryList = false;
    access_phys_mem = _access_phys_mem;
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
    // In FS mode, ruby memory will receive pio responses from devices
    // and it must forward these responses back to the particular CPU.
    DPRINTF(RubyPort,  "Pio response for address %#x\n", pkt->getAddr());

    assert(pkt->isResponse());

    // First we must retrieve the request port from the sender State
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
    DPRINTF(RubyPort,
            "Timing access caught for address %#x\n", pkt->getAddr());

    //dsm: based on SimpleTimingPort::recvTiming(pkt);

    // The received packets should only be M5 requests, which should never
    // get nacked.  There used to be code to hanldle nacks here, but
    // I'm pretty sure it didn't work correctly with the drain code,
    // so that would need to be fixed if we ever added it back.
    assert(pkt->isRequest());

    if (pkt->memInhibitAsserted()) {
        warn("memInhibitAsserted???");
        // snooper will supply based on copy of packet
        // still target's responsibility to delete packet
        delete pkt;
        return true;
    }

    // Save the port in the sender state object to be used later to
    // route the response
    pkt->senderState = new SenderState(this, pkt->senderState);

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (!isPhysMemAddress(pkt->getAddr())) {
        assert(ruby_port->pio_port != NULL);
        DPRINTF(RubyPort,
                "Request for address 0x%#x is assumed to be a pio request\n",
                pkt->getAddr());

        return ruby_port->pio_port->sendTiming(pkt);
    }

    // For DMA and CPU requests, translate them to ruby requests before
    // sending them to our assigned ruby port.
    RubyRequestType type = RubyRequestType_NULL;

    // If valid, copy the pc to the ruby request
    Addr pc = 0;
    if (pkt->req->hasPC()) {
        pc = pkt->req->getPC();
    }

    if (pkt->isLLSC()) {
        if (pkt->isWrite()) {
            DPRINTF(RubyPort, "Issuing SC\n");
            type = RubyRequestType_Store_Conditional;
        } else {
            DPRINTF(RubyPort, "Issuing LL\n");
            assert(pkt->isRead());
            type = RubyRequestType_Load_Linked;
        }
    } else if (pkt->req->isLocked()) {
        if (pkt->isWrite()) {
            DPRINTF(RubyPort, "Issuing Locked RMW Write\n");
            type = RubyRequestType_Locked_RMW_Write;
        } else {
            DPRINTF(RubyPort, "Issuing Locked RMW Read\n");
            assert(pkt->isRead());
            type = RubyRequestType_Locked_RMW_Read;
        }
    } else {
        if (pkt->isRead()) {
            if (pkt->req->isInstFetch()) {
                type = RubyRequestType_IFETCH;
            } else {
#if THE_ISA == X86_ISA
                uint32_t flags = pkt->req->getFlags();
                bool storeCheck = flags &
                        (TheISA::StoreCheck << TheISA::FlagShift);
#else
                bool storeCheck = false;
#endif // X86_ISA
                if (storeCheck) {
                    type = RubyRequestType_RMW_Read;
                } else {
                    type = RubyRequestType_LD;
                }
            }
        } else if (pkt->isWrite()) {
            //
            // Note: M5 packets do not differentiate ST from RMW_Write
            //
            type = RubyRequestType_ST;
        } else if (pkt->isFlush()) {
            type = RubyRequestType_FLUSH;
        } else {
            panic("Unsupported ruby packet type\n");
        }
    }

    RubyRequest ruby_request(pkt->getAddr(), pkt->getPtr<uint8_t>(true),
                             pkt->getSize(), pc, type,
                             RubyAccessMode_Supervisor, pkt);

    assert(ruby_request.m_PhysicalAddress.getOffset() + ruby_request.m_Size <=
        RubySystem::getBlockSizeBytes());

    // Submit the ruby request
    RequestStatus requestStatus = ruby_port->makeRequest(ruby_request);

    // If the request successfully issued then we should return true.
    // Otherwise, we need to delete the senderStatus we just created and return
    // false.
    if (requestStatus == RequestStatus_Issued) {
        DPRINTF(RubyPort, "Request %#x issued\n", pkt->getAddr());
        return true;
    }

    //
    // Unless one is using the ruby tester, record the stalled M5 port for 
    // later retry when the sequencer becomes free.
    //
    if (!ruby_port->m_usingRubyTester) {
        ruby_port->addToRetryList(this);
    }

    DPRINTF(RubyPort,
            "Request for address %#x did not issue because %s\n",
            pkt->getAddr(), RequestStatus_to_string(requestStatus));

    SenderState* senderState = safe_cast<SenderState*>(pkt->senderState);
    pkt->senderState = senderState->saved;
    delete senderState;
    return false;
}

void
RubyPort::ruby_hit_callback(PacketPtr pkt)
{
    // Retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->senderState);
    M5Port *port = senderState->port;
    assert(port != NULL);

    // pop the sender state from the packet
    pkt->senderState = senderState->saved;
    delete senderState;

    port->hitCallback(pkt);

    //
    // If we had to stall the M5Ports, wake them up because the sequencer
    // likely has free resources now.
    //
    if (waitingOnSequencer) {
        //
        // Record the current list of ports to retry on a temporary list before
        // calling sendRetry on those ports.  sendRetry will cause an 
        // immediate retry, which may result in the ports being put back on the
        // list. Therefore we want to clear the retryList before calling
        // sendRetry.
        //
        std::list<M5Port*> curRetryList(retryList);

        retryList.clear();
        waitingOnSequencer = false;
        
        for (std::list<M5Port*>::iterator i = curRetryList.begin();
             i != curRetryList.end(); ++i) {
            DPRINTF(RubyPort,
                    "Sequencer may now be free.  SendRetry to port %s\n",
                    (*i)->name());
            (*i)->onRetryList(false);
            (*i)->sendRetry();
        }
    }
}

void
RubyPort::M5Port::hitCallback(PacketPtr pkt)
{
    bool needsResponse = pkt->needsResponse();

    //
    // Unless specified at configuraiton, all responses except failed SC 
    // and Flush operations access M5 physical memory.
    //
    bool accessPhysMem = access_phys_mem;

    if (pkt->isLLSC()) {
        if (pkt->isWrite()) {
            if (pkt->req->getExtraData() != 0) {
                //
                // Successful SC packets convert to normal writes
                //
                pkt->convertScToWrite();
            } else {
                //
                // Failed SC packets don't access physical memory and thus
                // the RubyPort itself must convert it to a response.
                //
                accessPhysMem = false;
            }
        } else {
            //
            // All LL packets convert to normal loads so that M5 PhysMem does
            // not lock the blocks.
            //
            pkt->convertLlToRead();
        }
    }

    //
    // Flush requests don't access physical memory
    //
    if (pkt->isFlush()) {
        accessPhysMem = false;
    }

    DPRINTF(RubyPort, "Hit callback needs response %d\n", needsResponse);

    if (accessPhysMem) {
        ruby_port->physMemPort->sendAtomic(pkt);
    } else if (needsResponse) {
        pkt->makeResponse();
    }

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        DPRINTF(RubyPort, "Sending packet back over port\n");
        sendTiming(pkt);
    } else {
        delete pkt;
    }
    DPRINTF(RubyPort, "Hit callback done!\n");
}

bool
RubyPort::M5Port::sendTiming(PacketPtr pkt)
{
    //minimum latency, must be > 0
    schedSendTiming(pkt, curTick() + (1 * g_eventQueue_ptr->getClock()));
    return true;
}

bool
RubyPort::PioPort::sendTiming(PacketPtr pkt)
{
    //minimum latency, must be > 0
    schedSendTiming(pkt, curTick() + (1 * g_eventQueue_ptr->getClock()));
    return true;
}

bool
RubyPort::M5Port::isPhysMemAddress(Addr addr)
{
    AddrRangeList physMemAddrList;
    bool snoop = false;
    ruby_port->physMemPort->getPeerAddressRanges(physMemAddrList, snoop);
    for (AddrRangeIter iter = physMemAddrList.begin();
         iter != physMemAddrList.end();
         iter++) {
        if (addr >= iter->start && addr <= iter->end) {
            DPRINTF(RubyPort, "Request found in %#llx - %#llx range\n",
                    iter->start, iter->end);
            return true;
        }
    }
    return false;
}

unsigned
RubyPort::M5Port::deviceBlockSize() const
{
    return (unsigned) RubySystem::getBlockSizeBytes();
}
