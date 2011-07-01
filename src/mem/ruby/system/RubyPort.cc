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
#include "debug/Ruby.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/physical.hh"

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

    ruby_system = p->ruby_system;
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
                          ruby_system, access_phys_mem);
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
                                 ruby_system, access_phys_mem);

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

RubyPort::M5Port::M5Port(const std::string &_name, RubyPort *_port,
                         RubySystem *_system, bool _access_phys_mem)
    : SimpleTimingPort(_name, _port)
{
    DPRINTF(RubyPort, "creating port from ruby sequcner to cpu %s\n", _name);
    ruby_port = _port;
    ruby_system = _system;
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

bool
RubyPort::M5Port::doFunctionalRead(PacketPtr pkt)
{
    Address address(pkt->getAddr());
    Address line_address(address);
    line_address.makeLineAddress();

    AccessPermission accessPerm = AccessPermission_NotPresent;
    int num_controllers = ruby_system->m_abs_cntrl_vec.size();

    // In this loop, we try to figure which controller has a read only or
    // a read write copy of the given address. Any valid copy would suffice
    // for a functional read.

    DPRINTF(RubyPort, "Functional Read request for %s\n",address);
    for(int i = 0;i < num_controllers;++i)
    {
        accessPerm = ruby_system->m_abs_cntrl_vec[i]
                                          ->getAccessPermission(line_address);
        if(accessPerm == AccessPermission_Read_Only ||
           accessPerm == AccessPermission_Read_Write)
        {
            unsigned startByte = address.getAddress() - line_address.getAddress();

            uint8* data = pkt->getPtr<uint8_t>(true);
            unsigned int size_in_bytes = pkt->getSize();
            DataBlock& block = ruby_system->m_abs_cntrl_vec[i]
                                                 ->getDataBlock(line_address);

            DPRINTF(RubyPort, "reading from %s block %s\n",
                    ruby_system->m_abs_cntrl_vec[i]->name(), block);
            for (unsigned i = 0; i < size_in_bytes; ++i)
            {
                data[i] = block.getByte(i + startByte);
            }
            return true;
        }
    }
    return false;
}

bool
RubyPort::M5Port::doFunctionalWrite(PacketPtr pkt)
{
    Address addr(pkt->getAddr());
    Address line_addr = line_address(addr);
    AccessPermission accessPerm = AccessPermission_NotPresent;
    int num_controllers = ruby_system->m_abs_cntrl_vec.size();

    DPRINTF(RubyPort, "Functional Write request for %s\n",addr);

    unsigned int num_ro = 0;
    unsigned int num_rw = 0;
    unsigned int num_busy = 0;

    // In this loop we count the number of controllers that have the given
    // address in read only, read write and busy states.
    for(int i = 0;i < num_controllers;++i)
    {
        accessPerm = ruby_system->m_abs_cntrl_vec[i]->
                                            getAccessPermission(line_addr);
        if(accessPerm == AccessPermission_Read_Only) num_ro++;
        else if(accessPerm == AccessPermission_Read_Write) num_rw++;
        else if(accessPerm == AccessPermission_Busy) num_busy++;
    }

    // If the number of read write copies is more than 1, then there is bug in
    // coherence protocol. Otherwise, if all copies are in stable states, i.e.
    // num_busy == 0, we update all the copies. If there is at least one copy
    // in busy state, then we check if there is read write copy. If yes, then
    // also we let the access go through.

    DPRINTF(RubyPort, "num_busy = %d, num_ro = %d, num_rw = %d\n",
            num_busy, num_ro, num_rw);
    assert(num_rw <= 1);
    if((num_busy == 0 && num_ro > 0) || num_rw == 1)
    {
        uint8* data = pkt->getPtr<uint8_t>(true);
        unsigned int size_in_bytes = pkt->getSize();
        unsigned startByte = addr.getAddress() - line_addr.getAddress();

        for(int i = 0; i < num_controllers;++i)
        {
            accessPerm = ruby_system->m_abs_cntrl_vec[i]->
                                                getAccessPermission(line_addr);
            if(accessPerm == AccessPermission_Read_Only ||
               accessPerm == AccessPermission_Read_Write||
               accessPerm == AccessPermission_Maybe_Stale)
            {
                DataBlock& block = ruby_system->m_abs_cntrl_vec[i]
                                                      ->getDataBlock(line_addr);

                DPRINTF(RubyPort, "%s\n",block);
                for (unsigned i = 0; i < size_in_bytes; ++i)
                {
                  block.setByte(i + startByte, data[i]);
                }
                DPRINTF(RubyPort, "%s\n",block);
            }
        }
        return true;
    }
    return false;
}

void
RubyPort::M5Port::recvFunctional(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Functional access caught for address %#x\n",
                                                           pkt->getAddr());

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (!isPhysMemAddress(pkt->getAddr())) {
        assert(ruby_port->pio_port != NULL);
        DPRINTF(RubyPort, "Request for address 0x%#x is a pio request\n",
                                                           pkt->getAddr());
        panic("RubyPort::PioPort::recvFunctional() not implemented!\n");
    }

    assert(pkt->getAddr() + pkt->getSize() <=
                line_address(Address(pkt->getAddr())).getAddress() +
                RubySystem::getBlockSizeBytes());

    bool accessSucceeded = false;
    bool needsResponse = pkt->needsResponse();

    // Do the functional access on ruby memory
    if (pkt->isRead()) {
        accessSucceeded = doFunctionalRead(pkt);
    } else if (pkt->isWrite()) {
        accessSucceeded = doFunctionalWrite(pkt);
    } else {
        panic("RubyPort: unsupported functional command %s\n",
              pkt->cmdString());
    }

    // Unless the requester explicitly said otherwise, generate an error if
    // the functional request failed
    if (!accessSucceeded && !pkt->suppressFuncError()) {
        fatal("Ruby functional %s failed for address %#x\n",
              pkt->isWrite() ? "write" : "read", pkt->getAddr());
    }

    if (access_phys_mem) {
        // The attached physmem contains the official version of data.
        // The following command performs the real functional access.
        // This line should be removed once Ruby supplies the official version
        // of data.
        ruby_port->physMemPort->sendFunctional(pkt);
    }

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        pkt->setFunctionalResponseStatus(accessSucceeded);
        DPRINTF(RubyPort, "Sending packet back over port\n");
        sendFunctional(pkt);
    }
    DPRINTF(RubyPort, "Functional access %s!\n",
            accessSucceeded ? "successful":"failed");
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
