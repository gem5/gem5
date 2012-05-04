/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
 * Copyright (c) 2011 Mark D. Hill and David A. Wood
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

#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/Config.hh"
#include "debug/Ruby.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "sim/system.hh"

RubyPort::RubyPort(const Params *p)
    : MemObject(p), m_version(p->version), m_controller(NULL),
      m_mandatory_q_ptr(NULL),
      pio_port(csprintf("%s-pio-port", name()), this),
      m_usingRubyTester(p->using_ruby_tester), m_request_cnt(0),
      drainEvent(NULL), ruby_system(p->ruby_system), system(p->system),
      waitingOnSequencer(false), access_phys_mem(p->access_phys_mem)
{
    assert(m_version != -1);

    // create the slave ports based on the number of connected ports
    for (size_t i = 0; i < p->port_slave_connection_count; ++i) {
        slave_ports.push_back(new M5Port(csprintf("%s-slave%d", name(), i),
                                         this, ruby_system, access_phys_mem));
    }

    // create the master ports based on the number of connected ports
    for (size_t i = 0; i < p->port_master_connection_count; ++i) {
        master_ports.push_back(new PioPort(csprintf("%s-master%d", name(), i),
                                           this));
    }
}

void
RubyPort::init()
{
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
}

MasterPort &
RubyPort::getMasterPort(const std::string &if_name, int idx)
{
    if (if_name == "pio_port") {
        return pio_port;
    }

    // used by the x86 CPUs to connect the interrupt PIO and interrupt slave
    // port
    if (if_name != "master") {
        // pass it along to our super class
        return MemObject::getMasterPort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(master_ports.size())) {
            panic("RubyPort::getMasterPort: unknown index %d\n", idx);
        }

        return *master_ports[idx];
    }
}

SlavePort &
RubyPort::getSlavePort(const std::string &if_name, int idx)
{
    // used by the CPUs to connect the caches to the interconnect, and
    // for the x86 case also the interrupt master
    if (if_name != "slave") {
        // pass it along to our super class
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(slave_ports.size())) {
            panic("RubyPort::getSlavePort: unknown index %d\n", idx);
        }

        return *slave_ports[idx];
    }
}

RubyPort::PioPort::PioPort(const std::string &_name,
                           RubyPort *_port)
    : QueuedMasterPort(_name, _port, queue), queue(*_port, *this),
      ruby_port(_port)
{
    DPRINTF(RubyPort, "creating master port on ruby sequencer %s\n", _name);
}

RubyPort::M5Port::M5Port(const std::string &_name, RubyPort *_port,
                         RubySystem *_system, bool _access_phys_mem)
    : QueuedSlavePort(_name, _port, queue), queue(*_port, *this),
      ruby_port(_port), ruby_system(_system),
      _onRetryList(false), access_phys_mem(_access_phys_mem)
{
    DPRINTF(RubyPort, "creating slave port on ruby sequencer %s\n", _name);
}

Tick
RubyPort::M5Port::recvAtomic(PacketPtr pkt)
{
    panic("RubyPort::M5Port::recvAtomic() not implemented!\n");
    return 0;
}


bool
RubyPort::PioPort::recvTimingResp(PacketPtr pkt)
{
    // In FS mode, ruby memory will receive pio responses from devices
    // and it must forward these responses back to the particular CPU.
    DPRINTF(RubyPort,  "Pio response for address %#x\n", pkt->getAddr());

    // First we must retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
      safe_cast<RubyPort::SenderState *>(pkt->senderState);
    M5Port *port = senderState->port;
    assert(port != NULL);

    // pop the sender state from the packet
    pkt->senderState = senderState->saved;
    delete senderState;

    port->sendTimingResp(pkt);

    return true;
}

bool
RubyPort::M5Port::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(RubyPort,
            "Timing access caught for address %#x\n", pkt->getAddr());

    //dsm: based on SimpleTimingPort::recvTimingReq(pkt);

    // The received packets should only be M5 requests, which should never
    // get nacked.  There used to be code to hanldle nacks here, but
    // I'm pretty sure it didn't work correctly with the drain code,
    // so that would need to be fixed if we ever added it back.

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
        assert(ruby_port->pio_port.isConnected());
        DPRINTF(RubyPort,
                "Request for address 0x%#x is assumed to be a pio request\n",
                pkt->getAddr());

        return ruby_port->pio_port.sendNextCycle(pkt);
    }

    assert(Address(pkt->getAddr()).getOffset() + pkt->getSize() <=
           RubySystem::getBlockSizeBytes());

    // Submit the ruby request
    RequestStatus requestStatus = ruby_port->makeRequest(pkt);

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

    AccessPermission access_perm = AccessPermission_NotPresent;
    int num_controllers = ruby_system->m_abs_cntrl_vec.size();

    DPRINTF(RubyPort, "Functional Read request for %s\n",address);

    unsigned int num_ro = 0;
    unsigned int num_rw = 0;
    unsigned int num_busy = 0;
    unsigned int num_backing_store = 0;
    unsigned int num_invalid = 0;

    // In this loop we count the number of controllers that have the given
    // address in read only, read write and busy states.
    for (int i = 0; i < num_controllers; ++i) {
        access_perm = ruby_system->m_abs_cntrl_vec[i]->
                                            getAccessPermission(line_address);
        if (access_perm == AccessPermission_Read_Only)
            num_ro++;
        else if (access_perm == AccessPermission_Read_Write)
            num_rw++;
        else if (access_perm == AccessPermission_Busy)
            num_busy++;
        else if (access_perm == AccessPermission_Backing_Store)
            // See RubySlicc_Exports.sm for details, but Backing_Store is meant
            // to represent blocks in memory *for Broadcast/Snooping protocols*,
            // where memory has no idea whether it has an exclusive copy of data
            // or not.
            num_backing_store++;
        else if (access_perm == AccessPermission_Invalid ||
                 access_perm == AccessPermission_NotPresent)
            num_invalid++;
    }
    assert(num_rw <= 1);

    uint8* data = pkt->getPtr<uint8_t>(true);
    unsigned int size_in_bytes = pkt->getSize();
    unsigned startByte = address.getAddress() - line_address.getAddress();

    // This if case is meant to capture what happens in a Broadcast/Snoop
    // protocol where the block does not exist in the cache hierarchy. You
    // only want to read from the Backing_Store memory if there is no copy in
    // the cache hierarchy, otherwise you want to try to read the RO or RW
    // copies existing in the cache hierarchy (covered by the else statement).
    // The reason is because the Backing_Store memory could easily be stale, if
    // there are copies floating around the cache hierarchy, so you want to read
    // it only if it's not in the cache hierarchy at all.
    if (num_invalid == (num_controllers - 1) &&
            num_backing_store == 1)
    {
        DPRINTF(RubyPort, "only copy in Backing_Store memory, read from it\n");
        for (int i = 0; i < num_controllers; ++i) {
            access_perm = ruby_system->m_abs_cntrl_vec[i]
                                              ->getAccessPermission(line_address);
            if (access_perm == AccessPermission_Backing_Store) {
                DataBlock& block = ruby_system->m_abs_cntrl_vec[i]
                                                 ->getDataBlock(line_address);

                DPRINTF(RubyPort, "reading from %s block %s\n",
                        ruby_system->m_abs_cntrl_vec[i]->name(), block);
                for (unsigned i = 0; i < size_in_bytes; ++i) {
                    data[i] = block.getByte(i + startByte);
                }
                return true;
            }
        }
    } else {
        // In Broadcast/Snoop protocols, this covers if you know the block
        // exists somewhere in the caching hierarchy, then you want to read any
        // valid RO or RW block.  In directory protocols, same thing, you want
        // to read any valid readable copy of the block.
        DPRINTF(RubyPort, "num_busy = %d, num_ro = %d, num_rw = %d\n",
                num_busy, num_ro, num_rw);
        // In this loop, we try to figure which controller has a read only or
        // a read write copy of the given address. Any valid copy would suffice
        // for a functional read.
        for(int i = 0;i < num_controllers;++i) {
            access_perm = ruby_system->m_abs_cntrl_vec[i]
                                              ->getAccessPermission(line_address);
            if(access_perm == AccessPermission_Read_Only ||
               access_perm == AccessPermission_Read_Write)
            {
                DataBlock& block = ruby_system->m_abs_cntrl_vec[i]
                                                     ->getDataBlock(line_address);

                DPRINTF(RubyPort, "reading from %s block %s\n",
                        ruby_system->m_abs_cntrl_vec[i]->name(), block);
                for (unsigned i = 0; i < size_in_bytes; ++i) {
                    data[i] = block.getByte(i + startByte);
                }
                return true;
            }
        }
    }
    return false;
}

bool
RubyPort::M5Port::doFunctionalWrite(PacketPtr pkt)
{
    Address addr(pkt->getAddr());
    Address line_addr = line_address(addr);
    AccessPermission access_perm = AccessPermission_NotPresent;
    int num_controllers = ruby_system->m_abs_cntrl_vec.size();

    DPRINTF(RubyPort, "Functional Write request for %s\n",addr);

    unsigned int num_ro = 0;
    unsigned int num_rw = 0;
    unsigned int num_busy = 0;
    unsigned int num_backing_store = 0;
    unsigned int num_invalid = 0;

    // In this loop we count the number of controllers that have the given
    // address in read only, read write and busy states.
    for(int i = 0;i < num_controllers;++i) {
        access_perm = ruby_system->m_abs_cntrl_vec[i]->
                                            getAccessPermission(line_addr);
        if (access_perm == AccessPermission_Read_Only)
            num_ro++;
        else if (access_perm == AccessPermission_Read_Write)
            num_rw++;
        else if (access_perm == AccessPermission_Busy)
            num_busy++;
        else if (access_perm == AccessPermission_Backing_Store)
            // See RubySlicc_Exports.sm for details, but Backing_Store is meant
            // to represent blocks in memory *for Broadcast/Snooping protocols*,
            // where memory has no idea whether it has an exclusive copy of data
            // or not.
            num_backing_store++;
        else if (access_perm == AccessPermission_Invalid ||
                 access_perm == AccessPermission_NotPresent)
            num_invalid++;
    }

    // If the number of read write copies is more than 1, then there is bug in
    // coherence protocol. Otherwise, if all copies are in stable states, i.e.
    // num_busy == 0, we update all the copies. If there is at least one copy
    // in busy state, then we check if there is read write copy. If yes, then
    // also we let the access go through. Or, if there is no copy in the cache
    // hierarchy at all, we still want to do the write to the memory
    // (Backing_Store) instead of failing.

    DPRINTF(RubyPort, "num_busy = %d, num_ro = %d, num_rw = %d\n",
            num_busy, num_ro, num_rw);
    assert(num_rw <= 1);

    uint8* data = pkt->getPtr<uint8_t>(true);
    unsigned int size_in_bytes = pkt->getSize();
    unsigned startByte = addr.getAddress() - line_addr.getAddress();

    if ((num_busy == 0 && num_ro > 0) || num_rw == 1 ||
            (num_invalid == (num_controllers - 1) && num_backing_store == 1))
    {
        for(int i = 0; i < num_controllers;++i) {
            access_perm = ruby_system->m_abs_cntrl_vec[i]->
                                                getAccessPermission(line_addr);
            if(access_perm == AccessPermission_Read_Only ||
               access_perm == AccessPermission_Read_Write||
               access_perm == AccessPermission_Maybe_Stale ||
               access_perm == AccessPermission_Backing_Store)
            {
                DataBlock& block = ruby_system->m_abs_cntrl_vec[i]
                                                      ->getDataBlock(line_addr);

                DPRINTF(RubyPort, "%s\n",block);
                for (unsigned i = 0; i < size_in_bytes; ++i) {
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
        assert(ruby_port->pio_port.isConnected());
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
        ruby_port->system->getPhysMem().functionalAccess(pkt);
    }

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        pkt->setFunctionalResponseStatus(accessSucceeded);

        // @todo There should not be a reverse call since the response is
        // communicated through the packet pointer
        // DPRINTF(RubyPort, "Sending packet back over port\n");
        // sendFunctional(pkt);
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

    testDrainComplete();
}

void
RubyPort::testDrainComplete()
{
    //If we weren't able to drain before, we might be able to now.
    if (drainEvent != NULL) {
        unsigned int drainCount = getDrainCount(drainEvent);
        DPRINTF(Config, "Drain count: %u\n", drainCount);
        if (drainCount == 0) {
            drainEvent->process();
            // Clear the drain event once we're done with it.
            drainEvent = NULL;
        }
    }
}

unsigned int
RubyPort::getDrainCount(Event *de)
{
    int count = 0;
    //
    // If the sequencer is not empty, then requests need to drain.
    // The outstandingCount is the number of requests outstanding and thus the
    // number of times M5's timing port will process the drain event.
    //
    count += outstandingCount();

    DPRINTF(Config, "outstanding count %d\n", outstandingCount());

    // To simplify the draining process, the sequencer's deadlock detection
    // event should have been descheduled.
    assert(isDeadlockEventScheduled() == false);

    if (pio_port.isConnected()) {
        count += pio_port.drain(de);
        DPRINTF(Config, "count after pio check %d\n", count);
    }

    for (CpuPortIter p = slave_ports.begin(); p != slave_ports.end(); ++p) {
        count += (*p)->drain(de);
        DPRINTF(Config, "count after slave port check %d\n", count);
    }

    for (std::vector<PioPort*>::iterator p = master_ports.begin();
         p != master_ports.end(); ++p) {
        count += (*p)->drain(de);
        DPRINTF(Config, "count after master port check %d\n", count);
    }

    DPRINTF(Config, "final count %d\n", count);

    return count;
}

unsigned int
RubyPort::drain(Event *de)
{
    if (isDeadlockEventScheduled()) {
        descheduleDeadlockEvent();
    }

    int count = getDrainCount(de);

    // Set status
    if (count != 0) {
        drainEvent = de;

        changeState(SimObject::Draining);
        return count;
    }

    changeState(SimObject::Drained);
    return 0;
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
        ruby_port->system->getPhysMem().access(pkt);
    } else if (needsResponse) {
        pkt->makeResponse();
    }

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        DPRINTF(RubyPort, "Sending packet back over port\n");
        sendNextCycle(pkt);
    } else {
        delete pkt;
    }
    DPRINTF(RubyPort, "Hit callback done!\n");
}

bool
RubyPort::M5Port::sendNextCycle(PacketPtr pkt, bool send_as_snoop)
{
    //minimum latency, must be > 0
    queue.schedSendTiming(pkt, curTick() + (1 * g_eventQueue_ptr->getClock()),
                          send_as_snoop);
    return true;
}

bool
RubyPort::PioPort::sendNextCycle(PacketPtr pkt)
{
    //minimum latency, must be > 0
    queue.schedSendTiming(pkt, curTick() + (1 * g_eventQueue_ptr->getClock()));
    return true;
}

AddrRangeList
RubyPort::M5Port::getAddrRanges()
{
    // at the moment the assumption is that the master does not care
    AddrRangeList ranges;
    return ranges;
}

bool
RubyPort::M5Port::isPhysMemAddress(Addr addr)
{
    return ruby_port->system->isMemAddr(addr);
}

unsigned
RubyPort::M5Port::deviceBlockSize() const
{
    return (unsigned) RubySystem::getBlockSizeBytes();
}

void
RubyPort::ruby_eviction_callback(const Address& address)
{
    DPRINTF(RubyPort, "Sending invalidations.\n");
    // should this really be using funcMasterId?
    Request req(address.getAddress(), 0, 0, Request::funcMasterId);
    for (CpuPortIter p = slave_ports.begin(); p != slave_ports.end(); ++p) {
        if ((*p)->getMasterPort().isSnooping()) {
            Packet *pkt = new Packet(&req, MemCmd::InvalidationReq);
            // send as a snoop request
            (*p)->sendTimingSnoopReq(pkt);
        }
    }
}
