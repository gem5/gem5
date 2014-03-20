/*
 * Copyright (c) 2012-2013 ARM Limited
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
#include "debug/Drain.hh"
#include "debug/Ruby.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

RubyPort::RubyPort(const Params *p)
    : MemObject(p), m_version(p->version), m_controller(NULL),
      m_mandatory_q_ptr(NULL), m_usingRubyTester(p->using_ruby_tester),
      pioMasterPort(csprintf("%s.pio-master-port", name()), this),
      pioSlavePort(csprintf("%s.pio-slave-port", name()), this),
      memMasterPort(csprintf("%s.mem-master-port", name()), this),
      memSlavePort(csprintf("%s-mem-slave-port", name()), this,
          p->ruby_system, p->access_phys_mem, -1),
      gotAddrRanges(p->port_master_connection_count), drainManager(NULL),
      system(p->system), access_phys_mem(p->access_phys_mem)
{
    assert(m_version != -1);

    // create the slave ports based on the number of connected ports
    for (size_t i = 0; i < p->port_slave_connection_count; ++i) {
        slave_ports.push_back(new MemSlavePort(csprintf("%s.slave%d", name(),
            i), this, p->ruby_system, access_phys_mem, i));
    }

    // create the master ports based on the number of connected ports
    for (size_t i = 0; i < p->port_master_connection_count; ++i) {
        master_ports.push_back(new PioMasterPort(csprintf("%s.master%d",
            name(), i), this));
    }
}

void
RubyPort::init()
{
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
    m_mandatory_q_ptr->setSender(this);
}

BaseMasterPort &
RubyPort::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_master_port") {
        return memMasterPort;
    }

    if (if_name == "pio_master_port") {
        return pioMasterPort;
    }

    // used by the x86 CPUs to connect the interrupt PIO and interrupt slave
    // port
    if (if_name != "master") {
        // pass it along to our super class
        return MemObject::getMasterPort(if_name, idx);
    } else {
        if (idx >= static_cast<PortID>(master_ports.size())) {
            panic("RubyPort::getMasterPort: unknown index %d\n", idx);
        }

        return *master_ports[idx];
    }
}

BaseSlavePort &
RubyPort::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_slave_port") {
        return memSlavePort;
    }

    if (if_name == "pio_slave_port")
        return pioSlavePort;

    // used by the CPUs to connect the caches to the interconnect, and
    // for the x86 case also the interrupt master
    if (if_name != "slave") {
        // pass it along to our super class
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<PortID>(slave_ports.size())) {
            panic("RubyPort::getSlavePort: unknown index %d\n", idx);
        }

        return *slave_ports[idx];
    }
}

RubyPort::PioMasterPort::PioMasterPort(const std::string &_name,
                           RubyPort *_port)
    : QueuedMasterPort(_name, _port, queue), queue(*_port, *this)
{
    DPRINTF(RubyPort, "Created master pioport on sequencer %s\n", _name);
}

RubyPort::PioSlavePort::PioSlavePort(const std::string &_name,
                           RubyPort *_port)
    : QueuedSlavePort(_name, _port, queue), queue(*_port, *this)
{
    DPRINTF(RubyPort, "Created slave pioport on sequencer %s\n", _name);
}

RubyPort::MemMasterPort::MemMasterPort(const std::string &_name,
                           RubyPort *_port)
    : QueuedMasterPort(_name, _port, queue), queue(*_port, *this)
{
    DPRINTF(RubyPort, "Created master memport on ruby sequencer %s\n", _name);
}

RubyPort::MemSlavePort::MemSlavePort(const std::string &_name, RubyPort *_port,
                         RubySystem *_system, bool _access_phys_mem, PortID id)
    : QueuedSlavePort(_name, _port, queue, id), queue(*_port, *this),
      ruby_system(_system), access_phys_mem(_access_phys_mem)
{
    DPRINTF(RubyPort, "Created slave memport on ruby sequencer %s\n", _name);
}

bool
RubyPort::PioMasterPort::recvTimingResp(PacketPtr pkt)
{
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);
    DPRINTF(RubyPort, "Response for address: 0x%#x\n", pkt->getAddr());

    // send next cycle
    ruby_port->pioSlavePort.schedTimingResp(
            pkt, curTick() + g_system_ptr->clockPeriod());
    return true;
}

bool RubyPort::MemMasterPort::recvTimingResp(PacketPtr pkt)
{
    // got a response from a device
    assert(pkt->isResponse());

    // In FS mode, ruby memory will receive pio responses from devices
    // and it must forward these responses back to the particular CPU.
    DPRINTF(RubyPort,  "Pio response for address %#x, going to %d\n",
            pkt->getAddr(), pkt->getDest());

    // First we must retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->popSenderState());
    MemSlavePort *port = senderState->port;
    assert(port != NULL);
    delete senderState;

    // attempt to send the response in the next cycle
    port->schedTimingResp(pkt, curTick() + g_system_ptr->clockPeriod());

    return true;
}

bool
RubyPort::PioSlavePort::recvTimingReq(PacketPtr pkt)
{
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);

    for (size_t i = 0; i < ruby_port->master_ports.size(); ++i) {
        AddrRangeList l = ruby_port->master_ports[i]->getAddrRanges();
        for (auto it = l.begin(); it != l.end(); ++it) {
            if (it->contains(pkt->getAddr())) {
                ruby_port->master_ports[i]->sendTimingReq(pkt);
                return true;
            }
        }
    }
    panic("Should never reach here!\n");
}

bool
RubyPort::MemSlavePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Timing request for address %#x on port %d\n",
            pkt->getAddr(), id);
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);

    if (pkt->memInhibitAsserted())
        panic("RubyPort should never see an inhibited request\n");

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (!isPhysMemAddress(pkt->getAddr())) {
        assert(ruby_port->memMasterPort.isConnected());
        DPRINTF(RubyPort, "Request address %#x assumed to be a pio address\n",
                pkt->getAddr());

        // Save the port in the sender state object to be used later to
        // route the response
        pkt->pushSenderState(new SenderState(this));

        // send next cycle
        ruby_port->memMasterPort.schedTimingReq(pkt,
            curTick() + g_system_ptr->clockPeriod());
        return true;
    }

    // Save the port id to be used later to route the response
    pkt->setSrc(id);

    assert(Address(pkt->getAddr()).getOffset() + pkt->getSize() <=
           RubySystem::getBlockSizeBytes());

    // Submit the ruby request
    RequestStatus requestStatus = ruby_port->makeRequest(pkt);

    // If the request successfully issued then we should return true.
    // Otherwise, we need to tell the port to retry at a later point
    // and return false.
    if (requestStatus == RequestStatus_Issued) {
        DPRINTF(RubyPort, "Request %s 0x%x issued\n", pkt->cmdString(),
                pkt->getAddr());
        return true;
    }

    //
    // Unless one is using the ruby tester, record the stalled M5 port for 
    // later retry when the sequencer becomes free.
    //
    if (!ruby_port->m_usingRubyTester) {
        ruby_port->addToRetryList(this);
    }

    DPRINTF(RubyPort, "Request for address %#x did not issued because %s\n",
            pkt->getAddr(), RequestStatus_to_string(requestStatus));

    return false;
}

void
RubyPort::MemSlavePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Functional access for address: %#x\n", pkt->getAddr());
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (!isPhysMemAddress(pkt->getAddr())) {
        assert(ruby_port->memMasterPort.isConnected());
        DPRINTF(RubyPort, "Pio Request for address: 0x%#x\n", pkt->getAddr());
        panic("RubyPort::PioMasterPort::recvFunctional() not implemented!\n");
    }

    assert(pkt->getAddr() + pkt->getSize() <=
                line_address(Address(pkt->getAddr())).getAddress() +
                RubySystem::getBlockSizeBytes());

    bool accessSucceeded = false;
    bool needsResponse = pkt->needsResponse();

    // Do the functional access on ruby memory
    if (pkt->isRead()) {
        accessSucceeded = ruby_system->functionalRead(pkt);
    } else if (pkt->isWrite()) {
        accessSucceeded = ruby_system->functionalWrite(pkt);
    } else {
        panic("Unsupported functional command %s\n", pkt->cmdString());
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
    DPRINTF(RubyPort, "Hit callback for %s 0x%x\n", pkt->cmdString(),
            pkt->getAddr());

    // The packet was destined for memory and has not yet been turned
    // into a response
    assert(system->isMemAddr(pkt->getAddr()));
    assert(pkt->isRequest());

    // As it has not yet been turned around, the source field tells us
    // which port it came from.
    assert(pkt->getSrc() < slave_ports.size());

    slave_ports[pkt->getSrc()]->hitCallback(pkt);

    //
    // If we had to stall the MemSlavePorts, wake them up because the sequencer
    // likely has free resources now.
    //
    if (!retryList.empty()) {
        //
        // Record the current list of ports to retry on a temporary list before
        // calling sendRetry on those ports.  sendRetry will cause an 
        // immediate retry, which may result in the ports being put back on the
        // list. Therefore we want to clear the retryList before calling
        // sendRetry.
        //
        std::vector<MemSlavePort *> curRetryList(retryList);

        retryList.clear();

        for (auto i = curRetryList.begin(); i != curRetryList.end(); ++i) {
            DPRINTF(RubyPort,
                    "Sequencer may now be free.  SendRetry to port %s\n",
                    (*i)->name());
            (*i)->sendRetry();
        }
    }

    testDrainComplete();
}

void
RubyPort::testDrainComplete()
{
    //If we weren't able to drain before, we might be able to now.
    if (drainManager != NULL) {
        unsigned int drainCount = outstandingCount();
        DPRINTF(Drain, "Drain count: %u\n", drainCount);
        if (drainCount == 0) {
            DPRINTF(Drain, "RubyPort done draining, signaling drain done\n");
            drainManager->signalDrainDone();
            // Clear the drain manager once we're done with it.
            drainManager = NULL;
        }
    }
}

unsigned int
RubyPort::getChildDrainCount(DrainManager *dm)
{
    int count = 0;

    if (memMasterPort.isConnected()) {
        count += memMasterPort.drain(dm);
        DPRINTF(Config, "count after pio check %d\n", count);
    }

    for (CpuPortIter p = slave_ports.begin(); p != slave_ports.end(); ++p) {
        count += (*p)->drain(dm);
        DPRINTF(Config, "count after slave port check %d\n", count);
    }

    for (std::vector<PioMasterPort *>::iterator p = master_ports.begin();
         p != master_ports.end(); ++p) {
        count += (*p)->drain(dm);
        DPRINTF(Config, "count after master port check %d\n", count);
    }

    DPRINTF(Config, "final count %d\n", count);
    return count;
}

unsigned int
RubyPort::drain(DrainManager *dm)
{
    if (isDeadlockEventScheduled()) {
        descheduleDeadlockEvent();
    }

    //
    // If the RubyPort is not empty, then it needs to clear all outstanding
    // requests before it should call drainManager->signalDrainDone()
    //
    DPRINTF(Config, "outstanding count %d\n", outstandingCount());
    bool need_drain = outstandingCount() > 0;

    //
    // Also, get the number of child ports that will also need to clear
    // their buffered requests before they call drainManager->signalDrainDone()
    //
    unsigned int child_drain_count = getChildDrainCount(dm);

    // Set status
    if (need_drain) {
        drainManager = dm;

        DPRINTF(Drain, "RubyPort not drained\n");
        setDrainState(Drainable::Draining);
        return child_drain_count + 1;
    }

    drainManager = NULL;
    setDrainState(Drainable::Drained);
    return child_drain_count;
}

void
RubyPort::MemSlavePort::hitCallback(PacketPtr pkt)
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
        RubyPort *ruby_port = static_cast<RubyPort *>(&owner);
        ruby_port->system->getPhysMem().access(pkt);
    } else if (needsResponse) {
        pkt->makeResponse();
    }

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        DPRINTF(RubyPort, "Sending packet back over port\n");
        // send next cycle
        schedTimingResp(pkt, curTick() + g_system_ptr->clockPeriod());
    } else {
        delete pkt;
    }
    DPRINTF(RubyPort, "Hit callback done!\n");
}

AddrRangeList
RubyPort::PioSlavePort::getAddrRanges() const
{
    // at the moment the assumption is that the master does not care
    AddrRangeList ranges;
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);

    for (size_t i = 0; i < ruby_port->master_ports.size(); ++i) {
        ranges.splice(ranges.begin(),
                ruby_port->master_ports[i]->getAddrRanges());
    }
    for (AddrRangeConstIter r = ranges.begin(); r != ranges.end(); ++r)
        DPRINTF(RubyPort, "%s\n", r->to_string());
    return ranges;
}

bool
RubyPort::MemSlavePort::isPhysMemAddress(Addr addr) const
{
    RubyPort *ruby_port = static_cast<RubyPort *>(&owner);
    return ruby_port->system->isMemAddr(addr);
}

void
RubyPort::ruby_eviction_callback(const Address& address)
{
    DPRINTF(RubyPort, "Sending invalidations.\n");
    // This request is deleted in the stack-allocated packet destructor
    // when this function exits
    // TODO: should this really be using funcMasterId?
    RequestPtr req =
            new Request(address.getAddress(), 0, 0, Request::funcMasterId);
    // Use a single packet to signal all snooping ports of the invalidation.
    // This assumes that snooping ports do NOT modify the packet/request
    Packet pkt(req, MemCmd::InvalidationReq);
    for (CpuPortIter p = slave_ports.begin(); p != slave_ports.end(); ++p) {
        // check if the connected master port is snooping
        if ((*p)->isSnooping()) {
            // send as a snoop request
            (*p)->sendTimingSnoopReq(&pkt);
        }
    }
}

void
RubyPort::PioMasterPort::recvRangeChange()
{
    RubyPort &r = static_cast<RubyPort &>(owner);
    r.gotAddrRanges--;
    if (r.gotAddrRanges == 0 && FullSystem) {
        r.pioSlavePort.sendRangeChange();
    }
}
