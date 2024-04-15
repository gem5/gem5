/*
 * Copyright (c) 2012-2013,2020-2021 ARM Limited
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
 * Copyright (c) 2009-2013 Advanced Micro Devices, Inc.
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

#include "mem/ruby/system/RubyPort.hh"

#include "base/compiler.hh"
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/Config.hh"
#include "debug/Drain.hh"
#include "debug/Ruby.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/simple_mem.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

RubyPort::RubyPort(const Params &p)
    : ClockedObject(p),
      m_ruby_system(p.ruby_system),
      m_version(p.version),
      m_controller(NULL),
      m_mandatory_q_ptr(NULL),
      m_usingRubyTester(p.using_ruby_tester),
      system(p.system),
      pioRequestPort(csprintf("%s.pio-request-port", name()), *this),
      pioResponsePort(csprintf("%s.pio-response-port", name()), *this),
      memRequestPort(csprintf("%s.mem-request-port", name()), *this),
      memResponsePort(csprintf("%s-mem-response-port", name()), *this,
                      p.ruby_system->getAccessBackingStore(), -1,
                      p.no_retry_on_stall),
      gotAddrRanges(p.port_interrupt_out_port_connection_count),
      m_isCPUSequencer(p.is_cpu_sequencer)
{
    assert(m_version != -1);

    // create the response ports based on the number of connected ports
    for (size_t i = 0; i < p.port_in_ports_connection_count; ++i) {
        response_ports.push_back(new MemResponsePort(
            csprintf("%s.response_ports%d", name(), i), *this,
            p.ruby_system->getAccessBackingStore(), i, p.no_retry_on_stall));
    }

    // create the request ports based on the number of connected ports
    for (size_t i = 0; i < p.port_interrupt_out_port_connection_count; ++i) {
        request_ports.push_back(new PioRequestPort(
            csprintf("%s.request_ports%d", name(), i), *this));
    }
}

void
RubyPort::init()
{
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
    for (const auto &response_port : response_ports)
        response_port->sendRangeChange();
    if (gotAddrRanges == 0 && FullSystem) {
        pioResponsePort.sendRangeChange();
    }
}

Port &
RubyPort::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_request_port") {
        return memRequestPort;
    } else if (if_name == "pio_request_port") {
        return pioRequestPort;
    } else if (if_name == "mem_response_port") {
        return memResponsePort;
    } else if (if_name == "pio_response_port") {
        return pioResponsePort;
    } else if (if_name == "interrupt_out_port") {
        // used by the x86 CPUs to connect the interrupt PIO and interrupt
        // response port
        if (idx >= static_cast<PortID>(request_ports.size())) {
            panic("%s: unknown %s index (%d)\n", __func__, if_name, idx);
        }

        return *request_ports[idx];
    } else if (if_name == "in_ports") {
        // used by the CPUs to connect the caches to the interconnect, and
        // for the x86 case also the interrupt request port
        if (idx >= static_cast<PortID>(response_ports.size())) {
            panic("%s: unknown %s index (%d)\n", __func__, if_name, idx);
        }

        return *response_ports[idx];
    }

    // pass it along to our super class
    return ClockedObject::getPort(if_name, idx);
}

RubyPort::PioRequestPort::PioRequestPort(const std::string &_name,
                                         RubyPort &_port)
    : QueuedRequestPort(_name, reqQueue, snoopRespQueue),
      owner{ _port },
      reqQueue(_port, *this),
      snoopRespQueue(_port, *this)
{
    DPRINTF(RubyPort, "Created request pioport on sequencer %s\n", _name);
}

RubyPort::PioResponsePort::PioResponsePort(const std::string &_name,
                                           RubyPort &_port)
    : QueuedResponsePort(_name, queue), owner{ _port }, queue(_port, *this)
{
    DPRINTF(RubyPort, "Created response pioport on sequencer %s\n", _name);
}

RubyPort::MemRequestPort::MemRequestPort(const std::string &_name,
                                         RubyPort &_port)
    : QueuedRequestPort(_name, reqQueue, snoopRespQueue),
      owner{ _port },
      reqQueue(_port, *this),
      snoopRespQueue(_port, *this)
{
    DPRINTF(RubyPort, "Created request memport on ruby sequencer %s\n", _name);
}

RubyPort::MemResponsePort::MemResponsePort(const std::string &_name,
                                           RubyPort &_port,
                                           bool _access_backing_store,
                                           PortID id, bool _no_retry_on_stall)
    : QueuedResponsePort(_name, queue, id),
      owner{ _port },
      queue(_port, *this),
      access_backing_store(_access_backing_store),
      no_retry_on_stall(_no_retry_on_stall)
{
    DPRINTF(RubyPort, "Created response memport on ruby sequencer %s\n",
            _name);
}

bool
RubyPort::PioRequestPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Response for address: 0x%#x\n", pkt->getAddr());

    // send next cycle
    owner.pioResponsePort.schedTimingResp(
        pkt, curTick() + owner.m_ruby_system->clockPeriod());
    return true;
}

bool
RubyPort::MemRequestPort::recvTimingResp(PacketPtr pkt)
{
    // got a response from a device
    assert(pkt->isResponse());
    assert(!pkt->htmTransactionFailedInCache());

    // First we must retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->popSenderState());
    MemResponsePort *port = senderState->port;
    assert(port != NULL);
    delete senderState;

    // In FS mode, ruby memory will receive pio responses from devices
    // and it must forward these responses back to the particular CPU.
    DPRINTF(RubyPort, "Pio response for address %#x, going to %s\n",
            pkt->getAddr(), port->name());

    // attempt to send the response in the next cycle
    port->schedTimingResp(pkt, curTick() + owner.m_ruby_system->clockPeriod());

    return true;
}

bool
RubyPort::PioResponsePort::recvTimingReq(PacketPtr pkt)
{
    for (size_t i = 0; i < owner.request_ports.size(); ++i) {
        AddrRangeList l = owner.request_ports[i]->getAddrRanges();
        for (auto it = l.begin(); it != l.end(); ++it) {
            if (it->contains(pkt->getAddr())) {
                // generally it is not safe to assume success here as
                // the port could be blocked
                [[maybe_unused]] bool success =
                    owner.request_ports[i]->sendTimingReq(pkt);
                assert(success);
                return true;
            }
        }
    }
    panic("Should never reach here!\n");
}

Tick
RubyPort::PioResponsePort::recvAtomic(PacketPtr pkt)
{
    // Only atomic_noncaching mode supported!
    if (!owner.system->bypassCaches()) {
        panic("Ruby supports atomic accesses only in noncaching mode\n");
    }

    for (size_t i = 0; i < owner.request_ports.size(); ++i) {
        AddrRangeList l = owner.request_ports[i]->getAddrRanges();
        for (auto it = l.begin(); it != l.end(); ++it) {
            if (it->contains(pkt->getAddr())) {
                return owner.request_ports[i]->sendAtomic(pkt);
            }
        }
    }
    panic("Could not find address in Ruby PIO address ranges!\n");
}

bool
RubyPort::MemResponsePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Timing request for address %#x on port %d\n",
            pkt->getAddr(), id);

    if (pkt->cacheResponding())
        panic("RubyPort should never see request with the "
              "cacheResponding flag set\n");

    // ruby doesn't support cache maintenance operations at the
    // moment, as a workaround, we respond right away
    if (pkt->req->isCacheMaintenance()) {
        warn_once("Cache maintenance operations are not supported in Ruby.\n");
        pkt->makeResponse();
        schedTimingResp(pkt, curTick());
        return true;
    }
    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (pkt->cmd != MemCmd::MemSyncReq) {
        if (!pkt->req->isMemMgmt() && !isPhysMemAddress(pkt)) {
            assert(owner.memRequestPort.isConnected());
            DPRINTF(RubyPort,
                    "Request address %#x assumed to be a "
                    "pio address\n",
                    pkt->getAddr());

            // Save the port in the sender state object to be used later to
            // route the response
            pkt->pushSenderState(new SenderState(this));

            // send next cycle
            RubySystem *rs = owner.m_ruby_system;
            owner.memRequestPort.schedTimingReq(pkt,
                                                curTick() + rs->clockPeriod());
            return true;
        }
    }

    // Save the port in the sender state object to be used later to
    // route the response
    pkt->pushSenderState(new SenderState(this));

    // Submit the ruby request
    RequestStatus requestStatus = owner.makeRequest(pkt);

    // If the request successfully issued then we should return true.
    // Otherwise, we need to tell the port to retry at a later point
    // and return false.
    if (requestStatus == RequestStatus_Issued) {
        DPRINTF(RubyPort, "Request %s 0x%x issued\n", pkt->cmdString(),
                pkt->getAddr());
        return true;
    }

    // pop off sender state as this request failed to issue
    SenderState *ss = safe_cast<SenderState *>(pkt->popSenderState());
    delete ss;

    if (pkt->cmd != MemCmd::MemSyncReq) {
        DPRINTF(RubyPort,
                "Request %s for address %#x did not issue because %s\n",
                pkt->cmdString(), pkt->getAddr(),
                RequestStatus_to_string(requestStatus));
    }

    addToRetryList();

    return false;
}

Tick
RubyPort::MemResponsePort::recvAtomic(PacketPtr pkt)
{
    // Only atomic_noncaching mode supported!
    if (!owner.system->bypassCaches()) {
        panic("Ruby supports atomic accesses only in noncaching mode\n");
    }

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (pkt->cmd != MemCmd::MemSyncReq) {
        if (!isPhysMemAddress(pkt)) {
            assert(owner.memRequestPort.isConnected());
            DPRINTF(RubyPort,
                    "Request address %#x assumed to be a "
                    "pio address\n",
                    pkt->getAddr());

            // Save the port in the sender state object to be used later to
            // route the response
            pkt->pushSenderState(new SenderState(this));

            // send next cycle
            Tick req_ticks = owner.memRequestPort.sendAtomic(pkt);
            return owner.ticksToCycles(req_ticks);
        }

        assert(getOffset(pkt->getAddr()) + pkt->getSize() <=
               RubySystem::getBlockSizeBytes());
    }

    // Find the machine type of memory controller interface
    RubySystem *rs = owner.m_ruby_system;
    static int mem_interface_type = -1;
    if (mem_interface_type == -1) {
        if (rs->m_abstract_controls[MachineType_Directory].size() != 0) {
            mem_interface_type = MachineType_Directory;
        } else if (rs->m_abstract_controls[MachineType_Memory].size() != 0) {
            mem_interface_type = MachineType_Memory;
        } else {
            panic("Can't find the memory controller interface\n");
        }
    }

    // Find the controller for the target address
    MachineID id = owner.m_controller->mapAddressToMachine(
        pkt->getAddr(), (MachineType)mem_interface_type);
    AbstractController *mem_interface =
        rs->m_abstract_controls[mem_interface_type][id.getNum()];
    Tick latency = mem_interface->recvAtomic(pkt);
    if (access_backing_store)
        rs->getPhysMem()->access(pkt);
    return latency;
}

void
RubyPort::MemResponsePort::addToRetryList()
{
    //
    // Unless the request port do not want retries (e.g., the Ruby tester),
    // record the stalled M5 port for later retry when the sequencer
    // becomes free.
    //
    if (!no_retry_on_stall && !owner.onRetryList(this)) {
        owner.addToRetryList(this);
    }
}

void
RubyPort::MemResponsePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Functional access for address: %#x\n", pkt->getAddr());

    RubySystem *rs = owner.m_ruby_system;

    // Check for pio requests and directly send them to the dedicated
    // pio port.
    if (!isPhysMemAddress(pkt)) {
        DPRINTF(RubyPort, "Pio Request for address: 0x%#x\n", pkt->getAddr());
        assert(owner.pioRequestPort.isConnected());
        owner.pioRequestPort.sendFunctional(pkt);
        return;
    }

    assert(pkt->getAddr() + pkt->getSize() <=
           makeLineAddress(pkt->getAddr()) + RubySystem::getBlockSizeBytes());

    if (access_backing_store) {
        // The attached physmem contains the official version of data.
        // The following command performs the real functional access.
        // This line should be removed once Ruby supplies the official version
        // of data.
        rs->getPhysMem()->functionalAccess(pkt);
    } else {
        bool accessSucceeded = false;
        bool needsResponse = pkt->needsResponse();

        // Do the functional access on ruby memory
        if (pkt->isRead()) {
            accessSucceeded = rs->functionalRead(pkt);
        } else if (pkt->isWrite()) {
            accessSucceeded = rs->functionalWrite(pkt);
        } else {
            panic("Unsupported functional command %s\n", pkt->cmdString());
        }

        // Unless the request port explicitly said otherwise, generate an error
        // if the functional request failed
        if (!accessSucceeded && !pkt->suppressFuncError()) {
            fatal("Ruby functional %s failed for address %#x\n",
                  pkt->isWrite() ? "write" : "read", pkt->getAddr());
        }

        // turn packet around to go back to request port if response expected
        if (needsResponse) {
            // The pkt is already turned into a reponse if the directory
            // forwarded the request to the memory controller (see
            // AbstractController::functionalMemoryWrite and
            // AbstractMemory::functionalAccess)
            if (!pkt->isResponse())
                pkt->makeResponse();
            pkt->setFunctionalResponseStatus(accessSucceeded);
        }

        DPRINTF(RubyPort, "Functional access %s!\n",
                accessSucceeded ? "successful" : "failed");
    }
}

void
RubyPort::ruby_hit_callback(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Hit callback for %s 0x%x\n", pkt->cmdString(),
            pkt->getAddr());

    // The packet was destined for memory and has not yet been turned
    // into a response
    assert(system->isMemAddr(pkt->getAddr()) || system->isDeviceMemAddr(pkt));
    assert(pkt->isRequest());

    // First we must retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->popSenderState());
    MemResponsePort *port = senderState->port;
    assert(port != NULL);
    delete senderState;

    port->hitCallback(pkt);

    trySendRetries();
}

void
RubyPort::ruby_unaddressed_callback(PacketPtr pkt)
{
    DPRINTF(RubyPort, "Unaddressed callback for %s\n", pkt->cmdString());

    assert(pkt->isRequest());

    // First we must retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->popSenderState());
    MemResponsePort *port = senderState->port;
    assert(port != NULL);
    delete senderState;

    port->hitCallback(pkt);

    trySendRetries();
}

void
RubyPort::ruby_stale_translation_callback(Addr txnId)
{
    DPRINTF(RubyPort, "Stale Translation Callback\n");

    // Allocate the invalidate request and packet on the stack, as it is
    // assumed they will not be modified or deleted by receivers.
    // TODO: should this really be using funcRequestorId?
    auto request = std::make_shared<Request>(
        0, RubySystem::getBlockSizeBytes(), Request::TLBI_EXT_SYNC,
        Request::funcRequestorId);
    // Store the txnId in extraData instead of the address
    request->setExtraData(txnId);

    // Use a single packet to signal all snooping ports of the external sync.
    // This assumes that snooping ports do NOT modify the packet/request
    // TODO rename TlbiExtSync to StaleTranslation
    Packet pkt(request, MemCmd::TlbiExtSync);
    // TODO - see where response_ports is filled, might be we only want to send
    // to specific places
    for (auto &port : response_ports) {
        // check if the connected request port is snooping
        if (port->isSnooping()) {
            // send as a snoop request
            port->sendTimingSnoopReq(&pkt);
        }
    }
}

void
RubyPort::trySendRetries()
{
    //
    // If we had to stall the MemResponsePorts, wake them up because the
    // sequencer likely has free resources now.
    //
    if (!retryList.empty()) {
        // Record the current list of ports to retry on a temporary list
        // before calling sendRetryReq on those ports. sendRetryReq will cause
        // an immediate retry, which may result in the ports being put back on
        // the list. Therefore we want to clear the retryList before calling
        // sendRetryReq.
        std::vector<MemResponsePort *> curRetryList(retryList);

        retryList.clear();

        for (auto i = curRetryList.begin(); i != curRetryList.end(); ++i) {
            DPRINTF(RubyPort,
                    "Sequencer may now be free. SendRetry to port %s\n",
                    (*i)->name());
            (*i)->sendRetryReq();
        }
    }
}

void
RubyPort::testDrainComplete()
{
    // If we weren't able to drain before, we might be able to now.
    if (drainState() == DrainState::Draining) {
        unsigned int drainCount = outstandingCount();
        DPRINTF(Drain, "Drain count: %u\n", drainCount);
        if (drainCount == 0) {
            DPRINTF(Drain, "RubyPort done draining, signaling drain done\n");
            signalDrainDone();
        }
    }
}

DrainState
RubyPort::drain()
{
    if (isDeadlockEventScheduled()) {
        descheduleDeadlockEvent();
    }

    //
    // If the RubyPort is not empty, then it needs to clear all outstanding
    // requests before it should call signalDrainDone()
    //
    DPRINTF(Config, "outstanding count %d\n", outstandingCount());
    if (outstandingCount() > 0) {
        DPRINTF(Drain, "RubyPort not drained\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
RubyPort::MemResponsePort::hitCallback(PacketPtr pkt)
{
    bool needsResponse = pkt->needsResponse();

    // Unless specified at configuration, all responses except failed SC
    // and Flush operations access M5 physical memory.
    bool accessPhysMem = access_backing_store;

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

    // Flush, acquire, release requests don't access physical memory
    if (pkt->isFlush() || pkt->cmd == MemCmd::MemSyncReq ||
        pkt->cmd == MemCmd::WriteCompleteResp) {
        accessPhysMem = false;
    }

    if (pkt->req->isKernel()) {
        accessPhysMem = false;
        needsResponse = true;
    }

    DPRINTF(RubyPort, "Hit callback needs response %d\n", needsResponse);

    RubySystem *rs = owner.m_ruby_system;
    if (accessPhysMem) {
        // We must check device memory first in case it overlaps with the
        // system memory range.
        if (owner.system->isDeviceMemAddr(pkt)) {
            auto dmem = owner.system->getDeviceMemory(pkt);
            dmem->access(pkt);
        } else if (owner.system->isMemAddr(pkt->getAddr())) {
            rs->getPhysMem()->access(pkt);
        } else {
            panic("Packet is in neither device nor system memory!");
        }
    } else if (needsResponse) {
        pkt->makeResponse();
    }

    // turn packet around to go back to request port if response expected
    if (needsResponse || pkt->isResponse()) {
        DPRINTF(RubyPort, "Sending packet back over port\n");
        // Send a response in the same cycle. There is no need to delay the
        // response because the response latency is already incurred in the
        // Ruby protocol.
        schedTimingResp(pkt, curTick());
    } else {
        delete pkt;
    }

    DPRINTF(RubyPort, "Hit callback done!\n");
}

AddrRangeList
RubyPort::PioResponsePort::getAddrRanges() const
{
    // at the moment the assumption is that the request port does not care
    AddrRangeList ranges;

    for (size_t i = 0; i < owner.request_ports.size(); ++i) {
        ranges.splice(ranges.begin(), owner.request_ports[i]->getAddrRanges());
    }
    for ([[maybe_unused]] const auto &r : ranges)
        DPRINTF(RubyPort, "%s\n", r.to_string());
    return ranges;
}

bool
RubyPort::MemResponsePort::isShadowRomAddress(Addr addr) const
{
    AddrRangeList ranges = owner.system->getShadowRomRanges();

    for (auto it = ranges.begin(); it != ranges.end(); ++it) {
        if (it->contains(addr)) {
            return true;
        }
    }

    return false;
}

bool
RubyPort::MemResponsePort::isPhysMemAddress(PacketPtr pkt) const
{
    Addr addr = pkt->getAddr();
    return (owner.system->isMemAddr(addr) && !isShadowRomAddress(addr)) ||
           owner.system->isDeviceMemAddr(pkt);
}

void
RubyPort::ruby_eviction_callback(Addr address)
{
    DPRINTF(RubyPort, "Sending invalidations.\n");
    // Allocate the invalidate request and packet on the stack, as it is
    // assumed they will not be modified or deleted by receivers.
    // TODO: should this really be using funcRequestorId?
    auto request = std::make_shared<Request>(
        address, RubySystem::getBlockSizeBytes(), 0, Request::funcRequestorId);

    // Use a single packet to signal all snooping ports of the invalidation.
    // This assumes that snooping ports do NOT modify the packet/request
    Packet pkt(request, MemCmd::InvalidateReq);
    for (CpuPortIter p = response_ports.begin(); p != response_ports.end();
         ++p) {
        // check if the connected request port is snooping
        if ((*p)->isSnooping()) {
            // send as a snoop request
            (*p)->sendTimingSnoopReq(&pkt);
        }
    }
}

void
RubyPort::PioRequestPort::recvRangeChange()
{
    RubyPort &r = static_cast<RubyPort &>(owner);
    r.gotAddrRanges--;
    if (r.gotAddrRanges == 0 && FullSystem) {
        r.pioResponsePort.sendRangeChange();
    }
}

int
RubyPort::functionalWrite(Packet *func_pkt)
{
    int num_written = 0;
    for (auto port : response_ports) {
        if (port->trySatisfyFunctional(func_pkt)) {
            num_written += 1;
        }
    }
    return num_written;
}

} // namespace ruby
} // namespace gem5
