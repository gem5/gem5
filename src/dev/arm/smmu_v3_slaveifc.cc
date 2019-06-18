/*
 * Copyright (c) 2019 ARM Limited
 * All rights reserved
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
 * Authors: Stan Czerniawski
 *          Giacomo Travaglini
 */

#include "dev/arm/smmu_v3_slaveifc.hh"

#include "debug/SMMUv3.hh"
#include "dev/arm/smmu_v3.hh"
#include "dev/arm/smmu_v3_transl.hh"

SMMUv3SlaveInterface::SMMUv3SlaveInterface(
    const SMMUv3SlaveInterfaceParams *p) :
    MemObject(p),
    smmu(nullptr),
    microTLB(new SMMUTLB(p->utlb_entries,
                         p->utlb_assoc,
                         p->utlb_policy)),
    mainTLB(new SMMUTLB(p->tlb_entries,
                        p->tlb_assoc,
                        p->tlb_policy)),
    microTLBEnable(p->utlb_enable),
    mainTLBEnable(p->tlb_enable),
    slavePortSem(1),
    microTLBSem(p->utlb_slots),
    mainTLBSem(p->tlb_slots),
    microTLBLat(p->utlb_lat),
    mainTLBLat(p->tlb_lat),
    slavePort(new SMMUSlavePort(csprintf("%s.slave", name()), *this)),
    atsSlavePort(name() + ".atsSlave", *this),
    atsMasterPort(name() + ".atsMaster", *this),
    portWidth(p->port_width),
    wrBufSlotsRemaining(p->wrbuf_slots),
    xlateSlotsRemaining(p->xlate_slots),
    prefetchEnable(p->prefetch_enable),
    prefetchReserveLastWay(
        p->prefetch_reserve_last_way),
    deviceNeedsRetry(false),
    atsDeviceNeedsRetry(false),
    sendDeviceRetryEvent(*this),
    atsSendDeviceRetryEvent(this)
{}

void
SMMUv3SlaveInterface::sendRange()
{
    if (slavePort->isConnected()) {
        inform("Slave port is connected to %d\n",
                slavePort->getMasterPort().name());

        slavePort->sendRangeChange();
    } else {
        fatal("Slave port is not connected.\n");
    }
}

Port&
SMMUv3SlaveInterface::getPort(const std::string &name, PortID id)
{
    if (name == "ats_master") {
        return atsMasterPort;
    } else if (name == "slave") {
        return *slavePort;
    } else if (name == "ats_slave") {
        return atsSlavePort;
    } else {
        return MemObject::getPort(name, id);
    }
}

void
SMMUv3SlaveInterface::schedTimingResp(PacketPtr pkt)
{
    slavePort->schedTimingResp(pkt, nextCycle());
}

void
SMMUv3SlaveInterface::schedAtsTimingResp(PacketPtr pkt)
{
    atsSlavePort.schedTimingResp(pkt, nextCycle());

    if (atsDeviceNeedsRetry) {
        atsDeviceNeedsRetry = false;
        schedule(atsSendDeviceRetryEvent, nextCycle());
    }
}

Tick
SMMUv3SlaveInterface::recvAtomic(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[a] req from %s addr=%#x size=%#x\n",
            slavePort->getMasterPort().name(),
            pkt->getAddr(), pkt->getSize());

    std::string proc_name = csprintf("%s.port", name());
    SMMUTranslationProcess proc(proc_name, *smmu, *this);
    proc.beginTransaction(SMMUTranslRequest::fromPacket(pkt));

    SMMUAction a = smmu->runProcessAtomic(&proc, pkt);
    assert(a.type == ACTION_SEND_RESP);

    return a.delay;
}

bool
SMMUv3SlaveInterface::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[t] req from %s addr=%#x size=%#x\n",
            slavePort->getMasterPort().name(),
            pkt->getAddr(), pkt->getSize());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    unsigned nbeats =
        (pkt->getSize() + (portWidth-1)) / portWidth;

    if (xlateSlotsRemaining==0 ||
        (pkt->isWrite() && wrBufSlotsRemaining < nbeats))
    {
        deviceNeedsRetry = true;
        return false;
    }

    if (pkt->isWrite())
        wrBufSlotsRemaining -= nbeats;

    std::string proc_name = csprintf("%s.port", name());
    SMMUTranslationProcess *proc =
        new SMMUTranslationProcess(proc_name, *smmu, *this);
    proc->beginTransaction(SMMUTranslRequest::fromPacket(pkt));

    smmu->runProcessTiming(proc, pkt);

    return true;
}

Tick
SMMUv3SlaveInterface::atsSlaveRecvAtomic(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[a] ATS slave  req  addr=%#x size=%#x\n",
            pkt->getAddr(), pkt->getSize());

    std::string proc_name = csprintf("%s.atsport", name());
    const bool ats_request = true;
    SMMUTranslationProcess proc(
        proc_name, *smmu, *this);
    proc.beginTransaction(SMMUTranslRequest::fromPacket(pkt, ats_request));

    SMMUAction a = smmu->runProcessAtomic(&proc, pkt);
    assert(a.type == ACTION_SEND_RESP_ATS);

    return a.delay;
}

bool
SMMUv3SlaveInterface::atsSlaveRecvTimingReq(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[t] ATS slave  req  addr=%#x size=%#x\n",
            pkt->getAddr(), pkt->getSize());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    if (xlateSlotsRemaining == 0) {
        deviceNeedsRetry = true;
        return false;
    }

    std::string proc_name = csprintf("%s.atsport", name());
    const bool ats_request = true;
    SMMUTranslationProcess *proc =
        new SMMUTranslationProcess(proc_name, *smmu, *this);
    proc->beginTransaction(SMMUTranslRequest::fromPacket(pkt, ats_request));

    smmu->runProcessTiming(proc, pkt);

    return true;
}

bool
SMMUv3SlaveInterface::atsMasterRecvTimingResp(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[t] ATS master resp addr=%#x size=%#x\n",
            pkt->getAddr(), pkt->getSize());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    SMMUProcess *proc =
        safe_cast<SMMUProcess *>(pkt->popSenderState());

    smmu->runProcessTiming(proc, pkt);

    return true;
}

void
SMMUv3SlaveInterface::sendDeviceRetry()
{
    slavePort->sendRetryReq();
}

void
SMMUv3SlaveInterface::atsSendDeviceRetry()
{
    DPRINTF(SMMUv3, "ATS retry\n");
    atsSlavePort.sendRetryReq();
}

void
SMMUv3SlaveInterface::scheduleDeviceRetry()
{
    if (deviceNeedsRetry && !sendDeviceRetryEvent.scheduled()) {
        DPRINTF(SMMUv3, "sched slave retry\n");
        deviceNeedsRetry = false;
        schedule(sendDeviceRetryEvent, nextCycle());
    }
}

DrainState
SMMUv3SlaveInterface::drain()
{
    // Wait until all SMMU translations are completed
    if (xlateSlotsRemaining < params()->xlate_slots) {
        return DrainState::Draining;
    }
    return DrainState::Drained;
}

SMMUv3SlaveInterface*
SMMUv3SlaveInterfaceParams::create()
{
    return new SMMUv3SlaveInterface(this);
}
