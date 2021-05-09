/*
 * Copyright (c) 2012-2014 ARM Limited
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
 */

#include "dev/arm/energy_ctrl.hh"

#include "base/trace.hh"
#include "debug/EnergyCtrl.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/EnergyCtrl.hh"
#include "sim/dvfs_handler.hh"
#include "sim/serialize.hh"

namespace gem5
{

EnergyCtrl::EnergyCtrl(const Params &p)
    : BasicPioDevice(p, PIO_NUM_FIELDS * 4),        // each field is 32 bit
      dvfsHandler(p.dvfs_handler),
      domainID(0),
      domainIDIndexToRead(0),
      perfLevelAck(0),
      perfLevelToRead(0),
      updateAckEvent([this]{ updatePLAck(); }, name())
{
    fatal_if(!p.dvfs_handler, "EnergyCtrl: Needs a DVFSHandler for a "
             "functioning system.\n");
}

Tick
EnergyCtrl::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);

    Addr daddr = pkt->getAddr() - pioAddr;
    assert((daddr & 3) == 0);
    Registers reg = Registers(daddr / 4);

    if (!dvfsHandler->isEnabled()) {
        // NB: Zero is a good response if the handler is disabled
        pkt->setLE<uint32_t>(0);
        warn_once("EnergyCtrl: Disabled handler, ignoring read from reg %i\n",
                  reg);
        DPRINTF(EnergyCtrl, "dvfs handler disabled, return 0 for read from "\
                "reg %i\n", reg);
        pkt->makeAtomicResponse();
        return pioDelay;
    }

    uint32_t result = 0;
    Tick period;
    double voltage;

    switch(reg) {
      case DVFS_HANDLER_STATUS:
        result = 1;
        DPRINTF(EnergyCtrl, "dvfs handler enabled\n");
        break;
      case DVFS_NUM_DOMAINS:
        result = dvfsHandler->numDomains();
        DPRINTF(EnergyCtrl, "reading number of domains %d\n", result);
        break;
      case DVFS_DOMAINID_AT_INDEX:
        result = dvfsHandler->domainID(domainIDIndexToRead);
        DPRINTF(EnergyCtrl, "reading domain id at index %d as %d\n",
                domainIDIndexToRead, result);
        break;
      case DVFS_HANDLER_TRANS_LATENCY:
        // Return transition latency in nanoseconds
        result = dvfsHandler->transLatency() / sim_clock::as_int::ns;
        DPRINTF(EnergyCtrl, "reading dvfs handler trans latency %d ns\n",
                result);
        break;
      case DOMAIN_ID:
        result = domainID;
        DPRINTF(EnergyCtrl, "reading domain id:%d\n", result);
        break;
      case PERF_LEVEL:
        result = dvfsHandler->perfLevel(domainID);
        DPRINTF(EnergyCtrl, "reading domain %d perf level: %d\n",
                domainID, result);
        break;
      case PERF_LEVEL_ACK:
        result = perfLevelAck;
        DPRINTF(EnergyCtrl, "reading ack:%d\n", result);
        // Signal is set for a single read only
        if (result == 1)
            perfLevelAck = 0;
        break;
      case NUM_OF_PERF_LEVELS:
        result = dvfsHandler->numPerfLevels(domainID);
        DPRINTF(EnergyCtrl, "reading num of perf level:%d\n", result);
        break;
      case FREQ_AT_PERF_LEVEL:
        period = dvfsHandler->clkPeriodAtPerfLevel(domainID, perfLevelToRead);
        result = ticksTokHz(period);
        DPRINTF(EnergyCtrl, "reading freq %d KHz at perf level: %d\n",
                result, perfLevelToRead);
        break;
      case VOLT_AT_PERF_LEVEL:
        voltage = dvfsHandler->voltageAtPerfLevel(domainID, perfLevelToRead);
        result = toMicroVolt(voltage);
        DPRINTF(EnergyCtrl, "reading voltage %d u-volt at perf level: %d\n",
                result, perfLevelToRead);
        break;
      default:
        panic("Tried to read EnergyCtrl at offset %#x / reg %i\n", daddr,
              reg);
    }
    pkt->setLE<uint32_t>(result);
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
EnergyCtrl::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);

    uint32_t data;
    data = pkt->getLE<uint32_t>();

    Addr daddr = pkt->getAddr() - pioAddr;
    assert((daddr & 3) == 0);
    Registers reg = Registers(daddr / 4);

    if (!dvfsHandler->isEnabled()) {
        // Ignore writes to a disabled controller
        warn_once("EnergyCtrl: Disabled handler, ignoring write %u to "\
                  "reg %i\n", data, reg);
        DPRINTF(EnergyCtrl, "dvfs handler disabled, ignoring write %u to "\
                "reg %i\n", data, reg);
        pkt->makeAtomicResponse();
        return pioDelay;
    }

    switch(reg) {
      case DVFS_DOMAINID_AT_INDEX:
        domainIDIndexToRead = data;
        DPRINTF(EnergyCtrl, "writing domain id index:%d\n",
                domainIDIndexToRead);
        break;
      case DOMAIN_ID:
        // Extra check to ensure that a valid domain ID is being queried
        if (dvfsHandler->validDomainID(data)) {
            domainID = data;
            DPRINTF(EnergyCtrl, "writing domain id:%d\n", domainID);
        } else {
           DPRINTF(EnergyCtrl, "invalid domain id:%d\n", domainID);
        }
        break;
      case PERF_LEVEL:
        if (dvfsHandler->perfLevel(domainID, data)) {
            if (updateAckEvent.scheduled()) {
                // The OS driver is trying to change the perf level while
                // another change is in flight.  This is fine, but only a
                // single acknowledgment will be sent.
                DPRINTF(EnergyCtrl, "descheduling previous pending ack "\
                        "event\n");
                deschedule(updateAckEvent);
            }
            schedule(updateAckEvent, curTick() + dvfsHandler->transLatency());
            DPRINTF(EnergyCtrl, "writing domain %d perf level: %d\n",
                    domainID, data);
        } else {
            DPRINTF(EnergyCtrl, "invalid / ineffective perf level:%d for "\
                    "domain:%d\n", data, domainID);
        }
        break;
      case PERF_LEVEL_TO_READ:
        perfLevelToRead = data;
        DPRINTF(EnergyCtrl, "writing perf level to read opp at: %d\n",
                data);
        break;
      default:
        panic("Tried to write EnergyCtrl at offset %#x\n", daddr);
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
EnergyCtrl::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(domainID);
    SERIALIZE_SCALAR(domainIDIndexToRead);
    SERIALIZE_SCALAR(perfLevelToRead);
    SERIALIZE_SCALAR(perfLevelAck);

    Tick next_event = updateAckEvent.scheduled() ? updateAckEvent.when() : 0;
    SERIALIZE_SCALAR(next_event);
}

void
EnergyCtrl::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(domainID);
    UNSERIALIZE_SCALAR(domainIDIndexToRead);
    UNSERIALIZE_SCALAR(perfLevelToRead);
    UNSERIALIZE_SCALAR(perfLevelAck);
    Tick next_event = 0;
    UNSERIALIZE_SCALAR(next_event);

    // restore scheduled events
    if (next_event != 0) {
        schedule(updateAckEvent, next_event);
    }
}

void
EnergyCtrl::startup()
{
    if (!dvfsHandler->isEnabled()) {
        warn("Existing EnergyCtrl, but no enabled DVFSHandler found.\n");
    }
}

void
EnergyCtrl::init()
{
    BasicPioDevice::init();
}

} // namespace gem5
