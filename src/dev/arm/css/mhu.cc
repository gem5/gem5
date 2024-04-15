/*
 * Copyright (c) 2020 ARM Limited
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

#include "dev/arm/css/mhu.hh"

#include "debug/MHU.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/css/scp.hh"
#include "mem/packet_access.hh"
#include "params/Ap2ScpDoorbell.hh"
#include "params/MHU.hh"
#include "params/Scp2ApDoorbell.hh"

namespace gem5
{

Scp2ApDoorbell::Scp2ApDoorbell(const Scp2ApDoorbellParams &p)
    : MhuDoorbell(p), interrupt(p.interrupt->get())
{}

Ap2ScpDoorbell::Ap2ScpDoorbell(const Ap2ScpDoorbellParams &p) : MhuDoorbell(p)
{}

MHU::MHU(const MHUParams &p)
    : BasicPioDevice(p, p.pio_size),
      scpLow(p.lowp_scp2ap),
      scpHigh(p.highp_scp2ap),
      scpSec(p.sec_scp2ap),
      apLow(p.lowp_ap2scp),
      apHigh(p.highp_ap2scp),
      apSec(p.sec_ap2scp),
      pid{ 0x98, 0xb0, 0x1b, 0x0, 0x4 },
      compid{ 0x0d, 0xf0, 0x05, 0xb1 },
      scfg(0)
{
    apLow->setScp(p.scp);
    apHigh->setScp(p.scp);
    apSec->setScp(p.scp);
}

AddrRangeList
MHU::getAddrRanges() const
{
    return AddrRangeList({ RangeSize(pioAddr, pioSize) });
}

Tick
MHU::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    const bool secure = pkt->isSecure();

    uint32_t value = read32(addr, secure);

    DPRINTF(MHU, "Reading %#x at address: %#x\n", value, addr);

    pkt->setUintX(value, ByteOrder::little);
    pkt->makeAtomicResponse();
    return pioDelay;
}

uint32_t
MHU::read32(const Addr addr, bool secure_access)
{
    switch (addr) {
    case SCP_INTR_L_STAT:
        return scpLow->channel;
    case SCP_INTR_H_STAT:
        return scpHigh->channel;
    case CPU_INTR_L_STAT:
        return apLow->channel;
    case CPU_INTR_H_STAT:
        return apHigh->channel;
    case SCP_INTR_S_STAT:
        if (secure_access) {
            return scpSec->channel;
        } else {
            if (!bits(scfg, 0))
                scpSec->set(SVI_INT);
            return 0;
        }
    case CPU_INTR_S_STAT:
        if (secure_access) {
            return apSec->channel;
        } else {
            if (!bits(scfg, 0))
                scpSec->set(SVI_INT);
            return 0;
        }
    case MHU_SCFG:
        return scfg;
    case PID4:
        return pid[4];
    case PID0:
        return pid[0];
    case PID1:
        return pid[1];
    case PID2:
        return pid[2];
    case PID3:
        return pid[3];
    case COMPID0:
        return compid[0];
    case COMPID1:
        return compid[1];
    case COMPID2:
        return compid[2];
    case COMPID3:
        return compid[3];
    default:
        panic("Invalid register read at address: %#x\n", addr);
    }
}

Tick
MHU::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;

    assert(pkt->getSize() == sizeof(uint32_t));
    const uint32_t value = pkt->getLE<uint32_t>();

    DPRINTF(MHU, "Writing %#x at address: %#x\n", value, addr);

    switch (addr) {
    case SCP_INTR_L_SET:
        scpLow->set(value);
        break;
    case SCP_INTR_L_CLEAR:
        scpLow->clear(value);
        break;
    case SCP_INTR_H_SET:
        scpHigh->set(value);
        break;
    case SCP_INTR_H_CLEAR:
        scpHigh->clear(value);
        break;
    case CPU_INTR_L_SET:
        apLow->set(value);
        break;
    case CPU_INTR_L_CLEAR:
        apLow->clear(value);
        break;
    case CPU_INTR_H_SET:
        apHigh->set(value);
        break;
    case CPU_INTR_H_CLEAR:
        apHigh->clear(value);
        break;
    case SCP_INTR_S_SET:
        scpSec->set(value);
        break;
    case SCP_INTR_S_CLEAR:
        scpSec->clear(value);
        break;
    case CPU_INTR_S_SET:
        apSec->set(value);
        break;
    case CPU_INTR_S_CLEAR:
        apSec->clear(value);
        break;
    case MHU_SCFG:
        scfg = value;
        break;
    default:
        panic("Invalid register write at address: %#x\n", addr);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
MhuDoorbell::update(uint32_t new_val)
{
    const bool int_old = channel != 0;
    const bool int_new = new_val != 0;

    channel = new_val;
    if (int_old && !int_new) {
        clearInterrupt();
    } else if (!int_old && int_new) {
        raiseInterrupt();
    }
}

void
Scp2ApDoorbell::raiseInterrupt()
{
    interrupt->raise();
}

void
Scp2ApDoorbell::clearInterrupt()
{
    interrupt->clear();
}

void
Ap2ScpDoorbell::raiseInterrupt()
{
    scp->raiseInterrupt(this);
}

void
Ap2ScpDoorbell::clearInterrupt()
{
    scp->clearInterrupt(this);
}

} // namespace gem5
