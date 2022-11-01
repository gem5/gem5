/*
 * Copyright (c) 2019-2022 Arm Limited
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
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#include "dev/arm/gic_v3.hh"

#include "cpu/base.hh"
#include "debug/GIC.hh"
#include "debug/Interrupt.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "dev/arm/gic_v3_distributor.hh"
#include "dev/arm/gic_v3_its.hh"
#include "dev/arm/gic_v3_redistributor.hh"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5
{

void
Gicv3Registers::copyDistRegister(Gicv3Registers* from,
                                 Gicv3Registers* to,
                                 Addr daddr)
{
    auto val = from->readDistributor(daddr);
    DPRINTF(GIC, "copy dist 0x%x 0x%08x\n", daddr, val);
    to->writeDistributor(daddr, val);
}

void
Gicv3Registers::copyRedistRegister(Gicv3Registers* from,
                                   Gicv3Registers* to,
                                   const ArmISA::Affinity &aff, Addr daddr)
{
    auto val = from->readRedistributor(aff, daddr);
    DPRINTF(GIC,
            "copy redist (aff3: %d, aff2: %d, aff1: %d, aff0: %d) "
            "0x%x 0x%08x\n",
            aff.aff3, aff.aff2, aff.aff1, aff.aff0, daddr, val);

    to->writeRedistributor(aff, daddr, val);
}

void
Gicv3Registers::copyCpuRegister(Gicv3Registers* from,
                                Gicv3Registers* to,
                                const ArmISA::Affinity &aff,
                                ArmISA::MiscRegIndex misc_reg)
{
    auto val = from->readCpu(aff, misc_reg);
    DPRINTF(GIC,
            "copy cpu (aff3: %d, aff2: %d, aff1: %d, aff0: %d) "
            "%s 0x%08x\n",
            aff.aff3, aff.aff2, aff.aff1, aff.aff0,
            ArmISA::miscRegName[misc_reg], val);

    to->writeCpu(aff, misc_reg, val);
}

void
Gicv3Registers::clearRedistRegister(Gicv3Registers* to,
                                    const ArmISA::Affinity &aff, Addr daddr)
{
    to->writeRedistributor(aff, daddr, 0xFFFFFFFF);
}

void
Gicv3Registers::copyRedistRange(Gicv3Registers* from,
                                Gicv3Registers* to,
                                const ArmISA::Affinity &aff,
                                Addr daddr, size_t size)
{
    for (auto a = daddr; a < daddr + size; a += 4)
        copyRedistRegister(from, to, aff, a);
}

void
Gicv3Registers::copyDistRange(Gicv3Registers *from,
                              Gicv3Registers *to,
                              Addr daddr, size_t size)
{
    for (auto a = daddr; a < daddr + size; a += 4)
        copyDistRegister(from, to, a);
}

void
Gicv3Registers::clearDistRange(Gicv3Registers *to, Addr daddr, size_t size)
{
    for (auto a = daddr; a < daddr + size; a += 4)
        to->writeDistributor(a, 0xFFFFFFFF);
}


Gicv3::Gicv3(const Params &p)
    : BaseGic(p)
{
}

void
Gicv3::init()
{
    distributor = new Gicv3Distributor(this, params().it_lines);
    int threads = sys->threads.size();
    redistributors.resize(threads, nullptr);
    cpuInterfaces.resize(threads, nullptr);

    panic_if(threads > params().cpu_max,
        "Exceeding maximum number of PEs supported by GICv3: "
        "using %u while maximum is %u.", threads, params().cpu_max);

    for (int i = 0; i < threads; i++) {
        redistributors[i] = new Gicv3Redistributor(this, i);
        cpuInterfaces[i] = new Gicv3CPUInterface(this, sys->threads[i]);
    }

    distRange = RangeSize(params().dist_addr,
        Gicv3Distributor::ADDR_RANGE_SIZE);

    redistSize = redistributors[0]->addrRangeSize;
    redistRange = RangeSize(params().redist_addr, redistSize * threads);

    addrRanges = {distRange, redistRange};

    distributor->init();

    for (int i = 0; i < threads; i++) {
        redistributors[i]->init();
        cpuInterfaces[i]->init();
    }

    Gicv3Its *its = params().its;
    if (its)
        its->setGIC(this);

    BaseGic::init();
}

Tick
Gicv3::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    bool is_secure_access = pkt->isSecure();
    uint64_t resp = 0;
    Tick delay = 0;

    if (distRange.contains(addr)) {
        const Addr daddr = addr - distRange.start();
        panic_if(!distributor, "Distributor is null!");
        resp = distributor->read(daddr, size, is_secure_access);
        delay = params().dist_pio_delay;
        DPRINTF(GIC, "Gicv3::read(): (distributor) context_id %d register %#x "
                "size %d is_secure_access %d (value %#x)\n",
                pkt->req->contextId(), daddr, size, is_secure_access, resp);
    } else if (redistRange.contains(addr)) {
        Addr daddr = (addr - redistRange.start()) % redistSize;

        Gicv3Redistributor *redist = getRedistributorByAddr(addr);
        resp = redist->read(daddr, size, is_secure_access);

        delay = params().redist_pio_delay;
        DPRINTF(GIC, "Gicv3::read(): (redistributor %d) context_id %d "
                "register %#x size %d is_secure_access %d (value %#x)\n",
                redist->processorNumber(), pkt->req->contextId(), daddr, size,
                is_secure_access, resp);
    } else {
        panic("Gicv3::read(): unknown address %#x\n", addr);
    }

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeAtomicResponse();
    return delay;
}

Tick
Gicv3::write(PacketPtr pkt)
{
    const size_t size = pkt->getSize();
    uint64_t data = pkt->getUintX(ByteOrder::little);
    const Addr addr = pkt->getAddr();
    bool is_secure_access = pkt->isSecure();
    Tick delay = 0;

    if (distRange.contains(addr)) {
        const Addr daddr = addr - distRange.start();
        panic_if(!distributor, "Distributor is null!");
        DPRINTF(GIC, "Gicv3::write(): (distributor) context_id %d "
                "register %#x size %d is_secure_access %d value %#x\n",
                pkt->req->contextId(), daddr, size, is_secure_access, data);
        distributor->write(daddr, data, size, is_secure_access);
        delay = params().dist_pio_delay;
    } else if (redistRange.contains(addr)) {
        Addr daddr = (addr - redistRange.start()) % redistSize;

        Gicv3Redistributor *redist = getRedistributorByAddr(addr);
        DPRINTF(GIC, "Gicv3::write(): (redistributor %d) context_id %d "
                "register %#x size %d is_secure_access %d value %#x\n",
                redist->processorNumber(), pkt->req->contextId(), daddr, size,
                is_secure_access, data);

        redist->write(daddr, data, size, is_secure_access);

        delay = params().redist_pio_delay;
    } else {
        panic("Gicv3::write(): unknown address %#x\n", addr);
    }

    pkt->makeAtomicResponse();
    return delay;
}

void
Gicv3::sendInt(uint32_t int_id)
{
    DPRINTF(Interrupt, "Gicv3::sendInt(): received SPI %d\n", int_id);
    distributor->sendInt(int_id);
}

void
Gicv3::clearInt(uint32_t int_id)
{
    DPRINTF(Interrupt, "Gicv3::clearInt(): received SPI %d\n", int_id);
    distributor->clearInt(int_id);
}

void
Gicv3::sendPPInt(uint32_t int_id, uint32_t cpu)
{
    panic_if(cpu >= redistributors.size(), "Invalid cpuID sending PPI!");
    DPRINTF(Interrupt, "Gicv3::sendPPInt(): received PPI %d cpuTarget %#x\n",
            int_id, cpu);
    redistributors[cpu]->sendPPInt(int_id);
}

void
Gicv3::clearPPInt(uint32_t int_id, uint32_t cpu)
{
    panic_if(cpu >= redistributors.size(), "Invalid cpuID clearing PPI!");
    DPRINTF(Interrupt, "Gicv3::clearPPInt(): received PPI %d cpuTarget %#x\n",
            int_id, cpu);
    redistributors[cpu]->clearPPInt(int_id);
}

void
Gicv3::postInt(uint32_t cpu, ArmISA::InterruptTypes int_type)
{
    auto tc = sys->threads[cpu];
    tc->getCpuPtr()->postInterrupt(tc->threadId(), int_type, 0);
    ArmSystem::callClearStandByWfi(tc);
}

void
Gicv3::update()
{
    distributor->update();
}

bool
Gicv3::supportsVersion(GicVersion version)
{
    return (version == GicVersion::GIC_V3) ||
           (version == GicVersion::GIC_V4 && params().gicv4);
}

void
Gicv3::deassertInt(uint32_t cpu, ArmISA::InterruptTypes int_type)
{
    auto tc = sys->threads[cpu];
    tc->getCpuPtr()->clearInterrupt(tc->threadId(), int_type, 0);
}

void
Gicv3::deassertAll(uint32_t cpu)
{
    auto tc = sys->threads[cpu];
    tc->getCpuPtr()->clearInterrupts(tc->threadId());
}

bool
Gicv3::haveAsserted(uint32_t cpu) const
{
    auto tc = sys->threads[cpu];
    return tc->getCpuPtr()->checkInterrupts(tc->threadId());
}

Gicv3CPUInterface *
Gicv3::getCPUInterfaceByAffinity(const ArmISA::Affinity &aff) const
{
    return getRedistributorByAffinity(aff)->getCPUInterface();
}

Gicv3Redistributor *
Gicv3::getRedistributorByAffinity(const ArmISA::Affinity &aff) const
{
    for (auto & redistributor : redistributors) {
        if (redistributor->getAffinity() == aff) {
            return redistributor;
        }
    }

    return nullptr;
}

Gicv3Redistributor *
Gicv3::getRedistributorByAddr(Addr addr) const
{
    panic_if(!redistRange.contains(addr),
        "Address not pointing to a valid redistributor\n");

    const Addr daddr = addr - redistRange.start();
    const uint32_t redistributor_id = daddr / redistSize;

    panic_if(redistributor_id >= redistributors.size(),
             "Invalid redistributor_id!");
    panic_if(!redistributors[redistributor_id], "Redistributor is null!");

    return redistributors[redistributor_id];
}

uint32_t
Gicv3::readDistributor(Addr daddr)
{
    return distributor->read(daddr, 4, false);
}

uint32_t
Gicv3::readRedistributor(const ArmISA::Affinity &aff, Addr daddr)
{
    auto redistributor = getRedistributorByAffinity(aff);
    assert(redistributor);
    return redistributor->read(daddr, 4, false);
}

RegVal
Gicv3::readCpu(const ArmISA::Affinity &aff, ArmISA::MiscRegIndex misc_reg)
{
    auto cpu_interface = getCPUInterfaceByAffinity(aff);
    assert(cpu_interface);
    return cpu_interface->readMiscReg(misc_reg);
}

void
Gicv3::writeDistributor(Addr daddr, uint32_t data)
{
    distributor->write(daddr, data, sizeof(data), false);
}

void
Gicv3::writeRedistributor(const ArmISA::Affinity &aff, Addr daddr, uint32_t data)
{
    auto redistributor = getRedistributorByAffinity(aff);
    assert(redistributor);
    redistributor->write(daddr, data, sizeof(data), false);
}

void
Gicv3::writeCpu(const ArmISA::Affinity &aff, ArmISA::MiscRegIndex misc_reg,
                RegVal data)
{
    auto cpu_interface = getCPUInterfaceByAffinity(aff);
    assert(cpu_interface);
    cpu_interface->setMiscReg(misc_reg, data);
}

void
Gicv3::copyGicState(Gicv3Registers* from, Gicv3Registers* to)
{
    distributor->copy(from, to);
    for (auto& redistributor : redistributors) {
        redistributor->copy(from, to);
    }
    for (auto& cpu_interface : cpuInterfaces) {
        cpu_interface->copy(from, to);
    }
}

void
Gicv3::serialize(CheckpointOut & cp) const
{
    distributor->serializeSection(cp, "distributor");

    for (uint32_t redistributor_id = 0;
         redistributor_id < redistributors.size(); redistributor_id++)
        redistributors[redistributor_id]->serializeSection(cp,
            csprintf("redistributors.%i", redistributor_id));

    for (uint32_t cpu_interface_id = 0;
         cpu_interface_id < cpuInterfaces.size(); cpu_interface_id++)
        cpuInterfaces[cpu_interface_id]->serializeSection(cp,
            csprintf("cpuInterface.%i", cpu_interface_id));
}

void
Gicv3::unserialize(CheckpointIn & cp)
{
    getSystem()->setGIC(this);

    distributor->unserializeSection(cp, "distributor");

    for (uint32_t redistributor_id = 0;
         redistributor_id < redistributors.size(); redistributor_id++)
        redistributors[redistributor_id]->unserializeSection(cp,
            csprintf("redistributors.%i", redistributor_id));

    for (uint32_t cpu_interface_id = 0;
         cpu_interface_id < cpuInterfaces.size(); cpu_interface_id++)
        cpuInterfaces[cpu_interface_id]->unserializeSection(cp,
            csprintf("cpuInterface.%i", cpu_interface_id));
}

} // namespace gem5
