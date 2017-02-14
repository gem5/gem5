/*
 * Copyright (c) 2015-2017 ARM Limited
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
 * Authors: Andreas Sandberg
 *          Curtis Dunham
 */

#include "arch/arm/kvm/gic.hh"

#include <linux/kvm.h>

#include "arch/arm/kvm/base_cpu.hh"
#include "debug/Interrupt.hh"
#include "params/MuxingKvmGic.hh"

KvmKernelGicV2::KvmKernelGicV2(KvmVM &_vm, Addr cpu_addr, Addr dist_addr,
                               unsigned it_lines)
    : cpuRange(RangeSize(cpu_addr, KVM_VGIC_V2_CPU_SIZE)),
      distRange(RangeSize(dist_addr, KVM_VGIC_V2_DIST_SIZE)),
      vm(_vm),
      kdev(vm.createDevice(KVM_DEV_TYPE_ARM_VGIC_V2))
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_DIST, dist_addr);
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_CPU, cpu_addr);

    kdev.setAttr<uint32_t>(KVM_DEV_ARM_VGIC_GRP_NR_IRQS, 0, it_lines);
}

KvmKernelGicV2::~KvmKernelGicV2()
{
}

void
KvmKernelGicV2::setSPI(unsigned spi)
{
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, spi, true);
}

void
KvmKernelGicV2::clearSPI(unsigned spi)
{
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, spi, false);
}

void
KvmKernelGicV2::setPPI(unsigned vcpu, unsigned ppi)
{
    setIntState(KVM_ARM_IRQ_TYPE_PPI, vcpu, ppi, true);
}

void
KvmKernelGicV2::clearPPI(unsigned vcpu, unsigned ppi)
{
    setIntState(KVM_ARM_IRQ_TYPE_PPI, vcpu, ppi, false);
}

void
KvmKernelGicV2::setIntState(unsigned type, unsigned vcpu, unsigned irq,
                            bool high)
{
    assert(type <= KVM_ARM_IRQ_TYPE_MASK);
    assert(vcpu <= KVM_ARM_IRQ_VCPU_MASK);
    assert(irq <= KVM_ARM_IRQ_NUM_MASK);
    const uint32_t line(
        (type << KVM_ARM_IRQ_TYPE_SHIFT) |
        (vcpu << KVM_ARM_IRQ_VCPU_SHIFT) |
        (irq << KVM_ARM_IRQ_NUM_SHIFT));

    vm.setIRQLine(line, high);
}


MuxingKvmGic::MuxingKvmGic(const MuxingKvmGicParams *p)
    : Pl390(p),
      system(*p->system),
      kernelGic(nullptr),
      usingKvm(false)
{
    if (auto vm = system.getKvmVM()) {
        kernelGic = new KvmKernelGicV2(*vm, p->cpu_addr, p->dist_addr,
                                       p->it_lines);
    }
}

MuxingKvmGic::~MuxingKvmGic()
{
}

void
MuxingKvmGic::startup()
{
    usingKvm = (kernelGic != nullptr) && validKvmEnvironment();
}

void
MuxingKvmGic::drainResume()
{
    bool use_kvm = (kernelGic != nullptr) && validKvmEnvironment();
    if (use_kvm != usingKvm) {
        if (use_kvm) // from simulation to KVM emulation
            fromPl390ToKvm();
        else // from KVM emulation to simulation
            fromKvmToPl390();

        usingKvm = use_kvm;
    }
}

void
MuxingKvmGic::serialize(CheckpointOut &cp) const
{
    if (!usingKvm)
        return Pl390::serialize(cp);

    panic("Checkpointing unsupported\n");
}

void
MuxingKvmGic::unserialize(CheckpointIn &cp)
{
    if (!usingKvm)
        return Pl390::unserialize(cp);

    panic("Checkpointing unsupported\n");
}

Tick
MuxingKvmGic::read(PacketPtr pkt)
{
    if (!usingKvm)
        return Pl390::read(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

Tick
MuxingKvmGic::write(PacketPtr pkt)
{
    if (!usingKvm)
        return Pl390::write(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

void
MuxingKvmGic::sendInt(uint32_t num)
{
    if (!usingKvm)
        return Pl390::sendInt(num);

    DPRINTF(Interrupt, "Set SPI %d\n", num);
    kernelGic->setSPI(num);
}

void
MuxingKvmGic::clearInt(uint32_t num)
{
    if (!usingKvm)
        return Pl390::clearInt(num);

    DPRINTF(Interrupt, "Clear SPI %d\n", num);
    kernelGic->clearSPI(num);
}

void
MuxingKvmGic::sendPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return Pl390::sendPPInt(num, cpu);
    DPRINTF(Interrupt, "Set PPI %d:%d\n", cpu, num);
    kernelGic->setPPI(cpu, num);
}

void
MuxingKvmGic::clearPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return Pl390::clearPPInt(num, cpu);

    DPRINTF(Interrupt, "Clear PPI %d:%d\n", cpu, num);
    kernelGic->clearPPI(cpu, num);
}

bool
MuxingKvmGic::validKvmEnvironment() const
{
    if (system.threadContexts.empty())
        return false;

    for (auto tc : system.threadContexts) {
        if (dynamic_cast<BaseArmKvmCPU*>(tc->getCpuPtr()) == nullptr) {
            return false;
        }
    }
    return true;
}

void
MuxingKvmGic::fromPl390ToKvm()
{
    panic("Gic multiplexing not implemented.\n");
}

void
MuxingKvmGic::fromKvmToPl390()
{
    panic("Gic multiplexing not implemented.\n");
}

MuxingKvmGic *
MuxingKvmGicParams::create()
{
    return new MuxingKvmGic(this);
}
