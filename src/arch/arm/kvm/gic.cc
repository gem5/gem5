/*
 * Copyright (c) 2015 ARM Limited
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
 */

#include "arch/arm/kvm/gic.hh"

#include <linux/kvm.h>

#include "debug/Interrupt.hh"
#include "params/KvmGic.hh"

KvmGic::KvmGic(const KvmGicParams *p)
    : BaseGic(p),
      system(*p->system),
      vm(*p->kvmVM),
      kdev(vm.createDevice(KVM_DEV_TYPE_ARM_VGIC_V2)),
      distRange(RangeSize(p->dist_addr, KVM_VGIC_V2_DIST_SIZE)),
      cpuRange(RangeSize(p->cpu_addr, KVM_VGIC_V2_CPU_SIZE)),
      addrRanges{distRange, cpuRange}
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_DIST,
        p->dist_addr);
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_CPU,
        p->cpu_addr);
}

KvmGic::~KvmGic()
{
}

void
KvmGic::serialize(CheckpointOut &cp) const
{
    panic("Checkpointing unsupported\n");
}

void
KvmGic::unserialize(CheckpointIn &cp)
{
    panic("Checkpointing unsupported\n");
}

Tick
KvmGic::read(PacketPtr pkt)
{
    panic("KvmGic: PIO from gem5 is currently unsupported\n");
}

Tick
KvmGic::write(PacketPtr pkt)
{
    panic("KvmGic: PIO from gem5 is currently unsupported\n");
}

void
KvmGic::sendInt(uint32_t num)
{
    DPRINTF(Interrupt, "Set SPI %d\n", num);
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, num, true);
}

void
KvmGic::clearInt(uint32_t num)
{
    DPRINTF(Interrupt, "Clear SPI %d\n", num);
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, num, false);
}

void
KvmGic::sendPPInt(uint32_t num, uint32_t cpu)
{
    DPRINTF(Interrupt, "Set PPI %d:%d\n", cpu, num);
    setIntState(KVM_ARM_IRQ_TYPE_PPI, cpu, num, true);
}

void
KvmGic::clearPPInt(uint32_t num, uint32_t cpu)
{
    DPRINTF(Interrupt, "Clear PPI %d:%d\n", cpu, num);
    setIntState(KVM_ARM_IRQ_TYPE_PPI, cpu, num, false);
}

void
KvmGic::verifyMemoryMode() const
{
    if (!(system.isAtomicMode() && system.bypassCaches())) {
        fatal("The in-kernel KVM GIC can only be used with KVM CPUs, but the "
              "current memory mode does not support KVM.\n");
    }
}

void
KvmGic::setIntState(uint8_t type, uint8_t vcpu, uint16_t irq, bool high)
{
    assert(type < KVM_ARM_IRQ_TYPE_MASK);
    assert(vcpu < KVM_ARM_IRQ_VCPU_MASK);
    assert(irq < KVM_ARM_IRQ_NUM_MASK);
    const uint32_t line(
        (type << KVM_ARM_IRQ_TYPE_SHIFT) |
        (vcpu << KVM_ARM_IRQ_VCPU_SHIFT) |
        (irq << KVM_ARM_IRQ_NUM_SHIFT));

    vm.setIRQLine(line, high);
}


KvmGic *
KvmGicParams::create()
{
    return new KvmGic(this);
}
