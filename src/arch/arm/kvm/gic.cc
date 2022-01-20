/*
 * Copyright (c) 2015-2017, 2021 Arm Limited
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

#include "arch/arm/kvm/gic.hh"

#include <linux/kvm.h>

#include "arch/arm/kvm/base_cpu.hh"
#include "debug/GIC.hh"
#include "debug/Interrupt.hh"
#include "params/MuxingKvmGic.hh"

namespace gem5
{

KvmKernelGic::KvmKernelGic(KvmVM &_vm, uint32_t dev, unsigned it_lines)
    : vm(_vm),
      kdev(vm.createDevice(dev))
{
    // Tell the VM that we will emulate the GIC in the kernel. This
    // disables IRQ and FIQ handling in the KVM CPU model.
    vm.enableKernelIRQChip();

    kdev.setAttr<uint32_t>(KVM_DEV_ARM_VGIC_GRP_NR_IRQS, 0, it_lines);
}

KvmKernelGic::~KvmKernelGic()
{
}

void
KvmKernelGic::setSPI(unsigned spi)
{
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, spi, true);
}

void
KvmKernelGic::clearSPI(unsigned spi)
{
    setIntState(KVM_ARM_IRQ_TYPE_SPI, 0, spi, false);
}

void
KvmKernelGic::setPPI(unsigned vcpu, unsigned ppi)
{
    setIntState(KVM_ARM_IRQ_TYPE_PPI, vcpu, ppi, true);
}

void
KvmKernelGic::clearPPI(unsigned vcpu, unsigned ppi)
{
    setIntState(KVM_ARM_IRQ_TYPE_PPI, vcpu, ppi, false);
}

void
KvmKernelGic::setIntState(unsigned type, unsigned vcpu, unsigned irq,
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

KvmKernelGicV2::KvmKernelGicV2(KvmVM &_vm, Addr cpu_addr, Addr dist_addr,
                               unsigned it_lines)
    : KvmKernelGic(_vm, KVM_DEV_TYPE_ARM_VGIC_V2, it_lines),
      cpuRange(RangeSize(cpu_addr, KVM_VGIC_V2_CPU_SIZE)),
      distRange(RangeSize(dist_addr, KVM_VGIC_V2_DIST_SIZE))
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_DIST, dist_addr);
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_CPU, cpu_addr);
}

uint32_t
KvmKernelGicV2::getGicReg(unsigned group, unsigned vcpu, unsigned offset)
{
    uint64_t reg;

    assert(vcpu <= KVM_ARM_IRQ_VCPU_MASK);
    const uint64_t attr(
        ((uint64_t)vcpu << KVM_DEV_ARM_VGIC_CPUID_SHIFT) |
        (offset << KVM_DEV_ARM_VGIC_OFFSET_SHIFT));

    kdev.getAttrPtr(group, attr, &reg);
    return (uint32_t) reg;
}

void
KvmKernelGicV2::setGicReg(unsigned group, unsigned vcpu, unsigned offset,
                          unsigned value)
{
    uint64_t reg = value;

    assert(vcpu <= KVM_ARM_IRQ_VCPU_MASK);
    const uint64_t attr(
        ((uint64_t)vcpu << KVM_DEV_ARM_VGIC_CPUID_SHIFT) |
        (offset << KVM_DEV_ARM_VGIC_OFFSET_SHIFT));

    kdev.setAttrPtr(group, attr, &reg);
}

uint32_t
KvmKernelGicV2::readDistributor(ContextID ctx, Addr daddr)
{
    auto vcpu = vm.contextIdToVCpuId(ctx);
    return getGicReg(KVM_DEV_ARM_VGIC_GRP_DIST_REGS, vcpu, daddr);
}

uint32_t
KvmKernelGicV2::readCpu(ContextID ctx, Addr daddr)
{
    auto vcpu = vm.contextIdToVCpuId(ctx);
    return getGicReg(KVM_DEV_ARM_VGIC_GRP_CPU_REGS, vcpu, daddr);
}

void
KvmKernelGicV2::writeDistributor(ContextID ctx, Addr daddr, uint32_t data)
{
    auto vcpu = vm.contextIdToVCpuId(ctx);
    setGicReg(KVM_DEV_ARM_VGIC_GRP_DIST_REGS, vcpu, daddr, data);
}

void
KvmKernelGicV2::writeCpu(ContextID ctx, Addr daddr, uint32_t data)
{
    auto vcpu = vm.contextIdToVCpuId(ctx);
    setGicReg(KVM_DEV_ARM_VGIC_GRP_CPU_REGS, vcpu, daddr, data);
}

MuxingKvmGic::MuxingKvmGic(const MuxingKvmGicParams &p)
    : GicV2(p),
      system(*p.system),
      kernelGic(nullptr),
      usingKvm(false)
{
    auto vm = system.getKvmVM();
    if (vm && !p.simulate_gic) {
        kernelGic = new KvmKernelGicV2(*vm, p.cpu_addr, p.dist_addr,
                                       p.it_lines);
    }
}

MuxingKvmGic::~MuxingKvmGic()
{
}

void
MuxingKvmGic::startup()
{
    GicV2::startup();

    KvmVM *vm = system.getKvmVM();
    usingKvm = kernelGic && vm && vm->validEnvironment();
    if (usingKvm)
        fromGicToKvm();
}

DrainState
MuxingKvmGic::drain()
{
    if (usingKvm)
        fromKvmToGic();
    return GicV2::drain();
}

void
MuxingKvmGic::drainResume()
{
    GicV2::drainResume();

    KvmVM *vm = system.getKvmVM();
    bool use_kvm = kernelGic && vm && vm->validEnvironment();
    if (use_kvm != usingKvm) {
        // Should only occur due to CPU switches
        if (use_kvm) // from simulation to KVM emulation
            fromGicToKvm();
        // otherwise, drain() already sync'd the state back to the GicV2

        usingKvm = use_kvm;
    }
}

Tick
MuxingKvmGic::read(PacketPtr pkt)
{
    if (!usingKvm)
        return GicV2::read(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

Tick
MuxingKvmGic::write(PacketPtr pkt)
{
    if (!usingKvm)
        return GicV2::write(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

void
MuxingKvmGic::sendInt(uint32_t num)
{
    if (!usingKvm)
        return GicV2::sendInt(num);

    DPRINTF(Interrupt, "Set SPI %d\n", num);
    kernelGic->setSPI(num);
}

void
MuxingKvmGic::clearInt(uint32_t num)
{
    if (!usingKvm)
        return GicV2::clearInt(num);

    DPRINTF(Interrupt, "Clear SPI %d\n", num);
    kernelGic->clearSPI(num);
}

void
MuxingKvmGic::sendPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return GicV2::sendPPInt(num, cpu);
    DPRINTF(Interrupt, "Set PPI %d:%d\n", cpu, num);
    kernelGic->setPPI(cpu, num);
}

void
MuxingKvmGic::clearPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return GicV2::clearPPInt(num, cpu);

    DPRINTF(Interrupt, "Clear PPI %d:%d\n", cpu, num);
    kernelGic->clearPPI(cpu, num);
}

bool
MuxingKvmGic::blockIntUpdate() const
{
    // During Kvm->Gic state transfer, writes to the Gic will call
    // updateIntState() which can post an interrupt.  Since we're only
    // using the Gic model for holding state in this circumstance, we
    // short-circuit this behavior, as the GicV2 is not actually active.
    return usingKvm;
}

void
MuxingKvmGic::fromGicToKvm()
{
    copyGicState(static_cast<GicV2*>(this),
                 static_cast<KvmKernelGicV2*>(kernelGic));
}

void
MuxingKvmGic::fromKvmToGic()
{
    copyGicState(static_cast<KvmKernelGicV2*>(kernelGic),
                 static_cast<GicV2*>(this));

    // the values read for the Interrupt Priority Mask Register (PMR)
    // have been shifted by three bits due to its having been emulated by
    // a VGIC with only 5 PMR bits in its VMCR register.  Presently the
    // Linux kernel does not repair this inaccuracy, so we correct it here.
    for (int cpu = 0; cpu < system.threads.size(); ++cpu) {
       cpuPriority[cpu] <<= 3;
       assert((cpuPriority[cpu] & ~0xff) == 0);
    }
}

} // namespace gem5
