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
#include "arch/arm/regs/misc.hh"
#include "debug/GIC.hh"
#include "debug/Interrupt.hh"
#include "params/MuxingKvmGicV2.hh"

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
    const unsigned vcpu_index = vcpu & 0xff;
    const unsigned vcpu2_index = (vcpu >> 8) & 0xff;

    static const bool vcpu2_enabled = vm.kvm->capIRQLineLayout2();
    uint32_t kvm_vcpu = (vcpu_index << KVM_ARM_IRQ_VCPU_SHIFT);

#if defined(KVM_ARM_IRQ_VCPU2_SHIFT)
    if (vcpu2_enabled)
        kvm_vcpu |= vcpu2_index << KVM_ARM_IRQ_VCPU2_SHIFT;
#endif

    panic_if((!vcpu2_enabled && vcpu2_index) || kvm_vcpu > 0xffff,
              "VCPU out of range");

    assert(type <= KVM_ARM_IRQ_TYPE_MASK);
    assert(irq <= KVM_ARM_IRQ_NUM_MASK);
    const uint32_t line(
        kvm_vcpu |
        (type << KVM_ARM_IRQ_TYPE_SHIFT) |
        (irq << KVM_ARM_IRQ_NUM_SHIFT));

    vm.setIRQLine(line, high);
}

KvmKernelGicV2::KvmKernelGicV2(KvmVM &_vm,
                               const MuxingKvmGicV2Params &p)
    : KvmKernelGic(_vm, KVM_DEV_TYPE_ARM_VGIC_V2, p.it_lines),
      cpuRange(RangeSize(p.cpu_addr, KVM_VGIC_V2_CPU_SIZE)),
      distRange(RangeSize(p.dist_addr, KVM_VGIC_V2_DIST_SIZE))
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_DIST, p.dist_addr);
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V2_ADDR_TYPE_CPU, p.cpu_addr);
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

#ifndef SZ_64K
#define SZ_64K 0x00000040
#endif

KvmKernelGicV3::KvmKernelGicV3(KvmVM &_vm,
                               const MuxingKvmGicV3Params &p)
    : KvmKernelGic(_vm, KVM_DEV_TYPE_ARM_VGIC_V3, p.it_lines),
      redistRange(RangeSize(p.redist_addr, KVM_VGIC_V3_REDIST_SIZE)),
      distRange(RangeSize(p.dist_addr, KVM_VGIC_V3_DIST_SIZE))
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V3_ADDR_TYPE_DIST, p.dist_addr);
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_ADDR, KVM_VGIC_V3_ADDR_TYPE_REDIST, p.redist_addr);
}

void
KvmKernelGicV3::init()
{
    kdev.setAttr<uint64_t>(
        KVM_DEV_ARM_VGIC_GRP_CTRL, KVM_DEV_ARM_VGIC_CTRL_INIT, 0);
}

template <typename Ret>
Ret
KvmKernelGicV3::getGicReg(unsigned group, unsigned mpidr, unsigned offset)
{
    Ret reg;

    assert(mpidr <= KVM_DEV_ARM_VGIC_V3_MPIDR_MASK);
    const uint64_t attr(
        ((uint64_t)mpidr << KVM_DEV_ARM_VGIC_V3_MPIDR_SHIFT) |
        (offset << KVM_DEV_ARM_VGIC_OFFSET_SHIFT));

    kdev.getAttrPtr(group, attr, &reg);
    return reg;
}

template <typename Arg>
void
KvmKernelGicV3::setGicReg(unsigned group, unsigned mpidr, unsigned offset,
                          Arg value)
{
    Arg reg = value;

    assert(mpidr <= KVM_DEV_ARM_VGIC_V3_MPIDR_MASK);
    const uint64_t attr(
        ((uint64_t)mpidr << KVM_DEV_ARM_VGIC_V3_MPIDR_SHIFT) |
        (offset << KVM_DEV_ARM_VGIC_OFFSET_SHIFT));

    kdev.setAttrPtr(group, attr, &reg);
}

uint32_t
KvmKernelGicV3::readDistributor(Addr daddr)
{
    return getGicReg<uint32_t>(KVM_DEV_ARM_VGIC_GRP_DIST_REGS, 0, daddr);
}

uint32_t
KvmKernelGicV3::readRedistributor(const ArmISA::Affinity &aff, Addr daddr)
{
    return getGicReg<uint32_t>(KVM_DEV_ARM_VGIC_GRP_REDIST_REGS, aff, daddr);
}

RegVal
KvmKernelGicV3::readCpu(const ArmISA::Affinity &aff,
                        ArmISA::MiscRegIndex misc_reg)
{
    auto sys_reg = ArmISA::encodeAArch64SysReg(misc_reg).packed();
    return getGicReg<RegVal>(KVM_DEV_ARM_VGIC_GRP_CPU_SYSREGS, aff, sys_reg);
}

void
KvmKernelGicV3::writeDistributor(Addr daddr, uint32_t data)
{
    setGicReg<uint32_t>(KVM_DEV_ARM_VGIC_GRP_DIST_REGS, 0, daddr, data);
}

void
KvmKernelGicV3::writeRedistributor(const ArmISA::Affinity &aff, Addr daddr,
                                   uint32_t data)
{
    setGicReg<uint32_t>(KVM_DEV_ARM_VGIC_GRP_REDIST_REGS, aff, daddr, data);
}

void
KvmKernelGicV3::writeCpu(const ArmISA::Affinity &aff,
                         ArmISA::MiscRegIndex misc_reg,
                         RegVal data)
{
    auto sys_reg = ArmISA::encodeAArch64SysReg(misc_reg).packed();
    setGicReg<RegVal>(KVM_DEV_ARM_VGIC_GRP_CPU_SYSREGS, aff, sys_reg, data);
}

template <class Types>
MuxingKvmGic<Types>::MuxingKvmGic(const Params &p)
  : SimGic(p),
    system(*p.system),
    kernelGic(nullptr),
    usingKvm(false)
{
    auto vm = system.getKvmVM();
    if (vm && !p.simulate_gic) {
        kernelGic = new KvmGic(*vm, p);
    }
}

template <class Types>
void
MuxingKvmGic<Types>::startup()
{
    SimGic::startup();

    if (kernelGic) {
        kernelGic->init();

        KvmVM *vm = system.getKvmVM();
        if (vm && vm->validEnvironment()) {
            usingKvm = true;
            fromGicToKvm();
        }
    }
}

template <class Types>
DrainState
MuxingKvmGic<Types>::drain()
{
    if (usingKvm)
        fromKvmToGic();
    return SimGic::drain();
}

template <class Types>
void
MuxingKvmGic<Types>::drainResume()
{
    SimGic::drainResume();

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

template <class Types>
Tick
MuxingKvmGic<Types>::read(PacketPtr pkt)
{
    if (!usingKvm)
        return SimGic::read(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

template <class Types>
Tick
MuxingKvmGic<Types>::write(PacketPtr pkt)
{
    if (!usingKvm)
        return SimGic::write(pkt);

    panic("MuxingKvmGic: PIO from gem5 is currently unsupported\n");
}

template <class Types>
void
MuxingKvmGic<Types>::sendInt(uint32_t num)
{
    if (!usingKvm)
        return SimGic::sendInt(num);

    DPRINTF(Interrupt, "Set SPI %d\n", num);
    kernelGic->setSPI(num);
}

template <class Types>
void
MuxingKvmGic<Types>::clearInt(uint32_t num)
{
    if (!usingKvm)
        return SimGic::clearInt(num);

    DPRINTF(Interrupt, "Clear SPI %d\n", num);
    kernelGic->clearSPI(num);
}

template <class Types>
void
MuxingKvmGic<Types>::sendPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return SimGic::sendPPInt(num, cpu);
    DPRINTF(Interrupt, "Set PPI %d:%d\n", cpu, num);
    kernelGic->setPPI(cpu, num);
}

template <class Types>
void
MuxingKvmGic<Types>::clearPPInt(uint32_t num, uint32_t cpu)
{
    if (!usingKvm)
        return SimGic::clearPPInt(num, cpu);

    DPRINTF(Interrupt, "Clear PPI %d:%d\n", cpu, num);
    kernelGic->clearPPI(cpu, num);
}

template <class Types>
bool
MuxingKvmGic<Types>::blockIntUpdate() const
{
    // During Kvm->Gic state transfer, writes to the Gic will call
    // updateIntState() which can post an interrupt.  Since we're only
    // using the Gic model for holding state in this circumstance, we
    // short-circuit this behavior, as the GicV2 is not actually active.
    return usingKvm;
}

template <class Types>
void
MuxingKvmGic<Types>::fromGicToKvm()
{
    this->copyGicState(static_cast<SimGic*>(this),
                       static_cast<KvmGic*>(kernelGic));
}

template <class Types>
void
MuxingKvmGic<Types>::fromKvmToGic()
{
    this->copyGicState(static_cast<KvmGic*>(kernelGic),
                       static_cast<SimGic*>(this));

    // the values read for the Interrupt Priority Mask Register (PMR)
    // have been shifted by three bits due to its having been emulated by
    // a VGIC with only 5 PMR bits in its VMCR register.  Presently the
    // Linux kernel does not repair this inaccuracy, so we correct it here.
    if constexpr(std::is_same<SimGic, GicV2>::value) {
        for (int cpu = 0; cpu < system.threads.size(); ++cpu) {
           this->cpuPriority[cpu] <<= 3;
           assert((this->cpuPriority[cpu] & ~0xff) == 0);
        }
    }
}

template class MuxingKvmGic<GicV2Types>;
template class MuxingKvmGic<GicV3Types>;

} // namespace gem5
