/*
 * Copyright (c) 2012, 2015, 2017 ARM Limited
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

#include "arch/arm/kvm/base_cpu.hh"

#include <linux/kvm.h>
#include <mutex>

#include "arch/arm/interrupts.hh"
#include "base/uncontended_mutex.hh"
#include "debug/KvmInt.hh"
#include "dev/arm/generic_timer.hh"
#include "params/BaseArmKvmCPU.hh"
#include "params/GenericTimer.hh"

namespace gem5
{

using namespace ArmISA;

#define INTERRUPT_ID(type, vcpu, irq) (                    \
        ((type) << KVM_ARM_IRQ_TYPE_SHIFT) |               \
        ((vcpu) << KVM_ARM_IRQ_VCPU_SHIFT) |               \
        ((irq) << KVM_ARM_IRQ_NUM_SHIFT))

#define INTERRUPT_VCPU_IRQ(vcpu)                                \
    INTERRUPT_ID(KVM_ARM_IRQ_TYPE_CPU, vcpu, KVM_ARM_IRQ_CPU_IRQ)

#define INTERRUPT_VCPU_FIQ(vcpu)                                \
    INTERRUPT_ID(KVM_ARM_IRQ_TYPE_CPU, vcpu, KVM_ARM_IRQ_CPU_FIQ)

namespace {

/**
 * When the simulator returns from KVM for simulating other models, the
 * in-kernel timer doesn't stop. We have to save the virtual time and
 * restore before going into KVM next time. Moreover, setting virtual time
 * affacts all vcpus according to the kvm implementation. We maintain a global
 * virtual time here, restore it before the first vcpu going into KVM, and save
 * it after the last vcpu back from KVM.
 */
uint64_t vtime = 0;
uint64_t vtime_counter = 0;
UncontendedMutex vtime_mutex;

}  // namespace

BaseArmKvmCPU::BaseArmKvmCPU(const BaseArmKvmCPUParams &params)
    : BaseKvmCPU(params),
      irqAsserted(false), fiqAsserted(false),
      virtTimerPin(nullptr), prevDeviceIRQLevel(0)
{
}

BaseArmKvmCPU::~BaseArmKvmCPU()
{
}

void
BaseArmKvmCPU::startup()
{
    BaseKvmCPU::startup();

    /* TODO: This needs to be moved when we start to support VMs with
     * multiple threads since kvmArmVCpuInit requires that all CPUs in
     * the VM have been created.
     */
    struct kvm_vcpu_init target_config;
    memset(&target_config, 0, sizeof(target_config));

    vm->kvmArmPreferredTarget(target_config);
    if (!((ArmSystem *)system)->highestELIs64()) {
        target_config.features[0] |= (1 << KVM_ARM_VCPU_EL1_32BIT);
    }
    kvmArmVCpuInit(target_config);

    if (!vm->hasKernelIRQChip())
        virtTimerPin = static_cast<ArmSystem *>(system)\
            ->getGenericTimer()->params().int_el1_virt->get(tc);
}

Tick
BaseArmKvmCPU::kvmRun(Tick ticks)
{
    auto interrupt = static_cast<ArmISA::Interrupts *>(interrupts[0]);
    const bool simFIQ(interrupt->checkRaw(INT_FIQ));
    const bool simIRQ(interrupt->checkRaw(INT_IRQ));

    if (!vm->hasKernelIRQChip()) {
        if (fiqAsserted != simFIQ) {
            DPRINTF(KvmInt, "KVM: Update FIQ state: %i\n", simFIQ);
            vm->setIRQLine(INTERRUPT_VCPU_FIQ(vcpuID), simFIQ);
        }
        if (irqAsserted != simIRQ) {
            DPRINTF(KvmInt, "KVM: Update IRQ state: %i\n", simIRQ);
            vm->setIRQLine(INTERRUPT_VCPU_IRQ(vcpuID), simIRQ);
        }
    } else {
        warn_if(simFIQ && !fiqAsserted,
                "FIQ raised by the simulated interrupt controller " \
                "despite in-kernel GIC emulation. This is probably a bug.");

        warn_if(simIRQ && !irqAsserted,
                "IRQ raised by the simulated interrupt controller " \
                "despite in-kernel GIC emulation. This is probably a bug.");
    }

    irqAsserted = simIRQ;
    fiqAsserted = simFIQ;

    Tick kvmRunTicks = BaseKvmCPU::kvmRun(ticks);

    if (!vm->hasKernelIRQChip()) {
        uint64_t device_irq_level =
            getKvmRunState()->s.regs.device_irq_level;

        if (!(prevDeviceIRQLevel & KVM_ARM_DEV_EL1_VTIMER) &&
            (device_irq_level & KVM_ARM_DEV_EL1_VTIMER)) {

            DPRINTF(KvmInt, "In-kernel vtimer IRQ asserted\n");
            prevDeviceIRQLevel |= KVM_ARM_DEV_EL1_VTIMER;
            virtTimerPin->raise();

        } else if ((prevDeviceIRQLevel & KVM_ARM_DEV_EL1_VTIMER) &&
                   !(device_irq_level & KVM_ARM_DEV_EL1_VTIMER)) {

            DPRINTF(KvmInt, "In-kernel vtimer IRQ disasserted\n");
            prevDeviceIRQLevel &= ~KVM_ARM_DEV_EL1_VTIMER;
            virtTimerPin->clear();
        }
    }

    return kvmRunTicks;
}

void
BaseArmKvmCPU::ioctlRun()
{
    // Check if it's the first vcpu going into KVM. If yes, it should restore
    // the virtual time.
    {
        std::lock_guard<UncontendedMutex> l(vtime_mutex);
        if (vtime_counter++ == 0)
            setOneReg(KVM_REG_ARM_TIMER_CNT, vtime);
    }
    BaseKvmCPU::ioctlRun();
    // Check if it's the last vcpu back from KVM. If yes, it should save the
    // virtual time.
    {
        std::lock_guard<UncontendedMutex> l(vtime_mutex);
        if (--vtime_counter == 0)
            getOneReg(KVM_REG_ARM_TIMER_CNT, &vtime);
    }
}

const BaseArmKvmCPU::RegIndexVector &
BaseArmKvmCPU::getRegList() const
{
    // Do we need to request a list of registers from the kernel?
    if (_regIndexList.size() == 0) {
        // Start by probing for the size of the list. We do this
        // calling the ioctl with a struct size of 0. The kernel will
        // return the number of elements required to hold the list.
        kvm_reg_list regs_probe;
        regs_probe.n = 0;
        getRegList(regs_probe);

        // Request the actual register list now that we know how many
        // register we need to allocate space for.
        std::unique_ptr<kvm_reg_list, void(*)(void *p)>
            regs(nullptr, [](void *p) { operator delete(p); });
        const size_t size(sizeof(kvm_reg_list) +
                          regs_probe.n * sizeof(uint64_t));
        regs.reset((kvm_reg_list *)operator new(size));
        regs->n = regs_probe.n;
        if (!getRegList(*regs))
            panic("Failed to determine register list size.\n");

        _regIndexList.assign(regs->reg, regs->reg + regs->n);
    }

    return _regIndexList;
}

void
BaseArmKvmCPU::kvmArmVCpuInit(const struct kvm_vcpu_init &init)
{
    if (ioctl(KVM_ARM_VCPU_INIT, (void *)&init) == -1)
        panic("KVM: Failed to initialize vCPU; errno %d (%s)\n",
            errno, strerror(errno));
}

bool
BaseArmKvmCPU::getRegList(kvm_reg_list &regs) const
{
    if (ioctl(KVM_GET_REG_LIST, (void *)&regs) == -1) {
        if (errno == E2BIG) {
            return false;
        } else {
            panic("KVM: Failed to get vCPU register list (errno: %i)\n",
                  errno);
        }
    } else {
        return true;
    }
}

} // namespace gem5
