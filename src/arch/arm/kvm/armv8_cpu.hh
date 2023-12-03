/*
 * Copyright (c) 2015, 2017 ARM Limited
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

#ifndef __ARCH_ARM_KVM_ARMV8_CPU_HH__
#define __ARCH_ARM_KVM_ARMV8_CPU_HH__

#include <set>
#include <vector>

#include "arch/arm/kvm/base_cpu.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"

namespace gem5
{

struct ArmV8KvmCPUParams;

/**
 * This is an implementation of a KVM-based ARMv8-compatible CPU.
 *
 * Known limitations:
 * <ul>
 *
 *   <li>The system-register-based generic timer can only be simulated
 *       by the host kernel. Workaround: Use a memory mapped timer
 *       instead to simulate the timer in gem5.
 *
 *   <li>Simulating devices (e.g., the generic timer) in the host
 *       kernel requires that the host kernel also simulates the
 *       GIC.
 *
 *   <li>ID registers in the host and in gem5 must match for switching
 *       between simulated CPUs and KVM. This is particularly
 *       important for ID registers describing memory system
 *       capabilities (e.g., ASID size, physical address size).
 *
 *   <li>Switching between a virtualized CPU and a simulated CPU is
 *       currently not supported if in-kernel device emulation is
 *       used. This could be worked around by adding support for
 *       switching to the gem5 (e.g., the KvmGic) side of the device
 *       models. A simpler workaround is to avoid in-kernel device
 *       models altogether.
 *
 * </ul>
 *
 */
class ArmV8KvmCPU : public BaseArmKvmCPU
{
  public:
    ArmV8KvmCPU(const ArmV8KvmCPUParams &params);
    virtual ~ArmV8KvmCPU();

    void startup() override;

    void dump() const override;

  protected:
    void updateKvmState() override;
    void updateThreadContext() override;

  protected:
    /** Mapping between integer registers in gem5 and KVM */
    struct IntRegInfo
    {
        IntRegInfo(uint64_t _kvm, RegIndex _idx, const char *_name)
            : kvm(_kvm), idx(_idx), name(_name)
        {}

        /** Register index in KVM */
        uint64_t kvm;
        /** Register index in gem5 */
        RegIndex idx;
        /** Name to use in debug dumps */
        const char *name;
    };

    /** Mapping between misc registers in gem5 and registers in KVM */
    struct MiscRegInfo
    {
        MiscRegInfo(uint64_t _kvm, ArmISA::MiscRegIndex _idx,
                    const char *_name, bool _is_device = false)
            : kvm(_kvm), idx(_idx), name(_name), is_device(_is_device)
        {}

        /** Register index in KVM */
        uint64_t kvm;
        /** Register index in gem5 */
        ArmISA::MiscRegIndex idx;
        /** Name to use in debug dumps */
        const char *name;
        /** is device register? (needs 'effectful' state update) */
        bool is_device;
    };

    /**
     * Get a map between system registers in kvm and gem5 registers
     *
     * This method returns a mapping between system registers in kvm
     * and misc regs in gem5. The actual mapping is only created the
     * first time the method is called and stored in a cache
     * (ArmV8KvmCPU::sysRegMap).
     *
     * @return Vector of kvm<->misc reg mappings.
     */
    const std::vector<ArmV8KvmCPU::MiscRegInfo> &getSysRegMap() const;

    /** Mapping between gem5 integer registers and integer registers in kvm */
    static const std::vector<ArmV8KvmCPU::IntRegInfo> intRegMap;
    /** Mapping between gem5 misc registers and registers in kvm */
    static const std::vector<ArmV8KvmCPU::MiscRegInfo> miscRegMap;
    /** Device registers (needing "effectful" MiscReg writes) */
    static const std::set<ArmISA::MiscRegIndex> deviceRegSet;
    /** Mapping between gem5 ID misc registers and registers in kvm */
    static const std::vector<ArmV8KvmCPU::MiscRegInfo> miscRegIdMap;

    /** Cached mapping between system registers in kvm and misc regs in gem5 */
    mutable std::vector<ArmV8KvmCPU::MiscRegInfo> sysRegMap;
};

} // namespace gem5

#endif // __ARCH_ARM_KVM_ARMV8_CPU_HH__
