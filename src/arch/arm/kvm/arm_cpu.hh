/*
 * Copyright (c) 2012 ARM Limited
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

#ifndef __ARCH_ARM_KVM_ARM_CPU_HH__
#define __ARCH_ARM_KVM_ARM_CPU_HH__

#include <set>
#include <vector>

#include "cpu/kvm/base.hh"
#include "params/ArmKvmCPU.hh"

/**
 * ARM implementation of a KVM-based hardware virtualized CPU.
 * Architecture specific limitations:
 *  * LPAE is currently not supported by gem5. We therefore panic if LPAE
 *    is enabled when returning to gem5.
 *  * The co-processor based interface to the architected timer is
 *    unsupported. We can't support this due to limitations in the KVM
 *    API on ARM.
 *  * M5 ops are currently not supported. This requires either a kernel
 *  hack or a memory mapped device that handles the guest<->m5
 *  interface.
 */
class ArmKvmCPU : public BaseKvmCPU
{
  public:
    ArmKvmCPU(ArmKvmCPUParams *params);
    virtual ~ArmKvmCPU();

    void startup();

    void dump();

  protected:
    struct KvmIntRegInfo {
        /** KVM ID */
        const uint64_t id;
        /** gem5 index */
        const IntRegIndex idx;
        /** Name in debug output */
        const char *name;
    };

    struct KvmCoreMiscRegInfo {
        /** KVM ID */
        const uint64_t id;
        /** gem5 index */
        const MiscRegIndex idx;
        /** Name in debug output */
        const char *name;
    };

    typedef std::vector<uint64_t> RegIndexVector;

    Tick kvmRun(Tick ticks);

    void updateKvmState();
    void updateThreadContext();

    /**
     * Get a list of registers supported by getOneReg() and setOneReg().
     */
    const RegIndexVector &getRegList() const;

    void kvmArmVCpuInit(uint32_t target);
    void kvmArmVCpuInit(const struct kvm_vcpu_init &init);

    ArmISA::MiscRegIndex decodeCoProcReg(uint64_t id) const;

    ArmISA::MiscRegIndex decodeVFPCtrlReg(uint64_t id) const;

    /**
     * Determine if a register is invariant.
     *
     * Some registers must have the same value in both the host and
     * the guest. Such registers are referred to as "invariant"
     * registers in KVM. This is a restriction imposed by KVM as
     * having different values in ID registers (e.g., the cache
     * identification registers) would confuse the guest kernel.
     */
    bool isInvariantReg(uint64_t id);

    static KvmIntRegInfo kvmIntRegs[];
    static KvmCoreMiscRegInfo kvmCoreMiscRegs[];

  private:
    /**
     * Get a list of registers supported by getOneReg() and setOneReg().
     *
     * @return False if the number of elements allocated in the list
     * is too small to hold the complete register list (the required
     * value is written into n in this case). True on success.
     */
    bool getRegList(struct kvm_reg_list &regs) const;

    void dumpKvmStateCore();
    void dumpKvmStateMisc();
    void dumpKvmStateCoProc(uint64_t id);
    void dumpKvmStateVFP(uint64_t id);

    void updateKvmStateCore();
    void updateKvmStateMisc();
    void updateKvmStateCoProc(uint64_t id, bool show_warnings);
    void updateKvmStateVFP(uint64_t id, bool show_warnings);

    void updateTCStateCore();
    void updateTCStateMisc();
    void updateTCStateCoProc(uint64_t id, bool show_warnings);
    void updateTCStateVFP(uint64_t id, bool show_warnings);


    /** Cached state of the IRQ line */
    bool irqAsserted;
    /** Cached state of the FIQ line */
    bool fiqAsserted;

    /**
     * Cached copy of the list of co-processor registers supported by
     * KVM
     */
    mutable RegIndexVector _regIndexList;

    /**
     * List of co-processor registers that KVM requires to be
     * identical on both the host and the guest. KVM does not allow
     * writes to these registers.
     */
    static const std::set<uint64_t> invariant_regs;
};

#endif // __ARCH_ARM_KVM_ARM_CPU_HH__
