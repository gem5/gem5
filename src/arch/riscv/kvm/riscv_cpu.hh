/*
 * Copyright (c) 2026 Ozyegin University CAST Lab
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

#ifndef __ARCH_RISCV_KVM_CPU_HH__
#define __ARCH_RISCV_KVM_CPU_HH__

#include <unordered_set>
#include <vector>

#include "arch/riscv/pcstate.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/random.hh"
#include "cpu/kvm/base.hh"

struct kvm_reg_list;

namespace gem5
{

namespace RiscvISA
{
class ISA;
}

struct RiscvISAParams;
struct RiscvKvmCPUParams;

class RiscvKvmCPU : public BaseKvmCPU
{
  public:
    RiscvKvmCPU(const RiscvKvmCPUParams &params);
    ~RiscvKvmCPU() override = default;

    void startup() override;
    void dump() const override;

  protected:
    Tick kvmRun(Tick ticks) override;
    Tick handleKvmExit() override;
    void ioctlRun() override;

    void updateKvmState() override;
    void updateThreadContext() override;

    void
    stutterPC(PCStateBase &pc) const override
    {
        pc.as<RiscvISA::PCState>().setNPC(pc.instAddr());
    }

  private:
    using RegIndexVector = std::vector<uint64_t>;

    /** Mapping between an integer register in gem5 and KVM */
    struct IntRegInfo
    {
        uint64_t kvmId;
        RegIndex gem5Idx;
        const char *name;
    };

    /** Mapping between a CSR in gem5 and KVM */
    struct MiscRegInfo
    {
        uint64_t kvmId;
        RiscvISA::MiscRegIndex gem5Idx;
        const char *name;
    };

    enum class FpRegMode
    {
        None,
        F,
        D,
    };

    Tick handleKvmExitRiscvSBI();
    Tick handleKvmExitRiscvCSR();

    const RiscvISA::ISA &riscvIsa() const;
    const RiscvISAParams &riscvIsaParams() const;

    void refreshRegList();
    bool getRegList(struct kvm_reg_list &regs) const;
    bool hasReg(uint64_t id) const;

    void configureKvmFeatures();
    void configureKvmConfigRegs();
    void configureKvmIsaExts();

    void updateKvmStateCore();
    void updateKvmStateFP();
    void updateKvmStateCSR();
    void updateKvmStateVec();

    void updateTCCore();
    void updateTCFP();
    void updateTCCSR();
    void updateTCVec();

    /** Build a KVM register ID for a vector register of kvmVlenb bytes */
    uint64_t kvmVecRegId(int regIdx) const;

    /** Map VLENB to the corresponding KVM_REG_SIZE_* constant */
    static uint64_t kvmRegSizeForVlenb(uint64_t vlenb);

    static const std::vector<IntRegInfo> intRegMap;
    static const std::vector<MiscRegInfo> csrMap;

    /** Cached KVM one-reg list for this vCPU. */
    RegIndexVector regIndexList;
    std::unordered_set<uint64_t> regIndexSet;

    /** Deterministic entropy source for CSR_SEED emulation. */
    Random::RandomPtr seedEntropyRng;

    /** Active floating-point one-reg interface. */
    FpRegMode fpRegMode = FpRegMode::D;

    /** S-mode bits visible in sstatus for this guest configuration. */
    uint64_t kvmSstatusMask =
        RiscvISA::SSTATUS_MASKS[RiscvISA::RV64][enums::MSU];

    /** Host VLENB (bytes), queried from KVM at startup. */
    uint64_t kvmVlenb = 0;

    /** KVM timer register interface availability. */
    bool hasKvmTimer = false;

    /** Guest timebase frequency reported by KVM. */
    uint64_t kvmTimerFrequency = 0;
};

} // namespace gem5

#endif // __ARCH_RISCV_KVM_CPU_HH__
