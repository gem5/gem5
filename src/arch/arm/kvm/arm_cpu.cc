/*
 * Copyright (c) 2012, 2020 ARM Limited
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

#include "arch/arm/kvm/arm_cpu.hh"

#include <linux/kvm.h>

#include <algorithm>
#include <cerrno>
#include <memory>
#include <set>

#include "arch/arm/interrupts.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"
#include "cpu/kvm/base.hh"
#include "debug/Kvm.hh"
#include "debug/KvmContext.hh"
#include "debug/KvmInt.hh"
#include "sim/pseudo_inst.hh"

namespace gem5
{

using namespace ArmISA;

namespace
{

constexpr uint64_t
extractField(uint64_t val, uint64_t mask, size_t shift)
{
    return (val & mask) >> shift;
}

constexpr bool
regIsArm(uint64_t id)
{
    return (id & KVM_REG_ARCH_MASK) == KVM_REG_ARM;
}

constexpr bool
regIs32Bit(uint64_t id)
{
    return (id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U32;
}

constexpr bool
regIs64Bit(uint64_t id)
{
    return (id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U64;
}

constexpr bool
regIsCp(uint64_t id, uint64_t cp)
{
    return (id & KVM_REG_ARM_COPROC_MASK) == cp;
}

constexpr bool
regIsCore(uint64_t id)
{
    return regIsCp(id, KVM_REG_ARM_CORE);
}

constexpr bool
regIsVfp(uint64_t id)
{
    return regIsCp(id, KVM_REG_ARM_VFP);
}

constexpr uint64_t
regVfpReg(uint64_t id)
{
    return id & KVM_REG_ARM_VFP_MASK;
}

// HACK: These aren't really defined in any of the headers, so we'll
// assume some reasonable values for now.
constexpr bool
regIsVfpReg(uint64_t id)
{
    return regVfpReg(id) < 0x100;
}
constexpr bool
regIsVfpCtrl(uint64_t id)
{
    return regVfpReg(id) >= 0x100;
}

constexpr bool
regIsDemux(uint64_t id)
{
    return regIsCp(id, KVM_REG_ARM_DEMUX);
}


// There is no constant in the kernel headers defining the mask to use
// to get the core register index. We'll just do what they do
// internally.
constexpr uint64_t
regCoreIdx(uint64_t id)
{
    return ~(KVM_REG_ARCH_MASK | KVM_REG_SIZE_MASK | KVM_REG_ARM_CORE);
}

constexpr uint64_t
regCp(uint64_t id)
{
    return extractField(id, KVM_REG_ARM_COPROC_MASK, KVM_REG_ARM_COPROC_SHIFT);
}

constexpr uint64_t
regCrn(uint64_t id)
{
    return extractField(id, KVM_REG_ARM_32_CRN_MASK, KVM_REG_ARM_32_CRN_SHIFT);
}

constexpr uint64_t
regOpc1(uint64_t id)
{
    return extractField(id, KVM_REG_ARM_OPC1_MASK, KVM_REG_ARM_OPC1_SHIFT);
}

constexpr uint64_t
regCrm(uint64_t id)
{
    return extractField(id, KVM_REG_ARM_CRM_MASK, KVM_REG_ARM_CRM_SHIFT);
}

constexpr uint64_t
regOpc2(uint64_t id)
{
    return extractField(id, KVM_REG_ARM_32_OPC2_MASK,
            KVM_REG_ARM_32_OPC2_SHIFT);
}

constexpr uint64_t
regCp32(uint64_t cpnum, uint64_t crn, uint64_t opc1, uint64_t crm,
        uint64_t opc2)
{
    return KVM_REG_ARM | KVM_REG_SIZE_U32 |
        (cpnum << KVM_REG_ARM_COPROC_SHIFT) |
        (crn << KVM_REG_ARM_32_CRN_SHIFT) |
        (opc1 << KVM_REG_ARM_OPC1_SHIFT) |
        (crm << KVM_REG_ARM_CRM_SHIFT) |
        (opc2 << KVM_REG_ARM_32_OPC2_SHIFT);
}

constexpr uint64_t
regCp64(uint64_t cpnum, uint64_t opc1, uint64_t crm)
{
    return KVM_REG_ARM | KVM_REG_SIZE_U64 |
        (cpnum << KVM_REG_ARM_COPROC_SHIFT) |
        (opc1 << KVM_REG_ARM_OPC1_SHIFT) |
        (crm << KVM_REG_ARM_CRM_SHIFT);
}

constexpr KvmIntRegInfo
regCore32(off_t offset, RegIndex idx, const char *name)
{
    return { KVM_REG_ARM | KVM_REG_SIZE_U32 | KVM_REG_ARM_CORE | offset,
             idx, name };
}

constexpr uint64_t
regVfp32(uint64_t regno)
{
    return KVM_REG_ARM | KVM_REG_SIZE_U32 | KVM_REG_ARM_VFP | regno;
}

constexpr uint64_t
regVfp64(uint64_t regno)
{
    return KVM_REG_ARM | KVM_REG_SIZE_U64 | KVM_REG_ARM_VFP | regno;
}

constexpr uint64_t
regDemux32(uint64_t dmxid, uint64_t val)
{
    return KVM_REG_ARM | KVM_REG_SIZE_U32 | dmxid | val;
}

constexpr uint64_t
interruptId(uint64_t type, uint64_t vcpu, uint64_t irq)
{
    return (type << KVM_ARM_IRQ_TYPE_SHIFT) |
        (vcpu << KVM_ARM_IRQ_VCPU_SHIFT) |
        (irq << KVM_ARM_IRQ_NUM_SHIFT);
}

constexpr uint64_t
interruptVcpuIrq(uint64_t vcpu)
{
    return interruptId(KVM_ARM_IRQ_TYPE_CPU, vcpu, KVM_ARM_IRQ_CPU_IRQ);
}

constexpr uint64_t
interruptVcpuFiq(uint64_t vcpu)
{
    return interruptId(KVM_ARM_IRQ_TYPE_CPU, vcpu, KVM_ARM_IRQ_CPU_FIQ);
}

} // anonymous namespace

// Some of the co-processor registers are invariants and must have the
// same value on both the host and the guest. We need to keep a list
// of these to prevent gem5 from fiddling with them on the guest.
static uint64_t invariant_reg_vector[] = {
    regCp32(15, 0, 0, 0, 0), // MIDR
    regCp32(15, 0, 0, 0, 1), // CTR
    regCp32(15, 0, 0, 0, 2), // TCMTR
    regCp32(15, 0, 0, 0, 3), // TLBTR
    regCp32(15, 0, 0, 0, 6), // REVIDR

    regCp32(15, 0, 0, 1, 0), // ID_PFR0
    regCp32(15, 0, 0, 1, 1), // ID_PFR1
    regCp32(15, 0, 0, 1, 2), // ID_DFR0
    regCp32(15, 0, 0, 1, 3), // ID_AFR0
    regCp32(15, 0, 0, 1, 4), // ID_MMFR0
    regCp32(15, 0, 0, 1, 5), // ID_MMFR1
    regCp32(15, 0, 0, 1, 6), // ID_MMFR2
    regCp32(15, 0, 0, 1, 7), // ID_MMFR3

    regCp32(15, 0, 0, 2, 0), // ID_ISAR0
    regCp32(15, 0, 0, 2, 1), // ID_ISAR1
    regCp32(15, 0, 0, 2, 2), // ID_ISAR2
    regCp32(15, 0, 0, 2, 3), // ID_ISAR3
    regCp32(15, 0, 0, 2, 4), // ID_ISAR4
    regCp32(15, 0, 0, 2, 5), // ID_ISAR5
    regCp32(15, 0, 0, 2, 6), // ID_MMFR4
    regCp32(15, 0, 0, 2, 7), // ID_ISAR6

    regCp32(15, 0, 1, 0, 0), // CSSIDR
    regCp32(15, 0, 1, 0, 1), // CLIDR
    regCp32(15, 0, 1, 0, 7), // AIDR

    regVfp32(KVM_REG_ARM_VFP_MVFR0),
    regVfp32(KVM_REG_ARM_VFP_MVFR1),
    regVfp32(KVM_REG_ARM_VFP_FPSID),

    regDemux32(KVM_REG_ARM_DEMUX_ID_CCSIDR, 0),
};

const static uint64_t KVM_REG64_TTBR0(regCp64(15, 0, 2));
const static uint64_t KVM_REG64_TTBR1(regCp64(15, 1, 2));


const std::set<uint64_t> ArmKvmCPU::invariant_regs(
    std::begin(invariant_reg_vector), std::end(invariant_reg_vector));


ArmKvmCPU::KvmIntRegInfo ArmKvmCPU::kvmIntRegs[] = {
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r0), int_reg::R0, "R0"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r1), int_reg::R1, "R1"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r2), int_reg::R2, "R2"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r3), int_reg::R3, "R3"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r4), int_reg::R4, "R4"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r5), int_reg::R5, "R5"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r6), int_reg::R6, "R6"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r7), int_reg::R7, "R7"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r8), int_reg::R8, "R8"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r9), int_reg::R9, "R9"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_r10), int_reg::R10, "R10"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_fp), int_reg::R11, "R11"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_ip), int_reg::R12, "R12"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_sp), int_reg::R13, "R13(USR)"),
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_lr), int_reg::R14, "R14(USR)"),

    regCore32(KVM_REG_ARM_CORE_REG(svc_regs[0]), int_reg::SpSvc, "R13(SVC)"),
    regCore32(KVM_REG_ARM_CORE_REG(svc_regs[1]), int_reg::LrSvc, "R14(SVC)"),

    regCore32(KVM_REG_ARM_CORE_REG(abt_regs[0]), int_reg::SpAbt, "R13(ABT)"),
    regCore32(KVM_REG_ARM_CORE_REG(abt_regs[1]), int_reg::LrAbt, "R14(ABT)"),

    regCore32(KVM_REG_ARM_CORE_REG(und_regs[0]), int_reg::SpUnd, "R13(UND)"),
    regCore32(KVM_REG_ARM_CORE_REG(und_regs[1]), int_reg::LrUnd, "R14(UND)"),

    regCore32(KVM_REG_ARM_CORE_REG(irq_regs[0]), int_reg::SpIrq, "R13(IRQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(irq_regs[1]), int_reg::LrIrq, "R14(IRQ)"),


    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[0]), int_reg::R8Fiq, "R8(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[1]), int_reg::R9Fiq, "R9(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[2]), int_reg::R10Fiq, "R10(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[3]), int_reg::R11Fiq, "R11(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[4]), int_reg::R12Fiq, "R12(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[5]), int_reg::R13Fiq, "R13(FIQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[6]), int_reg::R14Fiq, "R14(FIQ)"),
    { 0, int_reg::NumRegs, NULL }
};

ArmKvmCPU::KvmCoreMiscRegInfo ArmKvmCPU::kvmCoreMiscRegs[] = {
    regCore32(KVM_REG_ARM_CORE_REG(usr_regs.ARM_cpsr), MISCREG_CPSR, "CPSR"),
    regCore32(KVM_REG_ARM_CORE_REG(svc_regs[2]), MISCREG_SPSR_SVC,
            "SPSR(SVC)"),
    regCore32(KVM_REG_ARM_CORE_REG(abt_regs[2]), MISCREG_SPSR_ABT,
            "SPSR(ABT)"),
    regCore32(KVM_REG_ARM_CORE_REG(und_regs[2]), MISCREG_SPSR_UND,
            "SPSR(UND)"),
    regCore32(KVM_REG_ARM_CORE_REG(irq_regs[2]), MISCREG_SPSR_IRQ,
            "SPSR(IRQ)"),
    regCore32(KVM_REG_ARM_CORE_REG(fiq_regs[2]), MISCREG_SPSR_FIQ,
            "SPSR(FIQ)"),
    { 0, NUM_MISCREGS }
};

ArmKvmCPU::ArmKvmCPU(const ArmKvmCPUParams &params)
    : BaseKvmCPU(params),
      irqAsserted(false), fiqAsserted(false)
{
}

ArmKvmCPU::~ArmKvmCPU()
{
}

void
ArmKvmCPU::startup()
{
    BaseKvmCPU::startup();

    /* TODO: This needs to be moved when we start to support VMs with
     * multiple threads since kvmArmVCpuInit requires that all CPUs in
     * the VM have been created.
     */
    /* TODO: The CPU type needs to be configurable once KVM on ARM
     * starts to support more CPUs.
     */
    kvmArmVCpuInit(KVM_ARM_TARGET_CORTEX_A15);
}

Tick
ArmKvmCPU::kvmRun(Tick ticks)
{
    auto interrupt = static_cast<ArmISA::Interrupts *>(interrupts[0]);
    const bool simFIQ = interrupt->checkRaw(INT_FIQ);
    const bool simIRQ = interrupt->checkRaw(INT_IRQ);

    if (fiqAsserted != simFIQ) {
        fiqAsserted = simFIQ;
        DPRINTF(KvmInt, "KVM: Update FIQ state: %i\n", simFIQ);
        vm->setIRQLine(interruptVcpuFiq(vcpuID), simFIQ);
    }
    if (irqAsserted != simIRQ) {
        irqAsserted = simIRQ;
        DPRINTF(KvmInt, "KVM: Update IRQ state: %i\n", simIRQ);
        vm->setIRQLine(interruptVcpuIrq(vcpuID), simIRQ);
    }

    return BaseKvmCPU::kvmRun(ticks);
}

void
ArmKvmCPU::dump()
{
    dumpKvmStateCore();
    dumpKvmStateMisc();
}

void
ArmKvmCPU::updateKvmState()
{
    DPRINTF(KvmContext, "Updating KVM state...\n");

    updateKvmStateCore();
    updateKvmStateMisc();
}

void
ArmKvmCPU::updateThreadContext()
{
    DPRINTF(KvmContext, "Updating gem5 state...\n");

    updateTCStateCore();
    updateTCStateMisc();
}

const ArmKvmCPU::RegIndexVector &
ArmKvmCPU::getRegList() const
{
    if (_regIndexList.size() == 0) {
        std::unique_ptr<struct kvm_reg_list, void(*)(void *p)>
            regs(nullptr, [](void *p) { operator delete(p); });
        uint64_t i = 1;

        do {
            i <<= 1;
            regs.reset((struct kvm_reg_list *)
                       operator new(sizeof(struct kvm_reg_list) +
                                    i * sizeof(uint64_t)));
            regs->n = i;
        } while (!getRegList(*regs));
        _regIndexList.assign(regs->reg,
                             regs->reg + regs->n);
    }

    return _regIndexList;
}

void
ArmKvmCPU::kvmArmVCpuInit(uint32_t target)
{
    struct kvm_vcpu_init init;

    memset(&init, 0, sizeof(init));

    init.target = target;

    kvmArmVCpuInit(init);
}

void
ArmKvmCPU::kvmArmVCpuInit(const struct kvm_vcpu_init &init)
{
    if (ioctl(KVM_ARM_VCPU_INIT, (void *)&init) == -1)
        panic("KVM: Failed to initialize vCPU\n");
}

MiscRegIndex
ArmKvmCPU::decodeCoProcReg(uint64_t id) const
{
    const unsigned cp = regCp(id);
    const bool is_reg32 = regIs32Bit(id);
    const bool is_reg64 = regIs64Bit(id);

    // CP numbers larger than 15 are reserved for KVM extensions
    if (cp > 15)
        return NUM_MISCREGS;

    const unsigned crm = regCrm(id);
    const unsigned crn = regCrn(id);
    const unsigned opc1 = regOpc1(id);
    const unsigned opc2 = regOpc2(id);

    if (is_reg32) {
        switch (cp) {
          case 14:
            return decodeCP14Reg(crn, opc1, crm, opc2);

          case 15:
            return decodeCP15Reg(crn, opc1, crm, opc2);

          default:
            return NUM_MISCREGS;
        }
    } else if (is_reg64) {
        return NUM_MISCREGS;
    } else {
        warn("Unhandled register length, register (0x%x) ignored.\n");
        return NUM_MISCREGS;
    }
}

ArmISA::MiscRegIndex
ArmKvmCPU::decodeVFPCtrlReg(uint64_t id) const
{
    if (!regIsArm(id) || !regIsVfp(id) || !regIsVfpCtrl(id))
        return NUM_MISCREGS;

    const unsigned vfp_reg = regVfpReg(id);
    switch (vfp_reg) {
      case KVM_REG_ARM_VFP_FPSID: return MISCREG_FPSID;
      case KVM_REG_ARM_VFP_FPSCR: return MISCREG_FPSCR;
      case KVM_REG_ARM_VFP_MVFR0: return MISCREG_MVFR0;
      case KVM_REG_ARM_VFP_MVFR1: return MISCREG_MVFR1;
      case KVM_REG_ARM_VFP_FPEXC: return MISCREG_FPEXC;

      case KVM_REG_ARM_VFP_FPINST:
      case KVM_REG_ARM_VFP_FPINST2:
        warn_once("KVM: FPINST not implemented.\n");
        return NUM_MISCREGS;

      default:
        return NUM_MISCREGS;
    }
}

bool
ArmKvmCPU::isInvariantReg(uint64_t id)
{
    /* Mask away the value field from multiplexed registers, we assume
     * that entire groups of multiplexed registers can be treated as
     * invariant. */
    if (regIsArm(id) && regIsDemux(id))
        id &= ~KVM_REG_ARM_DEMUX_VAL_MASK;

    return invariant_regs.find(id) != invariant_regs.end();
}

bool
ArmKvmCPU::getRegList(struct kvm_reg_list &regs) const
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

void
ArmKvmCPU::dumpKvmStateCore()
{
    /* Print core registers */
    uint32_t pc = getOneRegU32(REG_CORE32(usr_regs.ARM_pc));
    inform("PC: 0x%x\n", pc);

    for (const KvmIntRegInfo *ri(kvmIntRegs);
         ri->idx != int_reg::NumRegs; ++ri) {

        uint32_t value(getOneRegU32(ri->id));
        inform("%s: 0x%x\n", ri->name, value);
    }

    for (const KvmCoreMiscRegInfo *ri(kvmCoreMiscRegs);
         ri->idx != NUM_MISCREGS; ++ri) {

        uint32_t value(getOneRegU32(ri->id));
        inform("%s: 0x%x\n", miscRegName[ri->idx], value);
    }
}

void
ArmKvmCPU::dumpKvmStateMisc()
{
    /* Print co-processor registers */
    const RegIndexVector &reg_ids = getRegList();
    for (RegIndexVector::const_iterator it(reg_ids.begin());
         it != reg_ids.end(); ++it) {
        uint64_t id = *it;

        if (regIsArm(id) && regCp(id) <= 15) {
            dumpKvmStateCoProc(id);
        } else if (regIsArm(id) && regIsVfp(id)) {
            dumpKvmStateVFP(id);
        } else if (regIsArm(id) && regIsDemux(id)) {
            switch (id & KVM_REG_ARM_DEMUX_ID_MASK) {
              case KVM_REG_ARM_DEMUX_ID_CCSIDR:
                inform("CCSIDR [0x%x]: %s\n",
                       extractField(id, KVM_REG_ARM_DEMUX_VAL_MASK,
                           KVM_REG_ARM_DEMUX_VAL_SHIFT),
                       getAndFormatOneReg(id));
                break;
              default:
                inform("DEMUX [0x%x, 0x%x]: %s\n",
                       extractField(id, KVM_REG_ARM_DEMUX_ID_MASK,
                           KVM_REG_ARM_DEMUX_ID_SHIFT),
                       extractField(id, KVM_REG_ARM_DEMUX_VAL_MASK,
                           KVM_REG_ARM_DEMUX_VAL_SHIFT),
                       getAndFormatOneReg(id));
                break;
            }
        } else if (!regIsCore(id)) {
            inform("0x%x: %s\n", id, getAndFormatOneReg(id));
        }
    }
}

void
ArmKvmCPU::dumpKvmStateCoProc(uint64_t id)
{
    assert(regIsArm(id));
    assert(regCp(id) <= 15);

    if (regIs32Bit(id)) {
        // 32-bit co-proc registers
        MiscRegIndex idx = decodeCoProcReg(id);
        uint32_t value = getOneRegU32(id);

        if (idx != NUM_MISCREGS &&
            !(idx >= MISCREG_CP15_UNIMP_START && idx < MISCREG_CP15_END)) {
            const char *name = miscRegName[idx];
            const unsigned m5_ne = tc->readMiscRegNoEffect(idx);
            const unsigned m5_e = tc->readMiscReg(idx);
            inform("CP%i: [CRn: c%i opc1: %.2i CRm: c%i opc2: %i inv: %i]: "
                   "[%s]: 0x%x/0x%x\n",
                   regCp(id), regCrn(id), regOpc1(id), regCrm(id),
                   regOpc2(id), isInvariantReg(id),
                   name, value, m5_e);
            if (m5_e != m5_ne) {
                inform("readMiscReg: %x, readMiscRegNoEffect: %x\n",
                       m5_e, m5_ne);
            }
        } else {
            const char *name = idx != NUM_MISCREGS ? miscRegName[idx] : "-";
            inform("CP%i: [CRn: c%i opc1: %.2i CRm: c%i opc2: %i inv: %i]: "
                    "[%s]: 0x%x\n",
                   regCp(id), regCrn(id), regOpc1(id), regCrm(id),
                   regOpc2(id), isInvariantReg(id), name, value);
        }
    } else {
        inform("CP%i: [CRn: c%i opc1: %.2i CRm: c%i opc2: %i inv: %i "
               "len: 0x%x]: %s\n",
               regCp(id), regCrn(id), regOpc1(id), regCrm(id),
               regOpc2(id), isInvariantReg(id),
               extractField(id, KVM_REG_SIZE_MASK, KVM_REG_SIZE_SHIFT),
               getAndFormatOneReg(id));
    }
}

void
ArmKvmCPU::dumpKvmStateVFP(uint64_t id)
{
    assert(regIsArm(id));
    assert(regIsVfp(id));

    if (regIsVfpReg(id)) {
        const unsigned idx = id & KVM_REG_ARM_VFP_MASK;
        inform("VFP reg %i: %s", idx, getAndFormatOneReg(id));
    } else if (regIsVfpCtrl(id)) {
        MiscRegIndex idx = decodeVFPCtrlReg(id);
        if (idx != NUM_MISCREGS) {
            inform("VFP [%s]: %s", miscRegName[idx], getAndFormatOneReg(id));
        } else {
            inform("VFP [0x%x]: %s", id, getAndFormatOneReg(id));
        }
    } else {
        inform("VFP [0x%x]: %s", id, getAndFormatOneReg(id));
    }
}

void
ArmKvmCPU::updateKvmStateCore()
{
    for (const KvmIntRegInfo *ri(kvmIntRegs);
         ri->idx != init_reg::NumRegs; ++ri) {

        uint64_t value = tc->getReg(flatIntRegClass[ri->idx]);
        DPRINTF(KvmContext, "kvm(%s) := 0x%x\n", ri->name, value);
        setOneReg(ri->id, value);
    }

    DPRINTF(KvmContext, "kvm(PC) := 0x%x\n", tc->pcState().instAddr());
    setOneReg(REG_CORE32(usr_regs.ARM_pc), tc->pcState().instAddr());

    for (const KvmCoreMiscRegInfo *ri(kvmCoreMiscRegs);
         ri->idx != NUM_MISCREGS; ++ri) {

        uint64_t value = tc->readMiscReg(ri->idx);
        DPRINTF(KvmContext, "kvm(%s) := 0x%x\n", ri->name, value);
        setOneReg(ri->id, value);
    }

    if (debug::KvmContext)
        dumpKvmStateCore();
}

void
ArmKvmCPU::updateKvmStateMisc()
{
    static bool warned = false; // We can't use warn_once since we want
                                // to show /all/ registers

    const RegIndexVector &regs = getRegList();

    for (RegIndexVector::const_iterator it(regs.begin());
         it != regs.end();
         ++it) {

        if (!regIsArm(*it)) {
            if (!warned)
                warn("Skipping non-ARM register: 0x%x\n", *it);
        } else if (isInvariantReg(*it)) {
            DPRINTF(Kvm, "Skipping invariant register: 0x%x\n", *it);
        } else if (regIsCore(*it)) {
            // Core registers are handled in updateKvmStateCore
            continue;
        } else if (regCp(*it) <= 15) {
            updateKvmStateCoProc(*it, !warned);
        } else if (regIsVfp(*it)) {
            updateKvmStateVFP(*it, !warned);
        } else {
            if (!warned) {
                warn("Skipping register with unknown CP (%i) id: 0x%x\n",
                     regCp(*it), *it);
            }
        }

    }

    warned = true;
    if (debug::KvmContext)
        dumpKvmStateMisc();
}

void
ArmKvmCPU::updateKvmStateCoProc(uint64_t id, bool show_warnings)
{
    MiscRegIndex reg = decodeCoProcReg(id);

    assert(regIsArm(id));
    assert(regCp(id) <= 15);

    if (id == KVM_REG64_TTBR0 || id == KVM_REG64_TTBR1) {
        // HACK HACK HACK: Workaround for 64-bit TTBRx
        reg = (id == KVM_REG64_TTBR0 ? MISCREG_TTBR0 : MISCREG_TTBR1);
        if (show_warnings)
            hack("KVM: 64-bit TTBBRx workaround\n");
    }

    if (reg == NUM_MISCREGS) {
        if (show_warnings) {
            warn("KVM: Ignoring unknown KVM co-processor register (0x%.8x):\n",
                 id);
            warn("\t0x%x: [CP: %i 64: %i CRn: c%i opc1: %.2i CRm: c%i"
                 " opc2: %i]\n",
                 id, regCp(id), regIs64Bit(id), regCrn(id),
                 regOpc1(id), regCrm(id), regOpc2(id));
        }
    } else if (reg >= MISCREG_CP15_UNIMP_START && reg < MISCREG_CP15_END) {
        if (show_warnings)
            warn("KVM: Co-processor reg. %s not implemented by gem5.\n",
                 miscRegName[reg]);
    } else {
        setOneReg(id, tc->readMiscRegNoEffect(reg));
    }
}


void
ArmKvmCPU::updateKvmStateVFP(uint64_t id, bool show_warnings)
{
    assert(regIsArm(id));
    assert(regIsVfp(id));

    if (regIsVfpReg(id)) {
        if (!regIs64Bit(id)) {
            if (show_warnings)
                warn("Unexpected VFP register length (reg: 0x%x).\n", id);
            return;
        }
        const unsigned idx = id & KVM_REG_ARM_VFP_MASK;
        const unsigned idx_base = idx << 1;
        const unsigned idx_hi = idx_base + 1;
        const unsigned idx_lo = idx_base + 0;
        uint64_t value = (tc->getReg(floatRegClass[idx_hi]) << 32) |
            tc->getReg(floatRegClass[idx_lo]);

        setOneReg(id, value);
    } else if (regIsVfpCtrl(id)) {
        MiscRegIndex idx = decodeVFPCtrlReg(id);
        if (idx == NUM_MISCREGS) {
            if (show_warnings)
                warn("Unhandled VFP control register: 0x%x\n", id);
            return;
        }
        if (!regIs32Bit(id)) {
            if (show_warnings)
                warn("Ignoring VFP control register (%s) with "
                     "unexpected size.\n",
                     miscRegName[idx]);
            return;
        }
        setOneReg(id, (uint32_t)tc->readMiscReg(idx));
    } else {
        if (show_warnings)
            warn("Unhandled VFP register: 0x%x\n", id);
    }
}

void
ArmKvmCPU::updateTCStateCore()
{
    for (const KvmIntRegInfo *ri(kvmIntRegs);
         ri->idx != int_reg::NumRegs; ++ri) {

        tc->setReg(intRegClass[ri->idx], getOneRegU32(ri->id));
    }

    for (const KvmCoreMiscRegInfo *ri(kvmCoreMiscRegs);
         ri->idx != NUM_MISCREGS; ++ri) {

        tc->setMiscRegNoEffect(ri->idx, getOneRegU32(ri->id));
    }

    /* We want the simulator to execute all side-effects of the CPSR
     * update since this updates PC state and register maps.
     */
    tc->setMiscReg(MISCREG_CPSR, tc->readMiscRegNoEffect(MISCREG_CPSR));

    // We update the PC state after we have updated the CPSR the
    // contents of the CPSR affects how the npc is updated.
    PCState pc = tc->pcState().as<PCState>();
    pc.set(getOneRegU32(REG_CORE32(usr_regs.ARM_pc)));
    tc->pcState(pc);

    if (debug::KvmContext)
        dumpKvmStateCore();
}

void
ArmKvmCPU::updateTCStateMisc()
{
    static bool warned(false); // We can't use warn_once since we want
                               // to show /all/ registers

    const RegIndexVector &reg_ids = getRegList();
    for (RegIndexVector::const_iterator it(reg_ids.begin());
         it != reg_ids.end(); ++it) {

        if (!regIsArm(*it)) {
            if (!warned)
                warn("Skipping non-ARM register: 0x%x\n", *it);
        } else if (regIsCore(*it)) {
            // Core registers are handled in updateKvmStateCore
        } else if (regCp(*it) <= 15) {
            updateTCStateCoProc(*it, !warned);
        } else if (regIsVfp(*it)) {
            updateTCStateVFP(*it, !warned);
        } else {
            if (!warned) {
                warn("Skipping register with unknown CP (%i) id: 0x%x\n",
                     regCp(*it), *it);
            }
        }
    }

    warned = true;

    if (debug::KvmContext)
        dumpKvmStateMisc();
}

void
ArmKvmCPU::updateTCStateCoProc(uint64_t id, bool show_warnings)
{
    MiscRegIndex reg = decodeCoProcReg(id);

    assert(regIsArm(id));
    assert(regCp(id) <= 15);

    if (id == KVM_REG64_TTBR0 || id == KVM_REG64_TTBR1) {
        // HACK HACK HACK: We don't currently support 64-bit TTBR0/TTBR1
        hack_once("KVM: 64-bit TTBRx workaround\n");
        tc->setMiscRegNoEffect(
            id == KVM_REG64_TTBR0 ? MISCREG_TTBR0 : MISCREG_TTBR1,
            (uint32_t)(getOneRegU64(id) & 0xFFFFFFFF));
    } else if (reg == MISCREG_TTBCR) {
        uint32_t value = getOneRegU64(id);
        if (value & 0x80000000)
            panic("KVM: Guest tried to enable LPAE.\n");
        tc->setMiscRegNoEffect(reg, value);
    } else if (reg == NUM_MISCREGS) {
        if (show_warnings) {
            warn("KVM: Ignoring unknown KVM co-processor register:\n", id);
            warn("\t0x%x: [CP: %i 64: %i CRn: c%i opc1: %.2i CRm: c%i"
                 " opc2: %i]\n",
                 id, regCp(id), regIs64Bit(id), regCrn(id),
                 regOpc1(id), regCrm(id), regOpc2(id));
        }
    } else if (reg >= MISCREG_CP15_UNIMP_START && reg < MISCREG_CP15_END) {
        if (show_warnings)
            warn_once("KVM: Co-processor reg. %s not implemented by gem5.\n",
                      miscRegName[reg]);
    } else {
        tc->setMiscRegNoEffect(reg, getOneRegU32(id));
    }
}

void
ArmKvmCPU::updateTCStateVFP(uint64_t id, bool show_warnings)
{
    assert(regIsArm(id));
    assert(regIsVfp(id));

    if (regIsVfpReg(id)) {
        if (!regIs64Bit(id)) {
            if (show_warnings)
                warn("Unexpected VFP register length (reg: 0x%x).\n", id);
            return;
        }
        const unsigned idx = id & KVM_REG_ARM_VFP_MASK;
        const unsigned idx_base = idx << 1;
        const unsigned idx_hi = idx_base + 1;
        const unsigned idx_lo = idx_base + 0;
        uint64_t value = getOneRegU64(id);

        tc->setReg(floatRegClass[idx_hi], (value >> 32) & 0xFFFFFFFF);
        tc->setReg(floatRegClass[idx_lo], value & 0xFFFFFFFF);
    } else if (regIsVfpCtrl(id)) {
        MiscRegIndex idx = decodeVFPCtrlReg(id);
        if (idx == NUM_MISCREGS) {
            if (show_warnings)
                warn("Unhandled VFP control register: 0x%x\n", id);
            return;
        }
        if (!regIs32Bit(id)) {
            if (show_warnings)
                warn("Ignoring VFP control register (%s) with "
                     "unexpected size.\n",
                     miscRegName[idx]);
            return;
        }
        tc->setMiscReg(idx, getOneRegU64(id));
    } else {
        if (show_warnings)
            warn("Unhandled VFP register: 0x%x\n", id);
    }
}

} // namespace gem5
