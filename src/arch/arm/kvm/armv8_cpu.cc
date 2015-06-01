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

#include "arch/arm/kvm/armv8_cpu.hh"

#include <linux/kvm.h>

#include "debug/KvmContext.hh"
#include "params/ArmV8KvmCPU.hh"

// Unlike gem5, kvm doesn't count the SP as a normal integer register,
// which means we only have 31 normal integer registers.
constexpr static unsigned NUM_XREGS = NUM_ARCH_INTREGS - 1;
static_assert(NUM_XREGS == 31, "Unexpected number of aarch64 int. regs.");

// The KVM interface accesses vector registers of 4 single precision
// floats instead of individual registers.
constexpr static unsigned NUM_QREGS = NumFloatV8ArchRegs / 4;
static_assert(NUM_QREGS == 32, "Unexpected number of aarch64 vector regs.");

#define EXTRACT_FIELD(v, name) \
    (((v) & name ## _MASK) >> name ## _SHIFT)

#define CORE_REG(name, size)                               \
    (KVM_REG_ARM64 | KVM_REG_ARM_CORE |                    \
     KVM_REG_SIZE_ ## size |                               \
     KVM_REG_ARM_CORE_REG(name))

#define INT_REG(name) CORE_REG(name, U64)
#define SIMD_REG(name) CORE_REG(name, U128)

constexpr uint64_t
kvmXReg(const int num)
{
    return INT_REG(regs.regs[0]) +
        (INT_REG(regs.regs[1]) - INT_REG(regs.regs[0])) * num;
}

constexpr uint64_t
kvmFPReg(const int num)
{
    return SIMD_REG(fp_regs.vregs[0]) +
        (SIMD_REG(fp_regs.vregs[1]) - SIMD_REG(fp_regs.vregs[0])) * num;
}

union KvmFPReg {
    union {
        uint32_t i;
        float f;
    } s[4];

    union {
        uint64_t i;
        double f;
    } d[2];

    uint8_t data[32];
};

#define FP_REGS_PER_VFP_REG 4
static_assert(sizeof(FloatRegBits) == 4, "Unexpected float reg size");

const std::vector<ArmV8KvmCPU::IntRegInfo> ArmV8KvmCPU::intRegMap = {
    { INT_REG(regs.sp), INTREG_SP0, "SP(EL0)" },
    { INT_REG(sp_el1), INTREG_SP1, "SP(EL1)" },
};

const std::vector<ArmV8KvmCPU::MiscRegInfo> ArmV8KvmCPU::miscRegMap = {
    MiscRegInfo(INT_REG(regs.pstate), MISCREG_CPSR, "PSTATE"),
    MiscRegInfo(INT_REG(elr_el1), MISCREG_ELR_EL1, "ELR(EL1)"),
    MiscRegInfo(INT_REG(spsr[KVM_SPSR_EL1]), MISCREG_SPSR_EL1, "SPSR(EL1)"),
    MiscRegInfo(INT_REG(spsr[KVM_SPSR_ABT]), MISCREG_SPSR_ABT, "SPSR(ABT)"),
    MiscRegInfo(INT_REG(spsr[KVM_SPSR_UND]), MISCREG_SPSR_UND, "SPSR(UND)"),
    MiscRegInfo(INT_REG(spsr[KVM_SPSR_IRQ]), MISCREG_SPSR_IRQ, "SPSR(IRQ)"),
    MiscRegInfo(INT_REG(spsr[KVM_SPSR_FIQ]), MISCREG_SPSR_FIQ, "SPSR(FIQ)"),
    MiscRegInfo(INT_REG(fp_regs.fpsr), MISCREG_FPSR, "FPSR"),
    MiscRegInfo(INT_REG(fp_regs.fpcr), MISCREG_FPCR, "FPCR"),
};

ArmV8KvmCPU::ArmV8KvmCPU(ArmV8KvmCPUParams *params)
    : BaseArmKvmCPU(params)
{
}

ArmV8KvmCPU::~ArmV8KvmCPU()
{
}

void
ArmV8KvmCPU::dump()
{
    inform("Integer registers:\n");
    inform("  PC: %s\n", getAndFormatOneReg(INT_REG(regs.pc)));
    for (int i = 0; i < NUM_XREGS; ++i)
        inform("  X%i: %s\n", i, getAndFormatOneReg(kvmXReg(i)));

    for (int i = 0; i < NUM_QREGS; ++i)
        inform("  Q%i: %s\n", i, getAndFormatOneReg(kvmFPReg(i)));

    for (const auto &ri : intRegMap)
        inform("  %s: %s\n", ri.name, getAndFormatOneReg(ri.kvm));

    for (const auto &ri : miscRegMap)
        inform("  %s: %s\n", ri.name, getAndFormatOneReg(ri.kvm));

    for (const auto &reg : getRegList()) {
        const uint64_t arch(reg & KVM_REG_ARCH_MASK);
        if (arch != KVM_REG_ARM64) {
            inform("0x%x: %s\n", reg, getAndFormatOneReg(reg));
            continue;
        }

        const uint64_t type(reg & KVM_REG_ARM_COPROC_MASK);
        switch (type) {
          case KVM_REG_ARM_CORE:
            // These have already been printed
            break;

          case KVM_REG_ARM64_SYSREG: {
              const uint64_t op0(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP0));
              const uint64_t op1(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP1));
              const uint64_t crn(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_CRN));
              const uint64_t crm(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_CRM));
              const uint64_t op2(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP2));
              const MiscRegIndex idx(
                  decodeAArch64SysReg(op0, op1, crn, crm, op2));

              inform("  %s (op0: %i, op1: %i, crn: %i, crm: %i, op2: %i): %s",
                     miscRegName[idx], op0, op1, crn, crm, op2,
                     getAndFormatOneReg(reg));
          } break;

          case KVM_REG_ARM_DEMUX: {
              const uint64_t id(EXTRACT_FIELD(reg, KVM_REG_ARM_DEMUX_ID));
              const uint64_t val(EXTRACT_FIELD(reg, KVM_REG_ARM_DEMUX_VAL));
              if (id == KVM_REG_ARM_DEMUX_ID_CCSIDR) {
                  inform("  CSSIDR[%i]: %s\n", val,
                         getAndFormatOneReg(reg));
              } else {
                  inform("  UNKNOWN[%i:%i]: %s\n", id, val,
                         getAndFormatOneReg(reg));
              }
          } break;

          default:
            inform("0x%x: %s\n", reg, getAndFormatOneReg(reg));
        }
    }
}

void
ArmV8KvmCPU::updateKvmState()
{
    DPRINTF(KvmContext, "In updateKvmState():\n");
    for (const auto &ri : miscRegMap) {
        const uint64_t value(tc->readMiscReg(ri.idx));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        setOneReg(ri.kvm, value);
    }

    for (int i = 0; i < NUM_XREGS; ++i) {
        const uint64_t value(tc->readIntReg(INTREG_X0 + i));
        DPRINTF(KvmContext, "  X%i := 0x%x\n", i, value);
        setOneReg(kvmXReg(i), value);
    }

    for (const auto &ri : intRegMap) {
        const uint64_t value(tc->readIntReg(ri.idx));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        setOneReg(ri.kvm, value);
    }

    for (int i = 0; i < NUM_QREGS; ++i) {
        const RegIndex reg_base(i * FP_REGS_PER_VFP_REG);
        KvmFPReg reg;
        for (int j = 0; j < FP_REGS_PER_VFP_REG; j++)
            reg.s[j].i = tc->readFloatRegBits(reg_base + j);

        setOneReg(kvmFPReg(i), reg.data);
        DPRINTF(KvmContext, "  Q%i: %s\n", i, getAndFormatOneReg(kvmFPReg(i)));
    }

    for (const auto &ri : getSysRegMap()) {
        const uint64_t value(tc->readMiscReg(ri.idx));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        setOneReg(ri.kvm, value);
    }

    setOneReg(INT_REG(regs.pc), tc->instAddr());
    DPRINTF(KvmContext, "  PC := 0x%x\n", tc->instAddr());
}

void
ArmV8KvmCPU::updateThreadContext()
{
    DPRINTF(KvmContext, "In updateThreadContext():\n");

    // Update core misc regs first as they (particularly PSTATE/CPSR)
    // affect how other registers are mapped.
    for (const auto &ri : miscRegMap) {
        const auto value(getOneRegU64(ri.kvm));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        tc->setMiscRegNoEffect(ri.idx, value);
    }

    for (int i = 0; i < NUM_XREGS; ++i) {
        const auto value(getOneRegU64(kvmXReg(i)));
        DPRINTF(KvmContext, "  X%i := 0x%x\n", i, value);
        tc->setIntReg(INTREG_X0 + i, value);
    }

    for (const auto &ri : intRegMap) {
        const auto value(getOneRegU64(ri.kvm));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        tc->setIntReg(ri.idx, value);
    }

    for (int i = 0; i < NUM_QREGS; ++i) {
        const RegIndex reg_base(i * FP_REGS_PER_VFP_REG);
        KvmFPReg reg;
        DPRINTF(KvmContext, "  Q%i: %s\n", i, getAndFormatOneReg(kvmFPReg(i)));
        getOneReg(kvmFPReg(i), reg.data);
        for (int j = 0; j < FP_REGS_PER_VFP_REG; j++)
            tc->setFloatRegBits(reg_base + j, reg.s[j].i);
    }

    for (const auto &ri : getSysRegMap()) {
        const auto value(getOneRegU64(ri.kvm));
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        tc->setMiscRegNoEffect(ri.idx, value);
    }

    const CPSR cpsr(tc->readMiscRegNoEffect(MISCREG_CPSR));
    PCState pc(getOneRegU64(INT_REG(regs.pc)));
    pc.aarch64(inAArch64(tc));
    pc.thumb(cpsr.t);
    pc.nextAArch64(inAArch64(tc));
    // TODO: This is a massive assumption that will break when
    // switching to thumb.
    pc.nextThumb(cpsr.t);
    DPRINTF(KvmContext, "  PC := 0x%x (t: %i, a64: %i)\n",
            pc.instAddr(), pc.thumb(), pc.aarch64());
    tc->pcState(pc);
}

const std::vector<ArmV8KvmCPU::MiscRegInfo> &
ArmV8KvmCPU::getSysRegMap() const
{
    // Try to use the cached map
    if (!sysRegMap.empty())
        return sysRegMap;

    for (const auto &reg : getRegList()) {
        const uint64_t arch(reg & KVM_REG_ARCH_MASK);
        if (arch != KVM_REG_ARM64)
            continue;

        const uint64_t type(reg & KVM_REG_ARM_COPROC_MASK);
        if (type != KVM_REG_ARM64_SYSREG)
            continue;

        const uint64_t op0(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP0));
        const uint64_t op1(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP1));
        const uint64_t crn(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_CRN));
        const uint64_t crm(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_CRM));
        const uint64_t op2(EXTRACT_FIELD(reg, KVM_REG_ARM64_SYSREG_OP2));
        const MiscRegIndex idx(decodeAArch64SysReg(op0, op1, crn, crm, op2));
        const auto &info(miscRegInfo[idx]);
        const bool writeable(
            info[MISCREG_USR_NS_WR] || info[MISCREG_USR_S_WR] ||
            info[MISCREG_PRI_S_WR] || info[MISCREG_PRI_NS_WR] ||
            info[MISCREG_HYP_WR] ||
            info[MISCREG_MON_NS0_WR] || info[MISCREG_MON_NS1_WR]);
        const bool implemented(
            info[MISCREG_IMPLEMENTED] || info[MISCREG_WARN_NOT_FAIL]);

        // Only add implemented registers that we are going to be able
        // to write.
        if (implemented && writeable)
            sysRegMap.emplace_back(reg, idx, miscRegName[idx]);
    }

    return sysRegMap;
}

ArmV8KvmCPU *
ArmV8KvmCPUParams::create()
{
    return new ArmV8KvmCPU(this);
}
