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

#include "arch/riscv/kvm/riscv_cpu.hh"

#include <linux/kvm.h>

#include <asm/kvm.h>

#include "arch/riscv/interrupts.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/logging.hh"
#include "cpu/kvm/vm.hh"
#include "debug/KvmContext.hh"
#include "debug/KvmInt.hh"
#include "params/RiscvKvmCPU.hh"

namespace gem5
{

using namespace RiscvISA;

/*
 * KVM register ID helpers.
 *
 * RISC-V KVM register IDs are composed as:
 *   KVM_REG_RISCV | KVM_REG_SIZE_xxx | TYPE | offset
 *
 * Core registers (integer regs + mode) use KVM_REG_RISCV_CORE (type 2).
 * The user_regs_struct layout is: pc(0), ra(1), sp(2), ..., t6(31).
 * The mode field follows at offset 32.
 *
 * CSRs use KVM_REG_RISCV_CSR (type 3) with subtype GENERAL.
 * FP D-extension uses KVM_REG_RISCV_FP_D (type 6).
 */

#define RISCV_CORE_REG(off) \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CORE | (off))

#define RISCV_CSR(off) \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | \
     KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | (off))

#define RISCV_FP_D(off) \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | (off))

#define RISCV_FP_D_FCSR \
    (KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_D | \
     KVM_REG_RISCV_FP_D_REG(fcsr))

#define RISCV_TIMER_REG(name) \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | \
     KVM_REG_RISCV_TIMER_REG(name))

/* Core register offsets within kvm_riscv_core */
constexpr unsigned KVM_PC_OFF = KVM_REG_RISCV_CORE_REG(regs.pc);
constexpr unsigned KVM_MODE_OFF = KVM_REG_RISCV_CORE_REG(mode);

/* CSR offsets within kvm_riscv_csr */
constexpr unsigned KVM_CSR_SSTATUS   = KVM_REG_RISCV_CSR_REG(sstatus);
constexpr unsigned KVM_CSR_SIE       = KVM_REG_RISCV_CSR_REG(sie);
constexpr unsigned KVM_CSR_STVEC     = KVM_REG_RISCV_CSR_REG(stvec);
constexpr unsigned KVM_CSR_SSCRATCH  = KVM_REG_RISCV_CSR_REG(sscratch);
constexpr unsigned KVM_CSR_SEPC      = KVM_REG_RISCV_CSR_REG(sepc);
constexpr unsigned KVM_CSR_SCAUSE    = KVM_REG_RISCV_CSR_REG(scause);
constexpr unsigned KVM_CSR_STVAL     = KVM_REG_RISCV_CSR_REG(stval);
constexpr unsigned KVM_CSR_SIP       = KVM_REG_RISCV_CSR_REG(sip);
constexpr unsigned KVM_CSR_SATP      = KVM_REG_RISCV_CSR_REG(satp);
constexpr unsigned KVM_CSR_SCOUNTEREN = KVM_REG_RISCV_CSR_REG(scounteren);
constexpr unsigned KVM_CSR_SENVCFG   = KVM_REG_RISCV_CSR_REG(senvcfg);

/* Vector CSR register IDs */
#define RISCV_VEC_CSR(name) \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_VECTOR | \
     KVM_REG_RISCV_VECTOR_CSR_REG(name))

/*
 * sstatus mask: use gem5's authoritative RV64 MHSU mask which covers
 * SD, UXL, MXR, SUM, UBE, XS, FS, VS, SPP, SPIE, SIE — all bits
 * visible in S-mode for a system with H-extension support.
 */
const uint64_t SSTATUS_MASK = SSTATUS_MASKS[RV64][enums::MHSU];

/*
 * Integer register mapping table.
 *
 * gem5 int_reg enum:  zero(0), ra(1), sp(2), ... t6(31)
 * KVM core regs:      pc(0), ra(1), sp(2), ... t6(31)
 *
 * The orderings are identical for x1-x31. x0 is hardwired to zero
 * and is not synced. PC is handled separately.
 */
const std::vector<RiscvKvmCPU::IntRegInfo> RiscvKvmCPU::intRegMap = {
    { RISCV_CORE_REG(1),  int_reg::_RaIdx,  "ra"  },
    { RISCV_CORE_REG(2),  int_reg::_SpIdx,  "sp"  },
    { RISCV_CORE_REG(3),  int_reg::_GpIdx,  "gp"  },
    { RISCV_CORE_REG(4),  int_reg::_TpIdx,  "tp"  },
    { RISCV_CORE_REG(5),  int_reg::_T0Idx,  "t0"  },
    { RISCV_CORE_REG(6),  int_reg::_T1Idx,  "t1"  },
    { RISCV_CORE_REG(7),  int_reg::_T2Idx,  "t2"  },
    { RISCV_CORE_REG(8),  int_reg::_S0Idx,  "s0"  },
    { RISCV_CORE_REG(9),  int_reg::_S1Idx,  "s1"  },
    { RISCV_CORE_REG(10), int_reg::_A0Idx,  "a0"  },
    { RISCV_CORE_REG(11), int_reg::_A1Idx,  "a1"  },
    { RISCV_CORE_REG(12), int_reg::_A2Idx,  "a2"  },
    { RISCV_CORE_REG(13), int_reg::_A3Idx,  "a3"  },
    { RISCV_CORE_REG(14), int_reg::_A4Idx,  "a4"  },
    { RISCV_CORE_REG(15), int_reg::_A5Idx,  "a5"  },
    { RISCV_CORE_REG(16), int_reg::_A6Idx,  "a6"  },
    { RISCV_CORE_REG(17), int_reg::_A7Idx,  "a7"  },
    { RISCV_CORE_REG(18), int_reg::_S2Idx,  "s2"  },
    { RISCV_CORE_REG(19), int_reg::_S3Idx,  "s3"  },
    { RISCV_CORE_REG(20), int_reg::_S4Idx,  "s4"  },
    { RISCV_CORE_REG(21), int_reg::_S5Idx,  "s5"  },
    { RISCV_CORE_REG(22), int_reg::_S6Idx,  "s6"  },
    { RISCV_CORE_REG(23), int_reg::_S7Idx,  "s7"  },
    { RISCV_CORE_REG(24), int_reg::_S8Idx,  "s8"  },
    { RISCV_CORE_REG(25), int_reg::_S9Idx,  "s9"  },
    { RISCV_CORE_REG(26), int_reg::_S10Idx, "s10" },
    { RISCV_CORE_REG(27), int_reg::_S11Idx, "s11" },
    { RISCV_CORE_REG(28), int_reg::_T3Idx,  "t3"  },
    { RISCV_CORE_REG(29), int_reg::_T4Idx,  "t4"  },
    { RISCV_CORE_REG(30), int_reg::_T5Idx,  "t5"  },
    { RISCV_CORE_REG(31), int_reg::_T6Idx,  "t6"  },
};

/*
 * CSR mapping table.
 *
 * KVM exposes supervisor-mode CSRs via kvm_riscv_csr.
 * gem5 stores the full M-mode register in MISCREG_STATUS / MISCREG_IP /
 * MISCREG_IE; the S-mode masking is applied in updateKvmStateCSR and
 * updateTCCSR.
 */
const std::vector<RiscvKvmCPU::MiscRegInfo> RiscvKvmCPU::csrMap = {
    { RISCV_CSR(KVM_CSR_SSTATUS),    MISCREG_STATUS,     "sstatus"    },
    { RISCV_CSR(KVM_CSR_SIE),        MISCREG_IE,         "sie"        },
    { RISCV_CSR(KVM_CSR_STVEC),      MISCREG_STVEC,      "stvec"      },
    { RISCV_CSR(KVM_CSR_SSCRATCH),   MISCREG_SSCRATCH,   "sscratch"   },
    { RISCV_CSR(KVM_CSR_SEPC),       MISCREG_SEPC,       "sepc"       },
    { RISCV_CSR(KVM_CSR_SCAUSE),     MISCREG_SCAUSE,     "scause"     },
    { RISCV_CSR(KVM_CSR_STVAL),      MISCREG_STVAL,      "stval"      },
    { RISCV_CSR(KVM_CSR_SIP),        MISCREG_IP,         "sip"        },
    { RISCV_CSR(KVM_CSR_SATP),       MISCREG_SATP,       "satp"       },
    { RISCV_CSR(KVM_CSR_SCOUNTEREN), MISCREG_SCOUNTEREN, "scounteren" },
    { RISCV_CSR(KVM_CSR_SENVCFG),    MISCREG_SENVCFG,    "senvcfg"    },
};


/*
 * Map VLENB (vector register length in bytes) to KVM_REG_SIZE_* constant.
 */
uint64_t
RiscvKvmCPU::kvmRegSizeForVlenb(uint64_t vlenb)
{
    switch (vlenb) {
      case 8:   return KVM_REG_SIZE_U64;
      case 16:  return KVM_REG_SIZE_U128;
      case 32:  return KVM_REG_SIZE_U256;
      case 64:  return KVM_REG_SIZE_U512;
      case 128: return KVM_REG_SIZE_U1024;
      case 256: return KVM_REG_SIZE_U2048;
      default:
        panic("RiscvKvmCPU: Unsupported VLENB=%lu\n", vlenb);
    }
}

/*
 * Build a KVM ONE_REG ID for vector register regIdx (0-31).
 */
uint64_t
RiscvKvmCPU::kvmVecRegId(int regIdx) const
{
    return KVM_REG_RISCV | kvmRegSizeForVlenb(kvmVlenb) |
           KVM_REG_RISCV_VECTOR | KVM_REG_RISCV_VECTOR_REG(regIdx);
}


RiscvKvmCPU::RiscvKvmCPU(const RiscvKvmCPUParams &params)
    : BaseKvmCPU(params)
{
}

RiscvKvmCPU::~RiscvKvmCPU()
{
}

void
RiscvKvmCPU::startup()
{
    BaseKvmCPU::startup();

    /*
     * Query VLENB from KVM. This tells us the vector register size
     * the host supports, which we need to construct correct register
     * IDs for vector register sync.
     */
    kvmVlenb = getOneRegU64(RISCV_VEC_CSR(vlenb));
    DPRINTF(KvmContext, "KVM VLENB = %lu bytes (%lu bits)\n",
            kvmVlenb, kvmVlenb * 8);
}

Tick
RiscvKvmCPU::kvmRun(Tick ticks)
{
    auto *interrupt = dynamic_cast<Interrupts *>(interrupts[0]);

    /*
     * Sync S-mode interrupt pending bits to KVM by writing the
     * guest SIP CSR directly via the ONE_REG interface.
     *
     * SIP.SSIP (bit 1) and SIP.SEIP (bit 9) are writable
     * through KVM ONE_REG.  SIP.STIP (bit 5) is read-only
     * (managed by the hardware timer / KVM sstc extension).
     *
     * We read-modify-write to preserve bits we don't control.
     */
    uint64_t ip = interrupt->readIP();
    static const uint64_t sipMask =
        (1UL << INT_SOFTWARE_SUPER) |
        (1UL << INT_EXT_SUPER);
    uint64_t sipVal = getOneRegU64(RISCV_CSR(KVM_CSR_SIP));
    uint64_t newSip = (sipVal & ~sipMask) | (ip & sipMask);
    if (newSip != sipVal) {
        DPRINTF(KvmInt, "KVM: SIP 0x%lx -> 0x%lx\n",
                sipVal, newSip);
        setOneReg(RISCV_CSR(KVM_CSR_SIP), newSip);
    }

    return BaseKvmCPU::kvmRun(ticks);
}

Tick
RiscvKvmCPU::handleKvmExit()
{
    switch (getKvmRunState()->exit_reason) {
      case KVM_EXIT_RISCV_SBI:
        _status = Running;
        return handleKvmExitRiscvSBI();

      case KVM_EXIT_RISCV_CSR:
        _status = Running;
        return handleKvmExitRiscvCSR();

      default:
        return BaseKvmCPU::handleKvmExit();
    }
}

Tick
RiscvKvmCPU::handleKvmExitRiscvSBI()
{
    auto *run = getKvmRunState();

    DPRINTF(KvmInt, "KVM: SBI exit (ext_id=0x%lx, func_id=0x%lx, "
            "a0=0x%lx, a1=0x%lx)\n",
            run->riscv_sbi.extension_id,
            run->riscv_sbi.function_id,
            run->riscv_sbi.args[0],
            run->riscv_sbi.args[1]);

    /*
     * Most SBI extensions (timer, IPI, rfence, HSM, PMU) are handled
     * in-kernel by KVM. Only unsupported extensions reach userspace.
     * Return SBI_ERR_NOT_SUPPORTED so the guest falls back gracefully.
     */
    // SBI_ERR_NOT_SUPPORTED
    run->riscv_sbi.ret[0] =
        static_cast<unsigned long>(-2);

    return 0;
}

Tick
RiscvKvmCPU::handleKvmExitRiscvCSR()
{
    auto *run = getKvmRunState();

    warn_once("KVM: Unhandled CSR exit "
              "(csr=0x%lx, write_mask=0x%lx, "
              "new_value=0x%lx)\n",
              run->riscv_csr.csr_num,
              run->riscv_csr.write_mask,
              run->riscv_csr.new_value);

    /*
     * Return the current value unchanged. The guest will see its
     * write had no effect on the masked bits.
     */
    return 0;
}

void
RiscvKvmCPU::ioctlRun()
{
    /*
     * Restore the virtual timer counter before entering KVM and save
     * it after exit, similar to ARM's virtual timer handling.
     *
     * Skip on the very first run (savedTime == 0) — the timer
     * register may not accept writes before the first KVM_RUN.
     */
    if (savedTime != 0)
        setOneReg(RISCV_TIMER_REG(time), savedTime);

    BaseKvmCPU::ioctlRun();

    savedTime = getOneRegU64(RISCV_TIMER_REG(time));
}

void
RiscvKvmCPU::dump() const
{
    inform("RISC-V KVM CPU state:");
    inform("  PC: %s", getAndFormatOneReg(RISCV_CORE_REG(KVM_PC_OFF)));

    for (const auto &ri : intRegMap)
        inform("  %s: %s", ri.name, getAndFormatOneReg(ri.kvmId));

    inform("  mode: %s", getAndFormatOneReg(RISCV_CORE_REG(KVM_MODE_OFF)));

    for (const auto &ri : csrMap)
        inform("  %s: %s", ri.name, getAndFormatOneReg(ri.kvmId));

    for (int i = 0; i < 32; ++i)
        inform("  f%d: %s", i, getAndFormatOneReg(RISCV_FP_D(i)));

    if (kvmVlenb) {
        inform("  vlenb: %lu", kvmVlenb);
        inform("  vstart: %s", getAndFormatOneReg(RISCV_VEC_CSR(vstart)));
        inform("  vl: %s", getAndFormatOneReg(RISCV_VEC_CSR(vl)));
        inform("  vtype: %s", getAndFormatOneReg(RISCV_VEC_CSR(vtype)));
        inform("  vcsr: %s", getAndFormatOneReg(RISCV_VEC_CSR(vcsr)));

        for (int i = 0; i < 32; ++i)
            inform("  v%d: %s", i, getAndFormatOneReg(kvmVecRegId(i)));
    }
}

void
RiscvKvmCPU::updateKvmState()
{
    DPRINTF(KvmContext, "Updating KVM state...\n");

    updateKvmStateCore();
    updateKvmStateFP();
    updateKvmStateCSR();
    if (kvmVlenb)
        updateKvmStateVec();
}

void
RiscvKvmCPU::updateKvmStateCore()
{
    // PC
    uint64_t pc = tc->pcState().instAddr();
    DPRINTF(KvmContext, "  PC := 0x%x\n", pc);
    setOneReg(RISCV_CORE_REG(KVM_PC_OFF), pc);

    // Integer registers x1-x31 (x0 is hardwired zero)
    for (const auto &ri : intRegMap) {
        uint64_t value = tc->getReg(intRegClass[ri.gem5Idx]);
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        setOneReg(ri.kvmId, value);
    }

    /*
     * Privilege mode: KVM only exposes S and U modes to the guest.
     * gem5 initialises the thread context in M-mode (Reset fault),
     * but under KVM the guest's "M-mode" is virtualised as S-mode.
     * Map both PRV_M and PRV_S → KVM_RISCV_MODE_S.
     */
    RegVal prv = tc->readMiscRegNoEffect(MISCREG_PRV);
    uint64_t kvmMode = (prv == PRV_U) ? KVM_RISCV_MODE_U
                                      : KVM_RISCV_MODE_S;
    DPRINTF(KvmContext, "  mode := %d (prv=%d)\n", kvmMode, prv);
    setOneReg(RISCV_CORE_REG(KVM_MODE_OFF), kvmMode);
}

void
RiscvKvmCPU::updateKvmStateFP()
{
    // FP registers f0-f31 via D-extension interface
    for (int i = 0; i < 32; ++i) {
        RegVal value = tc->getReg(floatRegClass[i]);
        setOneReg(RISCV_FP_D(i), (uint64_t)value);
    }

    // FCSR = (FRM << 5) | FFLAGS
    uint32_t fflags = tc->readMiscRegNoEffect(MISCREG_FFLAGS) & 0x1F;
    uint32_t frm = tc->readMiscRegNoEffect(MISCREG_FRM) & 0x7;
    uint32_t fcsr = (frm << 5) | fflags;
    setOneReg(RISCV_FP_D_FCSR, fcsr);
}

void
RiscvKvmCPU::updateKvmStateCSR()
{
    for (const auto &ri : csrMap) {
        uint64_t value = tc->readMiscRegNoEffect(ri.gem5Idx);

        // Mask to S-mode view for registers that are S-mode projections
        if (ri.gem5Idx == MISCREG_STATUS)
            value &= SSTATUS_MASK;

        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        setOneReg(ri.kvmId, value);
    }
}

void
RiscvKvmCPU::updateThreadContext()
{
    DPRINTF(KvmContext, "Updating thread context...\n");

    updateTCCore();
    updateTCFP();
    updateTCCSR();
    if (kvmVlenb)
        updateTCVec();
}

void
RiscvKvmCPU::updateTCCore()
{
    // Integer registers x1-x31
    for (const auto &ri : intRegMap) {
        uint64_t value = getOneRegU64(ri.kvmId);
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);
        tc->setReg(intRegClass[ri.gem5Idx], value);
    }

    // Privilege mode
    uint64_t kvmMode = getOneRegU64(RISCV_CORE_REG(KVM_MODE_OFF));
    RegVal prv = (kvmMode == KVM_RISCV_MODE_S) ? PRV_S : PRV_U;
    DPRINTF(KvmContext, "  mode := %d (prv=%d)\n", kvmMode, prv);
    tc->setMiscRegNoEffect(MISCREG_PRV, prv);

    // PC
    uint64_t pc = getOneRegU64(RISCV_CORE_REG(KVM_PC_OFF));
    DPRINTF(KvmContext, "  PC := 0x%x\n", pc);
    tc->pcState(RiscvISA::PCState(pc));
}

void
RiscvKvmCPU::updateTCFP()
{
    // FP registers f0-f31
    for (int i = 0; i < 32; ++i) {
        uint64_t value = getOneRegU64(RISCV_FP_D(i));
        tc->setReg(floatRegClass[i], (RegVal)value);
    }

    // FCSR
    uint32_t fcsr = getOneRegU32(RISCV_FP_D_FCSR);
    tc->setMiscRegNoEffect(MISCREG_FFLAGS, fcsr & 0x1F);
    tc->setMiscRegNoEffect(MISCREG_FRM, (fcsr >> 5) & 0x7);
}

void
RiscvKvmCPU::updateTCCSR()
{
    for (const auto &ri : csrMap) {
        uint64_t value = getOneRegU64(ri.kvmId);
        DPRINTF(KvmContext, "  %s := 0x%x\n", ri.name, value);

        if (ri.gem5Idx == MISCREG_STATUS) {
            // Merge sstatus bits into the full mstatus register
            uint64_t mstatus = tc->readMiscRegNoEffect(MISCREG_STATUS);
            mstatus = (mstatus & ~SSTATUS_MASK) | (value & SSTATUS_MASK);
            tc->setMiscRegNoEffect(MISCREG_STATUS, mstatus);
        } else {
            tc->setMiscRegNoEffect(ri.gem5Idx, value);
        }
    }
}

void
RiscvKvmCPU::updateKvmStateVec()
{
    // Vector CSRs
    uint64_t vstart = tc->readMiscRegNoEffect(MISCREG_VSTART);
    uint64_t vl = tc->readMiscRegNoEffect(MISCREG_VL);
    uint64_t vtype = tc->readMiscRegNoEffect(MISCREG_VTYPE);
    uint64_t vcsr = (tc->readMiscRegNoEffect(MISCREG_VXRM) << 1) |
                     tc->readMiscRegNoEffect(MISCREG_VXSAT);

    setOneReg(RISCV_VEC_CSR(vstart), vstart);
    setOneReg(RISCV_VEC_CSR(vl), vl);
    setOneReg(RISCV_VEC_CSR(vtype), vtype);
    setOneReg(RISCV_VEC_CSR(vcsr), vcsr);

    DPRINTF(KvmContext, "  vstart=%lu vl=%lu vtype=0x%lx vcsr=0x%lx\n",
            vstart, vl, vtype, vcsr);

    // Vector registers v0-v31
    for (int i = 0; i < NumVecStandardRegs; ++i) {
        RiscvISA::VecRegContainer vc;
        tc->getReg(vecRegClass[i], &vc);
        setOneReg(kvmVecRegId(i), vc.as<uint8_t>());
    }
}

void
RiscvKvmCPU::updateTCVec()
{
    // Vector CSRs
    uint64_t vstart = getOneRegU64(RISCV_VEC_CSR(vstart));
    uint64_t vl = getOneRegU64(RISCV_VEC_CSR(vl));
    uint64_t vtype = getOneRegU64(RISCV_VEC_CSR(vtype));
    uint64_t vcsr = getOneRegU64(RISCV_VEC_CSR(vcsr));

    tc->setMiscRegNoEffect(MISCREG_VSTART, vstart);
    tc->setMiscRegNoEffect(MISCREG_VL, vl);
    tc->setMiscRegNoEffect(MISCREG_VTYPE, vtype);
    tc->setMiscRegNoEffect(MISCREG_VXSAT, vcsr & 0x1);
    tc->setMiscRegNoEffect(MISCREG_VXRM, (vcsr >> 1) & 0x3);

    DPRINTF(KvmContext, "  vstart=%lu vl=%lu vtype=0x%lx vcsr=0x%lx\n",
            vstart, vl, vtype, vcsr);

    // Vector registers v0-v31
    for (int i = 0; i < NumVecStandardRegs; ++i) {
        auto *vc = static_cast<RiscvISA::VecRegContainer *>(
            tc->getWritableReg(vecRegClass[i]));
        getOneReg(kvmVecRegId(i), vc->as<uint8_t>());
    }
}

} // namespace gem5
