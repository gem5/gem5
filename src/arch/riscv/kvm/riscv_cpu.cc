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

#include <asm/kvm.h>
#include <linux/kvm.h>

#include <algorithm>
#include <cerrno>
#include <memory>
#include <mutex>
#include <utility>

#include "arch/riscv/interrupts.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/uncontended_mutex.hh"
#include "cpu/kvm/vm.hh"
#include "debug/KvmContext.hh"
#include "debug/KvmInt.hh"
#include "params/RiscvISA.hh"
#include "params/RiscvKvmCPU.hh"

namespace gem5
{

using namespace RiscvISA;

namespace
{

/*
 * Like ARM's in-kernel timer, the RISC-V KVM timer keeps advancing while
 * gem5 is servicing userspace events. Preserve a VM-global guest time across
 * KVM entries so guest time only advances while the vCPU is actually running.
 */
uint64_t vtime = 0;
uint64_t vtimeCounter = 0;
bool vtimeInitialized = false;
UncontendedMutex vtimeMutex;

constexpr uint64_t SupervisorIEBits = (1ULL << INT_SOFTWARE_SUPER) |
                                      (1ULL << INT_TIMER_SUPER) |
                                      (1ULL << INT_EXT_SUPER);

/*
 * KVM handles supervisor software interrupts in-kernel for the SBI IPI
 * extension. Rewriting SSIP from gem5's Interrupts state on every guest entry
 * can therefore drop pending IPIs that were raised entirely inside KVM. Only
 * drive SEIP from gem5 here and preserve the KVM-managed SSIP/STIP bits.
 */
constexpr uint64_t SupervisorIPWriteBits = (1ULL << INT_EXT_SUPER);

constexpr uint64_t
misaBit(char ext)
{
    return 1ULL << (ext - 'a');
}

constexpr uint64_t SeedOpstEs16 = 0x2ULL << 30;
constexpr uint64_t SeedEntropyMask = 0xFFFF;
constexpr uint64_t CsrSeed = 0x015;

uint64_t
emulateSeedCsr(Random &rng)
{
    return SeedOpstEs16 |
           rng.random<uint16_t>(0, static_cast<uint16_t>(SeedEntropyMask));
}

} // anonymous namespace

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

#define RISCV_CORE_REG(off)                                                   \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CORE | (off))

#define RISCV_CSR(off)                                                        \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CSR |                   \
     KVM_REG_RISCV_CSR_GENERAL | (off))

#define RISCV_CSR_AIA(off)                                                    \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CSR |                   \
     KVM_REG_RISCV_CSR_AIA | (off))

#define RISCV_CSR_SMSTATEEN(off)                                              \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CSR |                   \
     KVM_REG_RISCV_CSR_SMSTATEEN | (off))

#define RISCV_CFG(name)                                                       \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_CONFIG |                \
     KVM_REG_RISCV_CONFIG_REG(name))

#define RISCV_FP_F(off)                                                       \
    (KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | (off))

#define RISCV_FP_F_FCSR                                                       \
    (KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F |                  \
     KVM_REG_RISCV_FP_F_REG(fcsr))

#define RISCV_FP_D(off)                                                       \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | (off))

#define RISCV_FP_D_FCSR                                                       \
    (KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_D |                  \
     KVM_REG_RISCV_FP_D_REG(fcsr))

#define RISCV_ISA_EXT_SINGLE(ext)                                             \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_ISA_EXT |               \
     KVM_REG_RISCV_ISA_SINGLE | (ext))

#define RISCV_TIMER_REG(name)                                                 \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER |                 \
     KVM_REG_RISCV_TIMER_REG(name))

/* Core register offsets within kvm_riscv_core */
constexpr unsigned KVM_PC_OFF = KVM_REG_RISCV_CORE_REG(regs.pc);
constexpr unsigned KVM_MODE_OFF = KVM_REG_RISCV_CORE_REG(mode);

/* CSR offsets within kvm_riscv_csr */
constexpr unsigned KVM_CSR_SSTATUS = KVM_REG_RISCV_CSR_REG(sstatus);
constexpr unsigned KVM_CSR_SIE = KVM_REG_RISCV_CSR_REG(sie);
constexpr unsigned KVM_CSR_STVEC = KVM_REG_RISCV_CSR_REG(stvec);
constexpr unsigned KVM_CSR_SSCRATCH = KVM_REG_RISCV_CSR_REG(sscratch);
constexpr unsigned KVM_CSR_SEPC = KVM_REG_RISCV_CSR_REG(sepc);
constexpr unsigned KVM_CSR_SCAUSE = KVM_REG_RISCV_CSR_REG(scause);
constexpr unsigned KVM_CSR_STVAL = KVM_REG_RISCV_CSR_REG(stval);
constexpr unsigned KVM_CSR_SIP = KVM_REG_RISCV_CSR_REG(sip);
constexpr unsigned KVM_CSR_SATP = KVM_REG_RISCV_CSR_REG(satp);
constexpr unsigned KVM_CSR_SCOUNTEREN = KVM_REG_RISCV_CSR_REG(scounteren);
constexpr unsigned KVM_CSR_SENVCFG = KVM_REG_RISCV_CSR_REG(senvcfg);
constexpr unsigned KVM_CSR_AIA_SISELECT = KVM_REG_RISCV_CSR_AIA_REG(siselect);
constexpr unsigned KVM_CSR_SMSTATEEN0 =
    KVM_REG_RISCV_CSR_SMSTATEEN_REG(sstateen0);

/* Vector CSR register IDs */
#define RISCV_VEC_CSR(name)                                                   \
    (KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_VECTOR |                \
     KVM_REG_RISCV_VECTOR_CSR_REG(name))

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
    {RISCV_CORE_REG(1), int_reg::_RaIdx, "ra"},
    {RISCV_CORE_REG(2), int_reg::_SpIdx, "sp"},
    {RISCV_CORE_REG(3), int_reg::_GpIdx, "gp"},
    {RISCV_CORE_REG(4), int_reg::_TpIdx, "tp"},
    {RISCV_CORE_REG(5), int_reg::_T0Idx, "t0"},
    {RISCV_CORE_REG(6), int_reg::_T1Idx, "t1"},
    {RISCV_CORE_REG(7), int_reg::_T2Idx, "t2"},
    {RISCV_CORE_REG(8), int_reg::_S0Idx, "s0"},
    {RISCV_CORE_REG(9), int_reg::_S1Idx, "s1"},
    {RISCV_CORE_REG(10), int_reg::_A0Idx, "a0"},
    {RISCV_CORE_REG(11), int_reg::_A1Idx, "a1"},
    {RISCV_CORE_REG(12), int_reg::_A2Idx, "a2"},
    {RISCV_CORE_REG(13), int_reg::_A3Idx, "a3"},
    {RISCV_CORE_REG(14), int_reg::_A4Idx, "a4"},
    {RISCV_CORE_REG(15), int_reg::_A5Idx, "a5"},
    {RISCV_CORE_REG(16), int_reg::_A6Idx, "a6"},
    {RISCV_CORE_REG(17), int_reg::_A7Idx, "a7"},
    {RISCV_CORE_REG(18), int_reg::_S2Idx, "s2"},
    {RISCV_CORE_REG(19), int_reg::_S3Idx, "s3"},
    {RISCV_CORE_REG(20), int_reg::_S4Idx, "s4"},
    {RISCV_CORE_REG(21), int_reg::_S5Idx, "s5"},
    {RISCV_CORE_REG(22), int_reg::_S6Idx, "s6"},
    {RISCV_CORE_REG(23), int_reg::_S7Idx, "s7"},
    {RISCV_CORE_REG(24), int_reg::_S8Idx, "s8"},
    {RISCV_CORE_REG(25), int_reg::_S9Idx, "s9"},
    {RISCV_CORE_REG(26), int_reg::_S10Idx, "s10"},
    {RISCV_CORE_REG(27), int_reg::_S11Idx, "s11"},
    {RISCV_CORE_REG(28), int_reg::_T3Idx, "t3"},
    {RISCV_CORE_REG(29), int_reg::_T4Idx, "t4"},
    {RISCV_CORE_REG(30), int_reg::_T5Idx, "t5"},
    {RISCV_CORE_REG(31), int_reg::_T6Idx, "t6"},
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
    {RISCV_CSR(KVM_CSR_SSTATUS), MISCREG_STATUS, "sstatus"},
    {RISCV_CSR(KVM_CSR_SIE), MISCREG_IE, "sie"},
    {RISCV_CSR(KVM_CSR_STVEC), MISCREG_STVEC, "stvec"},
    {RISCV_CSR(KVM_CSR_SSCRATCH), MISCREG_SSCRATCH, "sscratch"},
    {RISCV_CSR(KVM_CSR_SEPC), MISCREG_SEPC, "sepc"},
    {RISCV_CSR(KVM_CSR_SCAUSE), MISCREG_SCAUSE, "scause"},
    {RISCV_CSR(KVM_CSR_STVAL), MISCREG_STVAL, "stval"},
    {RISCV_CSR(KVM_CSR_SIP), MISCREG_IP, "sip"},
    {RISCV_CSR(KVM_CSR_SATP), MISCREG_SATP, "satp"},
    {RISCV_CSR(KVM_CSR_SCOUNTEREN), MISCREG_SCOUNTEREN, "scounteren"},
    {RISCV_CSR(KVM_CSR_SENVCFG), MISCREG_SENVCFG, "senvcfg"},
};

/*
 * Map VLENB (vector register length in bytes) to KVM_REG_SIZE_* constant.
 */
uint64_t
RiscvKvmCPU::kvmRegSizeForVlenb(uint64_t vlenb)
{
    switch (vlenb) {
        case 8:
            return KVM_REG_SIZE_U64;
        case 16:
            return KVM_REG_SIZE_U128;
        case 32:
            return KVM_REG_SIZE_U256;
        case 64:
            return KVM_REG_SIZE_U512;
        case 128:
            return KVM_REG_SIZE_U1024;
        case 256:
            return KVM_REG_SIZE_U2048;
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
    : BaseKvmCPU(params),
      seedEntropyRng(Random::genRandom(
          static_cast<uint32_t>(Random::globalSeed + params.cpu_id)))
{}

const ISA &
RiscvKvmCPU::riscvIsa() const
{
    auto *isa = dynamic_cast<ISA *>(tc->getIsaPtr());
    panic_if(!isa, "RiscvKvmCPU requires a RISC-V ISA instance.\n");
    return *isa;
}

const RiscvISAParams &
RiscvKvmCPU::riscvIsaParams() const
{
    return dynamic_cast<const RiscvISAParams &>(riscvIsa().params());
}

bool
RiscvKvmCPU::getRegList(struct kvm_reg_list &regs) const
{
    if (ioctl(KVM_GET_REG_LIST, (void *)&regs) == -1) {
        if (errno == E2BIG) {
            return false;
        }

        panic("KVM: Failed to get RISC-V vCPU register list (errno: %i)\n",
              errno);
    }

    return true;
}

void
RiscvKvmCPU::refreshRegList()
{
    kvm_reg_list regs_probe;
    regs_probe.n = 0;
    getRegList(regs_probe);

    std::unique_ptr<struct kvm_reg_list, void (*)(void *p)> regs(
        nullptr, [](void *p) { operator delete(p); });
    const size_t size(sizeof(struct kvm_reg_list) +
                      regs_probe.n * sizeof(uint64_t));
    regs.reset((struct kvm_reg_list *)operator new(size));
    regs->n = regs_probe.n;
    if (!getRegList(*regs)) {
        panic("Failed to determine RISC-V KVM register list size.\n");
    }

    regIndexList.assign(regs->reg, regs->reg + regs->n);
    regIndexSet.clear();
    regIndexSet.insert(regIndexList.begin(), regIndexList.end());
}

bool
RiscvKvmCPU::hasReg(uint64_t id) const
{
    return regIndexSet.find(id) != regIndexSet.end();
}

void
RiscvKvmCPU::configureKvmConfigRegs()
{
    const RegVal misa = tc->readMiscRegNoEffect(MISCREG_ISA);
    const auto &isa_params = riscvIsaParams();

    if (hasReg(RISCV_CFG(isa))) {
        setOneReg(RISCV_CFG(isa), static_cast<uint64_t>(misa & ISA_EXT_MASK));
        DPRINTF(KvmContext, "Configured KVM base ISA mask: 0x%lx\n",
                misa & ISA_EXT_MASK);
    }

    if (hasReg(RISCV_CFG(mvendorid))) {
        setOneReg(
            RISCV_CFG(mvendorid),
            static_cast<uint64_t>(tc->readMiscRegNoEffect(MISCREG_VENDORID)));
    }
    if (hasReg(RISCV_CFG(marchid))) {
        setOneReg(
            RISCV_CFG(marchid),
            static_cast<uint64_t>(tc->readMiscRegNoEffect(MISCREG_ARCHID)));
    }
    if (hasReg(RISCV_CFG(mimpid))) {
        setOneReg(
            RISCV_CFG(mimpid),
            static_cast<uint64_t>(tc->readMiscRegNoEffect(MISCREG_IMPID)));
    }

    if (hasReg(RISCV_CFG(satp_mode))) {
        setOneReg(RISCV_CFG(satp_mode),
                  static_cast<uint64_t>(AddrXlateMode::SV39));
    }

    if (hasReg(RISCV_CFG(zicbom_block_size))) {
        setOneReg(RISCV_CFG(zicbom_block_size),
                  static_cast<uint64_t>(
                      isa_params.enable_Zicbom_fs ? cacheLineSize() : 0));
    }

    if (hasReg(RISCV_CFG(zicboz_block_size))) {
        setOneReg(RISCV_CFG(zicboz_block_size),
                  static_cast<uint64_t>(
                      isa_params.enable_Zicboz_fs ? cacheLineSize() : 0));
    }
}

void
RiscvKvmCPU::configureKvmIsaExts()
{
    const auto &isa_params = riscvIsaParams();

    static const std::pair<uint64_t, const char *> requiredExts[] = {
        {KVM_RISCV_ISA_EXT_ZICNTR, "zicntr"},
        {KVM_RISCV_ISA_EXT_ZICSR, "zicsr"},
        {KVM_RISCV_ISA_EXT_ZIFENCEI, "zifencei"},
        {KVM_RISCV_ISA_EXT_ZIHPM, "zihpm"},
        {KVM_RISCV_ISA_EXT_ZBA, "zba"},
        {KVM_RISCV_ISA_EXT_ZBB, "zbb"},
        {KVM_RISCV_ISA_EXT_ZBS, "zbs"},
        {KVM_RISCV_ISA_EXT_SVNAPOT, "svnapot"},
    };

    for (const auto &[ext, name] : requiredExts) {
        const uint64_t reg = RISCV_ISA_EXT_SINGLE(ext);
        fatal_if(!hasReg(reg),
                 "KVM host does not expose the %s extension required by "
                 "the configured gem5 CPU model.\n",
                 name);

        setOneReg(reg, uint64_t(1));
    }

    /*
     * These extensions are controlled by the configured gem5 ISA parameters
     * rather than being unconditional KVM requirements.
     */
    const struct
    {
        uint64_t ext;
        bool enable;
        const char *name;
    } isaParamExts[] = {
        {KVM_RISCV_ISA_EXT_ZICBOM, isa_params.enable_Zicbom_fs, "zicbom"},
        {KVM_RISCV_ISA_EXT_ZICBOZ, isa_params.enable_Zicboz_fs, "zicboz"},
    };

    for (const auto &ext : isaParamExts) {
        const uint64_t reg = RISCV_ISA_EXT_SINGLE(ext.ext);
        if (!hasReg(reg)) {
            fatal_if(ext.enable,
                     "KVM host does not expose the %s extension required by "
                     "the configured gem5 CPU model.\n",
                     ext.name);
            continue;
        }

        setOneReg(reg, uint64_t(ext.enable));
    }

    /*
     * Linux enables supported host ISA extensions by default. Explicitly
     * disable extensions whose extra CSR state gem5 does not synchronize.
     */
    static const std::pair<uint64_t, const char *> unsupportedStateExts[] = {
        {KVM_RISCV_ISA_EXT_SSAIA, "ssaia"},
        {KVM_RISCV_ISA_EXT_SMSTATEEN, "smstateen"},
    };

    for (const auto &[ext, extName] : unsupportedStateExts) {
        const uint64_t reg = RISCV_ISA_EXT_SINGLE(ext);
        if (!hasReg(reg)) {
            continue;
        }

        DPRINTF(KvmContext, "Disabling unsupported KVM ISA extension: %s\n",
                extName);
        setOneReg(reg, uint64_t(0));
    }
}

void
RiscvKvmCPU::configureKvmFeatures()
{
    const auto &isa = riscvIsa();
    const auto &isa_params = riscvIsaParams();

    fatal_if(isa.rvType() != RV64,
             "RiscvKvmCPU currently only supports RV64 guests.\n");
    fatal_if(isa_params.privilege_mode_set == enums::MHSU,
             "RiscvKvmCPU does not support exposing the RISC-V H extension "
             "to the guest.\n");
    fatal_if(isa_params.privilege_mode_set != enums::MSU,
             "RiscvKvmCPU requires the RISC-V MSU privilege mode set.\n");

    kvmSstatusMask =
        SSTATUS_MASKS[isa.rvType()][isa_params.privilege_mode_set];

    configureKvmConfigRegs();
    configureKvmIsaExts();
}

void
RiscvKvmCPU::startup()
{
    BaseKvmCPU::startup();

    /*
     * Linux expects non-boot harts to be brought online via SBI HSM instead
     * of all entering the kernel image at the reset vector. Start secondary
     * vCPUs in KVM's stopped state so the guest boot hart can release them.
     */
    if (getVCpuID() > 0) {
        if (vm->kvm->capMPState()) {
            struct kvm_mp_state mpState = {};
            mpState.mp_state = KVM_MP_STATE_STOPPED;
            if (ioctl(KVM_SET_MP_STATE, &mpState) == -1) {
                panic("KVM: Failed to stop secondary RISC-V vCPU %li "
                      "before guest boot (errno: %i)\n",
                      getVCpuID(), errno);
            }
        } else {
            warn_once("KVM: Host kernel does not support MP state control for "
                      "RISC-V vCPUs; secondary harts will start immediately.");
        }
    }

    const auto &isa = riscvIsa();

    refreshRegList();
    configureKvmFeatures();
    refreshRegList();

    fatal_if(hasReg(RISCV_CSR_AIA(KVM_CSR_AIA_SISELECT)),
             "KVM host still exposes Ssaia CSR state after gem5 disabled "
             "the extension.\n");
    fatal_if(hasReg(RISCV_CSR_SMSTATEEN(KVM_CSR_SMSTATEEN0)),
             "KVM host still exposes Smstateen CSR state after gem5 "
             "disabled the extension.\n");

    const RegVal gem5Misa = tc->readMiscRegNoEffect(MISCREG_ISA);
    const bool wantD = gem5Misa & misaBit('d');
    const bool wantF = gem5Misa & misaBit('f');
    const bool wantV = gem5Misa & misaBit('v');

    const bool hasFpD = hasReg(RISCV_FP_D(0));
    const bool hasFpF = hasReg(RISCV_FP_F(0));

    if (wantD) {
        fatal_if(!hasFpD,
                 "KVM host does not expose the D extension required by the "
                 "configured gem5 CPU model.\n");
        fpRegMode = FpRegMode::D;
    } else if (wantF) {
        fatal_if(hasFpD, "KVM host still exposes the D extension after the "
                         "configured gem5 CPU model disabled it.\n");
        fatal_if(!hasFpF,
                 "KVM host does not expose the F extension required by the "
                 "configured gem5 CPU model.\n");
        fpRegMode = FpRegMode::F;
    } else {
        fatal_if(hasFpD || hasFpF,
                 "KVM host still exposes floating-point state after the "
                 "configured gem5 CPU model disabled F/D.\n");
        fpRegMode = FpRegMode::None;
    }

    if (!hasReg(RISCV_TIMER_REG(frequency))) {
        warn("KVM: Timer registers not available, "
             "guest time will not be paused while gem5 services exits.\n");
    } else {
        hasKvmTimer = true;
        getOneReg(RISCV_TIMER_REG(frequency), &kvmTimerFrequency);
        DPRINTF(KvmContext, "KVM timer frequency = %lu Hz\n",
                kvmTimerFrequency);

        std::lock_guard<UncontendedMutex> lock(vtimeMutex);
        if (!vtimeInitialized) {
            vtime = getOneRegU64(RISCV_TIMER_REG(time));
            vtimeInitialized = true;
        }
    }

    /*
     * Query VLENB from KVM. This tells us the vector register size
     * the host supports, which we need to construct correct register
     * IDs for vector register sync.
     */
    if (!wantV) {
        fatal_if(hasReg(RISCV_VEC_CSR(vlenb)),
                 "KVM host still exposes vector state after the configured "
                 "gem5 CPU model disabled RVV.\n");
        kvmVlenb = 0;
    } else if (!hasReg(RISCV_VEC_CSR(vlenb))) {
        fatal("KVM host does not expose vector state required by the "
              "configured gem5 CPU model.\n");
    } else {
        kvmVlenb = getOneRegU64(RISCV_VEC_CSR(vlenb));
        fatal_if(kvmVlenb != isa.getVectorLengthInBytes(),
                 "KVM host VLENB (%lu) does not match gem5 RVV VLENB (%li). "
                 "Disable RVV for the KVM CPU or configure isa[0].vlen to "
                 "match the host VLENB.\n",
                 kvmVlenb, isa.getVectorLengthInBytes());
        DPRINTF(KvmContext, "KVM VLENB = %lu bytes (%lu bits)\n", kvmVlenb,
                kvmVlenb * 8);
    }
}

Tick
RiscvKvmCPU::kvmRun(Tick ticks)
{
    auto *interrupt = dynamic_cast<Interrupts *>(interrupts[0]);
    panic_if(!interrupt,
             "KVM: RISC-V KVM CPU requires RiscvISA::Interrupts\n");

    /*
     * Sync S-mode interrupt pending bits to KVM by writing the
     * guest SIP CSR directly via the ONE_REG interface.
     *
     * SIP.SEIP (bit 9) is driven from gem5 because external interrupts
     * originate in the gem5 PLIC model. SIP.SSIP (bit 1) is also writable,
     * but KVM raises SBI IPIs internally, so userspace must preserve that
     * KVM-managed state instead of rewriting it from gem5's interrupt model.
     * SIP.STIP (bit 5) is read-only and managed by the KVM timer path.
     *
     * We read-modify-write to preserve bits we don't control.
     */
    uint64_t ip = interrupt->readIP();
    uint64_t sipVal = getOneRegU64(RISCV_CSR(KVM_CSR_SIP));
    uint64_t newSip =
        (sipVal & ~SupervisorIPWriteBits) | (ip & SupervisorIPWriteBits);
    if (newSip != sipVal) {
        DPRINTF(KvmInt, "KVM: SIP 0x%lx -> 0x%lx\n", sipVal, newSip);
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

    DPRINTF(KvmInt,
            "KVM: SBI exit (ext_id=0x%lx, func_id=0x%lx, "
            "a0=0x%lx, a1=0x%lx)\n",
            run->riscv_sbi.extension_id, run->riscv_sbi.function_id,
            run->riscv_sbi.args[0], run->riscv_sbi.args[1]);

    /*
     * Most SBI extensions (timer, IPI, rfence, HSM, PMU) are handled
     * in-kernel by KVM. Only unsupported extensions reach userspace.
     * Return SBI_ERR_NOT_SUPPORTED so the guest falls back gracefully.
     */
    // SBI_ERR_NOT_SUPPORTED
    run->riscv_sbi.ret[0] = static_cast<unsigned long>(-2);
    run->riscv_sbi.ret[1] = 0;

    return 0;
}

Tick
RiscvKvmCPU::handleKvmExitRiscvCSR()
{
    auto *run = getKvmRunState();
    const auto csr_num = static_cast<uint64_t>(run->riscv_csr.csr_num);
    const auto &isa = riscvIsa();
    const auto &csr_data = isa.getCSRDataMap();
    const auto csr_it = csr_data.find(csr_num);

    auto readCsr = [&](RegVal csr, const CSRMetadata &metadata) -> RegVal {
        RegVal read_val = tc->readMiscReg(metadata.physIndex);

        if (csr == CSR_FCSR) {
            read_val = tc->readMiscReg(MISCREG_FFLAGS) |
                       (tc->readMiscReg(MISCREG_FRM) << FRM_OFFSET);
        }

        const auto &csr_masks = isa.getCSRMaskMap();
        const auto mask_it = csr_masks.find(csr);
        const RegVal visible_mask =
            (mask_it == csr_masks.end()) ? ~RegVal(0) : mask_it->second;
        read_val &= visible_mask;

        switch (csr) {
            case CSR_SIP:
            case CSR_SIE:
            case CSR_HIP:
            case CSR_HIE:
                read_val &= tc->readMiscReg(MISCREG_MIDELEG);
                break;
            case CSR_VSIP:
            case CSR_VSIE:
                read_val &= tc->readMiscReg(MISCREG_HIDELEG);
                read_val >>= 1;
                break;
            case CSR_HVIP: {
                INTERRUPT mip = tc->readMiscReg(MISCREG_IP);
                read_val |= (mip.vssi << 2);
                break;
            }
            default:
                break;
        }

        return read_val;
    };

    auto writeCsr = [&](RegVal csr, const CSRMetadata &metadata,
                        RegVal write_data) {
        auto &csr_masks = isa.getCSRMaskMap();
        auto &csr_write_masks = isa.getCSRWriteMaskMap();

        switch (csr) {
            case CSR_SIP:
            case CSR_SIE:
            case CSR_HIP:
            case CSR_HIE:
                write_data &= tc->readMiscReg(MISCREG_MIDELEG);
                break;
            case CSR_VSIP:
            case CSR_VSIE:
                write_data <<= 1;
                write_data &= tc->readMiscReg(MISCREG_HIDELEG);
                break;
            default:
                break;
        }

        RegVal arch_write_mask = ~RegVal(0);
        const auto write_mask_it = csr_write_masks.find(csr);
        if (write_mask_it != csr_write_masks.end()) {
            arch_write_mask = write_mask_it->second;
        } else {
            const auto read_mask_it = csr_masks.find(csr);
            if (read_mask_it != csr_masks.end()) {
                arch_write_mask = read_mask_it->second;
            }
        }

        const RegVal write_data_masked = write_data & arch_write_mask;
        RegVal reg_data_all = tc->readMiscReg(metadata.physIndex);
        if (csr == CSR_FCSR) {
            reg_data_all = tc->readMiscReg(MISCREG_FFLAGS) |
                           (tc->readMiscReg(MISCREG_FRM) << FRM_OFFSET);
        }

        RegVal new_reg_data_all =
            (reg_data_all & ~arch_write_mask) | (write_data & arch_write_mask);

        switch (csr) {
            case CSR_FCSR:
                tc->setMiscReg(MISCREG_FFLAGS, bits(write_data_masked, 4, 0));
                tc->setMiscReg(MISCREG_FRM, bits(write_data_masked, 7, 5));
                break;
            case CSR_HVIP: {
                INTERRUPT mip = tc->readMiscReg(MISCREG_IP);
                mip.vssi = (new_reg_data_all & VSSI_MASK) >> 2;
                tc->setMiscReg(MISCREG_IP, mip);
                new_reg_data_all &= ~VSSI_MASK;
                tc->setMiscReg(metadata.physIndex, new_reg_data_all);
                break;
            }
            default:
                tc->setMiscReg(metadata.physIndex, new_reg_data_all);
                break;
        }
    };

    if (csr_num == CsrSeed) {
        /*
         * Linux exits to userspace for SEED so the VMM can provide a virtual
         * entropy source. The CSR write operand is architecturally ignored.
         */
        run->riscv_csr.ret_value = emulateSeedCsr(*seedEntropyRng);
        DPRINTF(KvmContext, "KVM: CSR_SEED exit handled (ret=0x%lx)\n",
                run->riscv_csr.ret_value);
        return 0;
    }

    if (csr_it == csr_data.end()) {
        panic("KVM: Unhandled CSR exit "
              "(csr=0x%lx, write_mask=0x%lx, new_value=0x%lx)\n",
              run->riscv_csr.csr_num, run->riscv_csr.write_mask,
              run->riscv_csr.new_value);
    }

    const RegVal old_value = readCsr(csr_num, csr_it->second);
    run->riscv_csr.ret_value = old_value;

    if (run->riscv_csr.write_mask) {
        const RegVal write_mask =
            static_cast<RegVal>(run->riscv_csr.write_mask);
        const RegVal new_value = static_cast<RegVal>(run->riscv_csr.new_value);
        const RegVal write_value =
            (old_value & ~write_mask) | (new_value & write_mask);
        writeCsr(csr_num, csr_it->second, write_value);
    }

    DPRINTF(KvmContext,
            "KVM: CSR exit handled (csr=0x%lx, old=0x%lx, "
            "write_mask=0x%lx, new_value=0x%lx)\n",
            run->riscv_csr.csr_num, old_value, run->riscv_csr.write_mask,
            run->riscv_csr.new_value);

    return 0;
}

void
RiscvKvmCPU::ioctlRun()
{
    if (!hasKvmTimer) {
        BaseKvmCPU::ioctlRun();
        return;
    }

    {
        std::lock_guard<UncontendedMutex> lock(vtimeMutex);
        if (vtimeCounter++ == 0) {
            setOneReg(RISCV_TIMER_REG(time), vtime);
        }
    }

    BaseKvmCPU::ioctlRun();

    {
        std::lock_guard<UncontendedMutex> lock(vtimeMutex);
        if (--vtimeCounter == 0) {
            vtime = getOneRegU64(RISCV_TIMER_REG(time));
        }
    }
}

void
RiscvKvmCPU::dump() const
{
    inform("RISC-V KVM CPU state:");
    inform("  PC: %s", getAndFormatOneReg(RISCV_CORE_REG(KVM_PC_OFF)));

    for (const auto &ri : intRegMap) {
        inform("  %s: %s", ri.name, getAndFormatOneReg(ri.kvmId));
    }

    inform("  mode: %s", getAndFormatOneReg(RISCV_CORE_REG(KVM_MODE_OFF)));

    for (const auto &ri : csrMap) {
        inform("  %s: %s", ri.name, getAndFormatOneReg(ri.kvmId));
    }

    if (hasKvmTimer) {
        inform("  timer.frequency: %s",
               getAndFormatOneReg(RISCV_TIMER_REG(frequency)));
        inform("  timer.time: %s", getAndFormatOneReg(RISCV_TIMER_REG(time)));
        inform("  timer.compare: %s",
               getAndFormatOneReg(RISCV_TIMER_REG(compare)));
        inform("  timer.state: %s",
               getAndFormatOneReg(RISCV_TIMER_REG(state)));
    }

    switch (fpRegMode) {
        case FpRegMode::F:
            for (int i = 0; i < 32; ++i) {
                inform("  f%d: %s", i, getAndFormatOneReg(RISCV_FP_F(i)));
            }
            break;
        case FpRegMode::D:
            for (int i = 0; i < 32; ++i) {
                inform("  f%d: %s", i, getAndFormatOneReg(RISCV_FP_D(i)));
            }
            break;
        case FpRegMode::None:
            break;
    }

    if (kvmVlenb) {
        inform("  vlenb: %lu", kvmVlenb);
        inform("  vstart: %s", getAndFormatOneReg(RISCV_VEC_CSR(vstart)));
        inform("  vl: %s", getAndFormatOneReg(RISCV_VEC_CSR(vl)));
        inform("  vtype: %s", getAndFormatOneReg(RISCV_VEC_CSR(vtype)));
        inform("  vcsr: %s", getAndFormatOneReg(RISCV_VEC_CSR(vcsr)));

        for (int i = 0; i < 32; ++i) {
            inform("  v%d: %s", i, getAndFormatOneReg(kvmVecRegId(i)));
        }
    }
}

void
RiscvKvmCPU::updateKvmState()
{
    DPRINTF(KvmContext, "Updating KVM state...\n");

    updateKvmStateCore();
    updateKvmStateFP();
    updateKvmStateCSR();
    if (kvmVlenb) {
        updateKvmStateVec();
    }
}

void
RiscvKvmCPU::updateKvmStateCore()
{
    // PC
    uint64_t pc = tc->pcState().instAddr();
    DPRINTF(KvmContext, "  PC := 0x%lx\n", pc);
    setOneReg(RISCV_CORE_REG(KVM_PC_OFF), pc);

    // Integer registers x1-x31 (x0 is hardwired zero)
    for (const auto &ri : intRegMap) {
        uint64_t value = tc->getReg(intRegClass[ri.gem5Idx]);
        DPRINTF(KvmContext, "  %s := 0x%lx\n", ri.name, value);
        setOneReg(ri.kvmId, value);
    }

    /*
     * Privilege mode: KVM only exposes S and U modes to the guest.
     * gem5 initialises the thread context in M-mode (Reset fault),
     * but under KVM the guest's "M-mode" is virtualised as S-mode.
     * Map both PRV_M and PRV_S → KVM_RISCV_MODE_S.
     */
    RegVal prv = tc->readMiscRegNoEffect(MISCREG_PRV);
    uint64_t kvmMode = (prv == PRV_U) ? KVM_RISCV_MODE_U : KVM_RISCV_MODE_S;
    DPRINTF(KvmContext, "  mode := %d (prv=%d)\n", kvmMode, prv);
    setOneReg(RISCV_CORE_REG(KVM_MODE_OFF), kvmMode);
}

void
RiscvKvmCPU::updateKvmStateFP()
{
    if (fpRegMode == FpRegMode::None) {
        return;
    }

    for (int i = 0; i < 32; ++i) {
        RegVal value = tc->getReg(floatRegClass[i]);
        if (fpRegMode == FpRegMode::D) {
            setOneReg(RISCV_FP_D(i), static_cast<uint64_t>(value));
        } else {
            setOneReg(RISCV_FP_F(i), static_cast<uint32_t>(unboxF32(value)));
        }
    }

    // FCSR = (FRM << 5) | FFLAGS
    uint32_t fflags = tc->readMiscRegNoEffect(MISCREG_FFLAGS) & 0x1F;
    uint32_t frm = tc->readMiscRegNoEffect(MISCREG_FRM) & 0x7;
    uint32_t fcsr = (frm << 5) | fflags;
    if (fpRegMode == FpRegMode::D) {
        setOneReg(RISCV_FP_D_FCSR, fcsr);
    } else {
        setOneReg(RISCV_FP_F_FCSR, fcsr);
    }
}

void
RiscvKvmCPU::updateKvmStateCSR()
{
    for (const auto &ri : csrMap) {
        uint64_t value = tc->readMiscRegNoEffect(ri.gem5Idx);

        // Mask to S-mode view for registers that are S-mode projections
        if (ri.gem5Idx == MISCREG_STATUS) {
            value &= kvmSstatusMask;
        } else if (ri.gem5Idx == MISCREG_IE) {
            value &= SupervisorIEBits;
        } else if (ri.gem5Idx == MISCREG_IP) {
            value &= SupervisorIPWriteBits;
        }

        DPRINTF(KvmContext, "  %s := 0x%lx\n", ri.name, value);
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
    if (kvmVlenb) {
        updateTCVec();
    }
}

void
RiscvKvmCPU::updateTCCore()
{
    // Integer registers x1-x31
    for (const auto &ri : intRegMap) {
        uint64_t value = getOneRegU64(ri.kvmId);
        DPRINTF(KvmContext, "  %s := 0x%lx\n", ri.name, value);
        tc->setReg(intRegClass[ri.gem5Idx], value);
    }

    // Privilege mode
    uint64_t kvmMode = getOneRegU64(RISCV_CORE_REG(KVM_MODE_OFF));
    RegVal prv = (kvmMode == KVM_RISCV_MODE_S) ? PRV_S : PRV_U;
    DPRINTF(KvmContext, "  mode := %d (prv=%d)\n", kvmMode, prv);
    tc->setMiscRegNoEffect(MISCREG_PRV, prv);

    // PC
    uint64_t pc = getOneRegU64(RISCV_CORE_REG(KVM_PC_OFF));
    DPRINTF(KvmContext, "  PC := 0x%lx\n", pc);
    tc->pcState(RiscvISA::PCState(pc));
}

void
RiscvKvmCPU::updateTCFP()
{
    if (fpRegMode == FpRegMode::None) {
        return;
    }

    for (int i = 0; i < 32; ++i) {
        if (fpRegMode == FpRegMode::D) {
            uint64_t value = getOneRegU64(RISCV_FP_D(i));
            tc->setReg(floatRegClass[i], static_cast<RegVal>(value));
        } else {
            uint32_t value = getOneRegU32(RISCV_FP_F(i));
            tc->setReg(floatRegClass[i], static_cast<RegVal>(boxF32(value)));
        }
    }

    uint32_t fcsr = (fpRegMode == FpRegMode::D)
                        ? getOneRegU32(RISCV_FP_D_FCSR)
                        : getOneRegU32(RISCV_FP_F_FCSR);
    tc->setMiscRegNoEffect(MISCREG_FFLAGS, fcsr & 0x1F);
    tc->setMiscRegNoEffect(MISCREG_FRM, (fcsr >> 5) & 0x7);
}

void
RiscvKvmCPU::updateTCCSR()
{
    for (const auto &ri : csrMap) {
        uint64_t value = getOneRegU64(ri.kvmId);
        DPRINTF(KvmContext, "  %s := 0x%lx\n", ri.name, value);

        if (ri.gem5Idx == MISCREG_STATUS) {
            // Merge sstatus bits into the full mstatus register
            uint64_t mstatus = tc->readMiscRegNoEffect(MISCREG_STATUS);
            mstatus = (mstatus & ~kvmSstatusMask) | (value & kvmSstatusMask);
            tc->setMiscRegNoEffect(MISCREG_STATUS, mstatus);
        } else if (ri.gem5Idx == MISCREG_IE || ri.gem5Idx == MISCREG_IP) {
            // Use setMiscReg so the Interrupts controller is updated
            tc->setMiscReg(ri.gem5Idx, value);
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

    DPRINTF(KvmContext, "  vstart=%lu vl=%lu vtype=0x%lx vcsr=0x%lx\n", vstart,
            vl, vtype, vcsr);

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

    DPRINTF(KvmContext, "  vstart=%lu vl=%lu vtype=0x%lx vcsr=0x%lx\n", vstart,
            vl, vtype, vcsr);

    // Vector registers v0-v31
    for (int i = 0; i < NumVecStandardRegs; ++i) {
        auto *vc = static_cast<RiscvISA::VecRegContainer *>(
            tc->getWritableReg(vecRegClass[i]));
        getOneReg(kvmVecRegId(i), vc->as<uint8_t>());
    }
}

} // namespace gem5
