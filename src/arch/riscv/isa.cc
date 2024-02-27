/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2022 Google LLC
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

#include "arch/riscv/isa.hh"

#include <ctime>
#include <set>
#include <sstream>

#include "arch/riscv/faults.hh"
#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/interrupts.hh"
#include "arch/riscv/mmu.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/pmp.hh"
#include "arch/riscv/pcstate.hh"
#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/LLSC.hh"
#include "debug/MatRegs.hh"
#include "debug/RiscvMisc.hh"
#include "debug/VecRegs.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/RiscvISA.hh"
#include "sim/pseudo_inst.hh"

namespace gem5
{

namespace RiscvISA
{

[[maybe_unused]] const std::array<const char *, NUM_MISCREGS> MiscRegNames = {{
    [MISCREG_PRV]           = "PRV",
    [MISCREG_ISA]           = "ISA",
    [MISCREG_VENDORID]      = "VENDORID",
    [MISCREG_ARCHID]        = "ARCHID",
    [MISCREG_IMPID]         = "IMPID",
    [MISCREG_HARTID]        = "HARTID",
    [MISCREG_STATUS]        = "STATUS",
    [MISCREG_IP]            = "IP",
    [MISCREG_IE]            = "IE",
    [MISCREG_CYCLE]         = "CYCLE",
    [MISCREG_TIME]          = "TIME",
    [MISCREG_INSTRET]       = "INSTRET",
    [MISCREG_HPMCOUNTER03]  = "HPMCOUNTER03",
    [MISCREG_HPMCOUNTER04]  = "HPMCOUNTER04",
    [MISCREG_HPMCOUNTER05]  = "HPMCOUNTER05",
    [MISCREG_HPMCOUNTER06]  = "HPMCOUNTER06",
    [MISCREG_HPMCOUNTER07]  = "HPMCOUNTER07",
    [MISCREG_HPMCOUNTER08]  = "HPMCOUNTER08",
    [MISCREG_HPMCOUNTER09]  = "HPMCOUNTER09",
    [MISCREG_HPMCOUNTER10]  = "HPMCOUNTER10",
    [MISCREG_HPMCOUNTER11]  = "HPMCOUNTER11",
    [MISCREG_HPMCOUNTER12]  = "HPMCOUNTER12",
    [MISCREG_HPMCOUNTER13]  = "HPMCOUNTER13",
    [MISCREG_HPMCOUNTER14]  = "HPMCOUNTER14",
    [MISCREG_HPMCOUNTER15]  = "HPMCOUNTER15",
    [MISCREG_HPMCOUNTER16]  = "HPMCOUNTER16",
    [MISCREG_HPMCOUNTER17]  = "HPMCOUNTER17",
    [MISCREG_HPMCOUNTER18]  = "HPMCOUNTER18",
    [MISCREG_HPMCOUNTER19]  = "HPMCOUNTER19",
    [MISCREG_HPMCOUNTER20]  = "HPMCOUNTER20",
    [MISCREG_HPMCOUNTER21]  = "HPMCOUNTER21",
    [MISCREG_HPMCOUNTER22]  = "HPMCOUNTER22",
    [MISCREG_HPMCOUNTER23]  = "HPMCOUNTER23",
    [MISCREG_HPMCOUNTER24]  = "HPMCOUNTER24",
    [MISCREG_HPMCOUNTER25]  = "HPMCOUNTER25",
    [MISCREG_HPMCOUNTER26]  = "HPMCOUNTER26",
    [MISCREG_HPMCOUNTER27]  = "HPMCOUNTER27",
    [MISCREG_HPMCOUNTER28]  = "HPMCOUNTER28",
    [MISCREG_HPMCOUNTER29]  = "HPMCOUNTER29",
    [MISCREG_HPMCOUNTER30]  = "HPMCOUNTER30",
    [MISCREG_HPMCOUNTER31]  = "HPMCOUNTER31",
    [MISCREG_HPMEVENT03]    = "HPMEVENT03",
    [MISCREG_HPMEVENT04]    = "HPMEVENT04",
    [MISCREG_HPMEVENT05]    = "HPMEVENT05",
    [MISCREG_HPMEVENT06]    = "HPMEVENT06",
    [MISCREG_HPMEVENT07]    = "HPMEVENT07",
    [MISCREG_HPMEVENT08]    = "HPMEVENT08",
    [MISCREG_HPMEVENT09]    = "HPMEVENT09",
    [MISCREG_HPMEVENT10]    = "HPMEVENT10",
    [MISCREG_HPMEVENT11]    = "HPMEVENT11",
    [MISCREG_HPMEVENT12]    = "HPMEVENT12",
    [MISCREG_HPMEVENT13]    = "HPMEVENT13",
    [MISCREG_HPMEVENT14]    = "HPMEVENT14",
    [MISCREG_HPMEVENT15]    = "HPMEVENT15",
    [MISCREG_HPMEVENT16]    = "HPMEVENT16",
    [MISCREG_HPMEVENT17]    = "HPMEVENT17",
    [MISCREG_HPMEVENT18]    = "HPMEVENT18",
    [MISCREG_HPMEVENT19]    = "HPMEVENT19",
    [MISCREG_HPMEVENT20]    = "HPMEVENT20",
    [MISCREG_HPMEVENT21]    = "HPMEVENT21",
    [MISCREG_HPMEVENT22]    = "HPMEVENT22",
    [MISCREG_HPMEVENT23]    = "HPMEVENT23",
    [MISCREG_HPMEVENT24]    = "HPMEVENT24",
    [MISCREG_HPMEVENT25]    = "HPMEVENT25",
    [MISCREG_HPMEVENT26]    = "HPMEVENT26",
    [MISCREG_HPMEVENT27]    = "HPMEVENT27",
    [MISCREG_HPMEVENT28]    = "HPMEVENT28",
    [MISCREG_HPMEVENT29]    = "HPMEVENT29",
    [MISCREG_HPMEVENT30]    = "HPMEVENT30",
    [MISCREG_HPMEVENT31]    = "HPMEVENT31",
    [MISCREG_TSELECT]       = "TSELECT",
    [MISCREG_TDATA1]        = "TDATA1",
    [MISCREG_TDATA2]        = "TDATA2",
    [MISCREG_TDATA3]        = "TDATA3",
    [MISCREG_DCSR]          = "DCSR",
    [MISCREG_DPC]           = "DPC",
    [MISCREG_DSCRATCH]      = "DSCRATCH",

    [MISCREG_MEDELEG]       = "MEDELEG",
    [MISCREG_MIDELEG]       = "MIDELEG",
    [MISCREG_MTVEC]         = "MTVEC",
    [MISCREG_MCOUNTEREN]    = "MCOUNTEREN",
    [MISCREG_MSCRATCH]      = "MSCRATCH",
    [MISCREG_MEPC]          = "MEPC",
    [MISCREG_MCAUSE]        = "MCAUSE",
    [MISCREG_MTVAL]         = "MTVAL",
    [MISCREG_PMPCFG0]       = "PMPCFG0",
    [MISCREG_PMPCFG1]       = "PMPCFG1",   // pmpcfg1 is rv32 only
    [MISCREG_PMPCFG2]       = "PMPCFG2",
    [MISCREG_PMPCFG3]       = "PMPCFG3",   // pmpcfg3 is rv32 only
    [MISCREG_PMPADDR00]     = "PMPADDR00",
    [MISCREG_PMPADDR01]     = "PMPADDR01",
    [MISCREG_PMPADDR02]     = "PMPADDR02",
    [MISCREG_PMPADDR03]     = "PMPADDR03",
    [MISCREG_PMPADDR04]     = "PMPADDR04",
    [MISCREG_PMPADDR05]     = "PMPADDR05",
    [MISCREG_PMPADDR06]     = "PMPADDR06",
    [MISCREG_PMPADDR07]     = "PMPADDR07",
    [MISCREG_PMPADDR08]     = "PMPADDR08",
    [MISCREG_PMPADDR09]     = "PMPADDR09",
    [MISCREG_PMPADDR10]     = "PMPADDR10",
    [MISCREG_PMPADDR11]     = "PMPADDR11",
    [MISCREG_PMPADDR12]     = "PMPADDR12",
    [MISCREG_PMPADDR13]     = "PMPADDR13",
    [MISCREG_PMPADDR14]     = "PMPADDR14",
    [MISCREG_PMPADDR15]     = "PMPADDR15",

    [MISCREG_SEDELEG]       = "SEDELEG",
    [MISCREG_SIDELEG]       = "SIDELEG",
    [MISCREG_STVEC]         = "STVEC",
    [MISCREG_SCOUNTEREN]    = "SCOUNTEREN",
    [MISCREG_SSCRATCH]      = "SSCRATCH",
    [MISCREG_SEPC]          = "SEPC",
    [MISCREG_SCAUSE]        = "SCAUSE",
    [MISCREG_STVAL]         = "STVAL",
    [MISCREG_SATP]          = "SATP",

    [MISCREG_UTVEC]         = "UTVEC",
    [MISCREG_USCRATCH]      = "USCRATCH",
    [MISCREG_UEPC]          = "UEPC",
    [MISCREG_UCAUSE]        = "UCAUSE",
    [MISCREG_UTVAL]         = "UTVAL",
    [MISCREG_FFLAGS]        = "FFLAGS",
    [MISCREG_FRM]           = "FRM",

    [MISCREG_VSTART]        = "VSTART",
    [MISCREG_VXSAT]         = "VXSAT",
    [MISCREG_VXRM]          = "VXRM",
    [MISCREG_VCSR]          = "VCSR",
    [MISCREG_VL]            = "VL",
    [MISCREG_VTYPE]         = "VTYPE",
    [MISCREG_VLENB]         = "VLENB",

    [MISCREG_NMIVEC]        = "NMIVEC",
    [MISCREG_NMIE]          = "NMIE",
    [MISCREG_NMIP]          = "NMIP",

    // following are rv32 only registers
    [MISCREG_MSTATUSH]      = "MSTATUSH",

    [MISCREG_CYCLEH]         = "CYCLEH",
    [MISCREG_TIMEH]          = "TIMEH",
    [MISCREG_INSTRETH]       = "INSTRETH",
    [MISCREG_HPMCOUNTER03H]  = "HPMCOUNTER03H",
    [MISCREG_HPMCOUNTER04H]  = "HPMCOUNTER04H",
    [MISCREG_HPMCOUNTER05H]  = "HPMCOUNTER05H",
    [MISCREG_HPMCOUNTER06H]  = "HPMCOUNTER06H",
    [MISCREG_HPMCOUNTER07H]  = "HPMCOUNTER07H",
    [MISCREG_HPMCOUNTER08H]  = "HPMCOUNTER08H",
    [MISCREG_HPMCOUNTER09H]  = "HPMCOUNTER09H",
    [MISCREG_HPMCOUNTER10H]  = "HPMCOUNTER10H",
    [MISCREG_HPMCOUNTER11H]  = "HPMCOUNTER11H",
    [MISCREG_HPMCOUNTER12H]  = "HPMCOUNTER12H",
    [MISCREG_HPMCOUNTER13H]  = "HPMCOUNTER13H",
    [MISCREG_HPMCOUNTER14H]  = "HPMCOUNTER14H",
    [MISCREG_HPMCOUNTER15H]  = "HPMCOUNTER15H",
    [MISCREG_HPMCOUNTER16H]  = "HPMCOUNTER16H",
    [MISCREG_HPMCOUNTER17H]  = "HPMCOUNTER17H",
    [MISCREG_HPMCOUNTER18H]  = "HPMCOUNTER18H",
    [MISCREG_HPMCOUNTER19H]  = "HPMCOUNTER19H",
    [MISCREG_HPMCOUNTER20H]  = "HPMCOUNTER20H",
    [MISCREG_HPMCOUNTER21H]  = "HPMCOUNTER21H",
    [MISCREG_HPMCOUNTER22H]  = "HPMCOUNTER22H",
    [MISCREG_HPMCOUNTER23H]  = "HPMCOUNTER23H",
    [MISCREG_HPMCOUNTER24H]  = "HPMCOUNTER24H",
    [MISCREG_HPMCOUNTER25H]  = "HPMCOUNTER25H",
    [MISCREG_HPMCOUNTER26H]  = "HPMCOUNTER26H",
    [MISCREG_HPMCOUNTER27H]  = "HPMCOUNTER27H",
    [MISCREG_HPMCOUNTER28H]  = "HPMCOUNTER28H",
    [MISCREG_HPMCOUNTER29H]  = "HPMCOUNTER29H",
    [MISCREG_HPMCOUNTER30H]  = "HPMCOUNTER30H",
    [MISCREG_HPMCOUNTER31H]  = "HPMCOUNTER31H",

    [MISCREG_FFLAGS_EXE]    = "FFLAGS_EXE",
}};

namespace
{

/* Not applicable to RISCV */
RegClass vecElemClass(VecElemClass, VecElemClassName, 0, debug::IntRegs);
RegClass vecPredRegClass(VecPredRegClass, VecPredRegClassName, 0,
        debug::IntRegs);
RegClass matRegClass(MatRegClass, MatRegClassName, 0, debug::MatRegs);
RegClass ccRegClass(CCRegClass, CCRegClassName, 0, debug::IntRegs);

} // anonymous namespace

ISA::ISA(const Params &p) : BaseISA(p),
    _rvType(p.riscv_type), checkAlignment(p.check_alignment),
    enableRvv(p.enable_rvv), vlen(p.vlen), elen(p.elen),
    _privilegeModeSet(p.privilege_mode_set)
{
    _regClasses.push_back(&intRegClass);
    _regClasses.push_back(&floatRegClass);
    _regClasses.push_back(&vecRegClass);
    _regClasses.push_back(&vecElemClass);
    _regClasses.push_back(&vecPredRegClass);
    _regClasses.push_back(&matRegClass);
    _regClasses.push_back(&ccRegClass);
    _regClasses.push_back(&miscRegClass);

    fatal_if( p.vlen < p.elen,
    "VLEN should be greater or equal",
        "than ELEN. Ch. 2RISC-V vector spec.");

    inform("RVV enabled, VLEN = %d bits, ELEN = %d bits",
            p.vlen, p.elen);


    miscRegFile.resize(NUM_PHYS_MISCREGS);
    clear();
}

bool ISA::inUserMode() const
{
    return miscRegFile[MISCREG_PRV] == PRV_U;
}

void
ISA::copyRegsFrom(ThreadContext *src)
{
    // First loop through the integer registers.
    for (auto &id: intRegClass)
        tc->setReg(id, src->getReg(id));

    // Second loop through the float registers.
    for (auto &id: floatRegClass)
        tc->setReg(id, src->getReg(id));

    // Third loop through the vector registers.
    RiscvISA::VecRegContainer vc;
    for (auto &id: vecRegClass) {
        src->getReg(id, &vc);
        tc->setReg(id, &vc);
    }

    // Copying Misc Regs
    for (int i = 0; i < NUM_PHYS_MISCREGS; i++)
        tc->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    // Lastly copy PC/NPC
    tc->pcState(src->pcState());
}

void ISA::clear()
{
    std::fill(miscRegFile.begin(), miscRegFile.end(), 0);

    miscRegFile[MISCREG_PRV] = PRV_M;
    miscRegFile[MISCREG_VENDORID] = 0;
    miscRegFile[MISCREG_ARCHID] = 0;
    miscRegFile[MISCREG_IMPID] = 0;

    MISA misa = 0;
    STATUS status = 0;

    // default config arch isa string is rv64(32)imafdc
    misa.rvi = misa.rvm = misa.rva = misa.rvf = misa.rvd = misa.rvc = 1;

    switch (getPrivilegeModeSet()) {
        case enums::M:
          break;
        case enums::MU:
          misa.rvu = 1;
          break;
        case enums::MNU:
          misa.rvu = misa.rvn = 1;
          break;
        case enums::MSU:
          misa.rvs = misa.rvu = 1;
          break;
        case enums::MNSU:
          misa.rvs = misa.rvu = misa.rvn = 1;
          break;
        default:
          panic("Privilege mode set config should not reach here");
    }

    // mark FS is initial
    status.fs = INITIAL;

    // _rvType dependent init.
    switch (_rvType) {
        case RV32:
          misa.rv32_mxl = 1;
          break;
        case RV64:
          misa.rv64_mxl = 2;
          status.uxl = status.sxl = 2;
          if (getEnableRvv()) {
              status.vs = VPUStatus::INITIAL;
              misa.rvv = 1;
          }
          break;
        default:
          panic("%s: Unknown _rvType: %d", name(), (int)_rvType);
    }

    miscRegFile[MISCREG_ISA] = misa;
    miscRegFile[MISCREG_STATUS] = status;
    miscRegFile[MISCREG_MCOUNTEREN] = 0x7;
    miscRegFile[MISCREG_SCOUNTEREN] = 0x7;
    // don't set it to zero; software may try to determine the supported
    // triggers, starting at zero. simply set a different value here.
    miscRegFile[MISCREG_TSELECT] = 1;
    // NMI is always enabled.
    miscRegFile[MISCREG_NMIE] = 1;
}

bool
ISA::hpmCounterEnabled(int misc_reg) const
{
    int hpmcounter = 0;
    if (misc_reg >= MISCREG_CYCLEH) {
        hpmcounter = misc_reg - MISCREG_CYCLEH;
    } else {
        hpmcounter = misc_reg - MISCREG_CYCLE;
    }

    if (hpmcounter < 0 || hpmcounter > 31)
        panic("Illegal HPM counter %d\n", hpmcounter);
    int counteren;
    switch (readMiscRegNoEffect(MISCREG_PRV)) {
      case PRV_M:
        return true;
      case PRV_S:
        counteren = MISCREG_MCOUNTEREN;
        break;
      case PRV_U:
        counteren = MISCREG_SCOUNTEREN;
        break;
      default:
        panic("Unknown privilege level %d\n", miscRegFile[MISCREG_PRV]);
        return false;
    }
    return (miscRegFile[counteren] & (1ULL << (hpmcounter))) > 0;
}

RegVal
ISA::readMiscRegNoEffect(RegIndex idx) const
{
    // Illegal CSR
    panic_if(idx > NUM_PHYS_MISCREGS, "Illegal CSR index %#x\n", idx);
    DPRINTF(RiscvMisc, "Reading MiscReg %s (%d): %#x.\n",
            MiscRegNames[idx], idx, miscRegFile[idx]);
    return miscRegFile[idx];
}

RegVal
ISA::readMiscReg(RegIndex idx)
{
    switch (idx) {
      case MISCREG_HARTID:
        return tc->contextId();
      case MISCREG_CYCLE:
        if (hpmCounterEnabled(MISCREG_CYCLE)) {
            DPRINTF(RiscvMisc, "Cycle counter at: %llu.\n",
                    tc->getCpuPtr()->curCycle());
            return static_cast<RegVal>(tc->getCpuPtr()->curCycle());
        } else {
            warn("Cycle counter disabled.\n");
            return 0;
        }
      case MISCREG_CYCLEH:
        if (hpmCounterEnabled(MISCREG_CYCLEH)) {
            DPRINTF(RiscvMisc, "Cycle counter at: %llu.\n",
                    tc->getCpuPtr()->curCycle());
            return bits<RegVal>(tc->getCpuPtr()->curCycle(), 63, 32);
        } else {
            warn("Cycle counter disabled.\n");
            return 0;
        }
      case MISCREG_TIME:
        if (hpmCounterEnabled(MISCREG_TIME)) {
            DPRINTF(RiscvMisc, "Wall-clock counter at: %llu.\n",
                    std::time(nullptr));
            return readMiscRegNoEffect(MISCREG_TIME);
        } else {
            warn("Wall clock disabled.\n");
            return 0;
        }
      case MISCREG_TIMEH:
        if (hpmCounterEnabled(MISCREG_TIMEH)) {
            DPRINTF(RiscvMisc, "Wall-clock counter at: %llu.\n",
                    std::time(nullptr));
            return readMiscRegNoEffect(MISCREG_TIMEH);
        } else {
            warn("Wall clock disabled.\n");
            return 0;
        }
      case MISCREG_INSTRET:
        if (hpmCounterEnabled(MISCREG_INSTRET)) {
            DPRINTF(RiscvMisc, "Instruction counter at: %llu.\n",
                    tc->getCpuPtr()->totalInsts());
            return static_cast<RegVal>(tc->getCpuPtr()->totalInsts());
        } else {
            warn("Instruction counter disabled.\n");
            return 0;
        }
      case MISCREG_INSTRETH:
        if (hpmCounterEnabled(MISCREG_INSTRETH)) {
            DPRINTF(RiscvMisc, "Instruction counter at: %llu.\n",
                    tc->getCpuPtr()->totalInsts());
            return bits<RegVal>(tc->getCpuPtr()->totalInsts(), 63, 32);
        } else {
            warn("Instruction counter disabled.\n");
            return 0;
        }
      case MISCREG_IP:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIP();
        }
      case MISCREG_IE:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIE();
        }
      case MISCREG_SEPC:
      case MISCREG_MEPC:
        {
            MISA misa = readMiscRegNoEffect(MISCREG_ISA);
            auto val = readMiscRegNoEffect(idx);
            // if compressed instructions are disabled, epc[1] is set to 0
            if (misa.rvc == 0)
                return mbits(val, 63, 2);
            // epc[0] is always 0
            else
                return mbits(val, 63, 1);
        }
      case MISCREG_STATUS:
        {
            // Updating the SD bit.
            // . Per RISC-V ISA Manual, vol II, section 3.1.6.6, page 26,
            // the SD bit is a read-only bit indicating whether any of
            // FS, VS, and XS fields being in the respective dirty state.
            // . Per section 3.1.6, page 20, the SD bit is the most
            // significant bit of the MSTATUS CSR for both RV32 and RV64.
            // . Per section 3.1.6.6, page 29, the explicit formula for
            // updating the SD is,
            //   SD = ((FS==DIRTY) | (XS==DIRTY) | (VS==DIRTY))
            // . Ideally, we want to update the SD after every relevant
            // instruction, however, lazily updating the Status register
            // upon its read produces the same effect as well.
            STATUS status = readMiscRegNoEffect(idx);
            uint64_t sd_bit = \
                (status.xs == 3) || (status.fs == 3) || (status.vs == 3);
            // For RV32, the SD bit is at index 31
            // For RV64, the SD bit is at index 63.
            switch (_rvType) {
                case RV32:
                    status.rv32_sd = sd_bit;
                    break;
                case RV64:
                    status.rv64_sd = sd_bit;
                    break;
                default:
                    panic("%s: Unknown _rvType: %d", name(), (int)_rvType);
            }
            // Check status.mpp
            MISA misa = readMiscRegNoEffect(MISCREG_ISA);
            switch(status.mpp) {
                case PRV_U:
                    status.mpp = (misa.rvu) ? PRV_U : PRV_M;
                    break;
                case PRV_S:
                    if (misa.rvs)
                        status.mpp = PRV_S;
                    else
                        status.mpp = (misa.rvu) ? PRV_U : PRV_M;
                    break;
                case PRV_M:
                    break;
                default:
                    status.mpp = (misa.rvu) ? PRV_U : PRV_M;
            }

            setMiscRegNoEffect(idx, status);

            return readMiscRegNoEffect(idx);
        }
      case MISCREG_VLENB:
        {
            auto rpc = tc->pcState().as<PCState>();
            return rpc.vlenb();
        }
      case MISCREG_VTYPE:
        {
            auto rpc = tc->pcState().as<PCState>();
            return rpc.vtype();
        }
      case MISCREG_VL:
        {
            auto rpc = tc->pcState().as<PCState>();
            return (RegVal)rpc.vl();
        }
      case MISCREG_VCSR:
        {
            return readMiscRegNoEffect(MISCREG_VXSAT) &
                  (readMiscRegNoEffect(MISCREG_VXRM) << 1);
        }
        break;
      case MISCREG_FFLAGS_EXE:
        {
            return readMiscRegNoEffect(MISCREG_FFLAGS) & FFLAGS_MASK;
        }
      default:
        // Try reading HPM counters
        // As a placeholder, all HPM counters are just cycle counters
        if (idx >= MISCREG_HPMCOUNTER03 &&
                idx <= MISCREG_HPMCOUNTER31) {
            if (hpmCounterEnabled(idx)) {
                DPRINTF(RiscvMisc, "HPM counter %d: %llu.\n",
                        idx - MISCREG_CYCLE, tc->getCpuPtr()->curCycle());
                return tc->getCpuPtr()->curCycle();
            } else {
                warn("HPM counter %d disabled.\n", idx - MISCREG_CYCLE);
                return 0;
            }
        } else if (idx >= MISCREG_HPMCOUNTER03H &&
                idx <= MISCREG_HPMCOUNTER31H) {
            if (hpmCounterEnabled(idx)) {
                DPRINTF(RiscvMisc, "HPM counter %d: %llu.\n",
                        idx - MISCREG_CYCLE, tc->getCpuPtr()->curCycle());
                return bits<RegVal>(tc->getCpuPtr()->curCycle(), 63, 32);
            } else {
                warn("HPM counter %d disabled.\n", idx - MISCREG_CYCLE);
                return 0;
            }
        }
        return readMiscRegNoEffect(idx);
    }
}

void
ISA::setMiscRegNoEffect(RegIndex idx, RegVal val)
{
    // Illegal CSR
    panic_if(idx > NUM_PHYS_MISCREGS, "Illegal CSR index %#x\n", idx);
    DPRINTF(RiscvMisc, "Setting MiscReg %s (%d) to %#x.\n",
            MiscRegNames[idx], idx, val);
    miscRegFile[idx] = val;
}

void
ISA::setMiscReg(RegIndex idx, RegVal val)
{
    if (idx >= MISCREG_CYCLE && idx <= MISCREG_HPMCOUNTER31) {
        // Ignore writes to HPM counters for now
        warn("Ignoring write to miscreg %s.\n", MiscRegNames[idx]);
    } else {
        switch (idx) {

          // From section 3.7.1 of RISCV priv. specs
          // V1.12, the odd-numbered configuration
          // registers are illegal for RV64 and
          // each 64 bit CFG register hold configurations
          // for 8 PMP entries.

          case MISCREG_PMPCFG0:
          case MISCREG_PMPCFG1:
          case MISCREG_PMPCFG2:
          case MISCREG_PMPCFG3:
            {
                // PMP registers should only be modified in M mode
                assert(readMiscRegNoEffect(MISCREG_PRV) == PRV_M);

                int regSize = 0;
                switch (_rvType) {
                    case RV32:
                        regSize = 4;
                    break;
                    case RV64:
                        regSize = 8;
                    break;
                    default:
                        panic("%s: Unknown _rvType: %d", name(), (int)_rvType);
                }

                // Specs do not seem to mention what should be
                // configured first, cfg or address regs!
                // qemu seems to update the tables when
                // pmp addr regs are written (with the assumption
                // that cfg regs are already written)
                RegVal res = 0;
                RegVal old_val = readMiscRegNoEffect(idx);

                for (int i=0; i < regSize; i++) {

                    uint8_t cfg_val = (val >> (8*i)) & 0xff;
                    auto mmu = dynamic_cast<RiscvISA::MMU *>
                                (tc->getMMUPtr());

                    // Form pmp_index using the index i and
                    // PMPCFG register number
                    uint32_t pmp_index = i+(4*(idx-MISCREG_PMPCFG0));
                    bool result = mmu->getPMP()->pmpUpdateCfg(pmp_index,cfg_val);
                    if (result) {
                        res |= ((RegVal)cfg_val << (8*i));
                    } else {
                        res |= (old_val & (0xFF << (8*i)));
                    }
                }

                setMiscRegNoEffect(idx, res);
            }
            break;
          case MISCREG_PMPADDR00 ... MISCREG_PMPADDR15:
            {
                // PMP registers should only be modified in M mode
                assert(readMiscRegNoEffect(MISCREG_PRV) == PRV_M);

                auto mmu = dynamic_cast<RiscvISA::MMU *>
                              (tc->getMMUPtr());
                uint32_t pmp_index = idx-MISCREG_PMPADDR00;
                if (mmu->getPMP()->pmpUpdateAddr(pmp_index, val)) {
                    setMiscRegNoEffect(idx, val);
                }
            }
            break;

          case MISCREG_IP:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIP(val);
            }
            break;
          case MISCREG_IE:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIE(val);
            }
            break;
          case MISCREG_SATP:
            {
                // we only support bare and Sv39 mode; setting a different mode
                // shall have no effect (see 4.1.12 in priv ISA manual)
                SATP cur_val = readMiscRegNoEffect(idx);
                SATP new_val = val;
                if (new_val.mode != AddrXlateMode::BARE &&
                    new_val.mode != AddrXlateMode::SV39)
                    new_val.mode = cur_val.mode;
                setMiscRegNoEffect(idx, new_val);
            }
            break;
          case MISCREG_TSELECT:
            {
                // we don't support debugging, so always set a different value
                // than written
                setMiscRegNoEffect(idx, val + 1);
            }
            break;
          case MISCREG_ISA:
            {
                MISA cur_misa = (MISA)readMiscRegNoEffect(MISCREG_ISA);
                MISA new_misa = (MISA)val;
                // only allow to disable compressed instructions
                // if the following instruction is 4-byte aligned
                if (new_misa.rvc == 0 &&
                        bits(tc->pcState().as<RiscvISA::PCState>().npc(),
                            2, 0) != 0) {
                    new_misa.rvc = new_misa.rvc | cur_misa.rvc;
                }
                if (!getEnableRvv()) {
                    new_misa.rvv = 0;
                }
                new_misa.rvn = cur_misa.rvn;
                new_misa.rvs = cur_misa.rvs;
                new_misa.rvu = cur_misa.rvu;
                setMiscRegNoEffect(idx, new_misa);
            }
            break;
          case MISCREG_STATUS:
            {
                if (_rvType != RV32) {
                    // SXL and UXL are hard-wired to 64 bit
                    auto cur = readMiscRegNoEffect(idx);
                    val &= ~(STATUS_SXL_MASK | STATUS_UXL_MASK);
                    val |= cur & (STATUS_SXL_MASK | STATUS_UXL_MASK);
                }
                if (!getEnableRvv()) {
                    // Always OFF is rvv is disabled.
                    val &= ~STATUS_VS_MASK;
                }
                setMiscRegNoEffect(idx, val);
            }
            break;
          case MISCREG_VXSAT:
            {
                setMiscRegNoEffect(idx, val & 0x1);
            }
            break;
          case MISCREG_VXRM:
            {
                setMiscRegNoEffect(idx, val & 0x3);
            }
            break;
          case MISCREG_VCSR:
            {
                setMiscRegNoEffect(MISCREG_VXSAT, val & 0x1);
                setMiscRegNoEffect(MISCREG_VXRM, (val & 0x6) >> 1);
            }
            break;
          case MISCREG_FFLAGS_EXE:
            {
                RegVal new_val = readMiscRegNoEffect(MISCREG_FFLAGS);
                new_val |= (val & FFLAGS_MASK);
                setMiscRegNoEffect(MISCREG_FFLAGS, new_val);
            }
            break;
          default:
            setMiscRegNoEffect(idx, val);
        }
    }
}

void
ISA::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Riscv Misc Registers\n");
    SERIALIZE_CONTAINER(miscRegFile);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Riscv Misc Registers\n");
    UNSERIALIZE_CONTAINER(miscRegFile);
}

void
ISA::handleLockedSnoop(PacketPtr pkt, Addr cacheBlockMask)
{
    Addr& load_reservation_addr = load_reservation_addrs[tc->contextId()];

    if (load_reservation_addr == INVALID_RESERVATION_ADDR)
        return;
    Addr snoop_addr = pkt->getAddr() & cacheBlockMask;
    DPRINTF(LLSC, "Locked snoop on address %x.\n", snoop_addr);
    if ((load_reservation_addr & cacheBlockMask) == snoop_addr)
        load_reservation_addr = INVALID_RESERVATION_ADDR;
}


void
ISA::handleLockedRead(const RequestPtr &req)
{
    Addr& load_reservation_addr = load_reservation_addrs[tc->contextId()];

    load_reservation_addr = req->getPaddr();
    DPRINTF(LLSC, "[cid:%d]: Reserved address %x.\n",
            req->contextId(), req->getPaddr());
}

bool
ISA::handleLockedWrite(const RequestPtr &req, Addr cacheBlockMask)
{
    Addr& load_reservation_addr = load_reservation_addrs[tc->contextId()];
    bool lr_addr_empty = (load_reservation_addr == INVALID_RESERVATION_ADDR);

    // Normally RISC-V uses zero to indicate success and nonzero to indicate
    // failure (right now only 1 is reserved), but in gem5 zero indicates
    // failure and one indicates success, so here we conform to that (it should
    // be switched in the instruction's implementation)

    DPRINTF(LLSC, "[cid:%d]: load_reservation_addrs empty? %s.\n",
            req->contextId(),
            lr_addr_empty ? "yes" : "no");
    if (!lr_addr_empty) {
        DPRINTF(LLSC, "[cid:%d]: addr = %x.\n", req->contextId(),
                req->getPaddr() & cacheBlockMask);
        DPRINTF(LLSC, "[cid:%d]: last locked addr = %x.\n", req->contextId(),
                load_reservation_addr & cacheBlockMask);
    }
    if (lr_addr_empty ||
            (load_reservation_addr & cacheBlockMask)
            != ((req->getPaddr() & cacheBlockMask))) {
        req->setExtraData(0);
        int stCondFailures = tc->readStCondFailures();
        tc->setStCondFailures(++stCondFailures);
        if (stCondFailures % WARN_FAILURE == 0) {
            warn("%i: context %d: %d consecutive SC failures.\n",
                    curTick(), tc->contextId(), stCondFailures);
        }

        // Must clear any reservations
        load_reservation_addr = INVALID_RESERVATION_ADDR;

        return false;
    }
    if (req->isUncacheable()) {
        req->setExtraData(2);
    }

    // Must clear any reservations
    load_reservation_addr = INVALID_RESERVATION_ADDR;

    DPRINTF(LLSC, "[cid:%d]: SC success! Current locked addr = %x.\n",
            req->contextId(), load_reservation_addr & cacheBlockMask);
    return true;
}

void
ISA::globalClearExclusive()
{
    tc->getCpuPtr()->wakeup(tc->threadId());
    Addr& load_reservation_addr = load_reservation_addrs[tc->contextId()];
    load_reservation_addr = INVALID_RESERVATION_ADDR;
}

void
ISA::resetThread()
{
    Reset().invoke(tc);
}

Addr
ISA::getFaultHandlerAddr(RegIndex idx, uint64_t cause, bool intr) const
{
    auto vec = tc->readMiscRegNoEffect(idx);
    Addr addr = mbits(vec, 63, 2);
    if (intr && bits(vec, 1, 0) == 1)
        addr += 4 * cause;
    return addr;
}

} // namespace RiscvISA
} // namespace gem5

std::ostream &
operator<<(std::ostream &os, gem5::RiscvISA::PrivilegeMode pm)
{
    switch (pm) {
    case gem5::RiscvISA::PRV_U:
        return os << "PRV_U";
    case gem5::RiscvISA::PRV_S:
        return os << "PRV_S";
    case gem5::RiscvISA::PRV_M:
        return os << "PRV_M";
    }
    return os << "PRV_<invalid>";
}
