/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2022 Google LLC
 * Copyright (c) 2024 University of Rostock
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

    // H-extension (RV64) registers

    [MISCREG_HVIP]          = "HVIP",

    [MISCREG_MTINST]        = "MTINST",
    [MISCREG_MTVAL2]        = "MTVAL2",

    [MISCREG_HSTATUS]       = "HSTATUS",
    [MISCREG_HEDELEG]       = "HEDELEG",
    [MISCREG_HIDELEG]       = "HIDELEG",
    [MISCREG_HCOUNTEREN]    = "HCOUNTEREN",
    [MISCREG_HGEIE]         = "HGEIE",
    [MISCREG_HTVAL]         = "HTVAL",
    [MISCREG_HTINST]        = "HTINST",
    [MISCREG_HGEIP]         = "HGEIP",

    [MISCREG_HENVCFG]       = "HENVCFG",
    [MISCREG_HGATP]         = "HGATP",
    [MISCREG_HCONTEXT]      = "HCONTEXT",
    [MISCREG_HTIMEDELTA]    = "HTIMEDELTA",

    [MISCREG_VSSTATUS]      = "VSSTATUS",
    [MISCREG_VSTVEC]        = "VSTVEC",
    [MISCREG_VSSCRATCH]     = "VSSCRATCH",
    [MISCREG_VSEPC]         = "VSEPC",
    [MISCREG_VSCAUSE]       = "VSCAUSE",
    [MISCREG_VSTVAL]        = "VSTVAL",
    [MISCREG_VSATP]         = "VSATP",
    [MISCREG_VIRT]          = "VIRT",

    // H-extension (RV64) registers end here

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

ISA::ISA(const Params &p) : BaseISA(p, "riscv"),
    _rvType(p.riscv_type), enableRvv(p.enable_rvv), vlen(p.vlen), elen(p.elen),
    _privilegeModeSet(p.privilege_mode_set),
    _wfiResumeOnPending(p.wfi_resume_on_pending)
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

    miscRegFile.resize(NUM_MISCREGS);
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
    for (int i = 0; i < NUM_MISCREGS; i++)
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
        case enums::MHSU:
          misa.rvh = misa.rvs = misa.rvu = 1;
          inform("RVH enabled.");
          break;
        default:
          panic("Privilege mode set config should not reach here");
    }

    // mark FS is initial
    status.fs = FPUStatus::INITIAL;

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
          if (misa.rvh) {
              HSTATUS hstatus = 0;
              hstatus.vsxl = 2;
              miscRegFile[MISCREG_HSTATUS] = hstatus;

              STATUS vsstatus = 0;
              vsstatus.uxl = 2;
              vsstatus.fs = FPUStatus::INITIAL;

              if (getEnableRvv())
                vsstatus.vs = VPUStatus::INITIAL;

              miscRegFile[MISCREG_VSSTATUS] = vsstatus;
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

Fault
ISA::hpmCounterCheck(int misc_reg, ExtMachInst machInst) const
{
    int hpmcounter = 0;
    if (misc_reg >= MISCREG_CYCLEH) {
        hpmcounter = misc_reg - MISCREG_CYCLEH;
    } else {
        hpmcounter = misc_reg - MISCREG_CYCLE;
    }

    if (hpmcounter < 0 || hpmcounter > 31)
        panic("Illegal HPM counter %d\n", hpmcounter);

    PrivilegeMode prv = (PrivilegeMode)readMiscRegNoEffect(MISCREG_PRV);
    MISA misa = readMiscRegNoEffect(MISCREG_ISA);

    RegVal mcounteren = miscRegFile[MISCREG_MCOUNTEREN];
    RegVal hcounteren = misa.rvh ? miscRegFile[MISCREG_HCOUNTEREN] : 0;
    RegVal scounteren = misa.rvs ? miscRegFile[MISCREG_SCOUNTEREN] : 0;

    // Paragraph 3.1.11 RISCV Privileged Spec 20211203
    // firstly, respect mcounteren disallowing the counter
    if (prv < PRV_M && (bits(mcounteren, hpmcounter) == 0)) {
        return std::make_shared<IllegalInstFault>(
            csprintf("Counter %s is disabled from mcounteren bit %d\n",
            MiscRegNames[misc_reg], hpmcounter), machInst);
    }

    // then, if h-extension is on and virtualization
    // is enabled, respect hcounteren for VS and VU
    if (misa.rvh && isV() && prv < PRV_HS &&
        bits(hcounteren, hpmcounter) == 0)
    {
        return std::make_shared<VirtualInstFault>(
            csprintf("Counter %s is disabled from hcounteren bit %d\n",
            MiscRegNames[misc_reg], hpmcounter), machInst);
    }

    // finally, respect scounteren for PRV_U (either with V = 1 or 0)
    if (prv == PRV_U && bits(scounteren, hpmcounter) == 0) {
        return std::make_shared<IllegalInstFault>(
            csprintf("Counter %s is disabled from scounteren bit %d\n",
            MiscRegNames[misc_reg], hpmcounter), machInst);
    }

    return NoFault;
}

RegVal
ISA::readMiscRegNoEffect(RegIndex idx) const
{
    // Illegal CSR
    panic_if(idx > NUM_MISCREGS, "Illegal CSR index %#x\n", idx);
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
            return static_cast<RegVal>(tc->getCpuPtr()->curCycle());
      case MISCREG_CYCLEH:
            return bits<RegVal>(tc->getCpuPtr()->curCycle(), 63, 32);
      case MISCREG_TIME:
            return readMiscRegNoEffect(MISCREG_TIME);
      case MISCREG_TIMEH:
            return readMiscRegNoEffect(MISCREG_TIMEH);
      case MISCREG_INSTRET:
            return static_cast<RegVal>(tc->getCpuPtr()->totalInsts());
      case MISCREG_INSTRETH:
            return bits<RegVal>(tc->getCpuPtr()->totalInsts(), 63, 32);
      case MISCREG_IP:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIP();
        }
      case MISCREG_HVIP:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readHVIP();
        }
      case MISCREG_IE:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIE();
        }
      case MISCREG_MIDELEG:
        {
            MISA misa = readMiscRegNoEffect(MISCREG_ISA);
            auto mideleg_val = readMiscRegNoEffect(MISCREG_MIDELEG);

            // Note: old versions of bbl don't like HS_INTERRUPTS on mideleg
            // In those cases, the bootloader panics.
            // You can replace with the line below, however H-extension
            // is incomplete this way.
            //return misa.rvh ? mideleg_val : mideleg_val;
            return misa.rvh ? mideleg_val | HS_INTERRUPTS : mideleg_val;

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
      case MISCREG_HSTATUS:
        {
            auto val = readMiscRegNoEffect(idx);
            // hardwire vsxl
            return val |= (2ULL << VSXL_OFFSET);
        }
      case MISCREG_VSSTATUS:
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
            STATUS oldstatus = status;
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

            if ((isV() && idx == MISCREG_VSSTATUS) ||
                         (idx == MISCREG_STATUS)) {
                RegVal bits_of_interest = STATUS_MPP_MASK |
                                          STATUS_MPRV_MASK |
                                          STATUS_MXR_MASK |
                                          STATUS_SUM_MASK;

                // If at least one of these changed, flush
                if ((oldstatus ^ status) & bits_of_interest) {
                    tc->getMMUPtr()->flushAll();
                }
            }

            setMiscRegNoEffect(idx, status);
            return readMiscRegNoEffect(idx);
        }
      case MISCREG_VLENB:
        {
            return getVecLenInBytes();
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
            return readMiscRegNoEffect(MISCREG_VXSAT) |
                  (readMiscRegNoEffect(MISCREG_VXRM) << 1);
        }
        break;
      default:
        // Try reading HPM counters
        // As a placeholder, all HPM counters are just cycle counters
        if (idx >= MISCREG_HPMCOUNTER03 &&
                idx <= MISCREG_HPMCOUNTER31) {
            return tc->getCpuPtr()->curCycle();

        } else if (idx >= MISCREG_HPMCOUNTER03H &&
                idx <= MISCREG_HPMCOUNTER31H) {
            return bits<RegVal>(tc->getCpuPtr()->curCycle(), 63, 32);
        }
        return readMiscRegNoEffect(idx);
    }
}

void
ISA::setMiscRegNoEffect(RegIndex idx, RegVal val)
{
    // Illegal CSR
    panic_if(idx > NUM_MISCREGS, "Illegal CSR index %#x\n", idx);
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
                tc->getMMUPtr()->flushAll();
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
                mmu->flushAll();
            }
            break;

          case MISCREG_IP:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIP(val);
            }
            break;
          case MISCREG_HVIP:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setHVIP(val);
            }
            break;
          case MISCREG_IE:
            {
                val = val & MI_MASK[getPrivilegeModeSet()];
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIE(val);
            }
            break;
          case MISCREG_SATP:
          case MISCREG_VSATP:
            {
                // we only support bare and Sv39 mode; setting a different mode
                // shall have no effect (see 4.1.12 in priv ISA manual)
                SATP cur_val = readMiscRegNoEffect(idx);
                SATP new_val = val;
                if (new_val.mode != AddrXlateMode::BARE &&
                    new_val.mode != AddrXlateMode::SV39)
                    new_val.mode = cur_val.mode;

                // Flush TLB if satp changed as Spike does
                if (new_val != cur_val) {
                    tc->getMMUPtr()->flushAll();
                }
                setMiscRegNoEffect(idx, new_val);
            }
            break;
          case MISCREG_HGATP:
            {
                tc->getMMUPtr()->flushAll();
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
                STATUS oldstatus = readMiscReg(idx);
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
                RegVal bits_of_interest = STATUS_MPP_MASK |
                                          STATUS_MPRV_MASK |
                                          STATUS_MXR_MASK |
                                          STATUS_SUM_MASK;

                // If at least one of these changed, flush
                if ((oldstatus ^ val) & bits_of_interest) {
                    tc->getMMUPtr()->flushAll();
                }
                setMiscRegNoEffect(idx, val);
            }
            break;
          case MISCREG_VSSTATUS:
            {

                auto cur = readMiscRegNoEffect(idx);
                auto wmask_map = CSRWriteMasks[RV64][getPrivilegeModeSet()];
                auto sstatus_wmask = wmask_map.find(CSR_VSSTATUS)->second;
                val = (cur & ~sstatus_wmask) | val;
                if (isV()) {
                    RegVal bits_of_interest = 0
                        | STATUS_MPP_MASK
                        | STATUS_MPRV_MASK
                        | STATUS_MXR_MASK
                        | STATUS_SUM_MASK;

                    // If at least one of these changed, flush
                    if ((cur ^ val) & bits_of_interest) {
                        tc->getMMUPtr()->flushAll();
                    }
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
          case MISCREG_PRV:
            {
                tc->getMMUPtr()->flushAll();
                setMiscRegNoEffect(idx, val);
            }
            break;
          case MISCREG_VIRT:
            {
                tc->getMMUPtr()->flushAll();
                setMiscRegNoEffect(idx, val);
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
    BaseISA::serialize(cp);

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

bool
ISA::isV() const
{
    // Virtualized when V-bit is 1
    return readMiscRegNoEffect(MISCREG_VIRT) == 1;
}

void
ISA::swapToVirtCSR(
    uint64_t& csr, RegIndex& midx, std::string& csrName)
{
    switch (csr) {
        case CSR_SSTATUS:  csr = CSR_VSSTATUS;  break;
        case CSR_STVEC:    csr = CSR_VSTVEC;    break;
        case CSR_SSCRATCH: csr = CSR_VSSCRATCH; break;
        case CSR_SEPC:     csr = CSR_VSEPC;     break;
        case CSR_SCAUSE:   csr = CSR_VSCAUSE;   break;
        case CSR_STVAL:    csr = CSR_VSTVAL;    break;
        case CSR_SATP:     csr = CSR_VSATP;     break;
        case CSR_SIP:      csr = CSR_VSIP;      break;
        case CSR_SIE:      csr = CSR_VSIE;      break;
        default:           return;              break;
    }

    // Lookup new midx and csrName
    auto csr_data_it = CSRData.find(csr);
    if (csr_data_it == CSRData.end()) {
        panic("Bad remapping of virtualized CSR");
    }
    midx = csr_data_it->second.physIndex;
    csrName = csr_data_it->second.name;
}

Fault
ISA::tvmChecks(uint64_t csr, PrivilegeMode pm, ExtMachInst machInst)
{
    if (csr == CSR_SATP) {
        STATUS status = readMiscReg(MISCREG_STATUS);
        if (pm != PRV_M && status.tvm == 1) {
            return std::make_shared<IllegalInstFault>(
                    "SATP access with TVM enabled\n",
                    machInst);
        }
    }
    else if (csr == CSR_VSATP) {
        if (isV()) {
        HSTATUS hstatus = readMiscReg(MISCREG_HSTATUS);
        if (hstatus.vtvm == 1)
            return std::make_shared<VirtualInstFault>(
                "VSATP access with hstatus.vtvm enabled",
                machInst);
        }
    }
    else if (csr == CSR_HGATP) {
        STATUS status = readMiscReg(MISCREG_STATUS);
        if (pm != PRV_M && status.tvm == 1) {
            return std::make_shared<IllegalInstFault>(
                    "HGATP access with TVM enabled\n",
                    machInst);
        }
    }


    return NoFault;
}

RegVal
ISA::backdoorReadCSRAllBits(uint64_t csr) {

    auto csr_it = CSRData.find(csr);

    // panic if the method was used with bad csr idx
    panic_if(csr_it == CSRData.end(),
        "Illegal CSR passed to backdoorReadCSRAllBits");

    auto midx = csr_it->second.physIndex;

    RegVal readval = readMiscReg(midx);

    // Special handling for FCSR
    if (csr == CSR_FCSR) {
        readval = (readMiscReg(MISCREG_FFLAGS) |
            (readMiscReg(MISCREG_FRM) << FRM_OFFSET));
    }

    return readval;
}

RegVal
ISA::readCSR(uint64_t csr)
{
    auto csr_it = CSRData.find(csr);

    // panic if the method was used with bad csr idx
    panic_if(csr_it == CSRData.end(), "Illegal CSR passed to readCSR");


    auto mask_it = getCSRMaskMap().find(csr);
    RegVal maskVal = (mask_it == getCSRMaskMap().end()) ?
                        mask(64) : mask_it->second;

    RegVal readval = backdoorReadCSRAllBits(csr) & maskVal;


    // Some registers need additional masking/shifting
    // to read the correct value
    switch (csr) {
        case CSR_SIP: case CSR_SIE:
        case CSR_HIP: case CSR_HIE:
        {
            RegVal mideleg = readMiscReg(MISCREG_MIDELEG);
            readval &= mideleg;
            break;
        }
        // VSIP and VSIE bits are stored one bit to the left
        // However reads expect them in SIP and SIE positions!
        case CSR_VSIP: case CSR_VSIE:
        {
            RegVal hideleg = readMiscReg(MISCREG_HIDELEG);
            readval &= hideleg;
            readval >>= 1;
            break;
        }

        case CSR_HVIP:
        {
            INTERRUPT mip = readMiscReg(MISCREG_IP);
            readval |= (mip.vssi << 2);
            break;
        }
        default: break;
    }

    return readval;
}

void
ISA::writeCSR(uint64_t csr, RegVal writeData)
{
    auto csr_it = CSRData.find(csr);

    // panic if the method was used with bad csr idx
    panic_if(csr_it == CSRData.end(), "Illegal CSR passed to writeCSR");

    // find physical register id (MISCREG == physical)
    auto midx = csr_it->second.physIndex;

    switch (csr) {
        case CSR_SIP: case CSR_SIE:
        case CSR_HIP: case CSR_HIE:
        {
            RegVal mideleg = readMiscReg(MISCREG_MIDELEG);
            writeData &= mideleg;
            break;
        }

        // Special case for VSIP & VSIE
        // We presented the bits in SIP and SIE (one position right)
        // But we shift them up one again so that they don't overwrite
        // the actual SIP and SIE but instead take their actual places.
        // This happens because XIP and XIE are shared registers
        // but VS-level should be unaware that its SIP and SIE bits are stored
        // elsewhere.
        case CSR_VSIP: case CSR_VSIE:
        {
            RegVal hideleg = readMiscReg(MISCREG_HIDELEG);
            writeData <<= 1;
            writeData &= hideleg;
            break;
        }
        default:
            break;
    }

    auto& csr_read_masks = getCSRMaskMap();
    auto& csr_write_masks = getCSRWriteMaskMap();

    // If no read mask or write mask, assume all bits are writable
    RegVal write_mask = mask(64);

    // To save space, we only specify write masks that differ
    // from the respective read masks in misc.hh
    // If there is no write mask, use the read mask to write
    // all "visible" aka readable bits. Else use mask(64).
    auto write_mask_it = csr_write_masks.find(csr);
    if (write_mask_it != csr_write_masks.end()) {
        write_mask = write_mask_it->second;
    } else {
        auto read_mask_it = csr_read_masks.find(csr);
        if (read_mask_it != csr_read_masks.end()) {
            write_mask = read_mask_it->second;
        }
    }

    auto writeDataMasked = writeData & write_mask;

    // CSRs are often aliases with different visibility
    // on the same physical register (MISCREG).
    // We must keep the values of the non-visible bits
    // intact for other privileges/contexts where
    // they are used.

    // Read all the CSR bits
    auto reg_data_all = backdoorReadCSRAllBits(csr);

    // Only modify those in writeMask
    auto new_reg_data_all = (reg_data_all & ~write_mask)
                          | (writeData & write_mask);


    switch (csr) {
        case CSR_FCSR: {
            setMiscReg(MISCREG_FFLAGS, bits(writeDataMasked, 4, 0));
            setMiscReg(MISCREG_FRM, bits(writeDataMasked, 7, 5));
            break;
        }

        case CSR_HVIP: {
            // vssi bit is an alias, propagate to mip
            INTERRUPT mip = readMiscReg(MISCREG_IP);
            mip.vssi = (new_reg_data_all & VSSI_MASK) >> 2;
            setMiscReg(MISCREG_IP, mip);

            // turn off vssi for write to HVIP
            new_reg_data_all &= ~VSSI_MASK;

            // finally write hvip
            setMiscReg(midx, new_reg_data_all);
            break;
        }
        // case CSR_MIP: case CSR_MIE:
        // case CSR_HIP: case CSR_HIE:
        // case CSR_SIP: case CSR_SIE:
        // case CSR_VSIP: case CSR_VSIE:
        // case CSR_UIP: case CSR_UIE:
        // case CSR_MSTATUS: case CSR_SSTATUS: case CSR_USTATUS:
        //     xc->setMiscReg(midx, new_reg_data_all);
        //     break;
        default:
            setMiscReg(midx, new_reg_data_all);
            break;
    }
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

// V-bit utilities (H-extension)
bool isV(ExecContext *xc) { return xc->readMiscReg(MISCREG_VIRT); }
bool isV(ThreadContext *tc) { return tc->readMiscReg(MISCREG_VIRT); }

void setV(ExecContext *xc) {
    assert(!isV(xc));
    xc->setMiscReg(MISCREG_VIRT, 1);
}
void setV(ThreadContext *tc) {
    assert(!isV(tc));
    tc->setMiscReg(MISCREG_VIRT, 1);
}

void resetV(ExecContext *xc) {
    assert(isV(xc));
    xc->setMiscReg(MISCREG_VIRT, 0);
}
void resetV(ThreadContext *tc) {
    assert(isV(tc));
    tc->setMiscReg(MISCREG_VIRT, 0);
}

// FPU status update function
Fault updateFPUStatus(ExecContext *xc, ExtMachInst machInst, bool set_dirty) {


    MISA misa = xc->readMiscReg(MISCREG_ISA);
    STATUS status = xc->readMiscReg(MISCREG_STATUS);
    STATUS vsstatus = misa.rvh && isV(xc) ?
        xc->readMiscReg(MISCREG_VSSTATUS) : 0;

    if (status.fs == FPUStatus::OFF ||
        (misa.rvh && isV(xc) && vsstatus.fs == FPUStatus::OFF))
        return std::make_shared<IllegalInstFault>("FPU is off", machInst);

    if (set_dirty) {
        status.fs = FPUStatus::DIRTY;
        xc->setMiscReg(MISCREG_STATUS, status);

        if (misa.rvh && isV(xc)) {
            vsstatus.fs = FPUStatus::DIRTY;
            xc->setMiscReg(MISCREG_VSSTATUS, vsstatus);
        }
    }

    return NoFault;
}

// VPU status update function
Fault updateVPUStatus(
    ExecContext *xc, ExtMachInst machInst, bool set_dirty, bool check_vill) {

    MISA misa = xc->readMiscReg(MISCREG_ISA);
    STATUS status = xc->readMiscReg(MISCREG_STATUS);
    STATUS vsstatus = misa.rvh && isV(xc) ?
        xc->readMiscReg(MISCREG_VSSTATUS) : 0;

    if (!misa.rvv || status.vs == VPUStatus::OFF ||
        (misa.rvh && isV(xc) && vsstatus.vs == VPUStatus::OFF))
        return std::make_shared<IllegalInstFault>(
            "RVV is disabled or VPU is off", machInst);


    if (check_vill && machInst.vill)
        return std::make_shared<IllegalInstFault>("VILL is set", machInst);


    if (set_dirty) {
        status.vs = VPUStatus::DIRTY;
        xc->setMiscReg(MISCREG_STATUS, status);

        if (misa.rvh && isV(xc)) {
            vsstatus.vs = VPUStatus::DIRTY;
            xc->setMiscReg(MISCREG_VSSTATUS, vsstatus);
        }
    }

    return NoFault;
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
    default:
        return os << "PRV_<invalid>";
    }
}
