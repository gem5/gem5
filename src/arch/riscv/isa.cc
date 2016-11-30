/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 *
 * Authors: Alec Roelke
 */
#include "arch/riscv/isa.hh"

#include <ctime>
#include <set>

#include "arch/riscv/registers.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "debug/RiscvMisc.hh"
#include "params/RiscvISA.hh"
#include "sim/core.hh"
#include "sim/pseudo_inst.hh"

namespace RiscvISA
{

std::map<int, std::string> ISA::miscRegNames = {
    {MISCREG_FFLAGS, "fflags"},
    {MISCREG_FRM, "frm"},
    {MISCREG_FCSR, "fcsr"},
    {MISCREG_CYCLE, "cycle"},
    {MISCREG_TIME, "time"},
    {MISCREG_INSTRET, "instret"},
    {MISCREG_CYCLEH, "cycleh"},
    {MISCREG_TIMEH, "timeh"},
    {MISCREG_INSTRETH, "instreth"},

    {MISCREG_SSTATUS, "sstatus"},
    {MISCREG_STVEC, "stvec"},
    {MISCREG_SIE, "sie"},
    {MISCREG_STIMECMP, "stimecmp"},
    {MISCREG_STIME, "stime"},
    {MISCREG_STIMEH, "stimeh"},
    {MISCREG_SSCRATCH, "sscratch"},
    {MISCREG_SEPC, "sepc"},
    {MISCREG_SCAUSE, "scause"},
    {MISCREG_SBADADDR, "sbadaddr"},
    {MISCREG_SIP, "sip"},
    {MISCREG_SPTBR, "sptbr"},
    {MISCREG_SASID, "sasid"},
    {MISCREG_CYCLEW, "cyclew"},
    {MISCREG_TIMEW, "timew"},
    {MISCREG_INSTRETW, "instretw"},
    {MISCREG_CYCLEHW, "cyclehw"},
    {MISCREG_TIMEHW, "timehw"},
    {MISCREG_INSTRETHW, "instrethw"},

    {MISCREG_HSTATUS, "hstatus"},
    {MISCREG_HTVEC, "htvec"},
    {MISCREG_HTDELEG, "htdeleg"},
    {MISCREG_HTIMECMP, "htimecmp"},
    {MISCREG_HTIME, "htime"},
    {MISCREG_HTIMEH, "htimeh"},
    {MISCREG_HSCRATCH, "hscratch"},
    {MISCREG_HEPC, "hepc"},
    {MISCREG_HCAUSE, "hcause"},
    {MISCREG_HBADADDR, "hbadaddr"},
    {MISCREG_STIMEW, "stimew"},
    {MISCREG_STIMEHW, "stimehw"},

    {MISCREG_MCPUID, "mcpuid"},
    {MISCREG_MIMPID, "mimpid"},
    {MISCREG_MHARTID, "mhartid"},
    {MISCREG_MSTATUS, "mstatus"},
    {MISCREG_MTVEC, "mtvec"},
    {MISCREG_MTDELEG, "mtdeleg"},
    {MISCREG_MIE, "mie"},
    {MISCREG_MTIMECMP, "mtimecmp"},
    {MISCREG_MTIME, "mtime"},
    {MISCREG_MTIMEH, "mtimeh"},
    {MISCREG_MSCRATCH, "mscratch"},
    {MISCREG_MEPC, "mepc"},
    {MISCREG_MCAUSE, "mcause"},
    {MISCREG_MBADADDR, "mbadaddr"},
    {MISCREG_MIP, "mip"},
    {MISCREG_MBASE, "mbase"},
    {MISCREG_MBOUND, "mbound"},
    {MISCREG_MIBASE, "mibase"},
    {MISCREG_MIBOUND, "mibound"},
    {MISCREG_MDBASE, "mdbase"},
    {MISCREG_MDBOUND, "mdbound"},
    {MISCREG_HTIMEW, "htimew"},
    {MISCREG_HTIMEHW, "htimehw"},
    {MISCREG_MTOHOST, "mtohost"},
    {MISCREG_MFROMHOST, "mfromhost"}
};

ISA::ISA(Params *p) : SimObject(p)
{
    miscRegFile.resize(NumMiscRegs);
    clear();
}

const RiscvISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void ISA::clear()
{
    std::fill(miscRegFile.begin(), miscRegFile.end(), 0);
}


MiscReg
ISA::readMiscRegNoEffect(int misc_reg) const
{
    DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n", miscRegNames[misc_reg],
        miscRegFile[misc_reg]);
    switch (misc_reg) {
      case MISCREG_FFLAGS:
        return bits(miscRegFile[MISCREG_FCSR], 4, 0);
      case MISCREG_FRM:
        return bits(miscRegFile[MISCREG_FCSR], 7, 5);
      case MISCREG_FCSR:
        return bits(miscRegFile[MISCREG_FCSR], 31, 0);
      case MISCREG_CYCLE:
        warn("Use readMiscReg to read the cycle CSR.");
        return 0;
      case MISCREG_TIME:
        return std::time(nullptr);
      case MISCREG_INSTRET:
        warn("Use readMiscReg to read the instret CSR.");
        return 0;
      case MISCREG_CYCLEH:
        warn("Use readMiscReg to read the cycleh CSR.");
        return 0;
      case MISCREG_TIMEH:
        return std::time(nullptr) >> 32;
      case MISCREG_INSTRETH:
        warn("Use readMiscReg to read the instreth CSR.");
        return 0;
      default:
        return miscRegFile[misc_reg];
    }
}

MiscReg
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_INSTRET:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            miscRegNames[misc_reg], miscRegFile[misc_reg]);
        return tc->getCpuPtr()->totalInsts();
      case MISCREG_CYCLE:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            miscRegNames[misc_reg], miscRegFile[misc_reg]);
        return tc->getCpuPtr()->curCycle();
      case MISCREG_INSTRETH:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            miscRegNames[misc_reg], miscRegFile[misc_reg]);
        return tc->getCpuPtr()->totalInsts() >> 32;
      case MISCREG_CYCLEH:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            miscRegNames[misc_reg], miscRegFile[misc_reg]);
        return tc->getCpuPtr()->curCycle() >> 32;
      default:
        return readMiscRegNoEffect(misc_reg);
    }
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    DPRINTF(RiscvMisc, "Setting CSR %s to 0x%016llx.\n",
        miscRegNames[misc_reg], miscRegNames[misc_reg], val);
    switch (misc_reg) {
      case MISCREG_FFLAGS:
        miscRegFile[MISCREG_FCSR] &= ~0x1F;
        miscRegFile[MISCREG_FCSR] |= bits(val, 4, 0);
        break;
      case MISCREG_FRM:
        miscRegFile[MISCREG_FCSR] &= ~0x70;
        miscRegFile[MISCREG_FCSR] |= bits(val, 2, 0) << 5;
        break;
      case MISCREG_FCSR:
        miscRegFile[MISCREG_FCSR] = bits(val, 7, 0);
        break;
      default:
        miscRegFile[misc_reg] = val;
        break;
    }
}

void
ISA::setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
{
    if (bits((unsigned)misc_reg, 11, 10) == 0x3) {
        warn("Ignoring write to read-only CSR.");
        return;
    }
    setMiscRegNoEffect(misc_reg, val);
}

}

RiscvISA::ISA *
RiscvISAParams::create()
{
    return new RiscvISA::ISA(this);
}
