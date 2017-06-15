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
#include <sstream>

#include "arch/riscv/registers.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "debug/RiscvMisc.hh"
#include "params/RiscvISA.hh"
#include "sim/core.hh"
#include "sim/pseudo_inst.hh"

namespace RiscvISA
{

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

    miscRegFile[MISCREG_MVENDORID] = 0;
    miscRegFile[MISCREG_MARCHID] = 0;
    miscRegFile[MISCREG_MIMPID] = 0;
    miscRegFile[MISCREG_MISA] = 0x8000000000101129ULL;
}


MiscReg
ISA::readMiscRegNoEffect(int misc_reg) const
{
    DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
        MiscRegNames.at(misc_reg), miscRegFile[misc_reg]);
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
      case MISCREG_MHARTID:
        warn("Use readMiscReg to read the mhartid CSR.");
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
            MiscRegNames.at(misc_reg), miscRegFile[misc_reg]);
        return tc->getCpuPtr()->totalInsts();
      case MISCREG_CYCLE:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            MiscRegNames.at(misc_reg), miscRegFile[misc_reg]);
        return tc->getCpuPtr()->curCycle();
      case MISCREG_INSTRETH:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            MiscRegNames.at(misc_reg), miscRegFile[misc_reg]);
        return tc->getCpuPtr()->totalInsts() >> 32;
      case MISCREG_CYCLEH:
        DPRINTF(RiscvMisc, "Reading CSR %s (0x%016llx).\n",
            MiscRegNames.at(misc_reg), miscRegFile[misc_reg]);
        return tc->getCpuPtr()->curCycle() >> 32;
      case MISCREG_MHARTID:
        return 0; // TODO: make this the hardware thread or cpu id
      default:
        return readMiscRegNoEffect(misc_reg);
    }
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    DPRINTF(RiscvMisc, "Setting CSR %s to 0x%016llx.\n",
        MiscRegNames.at(misc_reg), val);
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
