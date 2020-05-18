/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
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

#include "arch/riscv/interrupts.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/registers.hh"
#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/RiscvMisc.hh"
#include "params/RiscvISA.hh"
#include "sim/core.hh"
#include "sim/pseudo_inst.hh"

namespace RiscvISA
{

const std::array<const char *, NumMiscRegs> M5_VAR_USED MiscRegNames = {{
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
    // pmpcfg1 rv32 only
    [MISCREG_PMPCFG2]       = "PMPCFG2",
    // pmpcfg3 rv32 only
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
}};

ISA::ISA(Params *p) : BaseISA(p)
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

    miscRegFile[MISCREG_PRV] = PRV_M;
    miscRegFile[MISCREG_ISA] = (2ULL << MXL_OFFSET) | 0x14112D;
    miscRegFile[MISCREG_VENDORID] = 0;
    miscRegFile[MISCREG_ARCHID] = 0;
    miscRegFile[MISCREG_IMPID] = 0;
    miscRegFile[MISCREG_STATUS] = (2ULL << UXL_OFFSET) | (2ULL << SXL_OFFSET) |
                                  (1ULL << FS_OFFSET);
    miscRegFile[MISCREG_MCOUNTEREN] = 0x7;
    miscRegFile[MISCREG_SCOUNTEREN] = 0x7;
    // don't set it to zero; software may try to determine the supported
    // triggers, starting at zero. simply set a different value here.
    miscRegFile[MISCREG_TSELECT] = 1;
}

bool
ISA::hpmCounterEnabled(int misc_reg) const
{
    int hpmcounter = misc_reg - MISCREG_CYCLE;
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
ISA::readMiscRegNoEffect(int misc_reg) const
{
    if (misc_reg > NumMiscRegs || misc_reg < 0) {
        // Illegal CSR
        panic("Illegal CSR index %#x\n", misc_reg);
        return -1;
    }
    DPRINTF(RiscvMisc, "Reading MiscReg %s (%d): %#x.\n",
            MiscRegNames[misc_reg], misc_reg, miscRegFile[misc_reg]);
    return miscRegFile[misc_reg];
}

RegVal
ISA::readMiscReg(int misc_reg)
{
    switch (misc_reg) {
      case MISCREG_HARTID:
        return tc->contextId();
      case MISCREG_CYCLE:
        if (hpmCounterEnabled(MISCREG_CYCLE)) {
            DPRINTF(RiscvMisc, "Cycle counter at: %llu.\n",
                    tc->getCpuPtr()->curCycle());
            return tc->getCpuPtr()->curCycle();
        } else {
            warn("Cycle counter disabled.\n");
            return 0;
        }
      case MISCREG_TIME:
        if (hpmCounterEnabled(MISCREG_TIME)) {
            DPRINTF(RiscvMisc, "Wall-clock counter at: %llu.\n",
                    std::time(nullptr));
            return std::time(nullptr);
        } else {
            warn("Wall clock disabled.\n");
            return 0;
        }
      case MISCREG_INSTRET:
        if (hpmCounterEnabled(MISCREG_INSTRET)) {
            DPRINTF(RiscvMisc, "Instruction counter at: %llu.\n",
                    tc->getCpuPtr()->totalInsts());
            return tc->getCpuPtr()->totalInsts();
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
            auto misa = readMiscRegNoEffect(MISCREG_ISA);
            auto val = readMiscRegNoEffect(misc_reg);
            // if compressed instructions are disabled, epc[1] is set to 0
            if ((misa & ISA_EXT_C_MASK) == 0)
                return mbits(val, 63, 2);
            // epc[0] is always 0
            else
                return mbits(val, 63, 1);
        }
      default:
        // Try reading HPM counters
        // As a placeholder, all HPM counters are just cycle counters
        if (misc_reg >= MISCREG_HPMCOUNTER03 &&
                misc_reg <= MISCREG_HPMCOUNTER31) {
            if (hpmCounterEnabled(misc_reg)) {
                DPRINTF(RiscvMisc, "HPM counter %d: %llu.\n",
                        misc_reg - MISCREG_CYCLE, tc->getCpuPtr()->curCycle());
                return tc->getCpuPtr()->curCycle();
            } else {
                warn("HPM counter %d disabled.\n", misc_reg - MISCREG_CYCLE);
                return 0;
            }
        }
        return readMiscRegNoEffect(misc_reg);
    }
}

void
ISA::setMiscRegNoEffect(int misc_reg, RegVal val)
{
    if (misc_reg > NumMiscRegs || misc_reg < 0) {
        // Illegal CSR
        panic("Illegal CSR index %#x\n", misc_reg);
    }
    DPRINTF(RiscvMisc, "Setting MiscReg %s (%d) to %#x.\n",
            MiscRegNames[misc_reg], misc_reg, val);
    miscRegFile[misc_reg] = val;
}

void
ISA::setMiscReg(int misc_reg, RegVal val)
{
    if (misc_reg >= MISCREG_CYCLE && misc_reg <= MISCREG_HPMCOUNTER31) {
        // Ignore writes to HPM counters for now
        warn("Ignoring write to %s.\n", CSRData.at(misc_reg).name);
    } else {
        switch (misc_reg) {
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
                SATP cur_val = readMiscRegNoEffect(misc_reg);
                SATP new_val = val;
                if (new_val.mode != AddrXlateMode::BARE &&
                    new_val.mode != AddrXlateMode::SV39)
                    new_val.mode = cur_val.mode;
                setMiscRegNoEffect(misc_reg, new_val);
            }
            break;
          case MISCREG_TSELECT:
            {
                // we don't support debugging, so always set a different value
                // than written
                setMiscRegNoEffect(misc_reg, val + 1);
            }
            break;
          case MISCREG_ISA:
            {
                auto cur_val = readMiscRegNoEffect(misc_reg);
                // only allow to disable compressed instructions
                // if the following instruction is 4-byte aligned
                if ((val & ISA_EXT_C_MASK) == 0 &&
                    bits(tc->pcState().npc(), 2, 0) != 0)
                    val |= cur_val & ISA_EXT_C_MASK;
                setMiscRegNoEffect(misc_reg, val);
            }
            break;
          case MISCREG_STATUS:
            {
                // SXL and UXL are hard-wired to 64 bit
                auto cur = readMiscRegNoEffect(misc_reg);
                val &= ~(STATUS_SXL_MASK | STATUS_UXL_MASK);
                val |= cur & (STATUS_SXL_MASK | STATUS_UXL_MASK);
                setMiscRegNoEffect(misc_reg, val);
            }
            break;
          default:
            setMiscRegNoEffect(misc_reg, val);
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

}

RiscvISA::ISA *
RiscvISAParams::create()
{
    return new RiscvISA::ISA(this);
}
