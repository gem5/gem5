/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#include "base/bitfield.hh"

#include "arch/mips/regfile/misc_regfile.hh"
#include "arch/mips/mt_constants.hh"
#include "arch/mips/faults.hh"

#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
//#include "cpu/mixie/cpu.hh"

using namespace std;

std::string MiscRegFile::miscRegNames[NumMiscRegs] =
{"Index", "MVPControl", "MVPConf0", "MVPConf1", "", "", "", "",
 "Random", "VPEControl", "VPEConf0", "VPEConf1", "YQMask", "VPESchedule", "VPEScheFBack", "VPEOpt",
 "EntryLo0", "TCStatus", "TCBind", "TCRestart", "TCHalt", "TCContext", "TCSchedule", "TCScheFBack",
 "EntryLo1", "", "", "", "", "", "", "",
 "Context", "ContextConfig", "", "", "", "", "", "",
 "PageMask", "PageGrain", "", "", "", "", "", "",
 "Wired", "SRSConf0", "SRCConf1", "SRSConf2", "SRSConf3", "SRSConf4", "", "",
 "HWREna", "", "", "", "", "", "", "",
 "BadVAddr", "", "", "", "", "", "", "",
 "Count", "", "", "", "", "", "", "",
 "EntryHi", "", "", "", "", "", "", "",
 "Compare", "", "", "", "", "", "", "",
 "Status", "IntCtl", "SRSCtl", "SRSMap", "", "", "", "",
 "Cause", "", "", "", "", "", "", "",
 "EPC", "", "", "", "", "", "", "",
 "PRId", "EBase", "", "", "", "", "", "",
 "Config", "Config1", "Config2", "Config3", "", "", "", "",
 "LLAddr", "", "", "", "", "", "", "",
 "WatchLo0", "WatchLo1", "WatchLo2", "WatchLo3", "WatchLo4", "WatchLo5", "WatchLo6", "WatchLo7",
 "WatchHi0", "WatchHi1", "WatchHi2", "WatchHi3", "WatchHi4", "WatchHi5", "WatchHi6", "WatchHi7",
 "XCContext64", "", "", "", "", "", "", "",
 "", "", "", "", "", "", "", "",
 "", "", "", "", "", "", "", "",
 "Debug", "TraceControl1", "TraceControl2", "UserTraceData", "TraceBPC", "", "", "",
 "DEPC", "", "", "", "", "", "", "",
 "PerfCnt0", "PerfCnt1", "PerfCnt2", "PerfCnt3", "PerfCnt4", "PerfCnt5", "PerfCnt6", "PerfCnt7",
 "ErrCtl", "", "", "", "", "", "", "",
 "CacheErr0", "CacheErr1", "CacheErr2", "CacheErr3", "", "", "", "",
 "TagLo0", "DataLo1", "TagLo2", "DataLo3", "TagLo4", "DataLo5", "TagLo6", "DataLo7",
 "TagHi0", "DataHi1", "TagHi2", "DataHi3", "TagHi4", "DataHi5", "TagHi6", "DataHi7",
 "ErrorEPC", "", "", "", "", "", "", "",
 "DESAVE", "", "", "", "", "", "", "",
 "LLFlag"
};

MiscRegFile::MiscRegFile()
{
    init();
}

MiscRegFile::MiscRegFile(BaseCPU *_cpu)
{
    cpu = _cpu;
    init();
}

void
MiscRegFile::init()
{
    miscRegFile.resize(NumMiscRegs);
    bankType.resize(NumMiscRegs);

    for (int i=0; i < NumMiscRegs; i++) {
        miscRegFile[i].resize(1);
        bankType[i] = perProcessor;
    }

    clear(0);
}

void
MiscRegFile::clear(unsigned tid_or_vpn)
{
    for(int i = 0; i < NumMiscRegs; i++) {
        miscRegFile[i][tid_or_vpn] = 0;
    }
}

void
MiscRegFile::expandForMultithreading(unsigned num_threads, unsigned num_vpes)
{
    // Initialize all Per-VPE regs
    uint32_t per_vpe_regs[] = { VPEControl, VPEConf0, VPEConf1, YQMask,
                                VPESchedule, VPEScheFBack, VPEOpt, SRSConf0,
                                SRSConf1, SRSConf2, SRSConf3, SRSConf4,
                                EBase
                              };
    uint32_t num_vpe_regs = sizeof(per_vpe_regs) / 4;
    for (int i = 0; i < num_vpe_regs; i++) {
        if (num_vpes > 1) {
            miscRegFile[per_vpe_regs[i]].resize(num_vpes);
        }
        bankType[per_vpe_regs[i]] = perVirtProcessor;
    }

    // Initialize all Per-TC regs
    uint32_t per_tc_regs[] = { Status, TCStatus, TCBind, TCRestart, TCHalt,
                               TCContext, TCSchedule, TCScheFBack, Debug,
                               LLAddr
                             };
    uint32_t num_tc_regs = sizeof(per_tc_regs) /  4;

    for (int i = 0; i < num_tc_regs; i++) {
        miscRegFile[per_tc_regs[i]].resize(num_threads);
        bankType[per_tc_regs[i]] = perThreadContext;
    }


    if (num_vpes > 1) {
        for (int i=1; i < num_vpes; i++) {
            clear(i);
        }
    }

}

//@TODO: Use MIPS STYLE CONSTANTS (e.g. TCHALT_H instead of TCH_H)
void
MiscRegFile::reset(std::string core_name, unsigned num_threads,
                   unsigned num_vpes)
{
    DPRINTF(MipsPRA, "Resetting CP0 State with %i TCs and %i VPEs\n",
            num_threads, num_vpes);

    // Do Default CP0 initialization HERE

    // Do Initialization for MT cores here (eventually use
    // core_name parameter to toggle this initialization)
    // ===================================================
    // Config
    MiscReg cfg = readRegNoEffect(Config);
    replaceBits(cfg, CFG_M, 1);
    setRegNoEffect(Config, cfg);

    // Config1
    MiscReg cfg1 = readRegNoEffect(Config1);
    replaceBits(cfg1, CFG1_M, 1);
    setRegNoEffect(Config1, cfg1);

    // Config2
    MiscReg cfg2 = readRegNoEffect(Config2);
    replaceBits(cfg2, CFG2_M, 1);
    setRegNoEffect(Config2, cfg2);

    // Config3
    MiscReg cfg3 = readRegNoEffect(Config3);
    replaceBits(cfg3, CFG3_MT, 1);
    setRegNoEffect(Config3, cfg3);

    // MVPConf0
    MiscReg mvp_conf0 = readRegNoEffect(MVPConf0);
    replaceBits(mvp_conf0, MVPC0_TCA, 1);
    replaceBits(mvp_conf0, MVPC0_PVPE_HI, MVPC0_PVPE_LO, num_vpes - 1);
    replaceBits(mvp_conf0, MVPC0_PTC_HI, MVPC0_PTC_LO, num_threads - 1);
    setRegNoEffect(MVPConf0, mvp_conf0);

    // VPEConf0
    MiscReg vpe_conf0 = readRegNoEffect(VPEConf0);
    replaceBits(vpe_conf0, VPEC0_MVP, 1);
    setRegNoEffect(VPEConf0, vpe_conf0);

    // TCBind
    for (int tid = 0; tid < num_threads; tid++) {
        MiscReg tc_bind = readRegNoEffect(TCBind, tid);
        replaceBits(tc_bind, TCB_CUR_TC_HI, TCB_CUR_TC_LO, tid);
        setRegNoEffect(TCBind, tc_bind, tid);
    }

    // TCHalt
    MiscReg tc_halt = readRegNoEffect(TCHalt);
    replaceBits(tc_halt, TCH_H, 0);
    setRegNoEffect(TCHalt, tc_halt);
    /*for (int tid = 1; tid < num_threads; tid++) {
        // Set TCHalt Halt bit to 1 for all other threads
        tc_halt = readRegNoEffect(TCHalt, tid);
        replaceBits(tc_halt, TCH_H, 1);
        setReg(TCHalt, tc_halt, tid);
        }*/

    // TCStatus
    // Set TCStatus Activated to 1 for the initial thread that is running
    MiscReg tc_status = readRegNoEffect(TCStatus);
    replaceBits(tc_status, TCS_A, 1);
    setRegNoEffect(TCStatus, tc_status);

    // Set Dynamically Allocatable bit to 1 for all other threads
    for (int tid = 0; tid < num_threads; tid++) {
        tc_status = readRegNoEffect(TCStatus, tid);
        replaceBits(tc_status, TCSTATUS_DA, 1);
        setRegNoEffect(TCStatus, tc_status, tid);
    }
}

inline std::string
MipsISA::getMiscRegName(unsigned reg_idx)
{
    return MiscRegFile::miscRegNames[reg_idx];
}

inline unsigned
MiscRegFile::getVPENum(unsigned tid)
{
    unsigned tc_bind = miscRegFile[TCBind][tid];
    return bits(tc_bind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO);
}

MiscReg
MiscRegFile::readRegNoEffect(int misc_reg, unsigned tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    return miscRegFile[misc_reg][reg_sel];
}

//@TODO: MIPS MT's register view automatically connects
//       Status to TCStatus depending on current thread
MiscReg
MiscRegFile::readReg(int misc_reg,
                               ThreadContext *tc,  unsigned tid)
{
    DPRINTF(MipsPRA, "Reading CP0 Register:%u Select:%u (%s) with effect.\n",
            misc_reg / 8, misc_reg % 8, getMiscRegName(misc_reg));

    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    switch (misc_reg)
    {
      default:
        return miscRegFile[misc_reg][reg_sel];
    }
}

void
MiscRegFile::setRegNoEffect(int misc_reg, const MiscReg &val, unsigned tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    miscRegFile[misc_reg][reg_sel] = val;
}

// PROGRAMMER'S NOTES:
// (1) Some CP0 Registers have fields that cannot
// be overwritten. Make sure to handle those particular registers
// with care!
void
MiscRegFile::setReg(int misc_reg, const MiscReg &val,
                              ThreadContext *tc, unsigned tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    DPRINTF(MipsPRA, "[tid:%i]: Setting CP0 Register:%u Select:%u (%s) to %#x, with effect.\n",
            tid, misc_reg / 8, misc_reg % 8, getMiscRegName(misc_reg), val);

    MiscReg cp0_val = filterCP0Write(misc_reg, val);

    miscRegFile[misc_reg][reg_sel] = cp0_val;

    scheduleCP0Update();
}

void
MiscRegFile::scheduleCP0Update(int delay)
{
    if (!cp0Updated) {
        cp0Updated = true;

        //schedule UPDATE
        CP0Event *cp0_event = new CP0Event(this, cpu, UpdateCP0);
        cp0_event->schedule(curTick + cpu->cycles(delay));
    }
}

void
MiscRegFile::updateCPU()
{
    ///////////////////////////////////////////////////////////////////
    //
    // EVALUATE CP0 STATE FOR MIPS MT
    //
    ///////////////////////////////////////////////////////////////////
    unsigned mvp_conf0 = readRegNoEffect(MVPConf0);
    unsigned num_threads = bits(mvp_conf0, MVPC0_PTC_HI, MVPC0_PTC_LO) + 1;

    for (int tid = 0; tid < num_threads; tid++) {
        MiscReg tc_status = readRegNoEffect(TCStatus, tid);
        MiscReg tc_halt = readRegNoEffect(TCHalt, tid);

        //@todo: add vpe/mt check here thru mvpcontrol & vpecontrol regs
        if (bits(tc_halt, TCH_H) == 1 || bits(tc_status, TCS_A) == 0)  {
            haltThread(cpu->getContext(tid));
        } else if (bits(tc_halt, TCH_H) == 0 && bits(tc_status, TCS_A) == 1) {
            restoreThread(cpu->getContext(tid));
        }
    }

    num_threads = bits(mvp_conf0, MVPC0_PTC_HI, MVPC0_PTC_LO) + 1;

    // Toggle update flag after we finished updating
    cp0Updated = false;
}

MiscRegFile::CP0Event::CP0Event(CP0 *_cp0, BaseCPU *_cpu, CP0EventType e_type)
    : Event(&mainEventQueue, CPU_Tick_Pri), cp0(_cp0), cpu(_cpu), cp0EventType(e_type)
{  }

void
MiscRegFile::CP0Event::process()
{
    switch (cp0EventType)
    {
      case UpdateCP0:
        cp0->updateCPU();
        break;
    }

    //cp0EventRemoveList.push(this);
}

const char *
MiscRegFile::CP0Event::description()
{
    return "Coprocessor-0";
}

void
MiscRegFile::CP0Event::scheduleEvent(int delay)
{
    if (squashed())
        reschedule(curTick + cpu->cycles(delay));
    else if (!scheduled())
        schedule(curTick + cpu->cycles(delay));
}

void
MiscRegFile::CP0Event::unscheduleEvent()
{
    if (scheduled())
        squash();
}
