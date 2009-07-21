/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "arch/mips/isa.hh"
#include "arch/mips/mt_constants.hh"
#include "arch/mips/mt.hh"
#include "arch/mips/pra_constants.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"

namespace MipsISA
{

std::string
ISA::miscRegNames[NumMiscRegs] =
{
    "Index", "MVPControl", "MVPConf0", "MVPConf1", "", "", "", "",
    "Random", "VPEControl", "VPEConf0", "VPEConf1",
        "YQMask", "VPESchedule", "VPEScheFBack", "VPEOpt",
    "EntryLo0", "TCStatus", "TCBind", "TCRestart",
        "TCHalt", "TCContext", "TCSchedule", "TCScheFBack",
    "EntryLo1", "", "", "", "", "", "", "",
    "Context", "ContextConfig", "", "", "", "", "", "",
    "PageMask", "PageGrain", "", "", "", "", "", "",
    "Wired", "SRSConf0", "SRCConf1", "SRSConf2",
        "SRSConf3", "SRSConf4", "", "",
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
    "WatchLo0", "WatchLo1", "WatchLo2", "WatchLo3",
        "WatchLo4", "WatchLo5", "WatchLo6", "WatchLo7",
    "WatchHi0", "WatchHi1", "WatchHi2", "WatchHi3",
        "WatchHi4", "WatchHi5", "WatchHi6", "WatchHi7",
    "XCContext64", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "",
    "Debug", "TraceControl1", "TraceControl2", "UserTraceData",
        "TraceBPC", "", "", "",
    "DEPC", "", "", "", "", "", "", "",
    "PerfCnt0", "PerfCnt1", "PerfCnt2", "PerfCnt3",
        "PerfCnt4", "PerfCnt5", "PerfCnt6", "PerfCnt7",
    "ErrCtl", "", "", "", "", "", "", "",
    "CacheErr0", "CacheErr1", "CacheErr2", "CacheErr3", "", "", "", "",
    "TagLo0", "DataLo1", "TagLo2", "DataLo3",
        "TagLo4", "DataLo5", "TagLo6", "DataLo7",
    "TagHi0", "DataHi1", "TagHi2", "DataHi3",
        "TagHi4", "DataHi5", "TagHi6", "DataHi7",
    "ErrorEPC", "", "", "", "", "", "", "",
    "DESAVE", "", "", "", "", "", "", "",
    "LLFlag"
};

ISA::ISA()
{
    init();
}

ISA::ISA(BaseCPU *_cpu)
{
    cpu = _cpu;
    init();
}

void
ISA::init()
{
    miscRegFile.resize(NumMiscRegs);
    bankType.resize(NumMiscRegs);

    for (int i=0; i < NumMiscRegs; i++) {
        miscRegFile[i].resize(1);
        bankType[i] = perProcessor;
    }

    miscRegFile_WriteMask.resize(NumMiscRegs);

    for (int i = 0; i < NumMiscRegs; i++) {
        miscRegFile_WriteMask[i].push_back(0);
    }
    clear(0);
}

void
ISA::clear(unsigned tid_or_vpn)
{
    for(int i = 0; i < NumMiscRegs; i++) {
        miscRegFile[i][tid_or_vpn] = 0;
        miscRegFile_WriteMask[i][tid_or_vpn] = (long unsigned int)(-1);
    }
}

void
ISA::expandForMultithreading(ThreadID num_threads, unsigned num_vpes)
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
ISA::reset(std::string core_name, ThreadID num_threads,
                   unsigned num_vpes, BaseCPU *_cpu)
{
    DPRINTF(MipsPRA, "Resetting CP0 State with %i TCs and %i VPEs\n",
            num_threads, num_vpes);
    cpu = _cpu;

    MipsISA::CoreSpecific &cp = cpu->coreParams;

    // Do Default CP0 initialization HERE

    // Do Initialization for MT cores here (eventually use
    // core_name parameter to toggle this initialization)
    // ===================================================
    DPRINTF(MipsPRA, "Initializing CP0 State.... ");

    PRIdReg procId = readMiscRegNoEffect(PRId);
    procId.coOp = cp.CP0_PRId_CompanyOptions;
    procId.coId = cp.CP0_PRId_CompanyID;
    procId.procId = cp.CP0_PRId_ProcessorID;
    procId.rev = cp.CP0_PRId_Revision;
    setMiscRegNoEffect(PRId, procId);

    // Now, create Write Mask for ProcID register
    MiscReg ProcID_Mask = 0; // Read-Only register
    replaceBits(ProcID_Mask, 0, 32, 0);
    setRegMask(PRId, ProcID_Mask);

    // Config
    ConfigReg cfg = readMiscRegNoEffect(Config);
    cfg.be = cp.CP0_Config_BE;
    cfg.at = cp.CP0_Config_AT;
    cfg.ar = cp.CP0_Config_AR;
    cfg.mt = cp.CP0_Config_MT;
    cfg.vi = cp.CP0_Config_VI;
    cfg.m = 1;
    setMiscRegNoEffect(Config, cfg);
    // Now, create Write Mask for Config register
    MiscReg cfg_Mask = 0x7FFF0007;
    replaceBits(cfg_Mask, 0, 32, 0);
    setRegMask(Config, cfg_Mask);

    // Config1
    Config1Reg cfg1 = readMiscRegNoEffect(Config1);
    cfg1.mmuSize = cp.CP0_Config1_MMU;
    cfg1.is = cp.CP0_Config1_IS;
    cfg1.il = cp.CP0_Config1_IL;
    cfg1.ia = cp.CP0_Config1_IA;
    cfg1.ds = cp.CP0_Config1_DS;
    cfg1.dl = cp.CP0_Config1_DL;
    cfg1.da = cp.CP0_Config1_DA;
    cfg1.fp = cp.CP0_Config1_FP;
    cfg1.ep = cp.CP0_Config1_EP;
    cfg1.wr = cp.CP0_Config1_WR;
    cfg1.md = cp.CP0_Config1_MD;
    cfg1.c2 = cp.CP0_Config1_C2;
    cfg1.pc = cp.CP0_Config1_PC;
    cfg1.m = cp.CP0_Config1_M;
    setMiscRegNoEffect(Config1, cfg1);
    // Now, create Write Mask for Config register
    MiscReg cfg1_Mask = 0; // Read Only Register
    replaceBits(cfg1_Mask, 0, 32, 0);
    setRegMask(Config1, cfg1_Mask);

    // Config2
    Config2Reg cfg2 = readMiscRegNoEffect(Config2);
    cfg2.tu = cp.CP0_Config2_TU;
    cfg2.ts = cp.CP0_Config2_TS;
    cfg2.tl = cp.CP0_Config2_TL;
    cfg2.ta = cp.CP0_Config2_TA;
    cfg2.su = cp.CP0_Config2_SU;
    cfg2.ss = cp.CP0_Config2_SS;
    cfg2.sl = cp.CP0_Config2_SL;
    cfg2.sa = cp.CP0_Config2_SA;
    cfg2.m = cp.CP0_Config2_M;
    setMiscRegNoEffect(Config2, cfg2);
    // Now, create Write Mask for Config register
    MiscReg cfg2_Mask = 0x7000F000; // Read Only Register
    replaceBits(cfg2_Mask, 0, 32, 0);
    setRegMask(Config2, cfg2_Mask);

    // Config3
    Config3Reg cfg3 = readMiscRegNoEffect(Config3);
    cfg3.dspp = cp.CP0_Config3_DSPP;
    cfg3.lpa = cp.CP0_Config3_LPA;
    cfg3.veic = cp.CP0_Config3_VEIC;
    cfg3.vint = cp.CP0_Config3_VInt;
    cfg3.sp = cp.CP0_Config3_SP;
    cfg3.mt = cp.CP0_Config3_MT;
    cfg3.sm = cp.CP0_Config3_SM;
    cfg3.tl = cp.CP0_Config3_TL;
    setMiscRegNoEffect(Config3, cfg3);
    // Now, create Write Mask for Config register
    MiscReg cfg3_Mask = 0; // Read Only Register
    replaceBits(cfg3_Mask, 0, 32, 0);
    setRegMask(Config3, cfg3_Mask);

    // EBase - CPUNum
    EBaseReg eBase = readMiscRegNoEffect(EBase);
    eBase.cpuNum = cp.CP0_EBase_CPUNum;
    replaceBits(eBase, 31, 31, 1);
    setMiscRegNoEffect(EBase, eBase);
    // Now, create Write Mask for Config register
    MiscReg EB_Mask = 0x3FFFF000;// Except Exception Base, the
                                 // entire register is read only
    replaceBits(EB_Mask, 0, 32, 0);
    setRegMask(EBase, EB_Mask);

    // SRS Control - HSS (Highest Shadow Set)
    SRSCtlReg scsCtl = readMiscRegNoEffect(SRSCtl);
    scsCtl.hss = cp.CP0_SrsCtl_HSS;
    setMiscRegNoEffect(SRSCtl, scsCtl);
    // Now, create Write Mask for the SRS Ctl register
    MiscReg SC_Mask = 0x0000F3C0;
    replaceBits(SC_Mask, 0, 32, 0);
    setRegMask(SRSCtl, SC_Mask);

    // IntCtl - IPTI, IPPCI
    IntCtlReg intCtl = readMiscRegNoEffect(IntCtl);
    intCtl.ipti = cp.CP0_IntCtl_IPTI;
    intCtl.ippci = cp.CP0_IntCtl_IPPCI;
    setMiscRegNoEffect(IntCtl, intCtl);
    // Now, create Write Mask for the IntCtl register
    MiscReg IC_Mask = 0x000003E0;
    replaceBits(IC_Mask, 0, 32, 0);
    setRegMask(IntCtl, IC_Mask);

    // Watch Hi - M - FIXME (More than 1 Watch register)
    WatchHiReg watchHi = readMiscRegNoEffect(WatchHi0);
    watchHi.m = cp.CP0_WatchHi_M;
    setMiscRegNoEffect(WatchHi0, watchHi);
    // Now, create Write Mask for the IntCtl register
    MiscReg wh_Mask = 0x7FFF0FFF;
    replaceBits(wh_Mask, 0, 32, 0);
    setRegMask(WatchHi0, wh_Mask);

    // Perf Ctr - M - FIXME (More than 1 PerfCnt Pair)
    PerfCntCtlReg perfCntCtl = readMiscRegNoEffect(PerfCnt0);
    perfCntCtl.m = cp.CP0_PerfCtr_M;
    perfCntCtl.w = cp.CP0_PerfCtr_W;
    setMiscRegNoEffect(PerfCnt0, perfCntCtl);
    // Now, create Write Mask for the IntCtl register
    MiscReg pc_Mask = 0x00007FF;
    replaceBits(pc_Mask, 0, 32, 0);
    setRegMask(PerfCnt0, pc_Mask);

    // Random
    setMiscRegNoEffect(CP0_Random, 63);
    // Now, create Write Mask for the IntCtl register
    MiscReg random_Mask = 0;
    replaceBits(random_Mask, 0, 32, 0);
    setRegMask(CP0_Random, random_Mask);

    // PageGrain
    PageGrainReg pageGrain = readMiscRegNoEffect(PageGrain);
    pageGrain.esp = cp.CP0_Config3_SP;
    setMiscRegNoEffect(PageGrain, pageGrain);
    // Now, create Write Mask for the IntCtl register
    MiscReg pg_Mask = 0x10000000;
    replaceBits(pg_Mask, 0, 32, 0);
    setRegMask(PageGrain, pg_Mask);

    // Status
    StatusReg status = readMiscRegNoEffect(Status);
    // Only CU0 and IE are modified on a reset - everything else needs
    // to be controlled on a per CPU model basis

    // Enable CP0 on reset
    // status.cu0 = 1;

    // Enable ERL bit on a reset
    status.erl = 1;
    // Enable BEV bit on a reset
    status.bev = 1;

    setMiscRegNoEffect(Status, status);
    // Now, create Write Mask for the Status register
    MiscReg stat_Mask = 0xFF78FF17;
    replaceBits(stat_Mask, 0, 32, 0);
    setRegMask(Status, stat_Mask);


    // MVPConf0
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(MVPConf0);
    mvpConf0.tca = 1;
    mvpConf0.pvpe = num_vpes - 1;
    mvpConf0.ptc = num_threads - 1;
    setMiscRegNoEffect(MVPConf0, mvpConf0);

    // VPEConf0
    VPEConf0Reg vpeConf0 = readMiscRegNoEffect(VPEConf0);
    vpeConf0.mvp = 1;
    setMiscRegNoEffect(VPEConf0, vpeConf0);

    // TCBind
    for (ThreadID tid = 0; tid < num_threads; tid++) {
        TCBindReg tcBind = readMiscRegNoEffect(TCBind, tid);
        tcBind.curTC = tid;
        setMiscRegNoEffect(TCBind, tcBind, tid);
    }
    // TCHalt
    TCHaltReg tcHalt = readMiscRegNoEffect(TCHalt);
    tcHalt.h = 0;
    setMiscRegNoEffect(TCHalt, tcHalt);

    // TCStatus
    // Set TCStatus Activated to 1 for the initial thread that is running
    TCStatusReg tcStatus = readMiscRegNoEffect(TCStatus);
    tcStatus.a = 1;
    setMiscRegNoEffect(TCStatus, tcStatus);

    // Set Dynamically Allocatable bit to 1 for all other threads
    for (ThreadID tid = 1; tid < num_threads; tid++) {
        tcStatus = readMiscRegNoEffect(TCStatus, tid);
        tcStatus.da = 1;
        setMiscRegNoEffect(TCStatus, tcStatus, tid);
    }


    MiscReg Mask = 0x7FFFFFFF;

    // Now, create Write Mask for the Index register
    replaceBits(Mask, 0, 32, 0);
    setRegMask(Index, Mask);

    Mask = 0x3FFFFFFF;
    replaceBits(Mask, 0, 32, 0);
    setRegMask(EntryLo0, Mask);
    setRegMask(EntryLo1, Mask);

    Mask = 0xFF800000;
    replaceBits(Mask, 0, 32, 0);
    setRegMask(Context, Mask);

    Mask = 0x1FFFF800;
    replaceBits(Mask, 0, 32, 0);
    setRegMask(PageMask, Mask);

    Mask = 0x0;
    replaceBits(Mask, 0, 32, 0);
    setRegMask(BadVAddr, Mask);
    setRegMask(LLAddr, Mask);

    Mask = 0x08C00300;
    replaceBits(Mask, 0, 32, 0);
    setRegMask(Cause, Mask);

}

inline unsigned
ISA::getVPENum(ThreadID tid)
{
    TCBindReg tcBind = miscRegFile[TCBind - Ctrl_Base_DepTag][tid];
    return tcBind.curVPE;
}

MiscReg
ISA::readMiscRegNoEffect(int reg_idx, ThreadID tid)
{
    int misc_reg = reg_idx - Ctrl_Base_DepTag;
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA, "Reading CP0 Register:%u Select:%u (%s) (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);
    return miscRegFile[misc_reg][reg_sel];
}

//@TODO: MIPS MT's register view automatically connects
//       Status to TCStatus depending on current thread
//template <class TC>
MiscReg
ISA::readMiscReg(int reg_idx, ThreadContext *tc,  ThreadID tid)
{
    int misc_reg = reg_idx - Ctrl_Base_DepTag;
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "Reading CP0 Register:%u Select:%u (%s) with effect (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);

    return miscRegFile[misc_reg][reg_sel];
}

void
ISA::setMiscRegNoEffect(int reg_idx, const MiscReg &val, ThreadID tid)
{
    int misc_reg = reg_idx - Ctrl_Base_DepTag;
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i]: Setting (direct set) CP0 Register:%u "
            "Select:%u (%s) to %#x.\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);

    miscRegFile[misc_reg][reg_sel] = val;
}

void
ISA::setRegMask(int reg_idx, const MiscReg &val, ThreadID tid)
{
  //  return;
  int misc_reg = reg_idx - Ctrl_Base_DepTag;
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i]: Setting CP0 Register: %u Select: %u (%s) to %#x\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);
    miscRegFile_WriteMask[misc_reg][reg_sel] = val;
}

// PROGRAMMER'S NOTES:
// (1) Some CP0 Registers have fields that cannot
// be overwritten. Make sure to handle those particular registers
// with care!
void
ISA::setMiscReg(int reg_idx, const MiscReg &val,
                    ThreadContext *tc, ThreadID tid)
{
    int misc_reg = reg_idx - Ctrl_Base_DepTag;
    int reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    DPRINTF(MipsPRA,
            "[tid:%i]: Setting CP0 Register:%u "
            "Select:%u (%s) to %#x, with effect.\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);

    MiscReg cp0_val = filterCP0Write(misc_reg, reg_sel, val);

    miscRegFile[misc_reg][reg_sel] = cp0_val;

    scheduleCP0Update(1);
}

/**
 * This method doesn't need to adjust the Control Register Offset
 * since it has already been done in the calling method
 * (setRegWithEffect)
*/
MiscReg
ISA::filterCP0Write(int misc_reg, int reg_sel, const MiscReg &val)
{
    MiscReg retVal = val;

    // Mask off read-only regions
    retVal &= miscRegFile_WriteMask[misc_reg][reg_sel];
    MiscReg curVal = miscRegFile[misc_reg][reg_sel];
    // Mask off current alue with inverse mask (clear writeable bits)
    curVal &= (~miscRegFile_WriteMask[misc_reg][reg_sel]);
    retVal |= curVal; // Combine the two
    DPRINTF(MipsPRA,
            "filterCP0Write: Mask: %lx, Inverse Mask: %lx, write Val: %x, "
            "current val: %lx, written val: %x\n",
            miscRegFile_WriteMask[misc_reg][reg_sel],
            ~miscRegFile_WriteMask[misc_reg][reg_sel],
            val, miscRegFile[misc_reg][reg_sel], retVal);
    return retVal;
}

void
ISA::scheduleCP0Update(int delay)
{
    if (!cp0Updated) {
        cp0Updated = true;

        //schedule UPDATE
        CP0Event *cp0_event = new CP0Event(this, cpu, UpdateCP0);
        cpu->schedule(cp0_event, curTick + cpu->ticks(delay));
    }
}

void
ISA::updateCPU()
{
    ///////////////////////////////////////////////////////////////////
    //
    // EVALUATE CP0 STATE FOR MIPS MT
    //
    ///////////////////////////////////////////////////////////////////
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(MVPConf0);
    ThreadID num_threads = mvpConf0.ptc + 1;

    for (ThreadID tid = 0; tid < num_threads; tid++) {
        TCStatusReg tcStatus = readMiscRegNoEffect(TCStatus, tid);
        TCHaltReg tcHalt = readMiscRegNoEffect(TCHalt, tid);

        //@todo: add vpe/mt check here thru mvpcontrol & vpecontrol regs
        if (tcHalt.h == 1 || tcStatus.a == 0)  {
            haltThread(cpu->getContext(tid));
        } else if (tcHalt.h == 0 && tcStatus.a == 1) {
            restoreThread(cpu->getContext(tid));
        }
    }

    num_threads = mvpConf0.ptc + 1;

    // Toggle update flag after we finished updating
    cp0Updated = false;
}

ISA::CP0Event::CP0Event(CP0 *_cp0, BaseCPU *_cpu, CP0EventType e_type)
    : Event(CPU_Tick_Pri), cp0(_cp0), cpu(_cpu), cp0EventType(e_type)
{  }

void
ISA::CP0Event::process()
{
    switch (cp0EventType)
    {
      case UpdateCP0:
        cp0->updateCPU();
        break;
    }
}

const char *
ISA::CP0Event::description() const
{
    return "Coprocessor-0 event";
}

void
ISA::CP0Event::scheduleEvent(int delay)
{
    cpu->reschedule(this, curTick + cpu->ticks(delay), true);
}

void
ISA::CP0Event::unscheduleEvent()
{
    if (scheduled())
        squash();
}

}
