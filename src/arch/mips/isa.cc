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
 */

#include "arch/mips/isa.hh"

#include "arch/mips/mt.hh"
#include "arch/mips/mt_constants.hh"
#include "arch/mips/pra_constants.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/MipsPRA.hh"
#include "params/MipsISA.hh"

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

ISA::ISA(Params *p) : BaseISA(p), numThreads(p->num_threads),
    numVpes(p->num_vpes)
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

    // Initialize all Per-VPE regs
    uint32_t per_vpe_regs[] = { MISCREG_VPE_CONTROL,
                                MISCREG_VPE_CONF0, MISCREG_VPE_CONF1,
                                MISCREG_YQMASK,
                                MISCREG_VPE_SCHEDULE, MISCREG_VPE_SCHEFBACK,
                                MISCREG_VPE_OPT, MISCREG_SRS_CONF0,
                                MISCREG_SRS_CONF1, MISCREG_SRS_CONF2,
                                MISCREG_SRS_CONF3, MISCREG_SRS_CONF4,
                                MISCREG_EBASE
                              };
    uint32_t num_vpe_regs = sizeof(per_vpe_regs) / 4;
    for (int i = 0; i < num_vpe_regs; i++) {
        if (numVpes > 1) {
            miscRegFile[per_vpe_regs[i]].resize(numVpes);
        }
        bankType[per_vpe_regs[i]] = perVirtProcessor;
    }

    // Initialize all Per-TC regs
    uint32_t per_tc_regs[] = { MISCREG_STATUS,
                               MISCREG_TC_STATUS, MISCREG_TC_BIND,
                               MISCREG_TC_RESTART, MISCREG_TC_HALT,
                               MISCREG_TC_CONTEXT, MISCREG_TC_SCHEDULE,
                               MISCREG_TC_SCHEFBACK,
                               MISCREG_DEBUG, MISCREG_LLADDR
                             };
    uint32_t num_tc_regs = sizeof(per_tc_regs) /  4;

    for (int i = 0; i < num_tc_regs; i++) {
        miscRegFile[per_tc_regs[i]].resize(numThreads);
        bankType[per_tc_regs[i]] = perThreadContext;
    }

    clear();
}

const MipsISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void
ISA::clear()
{
    for (int i = 0; i < NumMiscRegs; i++) {
        for (int j = 0; j < miscRegFile[i].size(); j++)
            miscRegFile[i][j] = 0;

        for (int k = 0; k < miscRegFile_WriteMask[i].size(); k++)
            miscRegFile_WriteMask[i][k] = (long unsigned int)(-1);
    }
}


void
ISA::configCP()
{
    DPRINTF(MipsPRA, "Resetting CP0 State with %i TCs and %i VPEs\n",
            numThreads, numVpes);

    CoreSpecific cp;
    panic("CP state must be set before the following code is used");

    // Do Default CP0 initialization HERE

    // Do Initialization for MT cores here (eventually use
    // core_name parameter to toggle this initialization)
    // ===================================================
    DPRINTF(MipsPRA, "Initializing CP0 State.... ");

    PRIdReg procId = readMiscRegNoEffect(MISCREG_PRID);
    procId.coOp = cp.CP0_PRId_CompanyOptions;
    procId.coId = cp.CP0_PRId_CompanyID;
    procId.procId = cp.CP0_PRId_ProcessorID;
    procId.rev = cp.CP0_PRId_Revision;
    setMiscRegNoEffect(MISCREG_PRID, procId);

    // Now, create Write Mask for ProcID register
    RegVal procIDMask = 0; // Read-Only register
    replaceBits(procIDMask, 32, 0, 0);
    setRegMask(MISCREG_PRID, procIDMask);

    // Config
    ConfigReg cfg = readMiscRegNoEffect(MISCREG_CONFIG);
    cfg.be = cp.CP0_Config_BE;
    cfg.at = cp.CP0_Config_AT;
    cfg.ar = cp.CP0_Config_AR;
    cfg.mt = cp.CP0_Config_MT;
    cfg.vi = cp.CP0_Config_VI;
    cfg.m = 1;
    setMiscRegNoEffect(MISCREG_CONFIG, cfg);
    // Now, create Write Mask for Config register
    RegVal cfg_Mask = 0x7FFF0007;
    replaceBits(cfg_Mask, 32, 0, 0);
    setRegMask(MISCREG_CONFIG, cfg_Mask);

    // Config1
    Config1Reg cfg1 = readMiscRegNoEffect(MISCREG_CONFIG1);
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
    setMiscRegNoEffect(MISCREG_CONFIG1, cfg1);
    // Now, create Write Mask for Config register
    RegVal cfg1_Mask = 0; // Read Only Register
    replaceBits(cfg1_Mask, 32,0 , 0);
    setRegMask(MISCREG_CONFIG1, cfg1_Mask);

    // Config2
    Config2Reg cfg2 = readMiscRegNoEffect(MISCREG_CONFIG2);
    cfg2.tu = cp.CP0_Config2_TU;
    cfg2.ts = cp.CP0_Config2_TS;
    cfg2.tl = cp.CP0_Config2_TL;
    cfg2.ta = cp.CP0_Config2_TA;
    cfg2.su = cp.CP0_Config2_SU;
    cfg2.ss = cp.CP0_Config2_SS;
    cfg2.sl = cp.CP0_Config2_SL;
    cfg2.sa = cp.CP0_Config2_SA;
    cfg2.m = cp.CP0_Config2_M;
    setMiscRegNoEffect(MISCREG_CONFIG2, cfg2);
    // Now, create Write Mask for Config register
    RegVal cfg2_Mask = 0x7000F000; // Read Only Register
    replaceBits(cfg2_Mask, 32, 0, 0);
    setRegMask(MISCREG_CONFIG2, cfg2_Mask);

    // Config3
    Config3Reg cfg3 = readMiscRegNoEffect(MISCREG_CONFIG3);
    cfg3.dspp = cp.CP0_Config3_DSPP;
    cfg3.lpa = cp.CP0_Config3_LPA;
    cfg3.veic = cp.CP0_Config3_VEIC;
    cfg3.vint = cp.CP0_Config3_VInt;
    cfg3.sp = cp.CP0_Config3_SP;
    cfg3.mt = cp.CP0_Config3_MT;
    cfg3.sm = cp.CP0_Config3_SM;
    cfg3.tl = cp.CP0_Config3_TL;
    setMiscRegNoEffect(MISCREG_CONFIG3, cfg3);
    // Now, create Write Mask for Config register
    RegVal cfg3_Mask = 0; // Read Only Register
    replaceBits(cfg3_Mask, 32,0 , 0);
    setRegMask(MISCREG_CONFIG3, cfg3_Mask);

    // EBase - CPUNum
    EBaseReg eBase = readMiscRegNoEffect(MISCREG_EBASE);
    eBase.cpuNum = cp.CP0_EBase_CPUNum;
    replaceBits(eBase, 31, 31, 1);
    setMiscRegNoEffect(MISCREG_EBASE, eBase);
    // Now, create Write Mask for Config register
    RegVal EB_Mask = 0x3FFFF000;// Except Exception Base, the
                                 // entire register is read only
    replaceBits(EB_Mask, 32, 0, 0);
    setRegMask(MISCREG_EBASE, EB_Mask);

    // SRS Control - HSS (Highest Shadow Set)
    SRSCtlReg scsCtl = readMiscRegNoEffect(MISCREG_SRSCTL);
    scsCtl.hss = cp.CP0_SrsCtl_HSS;
    setMiscRegNoEffect(MISCREG_SRSCTL, scsCtl);
    // Now, create Write Mask for the SRS Ctl register
    RegVal SC_Mask = 0x0000F3C0;
    replaceBits(SC_Mask, 32, 0, 0);
    setRegMask(MISCREG_SRSCTL, SC_Mask);

    // IntCtl - IPTI, IPPCI
    IntCtlReg intCtl = readMiscRegNoEffect(MISCREG_INTCTL);
    intCtl.ipti = cp.CP0_IntCtl_IPTI;
    intCtl.ippci = cp.CP0_IntCtl_IPPCI;
    setMiscRegNoEffect(MISCREG_INTCTL, intCtl);
    // Now, create Write Mask for the IntCtl register
    RegVal IC_Mask = 0x000003E0;
    replaceBits(IC_Mask, 32, 0, 0);
    setRegMask(MISCREG_INTCTL, IC_Mask);

    // Watch Hi - M - FIXME (More than 1 Watch register)
    WatchHiReg watchHi = readMiscRegNoEffect(MISCREG_WATCHHI0);
    watchHi.m = cp.CP0_WatchHi_M;
    setMiscRegNoEffect(MISCREG_WATCHHI0, watchHi);
    // Now, create Write Mask for the IntCtl register
    RegVal wh_Mask = 0x7FFF0FFF;
    replaceBits(wh_Mask, 32, 0, 0);
    setRegMask(MISCREG_WATCHHI0, wh_Mask);

    // Perf Ctr - M - FIXME (More than 1 PerfCnt Pair)
    PerfCntCtlReg perfCntCtl = readMiscRegNoEffect(MISCREG_PERFCNT0);
    perfCntCtl.m = cp.CP0_PerfCtr_M;
    perfCntCtl.w = cp.CP0_PerfCtr_W;
    setMiscRegNoEffect(MISCREG_PERFCNT0, perfCntCtl);
    // Now, create Write Mask for the IntCtl register
    RegVal pc_Mask = 0x00007FF;
    replaceBits(pc_Mask, 32, 0, 0);
    setRegMask(MISCREG_PERFCNT0, pc_Mask);

    // Random
    setMiscRegNoEffect(MISCREG_CP0_RANDOM, 63);
    // Now, create Write Mask for the IntCtl register
    RegVal random_Mask = 0;
    replaceBits(random_Mask, 32, 0, 0);
    setRegMask(MISCREG_CP0_RANDOM, random_Mask);

    // PageGrain
    PageGrainReg pageGrain = readMiscRegNoEffect(MISCREG_PAGEGRAIN);
    pageGrain.esp = cp.CP0_Config3_SP;
    setMiscRegNoEffect(MISCREG_PAGEGRAIN, pageGrain);
    // Now, create Write Mask for the IntCtl register
    RegVal pg_Mask = 0x10000000;
    replaceBits(pg_Mask, 32, 0, 0);
    setRegMask(MISCREG_PAGEGRAIN, pg_Mask);

    // Status
    StatusReg status = readMiscRegNoEffect(MISCREG_STATUS);
    // Only CU0 and IE are modified on a reset - everything else needs
    // to be controlled on a per CPU model basis

    // Enable CP0 on reset
    // status.cu0 = 1;

    // Enable ERL bit on a reset
    status.erl = 1;
    // Enable BEV bit on a reset
    status.bev = 1;

    setMiscRegNoEffect(MISCREG_STATUS, status);
    // Now, create Write Mask for the Status register
    RegVal stat_Mask = 0xFF78FF17;
    replaceBits(stat_Mask, 32, 0, 0);
    setRegMask(MISCREG_STATUS, stat_Mask);


    // MVPConf0
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(MISCREG_MVP_CONF0);
    mvpConf0.tca = 1;
    mvpConf0.pvpe = numVpes - 1;
    mvpConf0.ptc = numThreads - 1;
    setMiscRegNoEffect(MISCREG_MVP_CONF0, mvpConf0);

    // VPEConf0
    VPEConf0Reg vpeConf0 = readMiscRegNoEffect(MISCREG_VPE_CONF0);
    vpeConf0.mvp = 1;
    setMiscRegNoEffect(MISCREG_VPE_CONF0, vpeConf0);

    // TCBind
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        TCBindReg tcBind = readMiscRegNoEffect(MISCREG_TC_BIND, tid);
        tcBind.curTC = tid;
        setMiscRegNoEffect(MISCREG_TC_BIND, tcBind, tid);
    }
    // TCHalt
    TCHaltReg tcHalt = readMiscRegNoEffect(MISCREG_TC_HALT);
    tcHalt.h = 0;
    setMiscRegNoEffect(MISCREG_TC_HALT, tcHalt);

    // TCStatus
    // Set TCStatus Activated to 1 for the initial thread that is running
    TCStatusReg tcStatus = readMiscRegNoEffect(MISCREG_TC_STATUS);
    tcStatus.a = 1;
    setMiscRegNoEffect(MISCREG_TC_STATUS, tcStatus);

    // Set Dynamically Allocatable bit to 1 for all other threads
    for (ThreadID tid = 1; tid < numThreads; tid++) {
        tcStatus = readMiscRegNoEffect(MISCREG_TC_STATUS, tid);
        tcStatus.da = 1;
        setMiscRegNoEffect(MISCREG_TC_STATUS, tcStatus, tid);
    }


    RegVal mask = 0x7FFFFFFF;

    // Now, create Write Mask for the Index register
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_INDEX, mask);

    mask = 0x3FFFFFFF;
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_ENTRYLO0, mask);
    setRegMask(MISCREG_ENTRYLO1, mask);

    mask = 0xFF800000;
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_CONTEXT, mask);

    mask = 0x1FFFF800;
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_PAGEMASK, mask);

    mask = 0x0;
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_BADVADDR, mask);
    setRegMask(MISCREG_LLADDR, mask);

    mask = 0x08C00300;
    replaceBits(mask, 32, 0, 0);
    setRegMask(MISCREG_CAUSE, mask);

}

inline unsigned
ISA::getVPENum(ThreadID tid) const
{
    TCBindReg tcBind = miscRegFile[MISCREG_TC_BIND][tid];
    return tcBind.curVPE;
}

RegVal
ISA::readMiscRegNoEffect(int misc_reg, ThreadID tid) const
{
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
RegVal
ISA::readMiscReg(int misc_reg, ThreadID tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "Reading CP0 Register:%u Select:%u (%s) with effect (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);

    return miscRegFile[misc_reg][reg_sel];
}

void
ISA::setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i] Setting (direct set) CP0 Register:%u "
            "Select:%u (%s) to %#x.\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);

    miscRegFile[misc_reg][reg_sel] = val;
}

void
ISA::setRegMask(int misc_reg, RegVal val, ThreadID tid)
{
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i] Setting CP0 Register: %u Select: %u (%s) to %#x\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);
    miscRegFile_WriteMask[misc_reg][reg_sel] = val;
}

// PROGRAMMER'S NOTES:
// (1) Some CP0 Registers have fields that cannot
// be overwritten. Make sure to handle those particular registers
// with care!
void
ISA::setMiscReg(int misc_reg, RegVal val, ThreadID tid)
{
    int reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);

    DPRINTF(MipsPRA,
            "[tid:%i] Setting CP0 Register:%u "
            "Select:%u (%s) to %#x, with effect.\n",
            tid, misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);

    RegVal cp0_val = filterCP0Write(misc_reg, reg_sel, val);

    miscRegFile[misc_reg][reg_sel] = cp0_val;

    scheduleCP0Update(tc->getCpuPtr(), Cycles(1));
}

/**
 * This method doesn't need to adjust the Control Register Offset
 * since it has already been done in the calling method
 * (setRegWithEffect)
*/
RegVal
ISA::filterCP0Write(int misc_reg, int reg_sel, RegVal val)
{
    RegVal retVal = val;

    // Mask off read-only regions
    retVal &= miscRegFile_WriteMask[misc_reg][reg_sel];
    RegVal curVal = miscRegFile[misc_reg][reg_sel];
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
ISA::scheduleCP0Update(BaseCPU *cpu, Cycles delay)
{
    if (!cp0Updated) {
        cp0Updated = true;

        //schedule UPDATE
        auto cp0_event = new EventFunctionWrapper(
            [this, cpu]{ processCP0Event(cpu, UpdateCP0); },
            "Coprocessor-0 event", true, Event::CPU_Tick_Pri);
        cpu->schedule(cp0_event, cpu->clockEdge(delay));
    }
}

void
ISA::updateCPU(BaseCPU *cpu)
{
    ///////////////////////////////////////////////////////////////////
    //
    // EVALUATE CP0 STATE FOR MIPS MT
    //
    ///////////////////////////////////////////////////////////////////
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(MISCREG_MVP_CONF0);
    ThreadID num_threads = mvpConf0.ptc + 1;

    for (ThreadID tid = 0; tid < num_threads; tid++) {
        TCStatusReg tcStatus = readMiscRegNoEffect(MISCREG_TC_STATUS, tid);
        TCHaltReg tcHalt = readMiscRegNoEffect(MISCREG_TC_HALT, tid);

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

void
ISA::processCP0Event(BaseCPU *cpu, CP0EventType cp0EventType)
{
    switch (cp0EventType)
    {
      case UpdateCP0:
        updateCPU(cpu);
        break;
    }
}

}

MipsISA::ISA *
MipsISAParams::create()
{
    return new MipsISA::ISA(this);
}
