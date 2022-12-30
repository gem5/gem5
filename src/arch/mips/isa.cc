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
#include "arch/mips/regs/float.hh"
#include "arch/mips/regs/int.hh"
#include "arch/mips/regs/misc.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "cpu/reg_class.hh"
#include "cpu/thread_context.hh"
#include "debug/MipsPRA.hh"
#include "params/MipsISA.hh"

namespace gem5
{

namespace MipsISA
{

std::string
ISA::miscRegNames[misc_reg::NumRegs] =
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

namespace
{

/* Not applicable to MIPS. */
constexpr RegClass vecRegClass(VecRegClass, VecRegClassName, 1,
        debug::IntRegs);
constexpr RegClass vecElemClass(VecElemClass, VecElemClassName, 2,
        debug::IntRegs);
constexpr RegClass vecPredRegClass(VecPredRegClass, VecPredRegClassName, 1,
        debug::IntRegs);
constexpr RegClass ccRegClass(CCRegClass, CCRegClassName, 0, debug::IntRegs);

} // anonymous namespace

ISA::ISA(const Params &p) : BaseISA(p), numThreads(p.num_threads),
    numVpes(p.num_vpes)
{
    _regClasses.push_back(&intRegClass);
    _regClasses.push_back(&floatRegClass);
    _regClasses.push_back(&vecRegClass);
    _regClasses.push_back(&vecElemClass);
    _regClasses.push_back(&vecPredRegClass);
    _regClasses.push_back(&ccRegClass);
    _regClasses.push_back(&miscRegClass);

    miscRegFile.resize(misc_reg::NumRegs);
    bankType.resize(misc_reg::NumRegs);

    for (int i = 0; i < misc_reg::NumRegs; i++) {
        miscRegFile[i].resize(1);
        bankType[i] = perProcessor;
    }

    miscRegFile_WriteMask.resize(misc_reg::NumRegs);

    for (int i = 0; i < misc_reg::NumRegs; i++) {
        miscRegFile_WriteMask[i].push_back(0);
    }

    // Initialize all Per-VPE regs
    uint32_t per_vpe_regs[] = { misc_reg::VpeControl,
                                misc_reg::VpeConf0, misc_reg::VpeConf1,
                                misc_reg::Yqmask,
                                misc_reg::VpeSchedule, misc_reg::VpeSchefback,
                                misc_reg::VpeOpt, misc_reg::SrsConf0,
                                misc_reg::SrsConf1, misc_reg::SrsConf2,
                                misc_reg::SrsConf3, misc_reg::SrsConf4,
                                misc_reg::Ebase
                              };
    uint32_t num_vpe_regs = sizeof(per_vpe_regs) / 4;
    for (int i = 0; i < num_vpe_regs; i++) {
        if (numVpes > 1) {
            miscRegFile[per_vpe_regs[i]].resize(numVpes);
        }
        bankType[per_vpe_regs[i]] = perVirtProcessor;
    }

    // Initialize all Per-TC regs
    uint32_t per_tc_regs[] = { misc_reg::Status,
                               misc_reg::TcStatus, misc_reg::TcBind,
                               misc_reg::TcRestart, misc_reg::TcHalt,
                               misc_reg::TcContext, misc_reg::TcSchedule,
                               misc_reg::TcSchefback,
                               misc_reg::Debug, misc_reg::Lladdr
                             };
    uint32_t num_tc_regs = sizeof(per_tc_regs) /  4;

    for (int i = 0; i < num_tc_regs; i++) {
        miscRegFile[per_tc_regs[i]].resize(numThreads);
        bankType[per_tc_regs[i]] = perThreadContext;
    }

    clear();
}

void
ISA::clear()
{
    for (int i = 0; i < misc_reg::NumRegs; i++) {
        for (int j = 0; j < miscRegFile[i].size(); j++)
            miscRegFile[i][j] = 0;

        for (int k = 0; k < miscRegFile_WriteMask[i].size(); k++)
            miscRegFile_WriteMask[i][k] = (long unsigned int)(-1);
    }
}

void
ISA::copyRegsFrom(ThreadContext *src)
{
    // First loop through the integer registers.
    for (auto &id: intRegClass)
        tc->setReg(id, src->getReg(id));

    // Then loop through the floating point registers.
    for (auto &id: floatRegClass)
        tc->setReg(id, src->getReg(id));

    // Copy misc. registers
    for (int i = 0; i < misc_reg::NumRegs; i++)
        tc->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    // Copy over the PC State
    tc->pcState(src->pcState());
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

    PRIdReg procId = readMiscRegNoEffect(misc_reg::Prid);
    procId.coOp = cp.CP0_PRId_CompanyOptions;
    procId.coId = cp.CP0_PRId_CompanyID;
    procId.procId = cp.CP0_PRId_ProcessorID;
    procId.rev = cp.CP0_PRId_Revision;
    setMiscRegNoEffect(misc_reg::Prid, procId);

    // Now, create Write Mask for ProcID register
    RegVal procIDMask = 0; // Read-Only register
    replaceBits(procIDMask, 32, 0, 0);
    setRegMask(misc_reg::Prid, procIDMask);

    // Config
    ConfigReg cfg = readMiscRegNoEffect(misc_reg::Config);
    cfg.be = cp.CP0_Config_BE;
    cfg.at = cp.CP0_Config_AT;
    cfg.ar = cp.CP0_Config_AR;
    cfg.mt = cp.CP0_Config_MT;
    cfg.vi = cp.CP0_Config_VI;
    cfg.m = 1;
    setMiscRegNoEffect(misc_reg::Config, cfg);
    // Now, create Write Mask for Config register
    RegVal cfg_Mask = 0x7FFF0007;
    replaceBits(cfg_Mask, 32, 0, 0);
    setRegMask(misc_reg::Config, cfg_Mask);

    // Config1
    Config1Reg cfg1 = readMiscRegNoEffect(misc_reg::Config1);
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
    setMiscRegNoEffect(misc_reg::Config1, cfg1);
    // Now, create Write Mask for Config register
    RegVal cfg1_Mask = 0; // Read Only Register
    replaceBits(cfg1_Mask, 32,0 , 0);
    setRegMask(misc_reg::Config1, cfg1_Mask);

    // Config2
    Config2Reg cfg2 = readMiscRegNoEffect(misc_reg::Config2);
    cfg2.tu = cp.CP0_Config2_TU;
    cfg2.ts = cp.CP0_Config2_TS;
    cfg2.tl = cp.CP0_Config2_TL;
    cfg2.ta = cp.CP0_Config2_TA;
    cfg2.su = cp.CP0_Config2_SU;
    cfg2.ss = cp.CP0_Config2_SS;
    cfg2.sl = cp.CP0_Config2_SL;
    cfg2.sa = cp.CP0_Config2_SA;
    cfg2.m = cp.CP0_Config2_M;
    setMiscRegNoEffect(misc_reg::Config2, cfg2);
    // Now, create Write Mask for Config register
    RegVal cfg2_Mask = 0x7000F000; // Read Only Register
    replaceBits(cfg2_Mask, 32, 0, 0);
    setRegMask(misc_reg::Config2, cfg2_Mask);

    // Config3
    Config3Reg cfg3 = readMiscRegNoEffect(misc_reg::Config3);
    cfg3.dspp = cp.CP0_Config3_DSPP;
    cfg3.lpa = cp.CP0_Config3_LPA;
    cfg3.veic = cp.CP0_Config3_VEIC;
    cfg3.vint = cp.CP0_Config3_VInt;
    cfg3.sp = cp.CP0_Config3_SP;
    cfg3.mt = cp.CP0_Config3_MT;
    cfg3.sm = cp.CP0_Config3_SM;
    cfg3.tl = cp.CP0_Config3_TL;
    setMiscRegNoEffect(misc_reg::Config3, cfg3);
    // Now, create Write Mask for Config register
    RegVal cfg3_Mask = 0; // Read Only Register
    replaceBits(cfg3_Mask, 32,0 , 0);
    setRegMask(misc_reg::Config3, cfg3_Mask);

    // EBase - CPUNum
    EBaseReg eBase = readMiscRegNoEffect(misc_reg::Ebase);
    eBase.cpuNum = cp.CP0_EBase_CPUNum;
    replaceBits(eBase, 31, 31, 1);
    setMiscRegNoEffect(misc_reg::Ebase, eBase);
    // Now, create Write Mask for Config register
    RegVal EB_Mask = 0x3FFFF000;// Except Exception Base, the
                                 // entire register is read only
    replaceBits(EB_Mask, 32, 0, 0);
    setRegMask(misc_reg::Ebase, EB_Mask);

    // SRS Control - HSS (Highest Shadow Set)
    SRSCtlReg scsCtl = readMiscRegNoEffect(misc_reg::Srsctl);
    scsCtl.hss = cp.CP0_SrsCtl_HSS;
    setMiscRegNoEffect(misc_reg::Srsctl, scsCtl);
    // Now, create Write Mask for the SRS Ctl register
    RegVal SC_Mask = 0x0000F3C0;
    replaceBits(SC_Mask, 32, 0, 0);
    setRegMask(misc_reg::Srsctl, SC_Mask);

    // IntCtl - IPTI, IPPCI
    IntCtlReg intCtl = readMiscRegNoEffect(misc_reg::Intctl);
    intCtl.ipti = cp.CP0_IntCtl_IPTI;
    intCtl.ippci = cp.CP0_IntCtl_IPPCI;
    setMiscRegNoEffect(misc_reg::Intctl, intCtl);
    // Now, create Write Mask for the IntCtl register
    RegVal IC_Mask = 0x000003E0;
    replaceBits(IC_Mask, 32, 0, 0);
    setRegMask(misc_reg::Intctl, IC_Mask);

    // Watch Hi - M - FIXME (More than 1 Watch register)
    WatchHiReg watchHi = readMiscRegNoEffect(misc_reg::Watchhi0);
    watchHi.m = cp.CP0_WatchHi_M;
    setMiscRegNoEffect(misc_reg::Watchhi0, watchHi);
    // Now, create Write Mask for the IntCtl register
    RegVal wh_Mask = 0x7FFF0FFF;
    replaceBits(wh_Mask, 32, 0, 0);
    setRegMask(misc_reg::Watchhi0, wh_Mask);

    // Perf Ctr - M - FIXME (More than 1 PerfCnt Pair)
    PerfCntCtlReg perfCntCtl = readMiscRegNoEffect(misc_reg::Perfcnt0);
    perfCntCtl.m = cp.CP0_PerfCtr_M;
    perfCntCtl.w = cp.CP0_PerfCtr_W;
    setMiscRegNoEffect(misc_reg::Perfcnt0, perfCntCtl);
    // Now, create Write Mask for the IntCtl register
    RegVal pc_Mask = 0x00007FF;
    replaceBits(pc_Mask, 32, 0, 0);
    setRegMask(misc_reg::Perfcnt0, pc_Mask);

    // Random
    setMiscRegNoEffect(misc_reg::Cp0Random, 63);
    // Now, create Write Mask for the IntCtl register
    RegVal random_Mask = 0;
    replaceBits(random_Mask, 32, 0, 0);
    setRegMask(misc_reg::Cp0Random, random_Mask);

    // PageGrain
    PageGrainReg pageGrain = readMiscRegNoEffect(misc_reg::Pagegrain);
    pageGrain.esp = cp.CP0_Config3_SP;
    setMiscRegNoEffect(misc_reg::Pagegrain, pageGrain);
    // Now, create Write Mask for the IntCtl register
    RegVal pg_Mask = 0x10000000;
    replaceBits(pg_Mask, 32, 0, 0);
    setRegMask(misc_reg::Pagegrain, pg_Mask);

    // Status
    StatusReg status = readMiscRegNoEffect(misc_reg::Status);
    // Only CU0 and IE are modified on a reset - everything else needs
    // to be controlled on a per CPU model basis

    // Enable CP0 on reset
    // status.cu0 = 1;

    // Enable ERL bit on a reset
    status.erl = 1;
    // Enable BEV bit on a reset
    status.bev = 1;

    setMiscRegNoEffect(misc_reg::Status, status);
    // Now, create Write Mask for the Status register
    RegVal stat_Mask = 0xFF78FF17;
    replaceBits(stat_Mask, 32, 0, 0);
    setRegMask(misc_reg::Status, stat_Mask);


    // MVPConf0
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(misc_reg::MvpConf0);
    mvpConf0.tca = 1;
    mvpConf0.pvpe = numVpes - 1;
    mvpConf0.ptc = numThreads - 1;
    setMiscRegNoEffect(misc_reg::MvpConf0, mvpConf0);

    // VPEConf0
    VPEConf0Reg vpeConf0 = readMiscRegNoEffect(misc_reg::VpeConf0);
    vpeConf0.mvp = 1;
    setMiscRegNoEffect(misc_reg::VpeConf0, vpeConf0);

    // TCBind
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        TCBindReg tcBind = readMiscRegNoEffect(misc_reg::TcBind, tid);
        tcBind.curTC = tid;
        setMiscRegNoEffect(misc_reg::TcBind, tcBind, tid);
    }
    // TCHalt
    TCHaltReg tcHalt = readMiscRegNoEffect(misc_reg::TcHalt);
    tcHalt.h = 0;
    setMiscRegNoEffect(misc_reg::TcHalt, tcHalt);

    // TCStatus
    // Set TCStatus Activated to 1 for the initial thread that is running
    TCStatusReg tcStatus = readMiscRegNoEffect(misc_reg::TcStatus);
    tcStatus.a = 1;
    setMiscRegNoEffect(misc_reg::TcStatus, tcStatus);

    // Set Dynamically Allocatable bit to 1 for all other threads
    for (ThreadID tid = 1; tid < numThreads; tid++) {
        tcStatus = readMiscRegNoEffect(misc_reg::TcStatus, tid);
        tcStatus.da = 1;
        setMiscRegNoEffect(misc_reg::TcStatus, tcStatus, tid);
    }


    RegVal mask = 0x7FFFFFFF;

    // Now, create Write Mask for the Index register
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Index, mask);

    mask = 0x3FFFFFFF;
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Entrylo0, mask);
    setRegMask(misc_reg::Entrylo1, mask);

    mask = 0xFF800000;
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Context, mask);

    mask = 0x1FFFF800;
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Pagemask, mask);

    mask = 0x0;
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Badvaddr, mask);
    setRegMask(misc_reg::Lladdr, mask);

    mask = 0x08C00300;
    replaceBits(mask, 32, 0, 0);
    setRegMask(misc_reg::Cause, mask);

}

inline unsigned
ISA::getVPENum(ThreadID tid) const
{
    TCBindReg tcBind = miscRegFile[misc_reg::TcBind][tid];
    return tcBind.curVPE;
}

RegVal
ISA::readMiscRegNoEffect(RegIndex idx, ThreadID tid) const
{
    unsigned reg_sel = (bankType[idx] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA, "Reading CP0 Register:%u Select:%u (%s) (%lx).\n",
            idx / 8, idx % 8, miscRegNames[idx], miscRegFile[idx][reg_sel]);
    return miscRegFile[idx][reg_sel];
}

//@TODO: MIPS MT's register view automatically connects
//       Status to TCStatus depending on current thread
//template <class TC>
RegVal
ISA::readMiscReg(RegIndex idx, ThreadID tid)
{
    unsigned reg_sel = (bankType[idx] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "Reading CP0 Register:%u Select:%u (%s) with effect (%lx).\n",
            idx / 8, idx % 8, miscRegNames[idx], miscRegFile[idx][reg_sel]);

    return miscRegFile[idx][reg_sel];
}

void
ISA::setMiscRegNoEffect(RegIndex idx, RegVal val, ThreadID tid)
{
    unsigned reg_sel = (bankType[idx] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i] Setting (direct set) CP0 Register:%u "
            "Select:%u (%s) to %#x.\n",
            tid, idx / 8, idx % 8, miscRegNames[idx], val);

    miscRegFile[idx][reg_sel] = val;
}

void
ISA::setRegMask(RegIndex idx, RegVal val, ThreadID tid)
{
    unsigned reg_sel = (bankType[idx] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "[tid:%i] Setting CP0 Register: %u Select: %u (%s) to %#x\n",
            tid, idx / 8, idx % 8, miscRegNames[idx], val);
    miscRegFile_WriteMask[idx][reg_sel] = val;
}

// PROGRAMMER'S NOTES:
// (1) Some CP0 Registers have fields that cannot
// be overwritten. Make sure to handle those particular registers
// with care!
void
ISA::setMiscReg(RegIndex idx, RegVal val, ThreadID tid)
{
    int reg_sel = (bankType[idx] == perThreadContext)
        ? tid : getVPENum(tid);

    DPRINTF(MipsPRA,
            "[tid:%i] Setting CP0 Register:%u "
            "Select:%u (%s) to %#x, with effect.\n",
            tid, idx / 8, idx % 8, miscRegNames[idx], val);

    RegVal cp0_val = filterCP0Write(idx, reg_sel, val);

    miscRegFile[idx][reg_sel] = cp0_val;

    scheduleCP0Update(tc->getCpuPtr(), Cycles(1));
}

/**
 * This method doesn't need to adjust the Control Register Offset
 * since it has already been done in the calling method
 * (setRegWithEffect)
*/
RegVal
ISA::filterCP0Write(RegIndex idx, int reg_sel, RegVal val)
{
    RegVal retVal = val;

    // Mask off read-only regions
    retVal &= miscRegFile_WriteMask[idx][reg_sel];
    RegVal curVal = miscRegFile[idx][reg_sel];
    // Mask off current alue with inverse mask (clear writeable bits)
    curVal &= (~miscRegFile_WriteMask[idx][reg_sel]);
    retVal |= curVal; // Combine the two
    DPRINTF(MipsPRA,
            "filterCP0Write: Mask: %lx, Inverse Mask: %lx, write Val: %x, "
            "current val: %lx, written val: %x\n",
            miscRegFile_WriteMask[idx][reg_sel],
            ~miscRegFile_WriteMask[idx][reg_sel],
            val, miscRegFile[idx][reg_sel], retVal);
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
    MVPConf0Reg mvpConf0 = readMiscRegNoEffect(misc_reg::MvpConf0);
    ThreadID num_threads = mvpConf0.ptc + 1;

    for (ThreadID tid = 0; tid < num_threads; tid++) {
        TCStatusReg tcStatus = readMiscRegNoEffect(misc_reg::TcStatus, tid);
        TCHaltReg tcHalt = readMiscRegNoEffect(misc_reg::TcHalt, tid);

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

} // namespace MipsISA
} // namespace gem5
