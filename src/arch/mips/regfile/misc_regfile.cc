/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *          Jaidev Patwardhan
 */

#include "base/bitfield.hh"

#include "arch/mips/regfile/misc_regfile.hh"
#include "arch/mips/mt_constants.hh"
#include "arch/mips/pra_constants.hh"

#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"

//#include "params/DerivO3CPU.hh"

using namespace std;
using namespace MipsISA;

std::string MiscRegFile::miscRegNames[NumMiscRegs] =
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

    miscRegFile_WriteMask.resize(NumMiscRegs);

    for (int i=0; i < NumMiscRegs; i++) {
      miscRegFile_WriteMask[i].push_back(0);
    }
    clear(0);
}

void
MiscRegFile::clear(unsigned tid_or_vpn)
{
    for(int i = 0; i < NumMiscRegs; i++) {
        miscRegFile[i][tid_or_vpn] = 0;
        miscRegFile_WriteMask[i][tid_or_vpn] = (long unsigned int)(-1);
    }
}

void
MiscRegFile::expandForMultithreading(ThreadID num_threads, unsigned num_vpes)
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
int MiscRegFile::getInstAsid()
{
  MiscReg Entry_Hi = readRegNoEffect(EntryHi);
  return bits(Entry_Hi,EntryHi_ASID_HI,EntryHi_ASID_LO);
}

int MiscRegFile:: getDataAsid()
{
  MiscReg EHi = readRegNoEffect(EntryHi);
  return bits(EHi,EntryHi_ASID_HI,EntryHi_ASID_LO);
}
//@TODO: Use MIPS STYLE CONSTANTS (e.g. TCHALT_H instead of TCH_H)
void
MiscRegFile::reset(std::string core_name, ThreadID num_threads,
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

    MiscReg ProcID = readRegNoEffect(PRId);
    replaceBits(ProcID,PRIdCoOp_HI,PRIdCoOp_LO,cp.CP0_PRId_CompanyOptions);
    replaceBits(ProcID,PRIdCoID_HI,PRIdCoID_LO,cp.CP0_PRId_CompanyID);
    replaceBits(ProcID,PRIdProc_ID_HI,PRIdProc_ID_LO,cp.CP0_PRId_ProcessorID);
    replaceBits(ProcID,PRIdRev_HI,PRIdRev_LO,cp.CP0_PRId_Revision);
    setRegNoEffect(PRId,ProcID);
    // Now, create Write Mask for ProcID register
    MiscReg ProcID_Mask = 0; // Read-Only register
    replaceBits(ProcID_Mask,0,32,0);
    setRegMask(PRId,ProcID_Mask);

    // Config
    MiscReg cfg = readRegNoEffect(Config);
    replaceBits(cfg, Config_BE_HI, Config_BE_LO, cp.CP0_Config_BE);
    replaceBits(cfg, Config_AT_HI, Config_AT_LO, cp.CP0_Config_AT);
    replaceBits(cfg, Config_AR_HI, Config_AR_LO, cp.CP0_Config_AR);
    replaceBits(cfg, Config_MT_HI, Config_MT_LO, cp.CP0_Config_MT);
    replaceBits(cfg, Config_VI_HI, Config_VI_LO, cp.CP0_Config_VI);
    replaceBits(cfg, Config_M, 1);
    setRegNoEffect(Config, cfg);
    // Now, create Write Mask for Config register
    MiscReg cfg_Mask = 0x7FFF0007;
    replaceBits(cfg_Mask,0,32,0);
    setRegMask(Config,cfg_Mask);

    // Config1
    MiscReg cfg1 = readRegNoEffect(Config1);
    replaceBits(cfg1, Config1_MMUSize_HI, Config1_MMUSize_LO,
                cp.CP0_Config1_MMU);
    replaceBits(cfg1, Config1_IS_HI, Config1_IS_LO, cp.CP0_Config1_IS);
    replaceBits(cfg1, Config1_IL_HI, Config1_IL_LO, cp.CP0_Config1_IL);
    replaceBits(cfg1, Config1_IA_HI, Config1_IA_LO, cp.CP0_Config1_IA);
    replaceBits(cfg1, Config1_DS_HI, Config1_DS_LO, cp.CP0_Config1_DS);
    replaceBits(cfg1, Config1_DL_HI, Config1_DL_LO, cp.CP0_Config1_DL);
    replaceBits(cfg1, Config1_DA_HI, Config1_DA_LO, cp.CP0_Config1_DA);
    replaceBits(cfg1, Config1_FP_HI, Config1_FP_LO, cp.CP0_Config1_FP);
    replaceBits(cfg1, Config1_EP_HI, Config1_EP_LO, cp.CP0_Config1_EP);
    replaceBits(cfg1, Config1_WR_HI, Config1_WR_LO, cp.CP0_Config1_WR);
    replaceBits(cfg1, Config1_MD_HI, Config1_MD_LO, cp.CP0_Config1_MD);
    replaceBits(cfg1, Config1_C2_HI, Config1_C2_LO, cp.CP0_Config1_C2);
    replaceBits(cfg1, Config1_PC_HI, Config1_PC_LO, cp.CP0_Config1_PC);
    replaceBits(cfg1, Config1_M, cp.CP0_Config1_M);
    setRegNoEffect(Config1, cfg1);
    // Now, create Write Mask for Config register
    MiscReg cfg1_Mask = 0; // Read Only Register
    replaceBits(cfg1_Mask,0,32,0);
    setRegMask(Config1,cfg1_Mask);

    // Config2
    MiscReg cfg2 = readRegNoEffect(Config2);
    replaceBits(cfg2, Config2_TU_HI, Config2_TU_LO, cp.CP0_Config2_TU);
    replaceBits(cfg2, Config2_TS_HI, Config2_TS_LO, cp.CP0_Config2_TS);
    replaceBits(cfg2, Config2_TL_HI, Config2_TL_LO, cp.CP0_Config2_TL);
    replaceBits(cfg2, Config2_TA_HI, Config2_TA_LO, cp.CP0_Config2_TA);
    replaceBits(cfg2, Config2_SU_HI, Config2_SU_LO, cp.CP0_Config2_SU);
    replaceBits(cfg2, Config2_SS_HI, Config2_SS_LO, cp.CP0_Config2_SS);
    replaceBits(cfg2, Config2_SL_HI, Config2_SL_LO, cp.CP0_Config2_SL);
    replaceBits(cfg2, Config2_SA_HI, Config2_SA_LO, cp.CP0_Config2_SA);
    replaceBits(cfg2, Config2_M, cp.CP0_Config2_M);
    setRegNoEffect(Config2, cfg2);
    // Now, create Write Mask for Config register
    MiscReg cfg2_Mask = 0x7000F000; // Read Only Register
    replaceBits(cfg2_Mask,0,32,0);
    setRegMask(Config2,cfg2_Mask);

    // Config3
    MiscReg cfg3 = readRegNoEffect(Config3);
    replaceBits(cfg3, Config3_DSPP_HI, Config3_DSPP_LO, cp.CP0_Config3_DSPP);
    replaceBits(cfg3, Config3_LPA_HI, Config3_LPA_LO, cp.CP0_Config3_LPA);
    replaceBits(cfg3, Config3_VEIC_HI, Config3_VEIC_LO, cp.CP0_Config3_VEIC);
    replaceBits(cfg3, Config3_VINT_HI, Config3_VINT_LO, cp.CP0_Config3_VInt);
    replaceBits(cfg3, Config3_SP_HI, Config3_SP_LO, cp.CP0_Config3_SP);
    replaceBits(cfg3, Config3_MT_HI, Config3_MT_LO, cp.CP0_Config3_MT);
    replaceBits(cfg3, Config3_SM_HI, Config3_SM_LO, cp.CP0_Config3_SM);
    replaceBits(cfg3, Config3_TL_HI, Config3_TL_LO, cp.CP0_Config3_TL);
    setRegNoEffect(Config3, cfg3);
    // Now, create Write Mask for Config register
    MiscReg cfg3_Mask = 0; // Read Only Register
    replaceBits(cfg3_Mask,0,32,0);
    setRegMask(Config3,cfg3_Mask);

    // EBase - CPUNum
    MiscReg EB = readRegNoEffect(EBase);
    replaceBits(EB, EBase_CPUNum_HI, EBase_CPUNum_LO, cp.CP0_EBase_CPUNum);
    replaceBits(EB, 31, 31, 1);
    setRegNoEffect(EBase, EB);
    // Now, create Write Mask for Config register
    MiscReg EB_Mask = 0x3FFFF000;// Except Exception Base, the
                                 // entire register is read only
    replaceBits(EB_Mask,0,32,0);
    setRegMask(EBase,EB_Mask);

    // SRS Control - HSS (Highest Shadow Set)
    MiscReg SC = readRegNoEffect(SRSCtl);
    replaceBits(SC, SRSCtl_HSS_HI,SRSCtl_HSS_LO,cp.CP0_SrsCtl_HSS);
    setRegNoEffect(SRSCtl, SC);
    // Now, create Write Mask for the SRS Ctl register
    MiscReg SC_Mask = 0x0000F3C0;
    replaceBits(SC_Mask,0,32,0);
    setRegMask(SRSCtl,SC_Mask);

    // IntCtl - IPTI, IPPCI
    MiscReg IC = readRegNoEffect(IntCtl);
    replaceBits(IC, IntCtl_IPTI_HI,IntCtl_IPTI_LO,cp.CP0_IntCtl_IPTI);
    replaceBits(IC, IntCtl_IPPCI_HI,IntCtl_IPPCI_LO,cp.CP0_IntCtl_IPPCI);
    setRegNoEffect(IntCtl, IC);
    // Now, create Write Mask for the IntCtl register
    MiscReg IC_Mask = 0x000003E0;
    replaceBits(IC_Mask,0,32,0);
    setRegMask(IntCtl,IC_Mask);

    // Watch Hi - M - FIXME (More than 1 Watch register)
    MiscReg WHi = readRegNoEffect(WatchHi0);
    replaceBits(WHi, WatchHi_M, cp.CP0_WatchHi_M);
    setRegNoEffect(WatchHi0, WHi);
    // Now, create Write Mask for the IntCtl register
    MiscReg wh_Mask = 0x7FFF0FFF;
    replaceBits(wh_Mask,0,32,0);
    setRegMask(WatchHi0,wh_Mask);

    // Perf Ctr - M - FIXME (More than 1 PerfCnt Pair)
    MiscReg PCtr = readRegNoEffect(PerfCnt0);
    replaceBits(PCtr, PerfCntCtl_M, cp.CP0_PerfCtr_M);
    replaceBits(PCtr, PerfCntCtl_W, cp.CP0_PerfCtr_W);
    setRegNoEffect(PerfCnt0, PCtr);
    // Now, create Write Mask for the IntCtl register
    MiscReg pc_Mask = 0x00007FF;
    replaceBits(pc_Mask,0,32,0);
    setRegMask(PerfCnt0,pc_Mask);

    // Random
    MiscReg random = readRegNoEffect(CP0_Random);
    random = 63;
    setRegNoEffect(CP0_Random, random);
    // Now, create Write Mask for the IntCtl register
    MiscReg random_Mask = 0;
    replaceBits(random_Mask,0,32,0);
    setRegMask(CP0_Random,random_Mask);

    // PageGrain
    MiscReg pagegrain = readRegNoEffect(PageGrain);
    replaceBits(pagegrain,PageGrain_ESP,cp.CP0_Config3_SP);
    setRegNoEffect(PageGrain, pagegrain);
    // Now, create Write Mask for the IntCtl register
    MiscReg pg_Mask = 0x10000000;
    replaceBits(pg_Mask,0,32,0);
    setRegMask(PageGrain,pg_Mask);

    // Status
    MiscReg stat = readRegNoEffect(Status);
    // Only CU0 and IE are modified on a reset - everything else needs
    // to be controlled on a per CPU model basis

    // Enable CP0 on reset
    // replaceBits(stat, Status_CU0_HI,Status_CU0_LO, 1);

    // Enable ERL bit on a reset
    replaceBits(stat, Status_ERL_HI, Status_ERL_LO, 1);

    // Enable BEV bit on a reset
    replaceBits(stat, Status_BEV_HI, Status_BEV_LO, 1);

    setRegNoEffect(Status, stat);
    // Now, create Write Mask for the Status register
    MiscReg stat_Mask = 0xFF78FF17;
    replaceBits(stat_Mask,0,32,0);
    setRegMask(Status,stat_Mask);


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
    for (ThreadID tid = 0; tid < num_threads; tid++) {
        MiscReg tc_bind = readRegNoEffect(TCBind, tid);
        replaceBits(tc_bind, TCB_CUR_TC_HI, TCB_CUR_TC_LO, tid);
        setRegNoEffect(TCBind, tc_bind, tid);
    }
    // TCHalt
    MiscReg tc_halt = readRegNoEffect(TCHalt);
    replaceBits(tc_halt, TCH_H, 0);
    setRegNoEffect(TCHalt, tc_halt);
    /*for (ThreadID tid = 1; tid < num_threads; tid++) {
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
    for (ThreadID tid = 1; tid < num_threads; tid++) {
        tc_status = readRegNoEffect(TCStatus, tid);
        replaceBits(tc_status, TCSTATUS_DA, 1);
        setRegNoEffect(TCStatus, tc_status, tid);
    }


    MiscReg Mask = 0x7FFFFFFF;

    // Now, create Write Mask for the Index register
    replaceBits(Mask,0,32,0);
    setRegMask(Index,Mask);

    Mask = 0x3FFFFFFF;
    replaceBits(Mask,0,32,0);
    setRegMask(EntryLo0,Mask);
    setRegMask(EntryLo1,Mask);

    Mask = 0xFF800000;
    replaceBits(Mask,0,32,0);
    setRegMask(Context,Mask);

    Mask = 0x1FFFF800;
    replaceBits(Mask,0,32,0);
    setRegMask(PageMask,Mask);

    Mask = 0x0;
    replaceBits(Mask,0,32,0);
    setRegMask(BadVAddr,Mask);
    setRegMask(LLAddr,Mask);

    Mask = 0x08C00300;
    replaceBits(Mask,0,32,0);
    setRegMask(Cause,Mask);

}

inline unsigned
MiscRegFile::getVPENum(ThreadID tid)
{
    unsigned tc_bind = miscRegFile[TCBind - Ctrl_Base_DepTag][tid];
    return bits(tc_bind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO);
}

MiscReg
MiscRegFile::readRegNoEffect(int reg_idx, ThreadID tid)
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
MiscRegFile::readReg(int reg_idx, ThreadContext *tc,  ThreadID tid)
{
    int misc_reg = reg_idx - Ctrl_Base_DepTag;
    unsigned reg_sel = (bankType[misc_reg] == perThreadContext)
        ? tid : getVPENum(tid);
    DPRINTF(MipsPRA,
            "Reading CP0 Register:%u Select:%u (%s) with effect (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);


    switch (misc_reg)
    {
      default:
        return miscRegFile[misc_reg][reg_sel];
    }
}

void
MiscRegFile::setRegNoEffect(int reg_idx, const MiscReg &val, ThreadID tid)
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
MiscRegFile::setRegMask(int reg_idx, const MiscReg &val, ThreadID tid)
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
//template <class TC>
void
MiscRegFile::setReg(int reg_idx, const MiscReg &val,
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
MiscRegFile::filterCP0Write(int misc_reg, int reg_sel, const MiscReg &val)
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
MiscRegFile::scheduleCP0Update(int delay)
{
    if (!cp0Updated) {
        cp0Updated = true;

        //schedule UPDATE
        CP0Event *cp0_event = new CP0Event(this, cpu, UpdateCP0);
        cpu->schedule(cp0_event, curTick + cpu->ticks(delay));
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
    ThreadID num_threads = bits(mvp_conf0, MVPC0_PTC_HI, MVPC0_PTC_LO) + 1;

    for (ThreadID tid = 0; tid < num_threads; tid++) {
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
    : Event(CPU_Tick_Pri), cp0(_cp0), cpu(_cpu), cp0EventType(e_type)
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
MiscRegFile::CP0Event::description() const
{
    return "Coprocessor-0 event";
}

void
MiscRegFile::CP0Event::scheduleEvent(int delay)
{
    cpu->reschedule(this, curTick + cpu->ticks(delay), true);
}

void
MiscRegFile::CP0Event::unscheduleEvent()
{
    if (scheduled())
        squash();
}
