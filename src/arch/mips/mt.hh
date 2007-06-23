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

#ifndef __ARCH_MIPS_MT_HH__
#define __ARCH_MIPS_MT_HH__

/**
 * @file
 *
 * ISA-specific helper functions for multithreaded execution.
 */

#include "arch/isa_traits.hh"
#include "arch/mips/faults.hh"
#include "arch/mips/mt_constants.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "base/misc.hh"

#include <iostream>
using namespace std;

namespace MipsISA
{


template <class TC>
inline unsigned
getVirtProcNum(TC *tc)
{
    MiscReg tcbind = tc->readMiscRegNoEffect(TCBind);
    return bits(tcbind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO);
}

template <class TC>
inline unsigned
getTargetThread(TC *tc)
{
    MiscReg vpec_ctrl = tc->readMiscRegNoEffect(VPEControl);
    return bits(vpec_ctrl, VPEC_TARG_TC_HI, VPEC_TARG_TC_LO);
}

template <class TC>
inline void
haltThread(TC *tc)
{
    if (tc->status() == TC::Active) {
        tc->halt();

        // Save last known PC in TCRestart
        // @TODO: Needs to check if this is a branch and if so, take previous instruction
        tc->setMiscReg(TCRestart, tc->readNextPC());

        warn("%i: Halting thread %i in %s @ PC %x, setting restart PC to %x", curTick, tc->getThreadNum(), tc->getCpuPtr()->name(),
             tc->readPC(), tc->readNextPC());
    }
}

template <class TC>
inline void
restoreThread(TC *tc)
{
    if (tc->status() != TC::Active) {
        // Restore PC from TCRestart
        IntReg pc = tc->readMiscRegNoEffect(TCRestart);

        // TODO: SET PC WITH AN EVENT INSTEAD OF INSTANTANEOUSLY
        // tc->setPCEvent(pc, pc + 4, pc + 8);
        tc->setPC(pc);
        tc->setNextPC(pc + 4);
        tc->setNextNPC(pc + 8);
        tc->activate(0);

        warn("%i: Restoring thread %i in %s @ PC %x", curTick, tc->getThreadNum(), tc->getCpuPtr()->name(),
             tc->readPC());
    }
}

template <class TC>
void
forkThread(TC *tc, Fault &fault, int Rd_bits, int Rs, int Rt)
{
    int num_threads = bits(tc->readMiscRegNoEffect(MVPConf0), MVPC0_PTC_HI, MVPC0_PTC_LO) + 1;

    int success = 0;
    for (int tid = 0; tid < num_threads && success == 0; tid++) {
        unsigned tid_TCBind = tc->readRegOtherThread(MipsISA::TCBind + Ctrl_Base_DepTag,
                                                     tid);
        unsigned tc_bind = tc->readMiscRegNoEffect(MipsISA::TCBind);

        if (bits(tid_TCBind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO) ==
            bits(tc_bind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO)) {

            unsigned tid_TCStatus = tc->readRegOtherThread(MipsISA::TCStatus + Ctrl_Base_DepTag,
                                                           tid);

            unsigned tid_TCHalt = tc->readRegOtherThread(MipsISA::TCHalt + Ctrl_Base_DepTag,
                                                         tid);

            if (bits(tid_TCStatus, TCS_DA) == 1 &&
                bits(tid_TCHalt, TCH_H) == 0    &&
                bits(tid_TCStatus, TCS_A) == 0  &&
                success == 0) {

                tc->setRegOtherThread(MipsISA::TCRestart + Ctrl_Base_DepTag, Rs, tid);

                tc->setRegOtherThread(Rd_bits, Rt, tid);

                unsigned status_ksu = bits(tc->readMiscReg(MipsISA::Status),
                                           S_KSU_HI, S_KSU_LO);
                unsigned tc_status_asid = bits(tc->readMiscReg(MipsISA::TCStatus),
                                          TCS_ASID_HI, TCS_ASID_LO);

                // Set Run-State to Running
                replaceBits(tid_TCStatus, TCSTATUS_RNST_HI, TCSTATUS_RNST_LO, 0);

                // Set Delay-Slot to 0
                replaceBits(tid_TCStatus, TCSTATUS_TDS, 0);

                // Set Dirty TC to 1
                replaceBits(tid_TCStatus, TCSTATUS_DT, 1);

                // Set Activated to 1
                replaceBits(tid_TCStatus, TCSTATUS_A, 1);

                // Set status to previous thread's status
                replaceBits(tid_TCStatus, TCSTATUS_TKSU_HI, TCSTATUS_TKSU_LO, status_ksu);

                // Set ASID to previous thread's state
                replaceBits(tid_TCStatus, TCSTATUS_ASID_HI, TCSTATUS_ASID_LO, tc_status_asid);

                // Write Status Register
                tc->setRegOtherThread(MipsISA::TCStatus + Ctrl_Base_DepTag,
                                      tid_TCStatus, tid);

                // Mark As Successful Fork
                success = 1;
            }
        } else {
            std::cerr << "Bad VPEs" << endl;
        }
    }

    if (success == 0) {
        unsigned vpe_control = tc->readMiscRegNoEffect(MipsISA::VPEControl);
        tc->setMiscReg(VPEControl, insertBits(vpe_control, VPEC_EXCPT_HI, VPEC_EXCPT_LO, 1));
        fault = new ThreadFault();
    }
}


template <class TC>
int
yieldThread(TC *tc, Fault &fault, int src_reg, uint32_t yield_mask)
{
    if (src_reg == 0) {
        unsigned mvpconf0 = tc->readMiscRegNoEffect(MVPConf0);
        int num_threads = bits(mvpconf0, MVPC0_PTC_HI, MVPC0_PTC_LO) + 1;

        int ok = 0;

        // Get Current VPE & TC numbers from calling thread
        unsigned tcbind = tc->readMiscRegNoEffect(TCBind);
        unsigned cur_vpe = bits(tcbind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO);
        unsigned cur_tc = bits(tcbind, TCB_CUR_TC_HI, TCB_CUR_TC_LO);

        for (int tid = 0; tid < num_threads; tid++) {
            unsigned tid_TCStatus = tc->readRegOtherThread(MipsISA::TCStatus + Ctrl_Base_DepTag,
                                                           tid);
            unsigned tid_TCHalt = tc->readRegOtherThread(MipsISA::TCHalt + Ctrl_Base_DepTag,
                                                         tid);
            unsigned tid_TCBind = tc->readRegOtherThread(MipsISA::TCBind + Ctrl_Base_DepTag,
                                                         tid);

            unsigned tid_vpe = bits(tid_TCBind, TCB_CUR_VPE_HI, TCB_CUR_VPE_LO);
            unsigned tid_tc = bits(tid_TCBind, TCB_CUR_TC_HI, TCB_CUR_TC_LO);
            unsigned tid_tcstatus_da = bits(tid_TCStatus, TCS_DA);
            unsigned tid_tcstatus_a = bits(tid_TCStatus, TCS_A);
            unsigned tid_tchalt_h = bits(tid_TCHalt, TCH_H);

            if (tid_vpe == cur_vpe &&
                tid_tc == cur_tc &&
                tid_tcstatus_da == 1 &&
                tid_tchalt_h == 0    &&
                tid_tcstatus_a == 1) {
                ok = 1;
            }
        }

        if (ok == 1) {
            unsigned tcstatus = tc->readMiscRegNoEffect(TCStatus);
            tc->setMiscReg(TCStatus, insertBits(tcstatus, TCS_A, TCS_A, 0));
            warn("%i: Deactivating Hardware Thread Context #%i", curTick, tc->getThreadNum());
        }
    } else if (src_reg > 0) {
        if (src_reg & !yield_mask != 0) {
            unsigned vpe_control = tc->readMiscReg(VPEControl);
            tc->setMiscReg(VPEControl, insertBits(vpe_control, VPEC_EXCPT_HI, VPEC_EXCPT_LO, 2));
            fault = new ThreadFault();
        } else {
            //tc->setThreadRescheduleCondition(src_reg & yield_mask);
        }
    } else if (src_reg != -2) {
        unsigned tcstatus = tc->readMiscRegNoEffect(TCStatus);
        unsigned vpe_control = tc->readMiscRegNoEffect(VPEControl);
        unsigned tcstatus_dt = bits(tcstatus, TCS_DT);
        unsigned vpe_control_ysi = bits(vpe_control, VPEC_YSI);

        if (vpe_control_ysi == 1 && tcstatus_dt == 1 ) {
            tc->setMiscReg(VPEControl, insertBits(vpe_control, VPEC_EXCPT_HI, VPEC_EXCPT_LO, 4));
            fault = new ThreadFault();
        } else {
            //tc->ScheduleOtherThreads();
            //std::cerr << "T" << tc->getThreadNum() << "YIELD: Schedule Other Threads.\n" << std::endl;
            //tc->suspend();
            // Save last known PC in TCRestart
            // @TODO: Needs to check if this is a branch and if so, take previous instruction
            //tc->setMiscRegWithEffect(TCRestart, tc->readNextPC());
        }
    }

    return src_reg & yield_mask;
}


// TC will usually be a object derived from ThreadContext
// (src/cpu/thread_context.hh)
template <class TC>
inline void
updateStatusView(TC *tc)
{
    // TCStatus' register view must be the same as
    // Status register view for CU, MX, KSU bits
    MiscReg tc_status = tc->readMiscRegNoEffect(TCStatus);
    MiscReg status = tc->readMiscRegNoEffect(Status);

    unsigned cu_bits = bits(tc_status, TCS_TCU_HI, TCS_TCU_LO);
    replaceBits(status, S_CU_HI, S_CU_LO, cu_bits);

    unsigned mx_bits = bits(tc_status, TCS_TMX);
    replaceBits(status, S_MX, S_MX, mx_bits);

    unsigned ksu_bits = bits(tc_status, TCS_TKSU_HI, TCS_TKSU_LO);
    replaceBits(status, S_KSU_HI, S_KSU_LO, ksu_bits);

    tc->setMiscRegNoEffect(Status, status);
}

// TC will usually be a object derived from ThreadContext
// (src/cpu/thread_context.hh)
template <class TC>
inline void
updateTCStatusView(TC *tc)
{
    // TCStatus' register view must be the same as
    // Status register view for CU, MX, KSU bits
    MiscReg tc_status = tc->readMiscRegNoEffect(TCStatus);
    MiscReg status = tc->readMiscRegNoEffect(Status);

    unsigned cu_bits = bits(status, S_CU_HI, S_CU_LO);
    replaceBits(tc_status, TCS_TCU_HI, TCS_TCU_LO, cu_bits);

    unsigned mx_bits = bits(status, S_MX, S_MX);
    replaceBits(tc_status, TCS_TMX, mx_bits);

    unsigned ksu_bits = bits(status, S_KSU_HI, S_KSU_LO);
    replaceBits(tc_status, TCS_TKSU_HI, TCS_TKSU_LO, ksu_bits);

    tc->setMiscRegNoEffect(TCStatus, tc_status);
}

} // namespace MipsISA


#endif
