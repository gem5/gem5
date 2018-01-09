/*
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
 */

#ifndef __ARCH_MIPS_MT_HH__
#define __ARCH_MIPS_MT_HH__

/**
 * @file
 *
 * ISA-specific helper functions for multithreaded execution.
 */

#include <iostream>

#include "arch/mips/faults.hh"
#include "arch/mips/isa_traits.hh"
#include "arch/mips/mt_constants.hh"
#include "arch/mips/pra_constants.hh"
#include "arch/mips/registers.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/trace.hh"

namespace MipsISA
{

template <class TC>
inline unsigned
getVirtProcNum(TC *tc)
{
    TCBindReg tcbind = tc->readMiscRegNoEffect(MISCREG_TC_BIND);
    return tcbind.curVPE;
}

template <class TC>
inline unsigned
getTargetThread(TC *tc)
{
    VPEControlReg vpeCtrl = tc->readMiscRegNoEffect(MISCREG_VPE_CONTROL);
    return vpeCtrl.targTC;
}

template <class TC>
inline void
haltThread(TC *tc)
{
    if (tc->status() == TC::Active) {
        tc->halt();

        // Save last known PC in TCRestart
        // @TODO: Needs to check if this is a branch and if so,
        // take previous instruction
        PCState pc = tc->pcState();
        tc->setMiscReg(MISCREG_TC_RESTART, pc.npc());

        warn("%i: Halting thread %i in %s @ PC %x, setting restart PC to %x",
                curTick(), tc->threadId(), tc->getCpuPtr()->name(),
                pc.pc(), pc.npc());
    }
}

template <class TC>
inline void
restoreThread(TC *tc)
{
    if (tc->status() != TC::Active) {
        // Restore PC from TCRestart
        Addr restartPC = tc->readMiscRegNoEffect(MISCREG_TC_RESTART);

        // TODO: SET PC WITH AN EVENT INSTEAD OF INSTANTANEOUSLY
        tc->pcState(restartPC);
        tc->activate();

        warn("%i: Restoring thread %i in %s @ PC %x",
                curTick(), tc->threadId(), tc->getCpuPtr()->name(), restartPC);
    }
}

template <class TC>
void
forkThread(TC *tc, Fault &fault, int Rd_bits, int Rs, int Rt)
{
    MVPConf0Reg mvpConf = tc->readMiscRegNoEffect(MISCREG_MVP_CONF0);
    int num_threads = mvpConf.ptc + 1;

    int success = 0;
    for (ThreadID tid = 0; tid < num_threads && success == 0; tid++) {
        TCBindReg tidTCBind =
            tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_BIND), tid);
        TCBindReg tcBind = tc->readMiscRegNoEffect(MISCREG_TC_BIND);

        if (tidTCBind.curVPE == tcBind.curVPE) {

            TCStatusReg tidTCStatus =
                tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_STATUS),
                                       tid);

            TCHaltReg tidTCHalt =
                tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_HALT),
                                       tid);

            if (tidTCStatus.da == 1 && tidTCHalt.h == 0 &&
                tidTCStatus.a == 0 && success == 0) {

                tc->setRegOtherThread(RegId(MiscRegClass, MISCREG_TC_RESTART),
                                      Rs, tid);
                tc->setRegOtherThread(RegId(IntRegClass, Rd_bits), Rt, tid);

                StatusReg status = tc->readMiscReg(MISCREG_STATUS);
                TCStatusReg tcStatus = tc->readMiscReg(MISCREG_TC_STATUS);

                // Set Run-State to Running
                tidTCStatus.rnst = 0;
                // Set Delay-Slot to 0
                tidTCStatus.tds = 0;
                // Set Dirty TC to 1
                tidTCStatus.dt = 1;
                // Set Activated to 1
                tidTCStatus.a = 1;
                // Set status to previous thread's status
                tidTCStatus.tksu = status.ksu;
                // Set ASID to previous thread's state
                tidTCStatus.asid = tcStatus.asid;

                // Write Status Register
                tc->setRegOtherThread(RegId(MiscRegClass, MISCREG_TC_STATUS),
                                      tidTCStatus, tid);

                // Mark As Successful Fork
                success = 1;
            }
        } else {
            std::cerr << "Bad VPEs" << std::endl;
        }
    }

    if (success == 0) {
        VPEControlReg vpeControl =
            tc->readMiscRegNoEffect(MISCREG_VPE_CONTROL);
        vpeControl.excpt = 1;
        tc->setMiscReg(MISCREG_VPE_CONTROL, vpeControl);
        fault = std::make_shared<ThreadFault>();
    }
}


template <class TC>
int
yieldThread(TC *tc, Fault &fault, int src_reg, uint32_t yield_mask)
{
    if (src_reg == 0) {
        MVPConf0Reg mvpConf0 = tc->readMiscRegNoEffect(MISCREG_MVP_CONF0);
        ThreadID num_threads = mvpConf0.ptc + 1;

        int ok = 0;

        // Get Current VPE & TC numbers from calling thread
        TCBindReg tcBind = tc->readMiscRegNoEffect(MISCREG_TC_BIND);

        for (ThreadID tid = 0; tid < num_threads; tid++) {
            TCStatusReg tidTCStatus =
                tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_STATUS),
                                       tid);
            TCHaltReg tidTCHalt =
                tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_HALT),
                                       tid);
            TCBindReg tidTCBind =
                tc->readRegOtherThread(RegId(MiscRegClass, MISCREG_TC_BIND),
                                       tid);

            if (tidTCBind.curVPE == tcBind.curVPE &&
                tidTCBind.curTC == tcBind.curTC &&
                tidTCStatus.da == 1 &&
                tidTCHalt.h == 0    &&
                tidTCStatus.a == 1) {
                ok = 1;
            }
        }

        if (ok == 1) {
            TCStatusReg tcStatus = tc->readMiscRegNoEffect(MISCREG_TC_STATUS);
            tcStatus.a = 0;
            tc->setMiscReg(MISCREG_TC_STATUS, tcStatus);
            warn("%i: Deactivating Hardware Thread Context #%i",
                    curTick(), tc->threadId());
        }
    } else if (src_reg > 0) {
        if (src_reg && !yield_mask != 0) {
            VPEControlReg vpeControl = tc->readMiscReg(MISCREG_VPE_CONTROL);
            vpeControl.excpt = 2;
            tc->setMiscReg(MISCREG_VPE_CONTROL, vpeControl);
            fault = std::make_shared<ThreadFault>();
        } else {
        }
    } else if (src_reg != -2) {
        TCStatusReg tcStatus = tc->readMiscRegNoEffect(MISCREG_TC_STATUS);
        VPEControlReg vpeControl =
            tc->readMiscRegNoEffect(MISCREG_VPE_CONTROL);

        if (vpeControl.ysi == 1 && tcStatus.dt == 1 ) {
            vpeControl.excpt = 4;
            fault = std::make_shared<ThreadFault>();
        } else {
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
    TCStatusReg tcStatus = tc->readMiscRegNoEffect(MISCREG_TC_STATUS);
    StatusReg status = tc->readMiscRegNoEffect(MISCREG_STATUS);

    status.cu = tcStatus.tcu;
    status.mx = tcStatus.tmx;
    status.ksu = tcStatus.tksu;

    tc->setMiscRegNoEffect(MISCREG_STATUS, status);
}

// TC will usually be a object derived from ThreadContext
// (src/cpu/thread_context.hh)
template <class TC>
inline void
updateTCStatusView(TC *tc)
{
    // TCStatus' register view must be the same as
    // Status register view for CU, MX, KSU bits
    TCStatusReg tcStatus = tc->readMiscRegNoEffect(MISCREG_TC_STATUS);
    StatusReg status = tc->readMiscRegNoEffect(MISCREG_STATUS);

    tcStatus.tcu = status.cu;
    tcStatus.tmx = status.mx;
    tcStatus.tksu = status.ksu;

    tc->setMiscRegNoEffect(MISCREG_TC_STATUS, tcStatus);
}

} // namespace MipsISA


#endif
