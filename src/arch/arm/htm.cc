/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "arch/arm/htm.hh"

#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

void
ArmISA::HTMCheckpoint::reset()
{
    rt = 0;
    nPc = 0;
    sp = 0;
    fpcr = 0;
    fpsr = 0;
    iccPmrEl1 = 0;
    nzcv = 0;
    daif = 0;
    tcreason = 0;
    x.fill(0);
    for (auto i = 0; i < NumVecRegs; ++i) {
      z[i].zero();
    }
    for (auto i = 0; i < NumVecPredRegs; ++i) {
      p[i].reset();
    }
    pcstateckpt = PCState();

    BaseHTMCheckpoint::reset();
}

void
ArmISA::HTMCheckpoint::save(ThreadContext *tc)
{
    sp = tc->getReg(int_reg::Spx);
    // below should be enabled on condition that GICV3 is enabled
    //tme_checkpoint->iccPmrEl1 = tc->readMiscReg(MISCREG_ICC_PMR_EL1);
    nzcv = tc->readMiscReg(MISCREG_NZCV);
    daif = tc->readMiscReg(MISCREG_DAIF);
    for (auto n = 0; n < int_reg::NumArchRegs; n++) {
        x[n] = tc->getReg(intRegClass[n]);
    }
    // TODO first detect if FP is enabled at this EL
    for (auto n = 0; n < NumVecRegs; n++)
        tc->getReg(vecRegClass[n], &z[n]);
    for (auto n = 0; n < NumVecPredRegs; n++)
        tc->getReg(vecPredRegClass[n], &p[n]);
    fpcr = tc->readMiscReg(MISCREG_FPCR);
    fpsr = tc->readMiscReg(MISCREG_FPSR);
    pcstateckpt = tc->pcState().as<PCState>();

    BaseHTMCheckpoint::save(tc);
}

void
ArmISA::HTMCheckpoint::restore(ThreadContext *tc, HtmFailureFaultCause cause)
{
    tc->setReg(int_reg::Spx, sp);
    // below should be enabled on condition that GICV3 is enabled
    //tc->setMiscReg(MISCREG_ICC_PMR_EL1, tme_checkpoint->iccPmrEl1);
    tc->setMiscReg(MISCREG_NZCV, nzcv);
    tc->setMiscReg(MISCREG_DAIF, daif);
    for (auto n = 0; n < int_reg::NumArchRegs; n++)
        tc->setReg(intRegClass[n], x[n]);
    // TODO first detect if FP is enabled at this EL
    for (auto n = 0; n < NumVecRegs; n++)
        tc->setReg(vecRegClass[n], &z[n]);
    for (auto n = 0; n < NumVecPredRegs; n++)
        tc->setReg(vecPredRegClass[n], &p[n]);
    tc->setMiscReg(MISCREG_FPCR, fpcr);
    tc->setMiscReg(MISCREG_FPSR, fpsr);

    // this code takes the generic HTM failure reason
    // and prepares an Arm/TME-specific error code
    // which is written to a destination register

    bool interrupt = false; // TODO get this from threadcontext
    bool retry = false;
    uint64_t error_code = 0;
    switch (cause) {
      case HtmFailureFaultCause::EXPLICIT:
        replaceBits(error_code, 14, 0, tcreason);
        replaceBits(error_code, 16, 1);
        retry = bits(tcreason, 15);
        break;
      case HtmFailureFaultCause::MEMORY:
        replaceBits(error_code, 17, 1);
        retry = true;
        break;
      case HtmFailureFaultCause::OTHER:
        replaceBits(error_code, 18, 1);
        break;
      case HtmFailureFaultCause::EXCEPTION:
        replaceBits(error_code, 19, 1);
        break;
      case HtmFailureFaultCause::SIZE:
        replaceBits(error_code, 20, 1);
        break;
      case HtmFailureFaultCause::NEST:
        replaceBits(error_code, 21, 1);
        break;
        // case HtmFailureFaultCause_DEBUG:
        //     replaceBits(error_code, 22, 1);
        //     break;
      default:
        panic("Unknown HTM failure reason\n");
    }
    assert(!retry || !interrupt);
    if (retry)
        replaceBits(error_code, 15, 1);
    if (interrupt)
        replaceBits(error_code, 23, 1);
    tc->setReg(intRegClass[rt], error_code);

    // set next PC
    pcstateckpt.uReset();
    pcstateckpt.advance();
    tc->pcState(pcstateckpt);

    BaseHTMCheckpoint::restore(tc, cause);
}

} // namespace gem5
