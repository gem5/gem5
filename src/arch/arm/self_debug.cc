/*
 * Copyright (c) 2019 Metempsy Technology LSC
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

#include "arch/arm/self_debug.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/miscregs_types.hh"
#include "base/bitfield.hh"

using namespace ArmISA;
using namespace std;

Fault
SelfDebug::testBreakPoints(ThreadContext *tc, Addr vaddr)
{
    if (!enableFlag)
        return NoFault;

    setAArch32(tc);

    to32 = targetAArch32(tc);

    init(tc);

    if (!isDebugEnabled(tc))
        return NoFault;

    ExceptionLevel el = (ExceptionLevel) currEL(tc);
    for (auto &p: arBrkPoints){
        PCState pcst = tc->pcState();
        Addr pc = vaddr;
        if (pcst.itstate() != 0x0)
            pc = pcst.pc();
        if (p.getEnable() && p.isActive(pc) &&(!to32 || !p.isSet())){
            const DBGBCR ctr = p.getControlReg(tc);
            if (p.isEnabled(tc, el, ctr.hmc, ctr.ssc, ctr.pmc)) {
                bool debug = p.test(tc, pc, el, ctr, false);
                if (debug){
                    if (to32)
                        p.setOnUse();
                    return triggerException(tc, pc);
                }
            }
        }
    }
    return NoFault;
}


Fault
SelfDebug::triggerException(ThreadContext * tc, Addr vaddr)
{
    if (isTo32()) {
        return std::make_shared<PrefetchAbort>(vaddr,
                                   ArmFault::DebugEvent, false,
                                   ArmFault::UnknownTran,
                                   ArmFault::BRKPOINT);
    } else {
        return std::make_shared<HardwareBreakpoint>(vaddr, 0x22);
    }
}

bool
SelfDebug::isDebugEnabledForEL64(ThreadContext *tc, ExceptionLevel el,
                         bool secure, bool mask)
{
    bool route_to_el2 =  ArmSystem::haveEL(tc, EL2)
                         && !secure && enableTdeTge;
    ExceptionLevel target_el = route_to_el2? EL2 : EL1;
    if (oslk || (bSDD && secure && ArmSystem::haveEL(tc, EL3))){
        return false;
    }
    if (el == target_el){
        return bKDE  && !mask;
    }else{
        return target_el > el;
    }
}

bool
SelfDebug::isDebugEnabledForEL32(ThreadContext *tc, ExceptionLevel el,
                         bool secure, bool mask)
{
    if (el==EL0 && !ELStateUsingAArch32(tc, EL1, secure)){
        return isDebugEnabledForEL64(tc, el, secure, mask);
    }
    if (oslk){
        return false;
    }

    bool enabled;
    if (secure && ArmSystem::haveEL(tc, EL3)){
        // We ignore the check for invasive External debug checking SPIDEN
        // and DBGEN signals. They are not implemented
        bool spd32 = bits(tc->readMiscReg(MISCREG_MDCR_EL3), 14);
        enabled = spd32;

        bool suiden = bits(tc->readMiscReg(MISCREG_SDER), 0);
        enabled  = el == EL0 ? (enabled || suiden) : enabled;
    }
    else
    {
        enabled = el != EL2;
    }
    return enabled;
}

bool
BrkPoint::testLinkedBk(ThreadContext *tc, Addr vaddr, ExceptionLevel el)
{
    bool debug = false;
    const DBGBCR ctr = getControlReg(tc);
    if ((ctr.bt & 0x1) && getEnable()){
        debug = test(tc, vaddr, el, ctr, true);
    }
    return debug;
}

bool
BrkPoint::test(ThreadContext *tc, Addr pc, ExceptionLevel el, DBGBCR ctr,
               bool from_link)
{
    bool v = false;
    switch (ctr.bt)
    {
        case 0x0:
            v = testAddrMatch(tc, pc, ctr.bas);
            break;
        case 0x1:
            v = testAddrMatch(tc, pc, ctr.bas); // linked
            if (v){
                v = (conf->getBrkPoint(ctr.lbn))->testLinkedBk(tc, pc, el);
            }
            break;
        case 0x2:
            v = testContextMatch(tc, true);
            break;
        case 0x3:
            if (from_link){
                v = testContextMatch(tc, true); //linked
            }
            break;
        case 0x4:
            v = testAddrMissMatch(tc, pc, ctr.bas);
            break;
        case 0x5:
            v = testAddrMissMatch(tc, pc, ctr.bas); // linked
            if (v && !from_link)
                v = v && (conf->getBrkPoint(ctr.lbn))->testLinkedBk(tc,
                                                                 pc, el);
            break;
        case 0x6:
            // VHE not implemented
            // v = testContextMatch(tc, true);
            break;
        case 0x7:
            // VHE not implemented
            // if (from_link)
            //     v = testContextMatch(tc, true);
            break;
        case 0x8:
            v = testVMIDMatch(tc);
            break;
        case 0x9:
            if (from_link && ArmSystem::haveEL(tc, EL2)){
                v = testVMIDMatch(tc); // linked
            }
            break;
        case 0xa:
            if (ArmSystem::haveEL(tc, EL2)){
                v = testContextMatch(tc, true);
                if (v && !from_link)
                v = v && testVMIDMatch(tc);
            }
            break;
        case 0xb:
            if (from_link && ArmSystem::haveEL(tc, EL2)){
                v = testContextMatch(tc, true);
                v = v && testVMIDMatch(tc);
            }
            break;
        case 0xc:
            // VHE not implemented
            // v = testContextMatch(tc, false); // CONTEXTIDR_EL2
            break;
        case 0xd:
            // VHE not implemented
            // if (from_link)
            //     v = testContextMatch(tc, false);
            // CONTEXTIDR_EL2 AND LINKED

            break;
       case 0xe:
            // VHE not implemented
            // v = testContextMatch(tc, true); // CONTEXTIDR_EL1
            // v = v && testContextMatch(tc, false); // CONTEXTIDR_EL2
            break;
        case 0xf:
            // VHE not implemented
            // if (from_link){
            //     v = testContextMatch(tc, true); // CONTEXTIDR_EL1
            //     v = v && testContextMatch(tc, false); // CONTEXTIDR_EL2
            // }
            break;
    }
    return v;
}

bool
BrkPoint::testAddrMatch(ThreadContext *tc, Addr in_pc, uint8_t bas)
{
    Addr pc_tocmp = getAddrfromReg(tc);
    Addr pc = bits(in_pc, maxAddrSize, 2);

    bool prs = true;
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    bool thumb = cpsr.t;

    if (thumb){
        if (bas == 0xc)
            prs = bits(in_pc, 1, 0) == 0x2;
        else if (bas == 0x3)
            prs = bits(in_pc, 1, 0) == 0x0;
    }
    return (pc == pc_tocmp) && prs;
}

bool
BrkPoint::testAddrMissMatch(ThreadContext *tc, Addr in_pc, uint8_t bas)
{
    if (bas == 0x0)
        return true;
    Addr pc_tocmp = getAddrfromReg(tc);
    Addr pc = bits(in_pc, maxAddrSize, 2);
    bool prs = false;
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    bool thumb = cpsr.t;

    if (thumb){
        if (bas == 0xc)
            prs = bits(in_pc, 1, 0) == 0x2;
        else if (bas == 0x3)
            prs = bits(in_pc, 1, 0) == 0x0;
    }
    return (pc != pc_tocmp) && !prs;
}

bool
BrkPoint::testContextMatch(ThreadContext *tc, bool ctx1)
{
    if (!isCntxtAware)
        return false;
    MiscRegIndex miscridx;
    ExceptionLevel el = currEL(tc);
    bool a32 = conf->isAArch32();

    if (ctx1){
        miscridx = a32? MISCREG_CONTEXTIDR: MISCREG_CONTEXTIDR_EL1;
        if ((el == EL3 && !a32) || el ==EL2)
            return false;
    }else{
        miscridx = MISCREG_CONTEXTIDR_EL2;
        if (el == EL2 && a32)
            return false;
    }

    RegVal ctxid = tc->readMiscReg(miscridx);
    RegVal v = getContextfromReg(tc, ctx1);
    return (v== ctxid);
}

bool
BrkPoint::testVMIDMatch(ThreadContext *tc)
{
    uint32_t vmid_index = 55;
    if (VMID16enabled)
        vmid_index = 63;
    ExceptionLevel el = currEL(tc);
    if (el == EL2)
        return false;

    uint32_t vmid = bits(tc->readMiscReg(MISCREG_VTTBR_EL2), vmid_index, 48);
    uint32_t v = getVMIDfromReg(tc);
    return (v == vmid);
}


bool
BrkPoint::isEnabled(ThreadContext* tc, ExceptionLevel el,
                    uint8_t hmc, uint8_t ssc, uint8_t pmc){
    bool v;
    bool aarch32 = conf->isAArch32();
    bool noEL2 = !ArmSystem::haveEL(tc, EL2);
    bool noEL3 = !ArmSystem::haveEL(tc, EL3);

    if (noEL3 && !noEL2 && (ssc==0x1 || ssc==0x2)
            && !(hmc && ssc == 0x1 && pmc==0x0)){
        return false;
    }
    else if (noEL3 && noEL2 &&( hmc != 0x0 || ssc !=0x0)
            && !(!aarch32 && ((hmc && ssc == 0x1  && pmc == 0x0)
                    || ssc == 0x3))){
        return false;
    }
    else if (noEL2 && hmc && ssc == 0x3 && pmc == 0x0){
        return false;
    }
    else if (ssc == 0x11 && pmc==0x1 &&
        !(!aarch32 && hmc && ssc == 0x3 &&pmc == 0x0)){
        // AND secureEL2 not implemented
        return false;
    }
    else if (hmc && ssc == 0x1 && pmc == 0x0){//AND secureEL2 not implemented
        return false;
    }
    switch (el) {
        case EL0:
            v = (pmc == 0x3) || (pmc == 0x2 && hmc == 0x0) ;
            if (aarch32)
                v = v || (pmc == 0x0 && ssc != 0x3 && hmc == 0x0);
            if (v && ssc == 0x3)
                panic("Unexpected EL in SelfDebug::isDebugEnabled.\n");
            break;
        case EL1:
            v = (pmc == 0x3) || (pmc == 0x1);
            if (aarch32)
                v = v || (pmc == 0x0 && hmc == 0x0 && ssc !=0x3);
            break;
        case EL2:
            v = (ssc == 0x3) ||
                ((hmc == 0x1) && !((ssc==0x2) && (pmc = 0x0)));
            if (v && pmc == 0x2)
                panic("Unexpected EL in SelfDebug::isDebugEnabled.\n");
            break;
        case EL3:
            if (ssc == 0x1)
                panic("Unexpected EL in SelfDebug::isDebugEnabled.\n");
            v = (hmc == 0x1) & (ssc != 0x3);
            break;
        default:
            panic("Unexpected EL %d in BrkPoint::isEnabled.\n", el);
    }
    return v && SelfDebug::securityStateMatch(tc, ssc, hmc || !aarch32);
}

uint32_t
BrkPoint::getVMIDfromReg(ThreadContext *tc)
{
    uint32_t vmid_index = 39;
    if (VMID16enabled)
        vmid_index = 47;
    return bits(tc->readMiscReg(valRegIndex), vmid_index, 32);
}

