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

#ifndef __ARCH_ARM_SELF_DEBUG_HH__
#define __ARCH_ARM_SELF_DEBUG_HH__


#include "arch/arm/faults.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/system.hh"
#include "arch/arm/types.hh"
#include "arch/arm/utility.hh"
#include "cpu/thread_context.hh"

class ThreadContext;

namespace ArmISA
{

class SelfDebug;

class BrkPoint
{
  private:
    MiscRegIndex ctrlRegIndex;
    MiscRegIndex valRegIndex;
    MiscRegIndex xRegIndex;
    SelfDebug * conf;
    bool isCntxtAware;
    bool VMID16enabled;
    Addr active_pc;
    bool enable;
    int maxAddrSize;
    bool onUse;

  public:
    BrkPoint(MiscRegIndex _ctrlIndex, MiscRegIndex _valIndex,
             MiscRegIndex _xIndex, SelfDebug* _conf, bool _ctxAw, bool lva,
             bool vmid16, bool aarch32):
                ctrlRegIndex(_ctrlIndex), valRegIndex(_valIndex),
                xRegIndex(_xIndex), conf(_conf), isCntxtAware(_ctxAw),
                VMID16enabled(vmid16), active_pc(0x0), enable(false)
    {
        maxAddrSize = lva ? 52: 48 ;
        maxAddrSize = aarch32 ? 31 : maxAddrSize;
        onUse=false;
    }
    void setOnUse()
    {
        onUse = true;
    }
    void unsetOnUse()
    {
        onUse = false;
    }
    bool isSet()
    {
        return onUse;
    }
    bool testLinkedBk(ThreadContext *tc, Addr vaddr, ExceptionLevel el);
    bool test(ThreadContext *tc, Addr pc, ExceptionLevel el, DBGBCR ctr,
              bool from_link);

  protected:
    inline Addr getAddrfromReg(ThreadContext *tc)
    {
        return bits(tc->readMiscReg(valRegIndex), maxAddrSize, 2);
    }

    inline RegVal getContextfromReg(ThreadContext *tc, bool ctxid1)
    {
        if (ctxid1)
            return bits(tc->readMiscReg(valRegIndex), 31, 0);
        else
            return bits(tc->readMiscReg(valRegIndex), 63, 32);
    }


    inline uint32_t getVMIDfromReg(ThreadContext *tc);

  public:
    bool testAddrMatch(ThreadContext *tc, Addr pc, uint8_t bas);
    bool testAddrMissMatch(ThreadContext *tc, Addr pc, uint8_t bas);
    bool testContextMatch(ThreadContext *tc, bool ctx1);
    bool testVMIDMatch(ThreadContext *tc);
    const DBGBCR getControlReg(ThreadContext *tc)
    {
        const DBGBCR ctr = tc->readMiscReg(ctrlRegIndex);
        return ctr;
    }
    bool isEnabled(ThreadContext* tc, ExceptionLevel el,
                   uint8_t hmc, uint8_t ssc, uint8_t pmc);
    bool isActive(Addr vaddr)
    {
        if (vaddr==active_pc){
            active_pc = 0x0;
            return false;
        }else{
            active_pc = vaddr;
            return true;
        }
    }
    inline void updateControl(DBGBCR val)
    {
        enable = val.e == 0x1;
    }
    bool getEnable()
    {
        return enable;
    }

};

class WatchPoint
{
  private:
    MiscRegIndex ctrlRegIndex;
    MiscRegIndex valRegIndex;
    SelfDebug * conf;
    bool enable;
    int maxAddrSize;

    inline int getMaxAddrSize()
    {
        return maxAddrSize;
    }


  public:
    WatchPoint(MiscRegIndex _ctrlIndex, MiscRegIndex _valIndex,
               SelfDebug* _conf, bool lva, bool aarch32):
                ctrlRegIndex(_ctrlIndex),
                valRegIndex(_valIndex), conf(_conf), enable(false)
    {
        maxAddrSize = lva ? 52: 48 ;
        maxAddrSize = aarch32 ? 31 : maxAddrSize;
    }

    bool compareAddress(ThreadContext *tc, Addr in_addr,
                        uint8_t bas, uint8_t mask, unsigned size);

    inline Addr getAddrfromReg(ThreadContext *tc)
    {
       return bits(tc->readMiscReg(valRegIndex), maxAddrSize, 0);

    }

    inline bool isDoubleAligned(Addr addr)
    {
        return addr & 0x4;
    }

    inline void updateControl(DBGWCR val)
    {
        enable = val.e == 0x1;
    }
    bool getEnable()
    {
        return enable;
    }

    bool isEnabled(ThreadContext* tc, ExceptionLevel el, bool hmc,
                   uint8_t ssc, uint8_t pac);
    bool test(ThreadContext *tc, Addr addr, ExceptionLevel el, bool& wrt,
              bool atomic, unsigned size);
};

class SoftwareStep
{

  private:
    static const uint8_t INACTIVE_STATE = 0;
    static const uint8_t ACTIVE_PENDING_STATE = 1;
    static const uint8_t ACTIVE_NOT_PENDING_STATE = 2;


    bool bSS;
    int stateSS;
    SelfDebug * conf;
    bool steppedLdx;
    bool prevSteppedLdx;
    bool cpsrD;

    bool ctrStepped;
    bool ctrActivate;


  public:
    SoftwareStep(SelfDebug* s): bSS(false), stateSS(INACTIVE_STATE),
                                conf(s), steppedLdx(false) { }

    ~SoftwareStep() { }

    bool debugExceptionReturnSS(ThreadContext *tc, CPSR spsr,
                                ExceptionLevel dest, bool aarch32);
    bool advanceSS(ThreadContext * tc);

    inline void setCPSRD(bool val)
    {
        cpsrD = val;
    }

    inline void setEnableSS(bool val)
    {
        bSS = val;
    }

    void setLdx()
    {
        prevSteppedLdx = steppedLdx;
        steppedLdx = true;
    }

    void clearLdx()
    {
        prevSteppedLdx = steppedLdx;
        steppedLdx = false;
    }

    bool getLdx()
    {
        return prevSteppedLdx;
    }
};

class SelfDebug
{
  private:
    std::vector<BrkPoint> arBrkPoints;
    std::vector<WatchPoint> arWatchPoints;
    SoftwareStep * softStep;

    bool initialized;
    bool enableTdeTge; // MDCR_EL2.TDE || HCR_EL2.TGE

    // THIS is MDSCR_EL1.MDE in aarch64 and DBGDSCRext.MDBGen in aarch32
    bool enableFlag;

    bool bSDD; // MDCR_EL3.SDD
    bool bKDE; // MDSCR_EL1.KDE
    bool oslk; // OS lock flag

    bool aarch32; // updates with stage1 aarch64/32
    bool to32;

  public:
    SelfDebug(): initialized(false), enableTdeTge(false),
                 enableFlag(false), bSDD(false), bKDE(false), oslk(false)
    {
        softStep = new SoftwareStep(this);
    }

    ~SelfDebug()
    {
        delete softStep;
    }

    Fault testBreakPoints(ThreadContext *tc, Addr vaddr);
    Fault testWatchPoints(ThreadContext *tc, Addr vaddr, bool write,
                          bool atomic, unsigned size, bool cm);
    Fault testVectorCatch(ThreadContext *tc, Addr addr, ArmFault* flt);

    Fault triggerException(ThreadContext * tc, Addr vaddr);
    Fault triggerWatchpointException(ThreadContext *tc, Addr vaddr,
                                     bool write, bool cm);

    inline BrkPoint* getBrkPoint(uint8_t index)
    {
        return &arBrkPoints[index];
    }

    static inline bool
    securityStateMatch(ThreadContext *tc, uint8_t ssc, bool hmc)
    {
        switch(ssc)
        {
            case 0x0: return true;
            case 0x1: return !inSecureState(tc);
            case 0x2: return inSecureState(tc);
            case 0x3:
                {
                    bool b = hmc? true: inSecureState(tc);
                    return b;
                }
            default: panic("Unreachable value");
        }
        return false;
    }

    bool isDebugEnabledForEL64(ThreadContext *tc, ExceptionLevel el,
                             bool secure, bool mask);
    bool isDebugEnabledForEL32(ThreadContext *tc, ExceptionLevel el,
                             bool secure, bool mask);

    void activateDebug()
    {
        for (auto &p: arBrkPoints){
            p.unsetOnUse();
        }
    }

    inline bool isDebugEnabled(ThreadContext *tc)
    {
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        ExceptionLevel el = (ExceptionLevel) currEL(tc);
        if (aarch32){
            return isDebugEnabledForEL32(tc, el, inSecureState(tc),
                                         (bool)cpsr.d == 1);
        }else{
            return isDebugEnabledForEL64(tc, el, inSecureState(tc),
                                         (bool)cpsr.d == 1 );
        }
    }

    inline void setbSDD(RegVal val)
    {
        bSDD = bits(val, 16);
    }

    inline void setMDSCRvals(RegVal val)
    {
        enableFlag = bits(val, 15);
        bKDE = bits(val, 13);
        softStep->setEnableSS((bool)bits(val, 0));
    }

    inline void setMDBGen(RegVal val)
    {
        enableFlag = bits(val, 15);
    }

    inline void setenableTDETGE(HCR hcr, HDCR mdcr)
    {
        enableTdeTge = (mdcr.tde == 0x1 || hcr.tge == 0x1);
    }

    inline void updateOSLock(RegVal val)
    {
        oslk = bool(bits(val, 0));
    }

    inline void updateDBGBCR(int index, DBGBCR val)
    {
        arBrkPoints[index].updateControl(val);
    }

    inline void updateDBGWCR(int index, DBGWCR val)
    {
        arWatchPoints[index].updateControl(val);
    }

    inline void setDebugMask(bool mask)
    {
        softStep->setCPSRD(mask);
    }
    inline bool isAArch32()
    {
        return aarch32;
    }

    inline bool isTo32()
    {
        return to32;
    }
    inline void setAArch32(ThreadContext * tc)
    {
        ExceptionLevel fromEL = (ExceptionLevel) currEL(tc);
        if (fromEL == EL0)
            aarch32 = ELIs32(tc, EL0) && ELIs32(tc, EL1);
        else
            aarch32 = ELIs32(tc, fromEL);
        return;
    }
    SoftwareStep * getSstep()
    {
        return softStep;
    }


    bool targetAArch32(ThreadContext * tc)
    {
        ExceptionLevel ELd = debugTargetFrom(tc, inSecureState(tc));
        return ELIs32(tc, ELd) && aarch32;
    }

    void init(ThreadContext *tc)
    {
        if (initialized)
            return;
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        aarch32 = cpsr.width == 1;

        const AA64DFR0 dfr = tc->readMiscReg(MISCREG_ID_AA64DFR0_EL1);
        const AA64MMFR2 mm_fr2 = tc->readMiscReg(MISCREG_ID_AA64MMFR2_EL1);
        const AA64MMFR1 mm_fr1 = tc->readMiscReg(MISCREG_ID_AA64MMFR1_EL1);
        const uint8_t nCtxtAwareBp = dfr.ctx_cmps;
        const bool VMIDBits = mm_fr1.vmidbits;
        for (int i=0; i<=dfr.brps; i++) {
            const bool isctxaw = i>=(dfr.brps-nCtxtAwareBp);

            BrkPoint  bkp = BrkPoint((MiscRegIndex)(MISCREG_DBGBCR0_EL1+i),
                                     (MiscRegIndex)(MISCREG_DBGBVR0_EL1+i),
                                     (MiscRegIndex)(MISCREG_DBGBXVR0+i),
                                     this, isctxaw, (bool)mm_fr2.varange,
                                     VMIDBits, aarch32);
            const DBGBCR ctr = tc->readMiscReg(MISCREG_DBGBCR0_EL1+i);

            bkp.updateControl(ctr);
            arBrkPoints.push_back(bkp);
        }

        for (int i=0; i<=dfr.wrps; i++) {
            WatchPoint  wtp = WatchPoint((MiscRegIndex)(MISCREG_DBGWCR0+i),
                                         (MiscRegIndex)(MISCREG_DBGWVR0+i),
                                         this, (bool)mm_fr2.varange, aarch32);
            const DBGWCR ctr = tc->readMiscReg(MISCREG_DBGWCR0+i);

            wtp.updateControl(ctr);
            arWatchPoints.push_back(wtp);
        }

        initialized = true;

        RegVal oslar_el1 = tc->readMiscReg(MISCREG_OSLAR_EL1);
        updateOSLock(oslar_el1);
        // Initialize preloaded control booleans
        uint64_t mdscr_el1 = tc->readMiscReg(MISCREG_MDSCR_EL1);
        setMDSCRvals(mdscr_el1);

        const uint64_t mdcr_el3 = tc->readMiscReg(MISCREG_MDCR_EL3);
        setbSDD(mdcr_el3);

        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
        setenableTDETGE(hcr, mdcr);

    }
};

}
#endif
