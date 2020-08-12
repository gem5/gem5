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
#include "arch/generic/tlb.hh"
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
    SelfDebug * conf;
    bool isCntxtAware;
    bool VMID16enabled;
    Addr activePc;
    bool enable;
    int maxAddrSize;
    bool onUse;

  public:
    friend class SelfDebug;

    BrkPoint(MiscRegIndex ctrl_index, MiscRegIndex val_index,
             SelfDebug* _conf, bool ctx_aw, bool lva,
             bool vmid16, bool aarch32):
                ctrlRegIndex(ctrl_index), valRegIndex(val_index),
                conf(_conf), isCntxtAware(ctx_aw),
                VMID16enabled(vmid16), activePc(0x0), enable(false)
    {
        maxAddrSize = lva ? 52: 48 ;
        maxAddrSize = aarch32 ? 31 : maxAddrSize;
        onUse = false;
    }

    bool testLinkedBk(ThreadContext *tc, Addr vaddr, ExceptionLevel el);
    bool test(ThreadContext *tc, Addr pc, ExceptionLevel el, DBGBCR ctr,
              bool from_link);

  protected:
    inline Addr
    getAddrfromReg(ThreadContext *tc) const
    {
        return bits(tc->readMiscReg(valRegIndex), maxAddrSize, 2);
    }

    inline RegVal
    getContextfromReg(ThreadContext *tc, bool ctxid1) const
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
    bool testContextMatch(ThreadContext *tc, bool ctx1, bool low_ctx);
    bool testContextMatch(ThreadContext *tc, bool ctx1);
    bool testVMIDMatch(ThreadContext *tc);

    const DBGBCR
    getControlReg(ThreadContext *tc)
    {
        return tc->readMiscReg(ctrlRegIndex);
    }

    bool isEnabled(ThreadContext* tc, ExceptionLevel el,
                   uint8_t hmc, uint8_t ssc, uint8_t pmc);

    bool
    isActive(Addr vaddr)
    {
        if (vaddr == activePc) {
            activePc = 0x0;
            return false;
        } else {
            activePc = vaddr;
            return true;
        }
    }

    inline void
    updateControl(DBGBCR val)
    {
        enable = val.e == 0x1;
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

  public:
    friend class SelfDebug;

    WatchPoint(MiscRegIndex ctrl_index, MiscRegIndex val_index,
               SelfDebug* _conf, bool lva, bool aarch32) :
               ctrlRegIndex(ctrl_index),
               valRegIndex(val_index), conf(_conf), enable(false)
    {
        maxAddrSize = lva ? 52: 48 ;
        maxAddrSize = aarch32 ? 31 : maxAddrSize;
    }

    bool compareAddress(ThreadContext *tc, Addr in_addr,
                        uint8_t bas, uint8_t mask, unsigned size);

    inline Addr
    getAddrfromReg(ThreadContext *tc)
    {
        return bits(tc->readMiscReg(valRegIndex), maxAddrSize, 0);
    }

    inline bool
    isDoubleAligned(Addr addr)
    {
        return addr & 0x4;
    }

    inline void
    updateControl(DBGWCR val)
    {
        enable = val.e == 0x1;
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
    SelfDebug *conf;
    bool steppedLdx;
    bool prevSteppedLdx;
    bool cpsrD;

  public:
    friend class SelfDebug;

    SoftwareStep(SelfDebug *s)
      : bSS(false), stateSS(INACTIVE_STATE),
        conf(s), steppedLdx(false)
    {}

    bool debugExceptionReturnSS(ThreadContext *tc, CPSR spsr,
                                ExceptionLevel dest);
    bool advanceSS(ThreadContext *tc);

    void
    setLdx()
    {
        prevSteppedLdx = steppedLdx;
        steppedLdx = true;
    }

    void
    clearLdx()
    {
        prevSteppedLdx = steppedLdx;
        steppedLdx = false;
    }

    bool
    getLdx() const
    {
        return prevSteppedLdx;
    }
};

class VectorCatch
{
  private:
    bool vcmatch;
    SelfDebug *conf;
    std::vector<Fault *> vectorTypes();

  public:
    VectorCatch(bool _vcmatch, SelfDebug* s) : vcmatch(_vcmatch), conf(s)
    {}

    bool addressMatching(ThreadContext *tc, Addr addr, ExceptionLevel el);
    bool exceptionTrapping(ThreadContext *tc, ExceptionLevel el,
                           ArmFault* fault);

    bool isVCMatch() const { return vcmatch; }

  private:
    Addr
    getVectorBase(ThreadContext *tc, bool monitor)
    {
        if (monitor) {
            return tc->readMiscReg(MISCREG_MVBAR) & ~0x1F;
        }
        SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
        if (sctlr.v) {
            return (Addr) 0xFFFF0000;
        } else {
            Addr vbar = tc->readMiscReg(MISCREG_VBAR) & ~0x1F;
            return vbar;
        }
    }

};

class SelfDebug
{
  private:
    std::vector<BrkPoint> arBrkPoints;
    std::vector<WatchPoint> arWatchPoints;
    SoftwareStep * softStep;
    VectorCatch * vcExcpt;

    bool initialized;
    bool enableTdeTge; // MDCR_EL2.TDE || HCR_EL2.TGE

    bool mde; // MDSCR_EL1.MDE, DBGDSCRext.MDBGen
    bool sdd; // MDCR_EL3.SDD
    bool kde; // MDSCR_EL1.KDE
    bool oslk; // OS lock flag

    bool aarch32; // updates with stage1 aarch64/32
    bool to32;

  public:
    SelfDebug()
      : initialized(false), enableTdeTge(false),
        mde(false), sdd(false), kde(false), oslk(false)
    {
        softStep = new SoftwareStep(this);
    }

    ~SelfDebug()
    {
        delete softStep;
        delete vcExcpt;
    }

    Fault testDebug(ThreadContext *tc, const RequestPtr &req,
                    BaseTLB::Mode mode);

  protected:
    Fault testBreakPoints(ThreadContext *tc, Addr vaddr);
    Fault testWatchPoints(ThreadContext *tc, Addr vaddr, bool write,
                          bool atomic, unsigned size, bool cm);

    Fault triggerException(ThreadContext * tc, Addr vaddr);
    Fault triggerWatchpointException(ThreadContext *tc, Addr vaddr,
                                     bool write, bool cm);
  public:
    Fault testVectorCatch(ThreadContext *tc, Addr addr, ArmFault* flt);

    bool enabled() const { return mde || softStep->bSS; };

    inline BrkPoint*
    getBrkPoint(uint8_t index)
    {
        return &arBrkPoints[index];
    }

    static inline bool
    securityStateMatch(ThreadContext *tc, uint8_t ssc, bool hmc)
    {
        switch (ssc) {
            case 0x0: return true;
            case 0x1: return !isSecure(tc);
            case 0x2: return isSecure(tc);
            case 0x3:
                {
                    bool b = hmc? true: isSecure(tc);
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

    void
    activateDebug()
    {
        for (auto &p: arBrkPoints){
            p.onUse = false;
        }
    }

    inline bool
    isDebugEnabled(ThreadContext *tc)
    {
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        ExceptionLevel el = (ExceptionLevel) currEL(tc);
        if (aarch32) {
            return isDebugEnabledForEL32(tc, el, isSecure(tc),
                                         (bool)cpsr.d == 1);
        } else {
            return isDebugEnabledForEL64(tc, el, isSecure(tc),
                                         (bool)cpsr.d == 1 );
        }
    }

    inline void
    setbSDD(RegVal val)
    {
        sdd = bits(val, 16);
    }

    inline void
    setMDSCRvals(RegVal val)
    {
        mde = bits(val, 15);
        kde = bits(val, 13);
        softStep->bSS = bits(val, 0);
    }

    inline void
    setMDBGen(RegVal val)
    {
        mde = bits(val, 15);
    }

    inline void
    setenableTDETGE(HCR hcr, HDCR mdcr)
    {
        enableTdeTge = (mdcr.tde == 0x1 || hcr.tge == 0x1);
    }

    inline void
    updateOSLock(RegVal val)
    {
        oslk = bool(bits(val, 0));
    }

    inline void
    updateDBGBCR(int index, DBGBCR val)
    {
        arBrkPoints[index].updateControl(val);
    }

    inline void
    updateDBGWCR(int index, DBGWCR val)
    {
        arWatchPoints[index].updateControl(val);
    }

    inline void
    setDebugMask(bool mask)
    {
        softStep->cpsrD = mask;
    }

    inline bool
    isAArch32() const
    {
        return aarch32;
    }

    inline void
    setAArch32(ThreadContext *tc)
    {
        ExceptionLevel from_el = (ExceptionLevel) currEL(tc);
        if (from_el == EL0)
            aarch32 = ELIs32(tc, EL0) && ELIs32(tc, EL1);
        else
            aarch32 = ELIs32(tc, from_el);
        return;
    }

    SoftwareStep *
    getSstep()
    {
        return softStep;
    }

    VectorCatch*
    getVectorCatch(ThreadContext *tc)
    {
        if (!initialized)
            init(tc);
        return vcExcpt;
    }

    bool
    targetAArch32(ThreadContext *tc)
    {
        ExceptionLevel ELd = debugTargetFrom(tc, isSecure(tc));
        return ELIs32(tc, ELd) && aarch32;
    }

    void init(ThreadContext *tc);
};

}
#endif
