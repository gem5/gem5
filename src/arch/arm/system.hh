/*
 * Copyright (c) 2010, 2012-2013, 2015-2020 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __ARCH_ARM_SYSTEM_HH__
#define __ARCH_ARM_SYSTEM_HH__

#include <memory>
#include <string>
#include <vector>

#include "kern/linux/events.hh"
#include "params/ArmSystem.hh"
#include "sim/full_system.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class GenericTimer;
class BaseGic;
class FVPBasePwrCtrl;
class ThreadContext;

class ArmSystem : public System
{
  protected:
    /**
     * True if this system implements the Security Extensions
     */
    const bool _haveSecurity;

    /**
     * True if this system implements the Large Physical Address Extension
     */
    const bool _haveLPAE;

    /**
     * True if this system implements the virtualization Extensions
     */
    const bool _haveVirtualization;

    /**
     * True if this system implements the Crypto Extension
     */
    const bool _haveCrypto;

    /**
     * Pointer to the Generic Timer wrapper.
     */
    GenericTimer *_genericTimer;
    BaseGic *_gic;

    /**
     * Pointer to the Power Controller (if any)
     */
    FVPBasePwrCtrl *_pwrCtrl;

    /**
     * Reset address (ARMv8)
     */
    Addr _resetAddr;

    /**
     * True if the register width of the highest implemented exception level is
     * 64 bits (ARMv8)
     */
    bool _highestELIs64;

    /**
     * Supported physical address range in bits if the highest implemented
     * exception level is 64 bits (ARMv8)
     */
    const uint8_t _physAddrRange64;

    /**
     * True if ASID is 16 bits in AArch64 (ARMv8)
     */
    const bool _haveLargeAsid64;

    /**
     * True if system implements the transactional memory extension (TME)
     */
    const bool _haveTME;

    /**
     * True if SVE is implemented (ARMv8)
     */
    const bool _haveSVE;

    /** SVE vector length at reset, in quadwords */
    const unsigned _sveVL;

    /**
     * True if LSE is implemented (ARMv8.1)
     */
    const bool _haveLSE;

    /** True if Priviledge Access Never is implemented */
    const unsigned _havePAN;

    /** True if Secure EL2 is implemented */
    const unsigned _haveSecEL2;

    /**
     * True if the Semihosting interface is enabled.
     */
    ArmSemihosting *const semihosting;

  public:
    static constexpr Addr PageBytes = ArmISA::PageBytes;
    static constexpr Addr PageShift = ArmISA::PageShift;

    typedef ArmSystemParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    ArmSystem(Params *p);

    /** true if this a multiprocessor system */
    bool multiProc;

    /** Returns true if this system implements the Security Extensions */
    bool haveSecurity() const { return _haveSecurity; }

    /** Returns true if this system implements the Large Physical Address
     * Extension */
    bool haveLPAE() const { return _haveLPAE; }

    /** Returns true if this system implements the virtualization
      * Extensions
      */
    bool haveVirtualization() const { return _haveVirtualization; }

    /** Returns true if this system implements the Crypto
      * Extension
      */
    bool haveCrypto() const { return _haveCrypto; }

    /** Sets the pointer to the Generic Timer. */
    void
    setGenericTimer(GenericTimer *generic_timer)
    {
        _genericTimer = generic_timer;
    }

    /** Sets the pointer to the GIC. */
    void setGIC(BaseGic *gic) { _gic = gic; }

    /** Sets the pointer to the Power Controller */
    void setPowerController(FVPBasePwrCtrl *pwr_ctrl)
    {
        _pwrCtrl = pwr_ctrl;
    }

    /** Get a pointer to the system's generic timer model */
    GenericTimer *getGenericTimer() const { return _genericTimer; }

    /** Get a pointer to the system's GIC */
    BaseGic *getGIC() const { return _gic; }

    /** Get a pointer to the system's power controller */
    FVPBasePwrCtrl *getPowerController() const { return _pwrCtrl; }

    /** Returns true if the register width of the highest implemented exception
     * level is 64 bits (ARMv8) */
    bool highestELIs64() const { return _highestELIs64; }

    /** Returns the highest implemented exception level */
    ArmISA::ExceptionLevel
    highestEL() const
    {
        if (_haveSecurity)
            return ArmISA::EL3;
        if (_haveVirtualization)
            return ArmISA::EL2;
        return ArmISA::EL1;
    }

    /** Returns the reset address if the highest implemented exception level is
     * 64 bits (ARMv8) */
    Addr resetAddr() const { return _resetAddr; }
    void setResetAddr(Addr addr) { _resetAddr = addr; }

    /** Returns true if ASID is 16 bits in AArch64 (ARMv8) */
    bool haveLargeAsid64() const { return _haveLargeAsid64; }

    /** Returns true if this system implements the transactional
      * memory extension (ARMv9)
      */
    bool haveTME() const { return _haveTME; }

    /** Returns true if SVE is implemented (ARMv8) */
    bool haveSVE() const { return _haveSVE; }

    /** Returns the SVE vector length at reset, in quadwords */
    unsigned sveVL() const { return _sveVL; }

    /** Returns true if LSE is implemented (ARMv8.1) */
    bool haveLSE() const { return _haveLSE; }

    /** Returns true if Priviledge Access Never is implemented */
    bool havePAN() const { return _havePAN; }

    /** Returns true if Priviledge Access Never is implemented */
    bool haveSecEL2() const { return _haveSecEL2; }

    /** Returns the supported physical address range in bits if the highest
     * implemented exception level is 64 bits (ARMv8) */
    uint8_t physAddrRange64() const { return _physAddrRange64; }

    /** Returns the supported physical address range in bits */
    uint8_t
    physAddrRange() const
    {
        if (_highestELIs64)
            return _physAddrRange64;
        if (_haveLPAE)
            return 40;
        return 32;
    }

    /** Returns the physical address mask */
    Addr physAddrMask() const { return mask(physAddrRange()); }

    /** Is Arm Semihosting support enabled? */
    bool haveSemihosting() const { return semihosting != nullptr; }

    /**
     * Returns a valid ArmSystem pointer if using ARM ISA, it fails
     * otherwise.
     */
    static ArmSystem*
    getArmSystem(ThreadContext *tc)
    {
        assert(FullSystem);
        return static_cast<ArmSystem *>(tc->getSystemPtr());
    }

    /** Returns true if the system of a specific thread context implements the
     * Security Extensions
     */
    static bool haveSecurity(ThreadContext *tc);

    /** Returns true if the system of a specific thread context implements the
     * virtualization Extensions
     */
    static bool haveVirtualization(ThreadContext *tc);

    /** Returns true if the system of a specific thread context implements the
     * Large Physical Address Extension
     */
    static bool haveLPAE(ThreadContext *tc);

    /** Returns true if the register width of the highest implemented exception
     * level for the system of a specific thread context is 64 bits (ARMv8)
     */
    static bool highestELIs64(ThreadContext *tc);

    /** Returns the highest implemented exception level for the system of a
     * specific thread context
     */
    static ArmISA::ExceptionLevel highestEL(ThreadContext *tc);

    /** Return true if the system implements a specific exception level */
    static bool haveEL(ThreadContext *tc, ArmISA::ExceptionLevel el);

    /** Returns true if the system of a specific thread context implements the
     * transactional memory extension (TME)
     */
    static bool haveTME(ThreadContext *tc);

    /** Returns the reset address if the highest implemented exception level
     * for the system of a specific thread context is 64 bits (ARMv8)
     */
    static Addr resetAddr(ThreadContext *tc);

    /** Returns the supported physical address range in bits for the system of a
     * specific thread context
     */
    static uint8_t physAddrRange(ThreadContext *tc);

    /** Returns the physical address mask for the system of a specific thread
     * context
     */
    static Addr physAddrMask(ThreadContext *tc);

    /** Returns true if ASID is 16 bits for the system of a specific thread
     * context while in AArch64 (ARMv8) */
    static bool haveLargeAsid64(ThreadContext *tc);

    /** Is Arm Semihosting support enabled? */
    static bool haveSemihosting(ThreadContext *tc);

    /** Make a Semihosting call from aarch64 */
    static bool callSemihosting64(ThreadContext *tc, bool gem5_ops=false);

    /** Make a Semihosting call from aarch32 */
    static bool callSemihosting32(ThreadContext *tc, bool gem5_ops=false);

    /** Make a Semihosting call from either aarch64 or aarch32 */
    static bool callSemihosting(ThreadContext *tc, bool gem5_ops=false);

    /** Make a call to notify the power controller of STANDBYWFI assertion */
    static void callSetStandByWfi(ThreadContext *tc);

    /** Make a call to notify the power controller of STANDBYWFI deassertion */
    static void callClearStandByWfi(ThreadContext *tc);

    /**
     * Notify the power controller of WAKEREQUEST assertion. Returns true
     * if WAKEREQUEST is enabled as a power-on mechanism, and the core is now
     * powered, false otherwise
     */
    static bool callSetWakeRequest(ThreadContext *tc);

    /** Notify the power controller of WAKEREQUEST deassertion */
    static void callClearWakeRequest(ThreadContext *tc);
};

#endif
