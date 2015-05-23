/*
 * Copyright (c) 2010, 2012-2013 ARM Limited
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
 *
 * Authors: Ali Saidi
 */

#ifndef __ARCH_ARM_SYSTEM_HH__
#define __ARCH_ARM_SYSTEM_HH__

#include <string>
#include <vector>

#include "kern/linux/events.hh"
#include "params/ArmSystem.hh"
#include "params/GenericArmSystem.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class GenericTimer;
class ThreadContext;

class ArmSystem : public System
{
  protected:
    /**
     * PC based event to skip the dprink() call and emulate its
     * functionality
     */
    Linux::DebugPrintkEvent *debugPrintkEvent;

    /**
     * Pointer to the bootloader object
     */
    ObjectFile *bootldr;

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
     * Pointer to the Generic Timer wrapper.
     */
    GenericTimer *_genericTimer;

    /**
     * True if the register width of the highest implemented exception level is
     * 64 bits (ARMv8)
     */
    bool _highestELIs64;

    /**
     * Reset address if the highest implemented exception level is 64 bits
     * (ARMv8)
     */
    const Addr _resetAddr64;

    /**
     * Supported physical address range in bits if the highest implemented
     * exception level is 64 bits (ARMv8)
     */
    const uint8_t _physAddrRange64;

    /**
     * True if ASID is 16 bits in AArch64 (ARMv8)
     */
    const bool _haveLargeAsid64;

  public:
    typedef ArmSystemParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    ArmSystem(Params *p);
    ~ArmSystem();

    /**
     * Initialise the system
     */
    virtual void initState();

    virtual Addr fixFuncEventAddr(Addr addr)
    {
        // Remove the low bit that thumb symbols have set
        // but that aren't actually odd aligned
        if (addr & 0x1)
            return addr & ~1;
        return addr;
    }

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

    /** Sets the pointer to the Generic Timer. */
    void setGenericTimer(GenericTimer *generic_timer)
    {
        _genericTimer = generic_timer;
    }

    /** Get a pointer to the system's generic timer model */
    GenericTimer *getGenericTimer() const { return _genericTimer; }

    /** Returns true if the register width of the highest implemented exception
     * level is 64 bits (ARMv8) */
    bool highestELIs64() const { return _highestELIs64; }

    /** Returns the highest implemented exception level */
    ExceptionLevel highestEL() const
    {
        if (_haveSecurity)
            return EL3;
        // @todo: uncomment this to enable Virtualization
        // if (_haveVirtualization)
        //     return EL2;
        return EL1;
    }

    /** Returns the reset address if the highest implemented exception level is
     * 64 bits (ARMv8) */
    Addr resetAddr64() const { return _resetAddr64; }

    /** Returns true if ASID is 16 bits in AArch64 (ARMv8) */
    bool haveLargeAsid64() const { return _haveLargeAsid64; }

    /** Returns the supported physical address range in bits if the highest
     * implemented exception level is 64 bits (ARMv8) */
    uint8_t physAddrRange64() const { return _physAddrRange64; }

    /** Returns the supported physical address range in bits */
    uint8_t physAddrRange() const
    {
        if (_highestELIs64)
            return _physAddrRange64;
        if (_haveLPAE)
            return 40;
        return 32;
    }

    /** Returns the physical address mask */
    Addr physAddrMask() const
    {
        return mask(physAddrRange());
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
    static ExceptionLevel highestEL(ThreadContext *tc);

    /** Returns the reset address if the highest implemented exception level for
     * the system of a specific thread context is 64 bits (ARMv8)
     */
    static Addr resetAddr64(ThreadContext *tc);

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
};

class GenericArmSystem : public ArmSystem
{
  public:
    typedef GenericArmSystemParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    GenericArmSystem(Params *p) : ArmSystem(p) {};
    virtual ~GenericArmSystem() {};

    /**
     * Initialise the system
     */
    virtual void initState();
};

#endif
