/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_FAULTS_HH__
#define __ARCH_X86_FAULTS_HH__

#include <string>

#include "arch/x86/tlb.hh"
#include "base/bitunion.hh"
#include "base/logging.hh"
#include "sim/faults.hh"

namespace X86ISA
{
    // Base class for all x86 "faults" where faults is in the m5 sense
    class X86FaultBase : public FaultBase
    {
      protected:
        const char * faultName;
        const char * mnem;
        uint8_t vector;
        uint64_t errorCode;

        X86FaultBase(const char * _faultName, const char * _mnem,
                     const uint8_t _vector, uint64_t _errorCode = (uint64_t)-1)
            : faultName(_faultName), mnem(_mnem),
              vector(_vector), errorCode(_errorCode)
        {
        }

        const char * name() const
        {
            return faultName;
        }

        virtual bool isBenign()
        {
            return true;
        }

        virtual const char * mnemonic() const
        {
            return mnem;
        }

        virtual bool isSoft()
        {
            return false;
        }

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);

        virtual std::string describe() const;

      public:
        /**
         * Get the vector of an interrupt.
         *
         * @return interrupt vector number.
         */
        virtual uint8_t getVector() const { return vector; }
    };

    // Base class for x86 faults which behave as if the underlying instruction
    // didn't happen.
    class X86Fault : public X86FaultBase
    {
      protected:
        X86Fault(const char * name, const char * mnem,
                 const uint8_t vector, uint64_t _errorCode = (uint64_t)-1)
            : X86FaultBase(name, mnem, vector, _errorCode)
        {}
    };

    // Base class for x86 traps which behave as if the underlying instruction
    // completed.
    class X86Trap : public X86FaultBase
    {
      protected:
        X86Trap(const char * name, const char * mnem,
                const uint8_t vector, uint64_t _errorCode = (uint64_t)-1)
            : X86FaultBase(name, mnem, vector, _errorCode)
        {}

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);
    };

    // Base class for x86 aborts which seem to be catastrophic failures.
    class X86Abort : public X86FaultBase
    {
      protected:
        X86Abort(const char * name, const char * mnem,
                const uint8_t vector, uint64_t _errorCode = (uint64_t)-1)
            : X86FaultBase(name, mnem, vector, _errorCode)
        {}

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);
    };

    // Base class for x86 interrupts.
    class X86Interrupt : public X86FaultBase
    {
      protected:
        X86Interrupt(const char * name, const char * mnem,
                const uint8_t _vector, uint64_t _errorCode = (uint64_t)-1)
            : X86FaultBase(name, mnem, _vector, _errorCode)
        {}
    };

    class UnimpInstFault : public FaultBase
    {
      public:
        const char * name() const
        {
            return "unimplemented_micro";
        }

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr)
        {
            panic("Unimplemented instruction!");
        }
    };

    // Below is a summary of the interrupt/exception information in the
    // architecture manuals.

    // Class  |  Type    | vector |               Cause                 | mnem
    //------------------------------------------------------------------------
    //Contrib   Fault     0         Divide Error                          #DE
    //Benign    Either    1         Debug                                 #DB
    //Benign    Interrupt 2         Non-Maskable-Interrupt                #NMI
    //Benign    Trap      3         Breakpoint                            #BP
    //Benign    Trap      4         Overflow                              #OF
    //Benign    Fault     5         Bound-Range                           #BR
    //Benign    Fault     6         Invalid-Opcode                        #UD
    //Benign    Fault     7         Device-Not-Available                  #NM
    //Benign    Abort     8         Double-Fault                          #DF
    //                    9         Coprocessor-Segment-Overrun
    //Contrib   Fault     10        Invalid-TSS                           #TS
    //Contrib   Fault     11        Segment-Not-Present                   #NP
    //Contrib   Fault     12        Stack                                 #SS
    //Contrib   Fault     13        General-Protection                    #GP
    //Either    Fault     14        Page-Fault                            #PF
    //                    15        Reserved
    //Benign    Fault     16        x87 Floating-Point Exception Pending  #MF
    //Benign    Fault     17        Alignment-Check                       #AC
    //Benign    Abort     18        Machine-Check                         #MC
    //Benign    Fault     19        SIMD Floating-Point                   #XF
    //                    20-29     Reserved
    //Contrib   ?         30        Security Exception                    #SX
    //                    31        Reserved
    //Benign    Interrupt 0-255     External Interrupts                   #INTR
    //Benign    Interrupt 0-255     Software Interrupts                   INTn

    // Note that
    class DivideError : public X86Fault
    {
      public:
        DivideError() :
            X86Fault("Divide-Error", "#DE", 0)
        {}
    };

    class DebugException : public X86FaultBase
    {
      public:
        DebugException() :
            X86FaultBase("Debug", "#DB", 1)
        {}
    };

    class NonMaskableInterrupt : public X86Interrupt
    {
      public:
        NonMaskableInterrupt(uint8_t _vector) :
            X86Interrupt("Non Maskable Interrupt", "#NMI", 2, _vector)
        {}
    };

    class Breakpoint : public X86Trap
    {
      public:
        Breakpoint() :
            X86Trap("Breakpoint", "#BP", 3)
        {}
    };

    class OverflowTrap : public X86Trap
    {
      public:
        OverflowTrap() :
            X86Trap("Overflow", "#OF", 4)
        {}
    };

    class BoundRange : public X86Fault
    {
      public:
        BoundRange() :
            X86Fault("Bound-Range", "#BR", 5)
        {}
    };

    class InvalidOpcode : public X86Fault
    {
      public:
        InvalidOpcode() :
            X86Fault("Invalid-Opcode", "#UD", 6)
        {}

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);
    };

    class DeviceNotAvailable : public X86Fault
    {
      public:
        DeviceNotAvailable() :
            X86Fault("Device-Not-Available", "#NM", 7)
        {}
    };

    class DoubleFault : public X86Abort
    {
      public:
        DoubleFault() :
            X86Abort("Double-Fault", "#DF", 8, 0)
        {}
    };

    class InvalidTSS : public X86Fault
    {
      public:
        InvalidTSS(uint32_t _errorCode) :
            X86Fault("Invalid-TSS", "#TS", 10, _errorCode)
        {}
    };

    class SegmentNotPresent : public X86Fault
    {
      public:
        SegmentNotPresent(uint32_t _errorCode) :
            X86Fault("Segment-Not-Present", "#NP", 11, _errorCode)
        {}
    };

    class StackFault : public X86Fault
    {
      public:
        StackFault(uint32_t _errorCode) :
            X86Fault("Stack", "#SS", 12, _errorCode)
        {}
    };

    class GeneralProtection : public X86Fault
    {
      public:
        GeneralProtection(uint32_t _errorCode) :
            X86Fault("General-Protection", "#GP", 13, _errorCode)
        {}
    };

    class PageFault : public X86Fault
    {
      protected:
        BitUnion32(PageFaultErrorCode)
            Bitfield<0> present;
            Bitfield<1> write;
            Bitfield<2> user;
            Bitfield<3> reserved;
            Bitfield<4> fetch;
        EndBitUnion(PageFaultErrorCode)

        Addr addr;

      public:
        PageFault(Addr _addr, uint32_t _errorCode) :
            X86Fault("Page-Fault", "#PF", 14, _errorCode), addr(_addr)
        {}

        PageFault(Addr _addr, bool present, BaseTLB::Mode mode,
                bool user, bool reserved) :
            X86Fault("Page-Fault", "#PF", 14, 0), addr(_addr)
        {
            PageFaultErrorCode code = 0;
            code.present = present;
            code.write = (mode == BaseTLB::Write);
            code.user = user;
            code.reserved = reserved;
            code.fetch = (mode == BaseTLB::Execute);
            errorCode = code;
        }

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);

        virtual std::string describe() const;
    };

    class X87FpExceptionPending : public X86Fault
    {
      public:
        X87FpExceptionPending() :
            X86Fault("x87 Floating-Point Exception Pending", "#MF", 16)
        {}
    };

    class AlignmentCheck : public X86Fault
    {
      public:
        AlignmentCheck() :
            X86Fault("Alignment-Check", "#AC", 17, 0)
        {}
    };

    class MachineCheck : public X86Abort
    {
      public:
        MachineCheck() :
            X86Abort("Machine-Check", "#MC", 18)
        {}
    };

    class SIMDFloatingPointFault : public X86Fault
    {
      public:
        SIMDFloatingPointFault() :
            X86Fault("SIMD Floating-Point", "#XF", 19)
        {}
    };

    class SecurityException : public X86FaultBase
    {
      public:
        SecurityException() :
            X86FaultBase("Security Exception", "#SX", 30)
        {}
    };

    class ExternalInterrupt : public X86Interrupt
    {
      public:
        ExternalInterrupt(uint8_t _vector) :
            X86Interrupt("External Interrupt", "#INTR", _vector)
        {}
    };

    class SystemManagementInterrupt : public X86Interrupt
    {
      public:
        SystemManagementInterrupt() :
            X86Interrupt("System Management Interrupt", "#SMI", 0)
        {}
    };

    class InitInterrupt : public X86Interrupt
    {
      public:
        InitInterrupt(uint8_t _vector) :
            X86Interrupt("INIT Interrupt", "#INIT", _vector)
        {}

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);
    };

    class StartupInterrupt : public X86Interrupt
    {
      public:
        StartupInterrupt(uint8_t _vector) :
            X86Interrupt("Startup Interrupt", "#SIPI", _vector)
        {}

        void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                    StaticInst::nullStaticInstPtr);
    };

    class SoftwareInterrupt : public X86Interrupt
    {
      public:
        SoftwareInterrupt(uint8_t _vector) :
            X86Interrupt("Software Interrupt", "#INTR", _vector)
        {}

        bool isSoft()
        {
            return true;
        }
    };
}

#endif // __ARCH_X86_FAULTS_HH__
