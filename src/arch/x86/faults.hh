/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "base/misc.hh"
#include "sim/faults.hh"

namespace X86ISA
{
    // Base class for all x86 "faults" where faults is in the m5 sense
    class X86FaultBase : public FaultBase
    {
      protected:
        const char * faultName;
        const char * mnem;
        uint64_t errorCode;

        X86FaultBase(const char * _faultName, const char * _mnem,
                uint64_t _errorCode = 0) :
            faultName(_faultName), mnem(_mnem), errorCode(_errorCode)
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

        void
        invoke(ThreadContext * tc)
        {
            panic("Unimplemented fault %s.\n", name());
        }
    };

    // Base class for x86 faults which behave as if the underlying instruction
    // didn't happen.
    class X86Fault : public X86FaultBase
    {
      protected:
        X86Fault(const char * name, const char * mnem,
                uint64_t _errorCode = 0) :
            X86FaultBase(name, mnem, _errorCode)
        {}
    };

    // Base class for x86 traps which behave as if the underlying instruction
    // completed.
    class X86Trap : public X86FaultBase
    {
      protected:
        X86Trap(const char * name, const char * mnem,
                uint64_t _errorCode = 0) :
            X86FaultBase(name, mnem, _errorCode)
        {}

#if FULL_SYSTEM
        void invoke(ThreadContext * tc);
#endif
    };

    // Base class for x86 aborts which seem to be catastrophic failures.
    class X86Abort : public X86FaultBase
    {
      protected:
        X86Abort(const char * name, const char * mnem,
                uint64_t _errorCode = 0) :
            X86FaultBase(name, mnem, _errorCode)
        {}

#if FULL_SYSTEM
        void invoke(ThreadContext * tc);
#endif
    };

    // Base class for x86 interrupts.
    class X86Interrupt : public X86FaultBase
    {
      protected:
        uint8_t vector;
        X86Interrupt(const char * name, const char * mnem, uint8_t _vector,
                uint64_t _errorCode = 0) :
            X86FaultBase(name, mnem, _errorCode), vector(_vector)
        {}

#if FULL_SYSTEM
        void invoke(ThreadContext * tc);
#endif
    };

    class UnimpInstFault : public FaultBase
    {
      public:
        const char * name() const
        {
            return "unimplemented_micro";
        }

        void invoke(ThreadContext * tc)
        {
            panic("Unimplemented instruction!");
        }
    };

    static inline Fault genMachineCheckFault()
    {
        panic("Machine check fault not implemented in x86!\n");
    }

    // Below is a summary of the interrupt/exception information in the
    // architecture manuals.

    // Class  |  Type    | vector |               Cause                 | mnem
    //------------------------------------------------------------------------
    //Contrib   Fault     0         Divide-by-Zero-Error                  #DE
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

    class DivideByZero : public X86Fault
    {
      public:
        DivideByZero() :
            X86Fault("Divide-by-Zero-Error", "#DE")
        {}
    };

    class DebugException : public X86FaultBase
    {
      public:
        DebugException() :
            X86FaultBase("Debug", "#DB")
        {}
    };

    class NonMaskableInterrupt : public X86Interrupt
    {
      public:
        NonMaskableInterrupt(uint8_t _vector) :
            X86Interrupt("Non Maskable Interrupt", "#NMI", _vector)
        {}
    };

    class Breakpoint : public X86Trap
    {
      public:
        Breakpoint() :
            X86Trap("Breakpoint", "#BP")
        {}
    };

    class OverflowTrap : public X86Trap
    {
      public:
        OverflowTrap() :
            X86Trap("Overflow", "#OF")
        {}
    };

    class BoundRange : public X86Fault
    {
      public:
        BoundRange() :
            X86Fault("Bound-Range", "#BR")
        {}
    };

    class InvalidOpcode : public X86Fault
    {
      public:
        InvalidOpcode() :
            X86Fault("Invalid-Opcode", "#UD")
        {}
    };

    class DeviceNotAvailable : public X86Fault
    {
      public:
        DeviceNotAvailable() :
            X86Fault("Device-Not-Available", "#NM")
        {}
    };

    class DoubleFault : public X86Abort
    {
      public:
        DoubleFault() :
            X86Abort("Double-Fault", "#DF")
        {}
    };

    class InvalidTSS : public X86Fault
    {
      public:
        InvalidTSS() :
            X86Fault("Invalid-TSS", "#TS")
        {}
    };

    class SegmentNotPresent : public X86Fault
    {
      public:
        SegmentNotPresent() :
            X86Fault("Segment-Not-Present", "#NP")
        {}
    };

    class StackFault : public X86Fault
    {
      public:
        StackFault() :
            X86Fault("Stack", "#SS")
        {}
    };

    class GeneralProtection : public X86Fault
    {
      public:
        GeneralProtection(uint64_t _errorCode) :
            X86Fault("General-Protection", "#GP", _errorCode)
        {}
    };

    class PageFault : public X86Fault
    {
      public:
        PageFault() :
            X86Fault("Page-Fault", "#PF")
        {}
    };

    class X87FpExceptionPending : public X86Fault
    {
      public:
        X87FpExceptionPending() :
            X86Fault("x87 Floating-Point Exception Pending", "#MF")
        {}
    };

    class AlignmentCheck : public X86Fault
    {
      public:
        AlignmentCheck() :
            X86Fault("Alignment-Check", "#AC")
        {}
    };

    class MachineCheck : public X86Abort
    {
      public:
        MachineCheck() :
            X86Abort("Machine-Check", "#MC")
        {}
    };

    class SIMDFloatingPointFault : public X86Fault
    {
      public:
        SIMDFloatingPointFault() :
            X86Fault("SIMD Floating-Point", "#XF")
        {}
    };

    class SecurityException : public X86FaultBase
    {
      public:
        SecurityException() :
            X86FaultBase("Security Exception", "#SX")
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
        uint8_t vector;
      public:
        InitInterrupt(uint8_t _vector) :
            X86Interrupt("INIT Interrupt", "#INIT", _vector)
        {}
    };

    class SoftwareInterrupt : public X86Interrupt
    {
      public:
        SoftwareInterrupt(uint8_t _vector) :
            X86Interrupt("Software Interrupt", "INTn", _vector)
        {}
    };

    // These faults aren't part of the ISA definition. They trigger filling
    // the tlb on a miss and are to take the place of a hardware table walker.
    class FakeITLBFault : public X86Fault
    {
      protected:
        Addr vaddr;
      public:
        FakeITLBFault(Addr _vaddr) :
            X86Fault("fake instruction tlb fault", "itlb"),
            vaddr(_vaddr)
        {}

        void invoke(ThreadContext * tc);
    };

    class FakeDTLBFault : public X86Fault
    {
      protected:
        Addr vaddr;
      public:
        FakeDTLBFault(Addr _vaddr) :
            X86Fault("fake data tlb fault", "dtlb"),
            vaddr(_vaddr)
        {}

        void invoke(ThreadContext * tc);
    };
};

#endif // __ARCH_X86_FAULTS_HH__
