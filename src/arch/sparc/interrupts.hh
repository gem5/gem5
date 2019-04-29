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
 * Authors: Ali Saidi
 *          Lisa Hsu
 */

#ifndef __ARCH_SPARC_INTERRUPT_HH__
#define __ARCH_SPARC_INTERRUPT_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/registers.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"
#include "params/SparcInterrupts.hh"
#include "sim/sim_object.hh"

namespace SparcISA
{

enum InterruptTypes
{
    IT_TRAP_LEVEL_ZERO,
    IT_HINTP,
    IT_INT_VEC,
    IT_CPU_MONDO,
    IT_DEV_MONDO,
    IT_RES_ERROR,
    IT_SOFT_INT,
    NumInterruptTypes
};

class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;

    uint64_t interrupts[NumInterruptTypes];
    uint64_t intStatus;

  public:

    void
    setCPU(BaseCPU * _cpu)
    {
        cpu = _cpu;
    }

    typedef SparcInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p) : SimObject(p), cpu(NULL)
    {
        clearAll();
    }

    int
    InterruptLevel(uint64_t softint)
    {
        if (softint & 0x10000 || softint & 0x1)
            return 14;

        int level = 15;
        while (level > 0 && !(1 << level & softint))
            level--;
        if (1 << level & softint)
            return level;
        return 0;
    }

    void
    post(int int_num, int index)
    {
        DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);
        assert(int_num >= 0 && int_num < NumInterruptTypes);
        assert(index >= 0 && index < 64);

        interrupts[int_num] |= ULL(1) << index;
        intStatus |= ULL(1) << int_num;
    }

    void
    clear(int int_num, int index)
    {
        DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);
        assert(int_num >= 0 && int_num < NumInterruptTypes);
        assert(index >= 0 && index < 64);

        interrupts[int_num] &= ~(ULL(1) << index);
        if (!interrupts[int_num])
            intStatus &= ~(ULL(1) << int_num);
    }

    void
    clearAll()
    {
        for (int i = 0; i < NumInterruptTypes; ++i) {
            interrupts[i] = 0;
        }
        intStatus = 0;
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        if (!intStatus)
            return false;

        HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);
        PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);

        // THESE ARE IN ORDER OF PRIORITY
        // since there are early returns, and the highest
        // priority interrupts should get serviced,
        // it is v. important that new interrupts are inserted
        // in the right order of processing
        if (hpstate.hpriv) {
            if (pstate.ie) {
                if (interrupts[IT_HINTP]) {
                    // This will be cleaned by a HINTP write
                    return true;
                }
                if (interrupts[IT_INT_VEC]) {
                    // this will be cleared by an ASI read (or write)
                    return true;
                }
            }
        } else {
            if (interrupts[IT_TRAP_LEVEL_ZERO]) {
                    // this is cleared by deasserting HPSTATE::tlz
                return true;
            }
            // HStick matches always happen in priv mode (ie doesn't matter)
            if (interrupts[IT_HINTP]) {
                return true;
            }
            if (interrupts[IT_INT_VEC]) {
                // this will be cleared by an ASI read (or write)
                return true;
            }
            if (pstate.ie) {
                if (interrupts[IT_CPU_MONDO]) {
                    return true;
                }
                if (interrupts[IT_DEV_MONDO]) {
                    return true;
                }
                if (interrupts[IT_SOFT_INT]) {
                    return true;
                }

                if (interrupts[IT_RES_ERROR]) {
                    return true;
                }
            } // !hpriv && pstate.ie
        }  // !hpriv

        return false;
    }

    Fault
    getInterrupt(ThreadContext *tc)
    {
        assert(checkInterrupts(tc));

        HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);
        PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);

        // THESE ARE IN ORDER OF PRIORITY
        // since there are early returns, and the highest
        // priority interrupts should get serviced,
        // it is v. important that new interrupts are inserted
        // in the right order of processing
        if (hpstate.hpriv) {
            if (pstate.ie) {
                if (interrupts[IT_HINTP]) {
                    // This will be cleaned by a HINTP write
                    return std::make_shared<HstickMatch>();
                }
                if (interrupts[IT_INT_VEC]) {
                    // this will be cleared by an ASI read (or write)
                    return std::make_shared<InterruptVector>();
                }
            }
        } else {
            if (interrupts[IT_TRAP_LEVEL_ZERO]) {
                    // this is cleared by deasserting HPSTATE::tlz
                return std::make_shared<TrapLevelZero>();
            }
            // HStick matches always happen in priv mode (ie doesn't matter)
            if (interrupts[IT_HINTP]) {
                return std::make_shared<HstickMatch>();
            }
            if (interrupts[IT_INT_VEC]) {
                // this will be cleared by an ASI read (or write)
                return std::make_shared<InterruptVector>();
            }
            if (pstate.ie) {
                if (interrupts[IT_CPU_MONDO]) {
                    return std::make_shared<CpuMondo>();
                }
                if (interrupts[IT_DEV_MONDO]) {
                    return std::make_shared<DevMondo>();
                }
                if (interrupts[IT_SOFT_INT]) {
                    int level = InterruptLevel(interrupts[IT_SOFT_INT]);
                    return std::make_shared<InterruptLevelN>(level);
                }

                if (interrupts[IT_RES_ERROR]) {
                    return std::make_shared<ResumableError>();
                }
            } // !hpriv && pstate.ie
        }  // !hpriv
        return NoFault;
    }

    void
    updateIntrInfo(ThreadContext *tc)
    {}

    uint64_t
    get_vec(int int_num)
    {
        assert(int_num >= 0 && int_num < NumInterruptTypes);
        return interrupts[int_num];
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        SERIALIZE_ARRAY(interrupts,NumInterruptTypes);
        SERIALIZE_SCALAR(intStatus);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        UNSERIALIZE_ARRAY(interrupts,NumInterruptTypes);
        UNSERIALIZE_SCALAR(intStatus);
    }
};
} // namespace SPARC_ISA

#endif // __ARCH_SPARC_INTERRUPT_HH__
