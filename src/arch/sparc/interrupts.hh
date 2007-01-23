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
 */

#ifndef __ARCH_SPARC_INTERRUPT_HH__
#define __ARCH_SPARC_INTERRUPT_HH__

#include "arch/sparc/faults.hh"
#include "cpu/thread_context.hh"

namespace SparcISA
{

enum interrupts_t {
    trap_level_zero,
    hstick_match,
    interrupt_vector,
    cpu_mondo,
    dev_mondo,
    resumable_error,
    soft_interrupt,
    num_interrupt_types
};

    class Interrupts
    {

      private:

        bool interrupts[num_interrupt_types];
        int numPosted;

      public:
        Interrupts()
        {
            for (int i = 0; i < num_interrupt_types; ++i) {
                interrupts[i] = false;
            }
            numPosted = 0;
        }

        void post(int int_type)
        {
            if (int_type < 0 || int_type >= num_interrupt_types)
                panic("posting unknown interrupt!\n");
            interrupts[int_type] = true;
            ++numPosted;
        }

        void post(int int_num, int index)
        {

        }

        void clear(int int_num, int index)
        {

        }

        void clear_all()
        {

        }

        bool check_interrupts(ThreadContext * tc) const
        {
            if (numPosted)
                return true;
            else
                return false;
        }

        Fault getInterrupt(ThreadContext * tc)
        {
            int hpstate = tc->readMiscReg(MISCREG_HPSTATE);
            int pstate = tc->readMiscReg(MISCREG_PSTATE);
            bool ie = pstate & PSTATE::ie;

            // THESE ARE IN ORDER OF PRIORITY
            // since there are early returns, and the highest
            // priority interrupts should get serviced,
            // it is v. important that new interrupts are inserted
            // in the right order of processing
            if (hpstate & HPSTATE::hpriv) {
                if (ie) {
                    if (interrupts[hstick_match]) {
                        if (tc->readMiscReg(MISCREG_HINTP) & 1) {
                            interrupts[hstick_match] = false;
                            --numPosted;
                            return new HstickMatch;
                        }
                    }
                    if (interrupts[interrupt_vector]) {
                        interrupts[interrupt_vector] = false;
                        --numPosted;
                        //HAVEN'T IMPLed THIS YET
                        return NoFault;
                    }
                } else {
                    if (interrupts[hstick_match]) {
                        return NoFault;
                    }

                }
            } else {
                if (interrupts[trap_level_zero]) {
                    if ((pstate & HPSTATE::tlz) && (tc->readMiscReg(MISCREG_TL) == 0)) {
                        interrupts[trap_level_zero] = false;
                        --numPosted;
                        return new TrapLevelZero;
                    }
                }
                if (interrupts[hstick_match]) {
                    if (tc->readMiscReg(MISCREG_HINTP) & 1) {
                        interrupts[hstick_match] = false;
                        --numPosted;
                        return new HstickMatch;
                        }
                }
                if (ie) {
                    if (interrupts[cpu_mondo]) {
                        interrupts[cpu_mondo] = false;
                        --numPosted;
                        return new CpuMondo;
                    }
                    if (interrupts[dev_mondo]) {
                        interrupts[dev_mondo] = false;
                        --numPosted;
                        return new DevMondo;
                    }
                    if (interrupts[soft_interrupt]) {
                        int il = InterruptLevel(tc->readMiscReg(MISCREG_SOFTINT));
                        // it seems that interrupt vectors are right in
                        // the middle of interrupt levels with regard to
                        // priority, so have to check
                        if ((il < 6) &&
                            interrupts[interrupt_vector]) {
                                // may require more details here since there
                                // may be lots of interrupts embedded in an
                                // platform interrupt vector
                                interrupts[interrupt_vector] = false;
                                --numPosted;
                                //HAVEN'T IMPLed YET
                                return NoFault;
                        } else {
                            if (il > tc->readMiscReg(MISCREG_PIL)) {
                                uint64_t si = tc->readMiscReg(MISCREG_SOFTINT);
                                uint64_t more = si & ~(1 << (il + 1));
                                if (!InterruptLevel(more)) {
                                    interrupts[soft_interrupt] = false;
                                    --numPosted;
                                }
                                return new InterruptLevelN(il);
                            }
                        }
                    }
                    if (interrupts[resumable_error]) {
                        interrupts[resumable_error] = false;
                        --numPosted;
                        return new ResumableError;
                    }
                }
            }
            return NoFault;
        }

        void updateIntrInfo(ThreadContext * tc)
        {

        }

        void serialize(std::ostream &os)
        {
        }

        void unserialize(Checkpoint *cp, const std::string &section)
        {
        }
    };
}

#endif // __ARCH_SPARC_INTERRUPT_HH__
