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
 * Authors: Steve Reinhardt
 *          Kevin Lim
 */

#ifndef __ARCH_ALPHA_INTERRUPT_HH__
#define __ARCH_ALPHA_INTERRUPT_HH__

#include "arch/alpha/faults.hh"
#include "arch/alpha/isa_traits.hh"
#include "cpu/thread_context.hh"

namespace AlphaISA
{
    class Interrupts
    {
      protected:
        uint64_t interrupts[NumInterruptLevels];
        uint64_t intstatus;

      public:
        Interrupts()
        {
            memset(interrupts, 0, sizeof(interrupts));
            intstatus = 0;
            newInfoSet = false;
        }

        void post(int int_num, int index)
        {
            DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);

            if (int_num < 0 || int_num >= NumInterruptLevels)
                panic("int_num out of bounds\n");

            if (index < 0 || index >= sizeof(uint64_t) * 8)
                panic("int_num out of bounds\n");

            interrupts[int_num] |= 1 << index;
            intstatus |= (ULL(1) << int_num);
        }

        void clear(int int_num, int index)
        {
            DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);

            if (int_num < 0 || int_num >= TheISA::NumInterruptLevels)
                panic("int_num out of bounds\n");

            if (index < 0 || index >= sizeof(uint64_t) * 8)
                panic("int_num out of bounds\n");

            interrupts[int_num] &= ~(1 << index);
            if (interrupts[int_num] == 0)
                intstatus &= ~(ULL(1) << int_num);
        }

        void clear_all()
        {
            DPRINTF(Interrupt, "Interrupts all cleared\n");

            memset(interrupts, 0, sizeof(interrupts));
            intstatus = 0;
        }

        void serialize(std::ostream &os)
        {
            SERIALIZE_ARRAY(interrupts, NumInterruptLevels);
            SERIALIZE_SCALAR(intstatus);
        }

        void unserialize(Checkpoint *cp, const std::string &section)
        {
            UNSERIALIZE_ARRAY(interrupts, NumInterruptLevels);
            UNSERIALIZE_SCALAR(intstatus);
        }

        bool check_interrupts(ThreadContext * tc) const
        {
            return (intstatus != 0) && !(tc->readPC() & 0x3);
        }

        Fault getInterrupt(ThreadContext * tc)
        {
            int ipl = 0;
            int summary = 0;

            if (tc->readMiscReg(IPR_ASTRR))
                panic("asynchronous traps not implemented\n");

            if (tc->readMiscReg(IPR_SIRR)) {
                for (int i = INTLEVEL_SOFTWARE_MIN;
                     i < INTLEVEL_SOFTWARE_MAX; i++) {
                    if (tc->readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
                        // See table 4-19 of 21164 hardware reference
                        ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                        summary |= (ULL(1) << i);
                    }
                }
            }

            uint64_t interrupts = intstatus;
            if (interrupts) {
                for (int i = INTLEVEL_EXTERNAL_MIN;
                    i < INTLEVEL_EXTERNAL_MAX; i++) {
                    if (interrupts & (ULL(1) << i)) {
                        // See table 4-19 of 21164 hardware reference
                        ipl = i;
                        summary |= (ULL(1) << i);
                    }
                }
            }

            if (ipl && ipl > tc->readMiscReg(IPR_IPLR)) {
//                assert(!newInfoSet);
                newIpl = ipl;
                newSummary = newSummary;
                newInfoSet = true;
                DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                        tc->readMiscReg(IPR_IPLR), ipl, summary);

                return new InterruptFault;
            } else {
                return NoFault;
            }
        }

        void updateIntrInfo(ThreadContext *tc)
        {
            assert(newInfoSet);
            tc->setMiscReg(IPR_ISR, newSummary);
            tc->setMiscReg(IPR_INTID, newIpl);
            newInfoSet = false;
        }

      private:
        bool newInfoSet;
        int newIpl;
        int newSummary;
    };
}

#endif

