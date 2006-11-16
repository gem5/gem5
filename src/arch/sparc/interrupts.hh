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
 * Authors: Gabe Black
 */

#ifndef __ARCH_SPARC_INTERRUPT_HH__
#define __ARCH_SPARC_INTERRUPT_HH__

#include "arch/sparc/faults.hh"

namespace SparcISA
{
    class Interrupts
    {
      protected:
        Fault interrupts[NumInterruptLevels];
        bool requested[NumInterruptLevels];

      public:
        Interrupts()
        {
            for(int x = 0; x < NumInterruptLevels; x++)
            {
                interrupts[x] = new InterruptLevelN(x);
                requested[x] = false;
            }
        }
        void post(int int_num, int index)
        {
            if(int_num < 0 || int_num >= NumInterruptLevels)
                panic("int_num out of bounds\n");

            requested[int_num] = true;
        }

        void clear(int int_num, int index)
        {
            requested[int_num] = false;
        }

        void clear_all()
        {
            for(int x = 0; x < NumInterruptLevels; x++)
                requested[x] = false;
        }

        bool check_interrupts(ThreadContext * tc) const
        {
            return true;
        }

        Fault getInterrupt(ThreadContext * tc)
        {
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
