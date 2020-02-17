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
 */

#include "arch/x86/insts/microop.hh"

#include "arch/x86/regs/misc.hh"

namespace X86ISA
{

    bool X86MicroopBase::checkCondition(uint64_t flags, int condition) const
    {
        CCFlagBits ccflags = flags;
        switch(condition)
        {
          case ConditionTests::True:
            return true;
          case ConditionTests::ECF:
            return ccflags.ecf;
          case ConditionTests::EZF:
            return ccflags.ezf;
          case ConditionTests::SZnZF:
            return !(!ccflags.ezf && ccflags.zf);
          case ConditionTests::MSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::STRZ:
            panic("This condition is not implemented!");
          case ConditionTests::MSTRC:
            panic("This condition is not implemented!");
          case ConditionTests::STRZnEZF:
            return !ccflags.ezf && ccflags.zf;
                //And no interrupts or debug traps are waiting
          case ConditionTests::OF:
            return ccflags.of;
          case ConditionTests::CF:
            return ccflags.cf;
          case ConditionTests::ZF:
            return ccflags.zf;
          case ConditionTests::CvZF:
            return ccflags.cf | ccflags.zf;
          case ConditionTests::SF:
            return ccflags.sf;
          case ConditionTests::PF:
            return ccflags.pf;
          case ConditionTests::SxOF:
            return ccflags.sf ^ ccflags.of;
          case ConditionTests::SxOvZF:
            return (ccflags.sf ^ ccflags.of) | ccflags.zf;
          case ConditionTests::False:
            return false;
          case ConditionTests::NotECF:
            return !ccflags.ecf;
          case ConditionTests::NotEZF:
            return !ccflags.ezf;
          case ConditionTests::NotSZnZF:
            return !ccflags.ezf && ccflags.zf;
          case ConditionTests::NotMSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::NotSTRZ:
            panic("This condition is not implemented!");
          case ConditionTests::NotMSTRC:
            panic("This condition is not implemented!");
          case ConditionTests::STRnZnEZF:
            return !ccflags.ezf && !ccflags.zf;
                //And no interrupts or debug traps are waiting
          case ConditionTests::NotOF:
            return !ccflags.of;
          case ConditionTests::NotCF:
            return !ccflags.cf;
          case ConditionTests::NotZF:
            return !ccflags.zf;
          case ConditionTests::NotCvZF:
            return !(ccflags.cf | ccflags.zf);
          case ConditionTests::NotSF:
            return !ccflags.sf;
          case ConditionTests::NotPF:
            return !ccflags.pf;
          case ConditionTests::NotSxOF:
            return !(ccflags.sf ^ ccflags.of);
          case ConditionTests::NotSxOvZF:
            return !((ccflags.sf ^ ccflags.of) | ccflags.zf);
        }
        panic("Unknown condition: %d\n", condition);
        return true;
    }
}
