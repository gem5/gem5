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

#ifndef REGSTATE_I386_HH
#define REGSTATE_I386_HH

#include <linux/user.h>
#include <sys/types.h>
#include <sys/ptrace.h>
#include <assert.h>
#include <string>

#include "tracechild.hh"

class I386TraceChild : public TraceChild
{
public:
        enum RegNum
        {
                //GPRs
                EAX, EBX, ECX, EDX,
                //Index registers
                ESI, EDI,
                //Base pointer and stack pointer
                EBP, ESP,
                //Segmentation registers
                CS, DS, ES, FS, GS, SS,
                //PC
                EIP,
                numregs
        };
private:
        char printBuffer [256];
        static char * regNames[numregs];
        int64_t getRegs(user_regs_struct & myregs, int num);
        user_regs_struct regs;
        user_regs_struct oldregs;
        bool regDiffSinceUpdate[numregs];

protected:
        bool update(int pid);

public:

        I386TraceChild();

        int getNumRegs()
        {
                return numregs;
        }

        bool diffSinceUpdate(int num)
        {
                assert(num < numregs && num >= 0);
                return regDiffSinceUpdate[num];
        }

        std::string getRegName(int num)
        {
                assert(num < numregs && num >= 0);
                return regNames[num];
        }

        int64_t getRegVal(int num);
        int64_t getOldRegVal(int num);
        uint64_t getPC() {return getRegVal(EIP);}
        uint64_t getSP() {return getRegVal(ESP);}
        std::ostream & outputStartState(std::ostream & output)
        {
            output << "Printing i386 initial state not yet implemented"
                << std::endl;
            return output;
        }

        char * printReg(int num);
};

#endif
