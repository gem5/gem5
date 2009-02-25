/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Ali Saidi
 */

#include "arch/sparc/intregfile.hh"
#include "base/trace.hh"
#include "base/misc.hh"
#include "sim/serialize.hh"

#include <string.h>

using namespace SparcISA;
using namespace std;

class Checkpoint;

string SparcISA::getIntRegName(RegIndex index)
{
    static std::string intRegName[NumIntArchRegs] =
        {"g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7",
         "o0", "o1", "o2", "o3", "o4", "o5", "o6", "o7",
         "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
         "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7"};
    return intRegName[index];
}

void IntRegFile::clear()
{
    memset(regs, 0, sizeof(IntReg) * NumIntRegs);
}

IntRegFile::IntRegFile()
{
    clear();
}

IntReg IntRegFile::readReg(int intReg)
{
    DPRINTF(IntRegs, "Read register %d = 0x%x\n", intReg, regs[intReg]);
    return regs[intReg];
}

void IntRegFile::setReg(int intReg, const IntReg &val)
{
    if(intReg)
    {
        DPRINTF(IntRegs, "Wrote register %d = 0x%x\n", intReg, val);
        regs[intReg] = val;
    }
    return;
}

void IntRegFile::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(regs, NumIntRegs);
    SERIALIZE_ARRAY(microRegs, NumMicroIntRegs);
}

void IntRegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(regs, NumIntRegs);
    UNSERIALIZE_ARRAY(microRegs, NumMicroIntRegs);
}
