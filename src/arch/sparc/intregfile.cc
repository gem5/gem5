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
#include "sim/serialize.hh"

#include <string.h>

using namespace SparcISA;
using namespace std;

class Checkpoint;

string SparcISA::getIntRegName(RegIndex index)
{
    static std::string intRegName[NumIntRegs] =
        {"g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7",
         "o0", "o1", "o2", "o3", "o4", "o5", "o6", "o7",
         "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
         "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7"};
    return intRegName[index];
}

int IntRegFile::flattenIndex(int reg)
{
    int flatIndex = offset[reg >> FrameOffsetBits]
        | (reg & FrameOffsetMask);
    DPRINTF(Sparc, "Flattened index %d into %d.\n", reg, flatIndex);
    return flatIndex;
}

void IntRegFile::clear()
{
    int x;
    for (x = 0; x < MaxGL; x++)
        memset(regGlobals[x], 0, sizeof(IntReg) * RegsPerFrame);
    for(int x = 0; x < 2 * NWindows; x++)
        memset(regSegments[x], 0, sizeof(IntReg) * RegsPerFrame);
}

IntRegFile::IntRegFile()
{
    offset[Globals] = 0;
    regView[Globals] = regGlobals[0];
    setCWP(0);
    clear();
}

IntReg IntRegFile::readReg(int intReg)
{
    IntReg val;
    if(intReg < NumRegularIntRegs)
        val = regView[intReg >> FrameOffsetBits][intReg & FrameOffsetMask];
    else if((intReg -= NumRegularIntRegs) < NumMicroIntRegs)
        val = microRegs[intReg];
    else
        panic("Tried to read non-existant integer register\n");

    DPRINTF(Sparc, "Read register %d = 0x%x\n", intReg, val);
    return val;
}

Fault IntRegFile::setReg(int intReg, const IntReg &val)
{
    if(intReg)
    {
        DPRINTF(Sparc, "Wrote register %d = 0x%x\n", intReg, val);
        if(intReg < NumRegularIntRegs)
            regView[intReg >> FrameOffsetBits][intReg & FrameOffsetMask] = val;
        else if((intReg -= NumRegularIntRegs) < NumMicroIntRegs)
            microRegs[intReg] = val;
        else
            panic("Tried to set non-existant integer register\n");
    }
    return NoFault;
}

//This doesn't effect the actual CWP register.
//It's purpose is to adjust the view of the register file
//to what it would be if CWP = cwp.
void IntRegFile::setCWP(int cwp)
{
    int index = ((NWindows - cwp) % NWindows) * 2;
    offset[Outputs] = FrameOffset + (index * RegsPerFrame);
    offset[Locals] = FrameOffset + ((index+1) * RegsPerFrame);
    offset[Inputs] = FrameOffset +
        (((index+2) % (NWindows * 2)) * RegsPerFrame);
    regView[Outputs] = regSegments[index];
    regView[Locals] = regSegments[index+1];
    regView[Inputs] = regSegments[(index+2) % (NWindows * 2)];

    DPRINTF(Sparc, "Changed the CWP value to %d\n", cwp);
}

void IntRegFile::setGlobals(int gl)
{
    DPRINTF(Sparc, "Now using %d globals", gl);

    regView[Globals] = regGlobals[gl];
    offset[Globals] = RegGlobalOffset + gl * RegsPerFrame;
}

void IntRegFile::serialize(std::ostream &os)
{
    unsigned int x;
    for(x = 0; x < MaxGL; x++)
        SERIALIZE_ARRAY(regGlobals[x], RegsPerFrame);
    for(x = 0; x < 2 * NWindows; x++)
        SERIALIZE_ARRAY(regSegments[x], RegsPerFrame);
    SERIALIZE_ARRAY(microRegs, NumMicroIntRegs);
}

void IntRegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    unsigned int x;
    for(x = 0; x < MaxGL; x++)
        UNSERIALIZE_ARRAY(regGlobals[x], RegsPerFrame);
    for(unsigned int x = 0; x < 2 * NWindows; x++)
        UNSERIALIZE_ARRAY(regSegments[x], RegsPerFrame);
    UNSERIALIZE_ARRAY(microRegs, NumMicroIntRegs);
}
