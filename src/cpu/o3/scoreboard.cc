/*
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 *          Kevin Lim
 */

#include "config/the_isa.hh"
#include "cpu/o3/scoreboard.hh"
#include "debug/Scoreboard.hh"

Scoreboard::Scoreboard(unsigned activeThreads,
                       unsigned _numLogicalIntRegs,
                       unsigned _numPhysicalIntRegs,
                       unsigned _numLogicalFloatRegs,
                       unsigned _numPhysicalFloatRegs,
                       unsigned _numMiscRegs,
                       unsigned _zeroRegIdx)
    : numLogicalIntRegs(_numLogicalIntRegs),
      numPhysicalIntRegs(_numPhysicalIntRegs),
      numLogicalFloatRegs(_numLogicalFloatRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs),
      numMiscRegs(_numMiscRegs),
      zeroRegIdx(_zeroRegIdx)
{
    //Get Register Sizes
    numLogicalRegs = numLogicalIntRegs  + numLogicalFloatRegs;
    numPhysicalRegs = numPhysicalIntRegs  + numPhysicalFloatRegs;

    //Resize scoreboard appropriately
    resize(numPhysicalRegs + (numMiscRegs * activeThreads));

    //Initialize values
    for (int i=0; i < numLogicalIntRegs * activeThreads; i++) {
        assert(indexInBounds(i));
        regScoreBoard[i] = 1;
    }

    for (int i= numPhysicalIntRegs;
         i < numPhysicalIntRegs + (numLogicalFloatRegs * activeThreads);
         i++) {
        assert(indexInBounds(i));
        regScoreBoard[i] = 1;
    }

    for (int i = numPhysicalRegs;
         i < numPhysicalRegs + (numMiscRegs * activeThreads);
         i++) {
        assert(indexInBounds(i));
        regScoreBoard[i] = 1;
    }
}

std::string
Scoreboard::name() const
{
    return "cpu.scoreboard";
}

bool
Scoreboard::getReg(PhysRegIndex phys_reg)
{
#if THE_ISA == ALPHA_ISA
    // Always ready if int or fp zero reg.
    if (phys_reg == zeroRegIdx ||
        phys_reg == (zeroRegIdx + numPhysicalIntRegs)) {
        return 1;
    }
#else
    // Always ready if int zero reg.
    if (phys_reg == zeroRegIdx) {
        return 1;
    }
#endif

    assert(indexInBounds(phys_reg));
    return regScoreBoard[phys_reg];
}

void
Scoreboard::setReg(PhysRegIndex phys_reg)
{
    DPRINTF(Scoreboard, "Setting reg %i as ready\n", phys_reg);

    assert(indexInBounds(phys_reg));
    regScoreBoard[phys_reg] = 1;
}

void
Scoreboard::unsetReg(PhysRegIndex ready_reg)
{
#if THE_ISA == ALPHA_ISA
    if (ready_reg == zeroRegIdx ||
        ready_reg == (zeroRegIdx + numPhysicalIntRegs)) {
        // Don't do anything if int or fp zero reg.
        return;
    }
#else
    if (ready_reg == zeroRegIdx) {
        // Don't do anything if int zero reg.
        return;
    }
#endif

    assert(indexInBounds(ready_reg));
    regScoreBoard[ready_reg] = 0;
}
