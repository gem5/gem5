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

#ifndef __CPU_O3_SCOREBOARD_HH__
#define __CPU_O3_SCOREBOARD_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "base/trace.hh"
#include "cpu/o3/comm.hh"

/**
 * Implements a simple scoreboard to track which registers are ready.
 * This class assumes that the fp registers start, index wise, right after
 * the integer registers. The misc. registers start, index wise, right after
 * the fp registers.
 * @todo: Fix up handling of the zero register in case the decoder does not
 * automatically make insts that write the zero register into nops.
 */
class Scoreboard
{
  public:
    /** Constructs a scoreboard.
     *  @param activeThreads The number of active threads.
     *  @param _numLogicalIntRegs Number of logical integer registers.
     *  @param _numPhysicalIntRegs Number of physical integer registers.
     *  @param _numLogicalFloatRegs Number of logical fp registers.
     *  @param _numPhysicalFloatRegs Number of physical fp registers.
     *  @param _numMiscRegs Number of miscellaneous registers.
     *  @param _zeroRegIdx Index of the zero register.
     */
    Scoreboard(unsigned activeThreads,
               unsigned _numLogicalIntRegs,
               unsigned _numPhysicalIntRegs,
               unsigned _numLogicalFloatRegs,
               unsigned _numPhysicalFloatRegs,
               unsigned _numMiscRegs,
               unsigned _zeroRegIdx);

    /** Destructor. */
    ~Scoreboard() {}

    /** Returns the name of the scoreboard. */
    std::string name() const;

    /** Checks if the register is ready. */
    bool getReg(PhysRegIndex ready_reg);

    /** Sets the register as ready. */
    void setReg(PhysRegIndex phys_reg);

    /** Sets the register as not ready. */
    void unsetReg(PhysRegIndex ready_reg);

  private:
    /** Scoreboard of physical integer registers, saying whether or not they
     *  are ready.
     */
    std::vector<bool> regScoreBoard;

    /** Number of logical integer registers. */
    int numLogicalIntRegs;

    /** Number of physical integer registers. */
    int numPhysicalIntRegs;

    /** Number of logical floating point registers. */
    int numLogicalFloatRegs;

    /** Number of physical floating point registers. */
    int numPhysicalFloatRegs;

    /** Number of miscellaneous registers. */
    int numMiscRegs;

    /** Number of logical integer + float registers. */
    int numLogicalRegs;

    /** Number of physical integer + float registers. */
    int numPhysicalRegs;

    /** The logical index of the zero register. */
    int zeroRegIdx;

    int currentSize;

    void
    resize(int newSize)
    {
        currentSize = newSize;
        regScoreBoard.resize(newSize);
    }

    bool
    indexInBounds(int idx)
    {
        return idx < currentSize;
    }
};

#endif
