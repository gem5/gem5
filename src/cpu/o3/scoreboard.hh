/*
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Steve Reinhardt
 */

#ifndef __CPU_O3_SCOREBOARD_HH__
#define __CPU_O3_SCOREBOARD_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "debug/Scoreboard.hh"

/**
 * Implements a simple scoreboard to track which registers are
 * ready. This class operates on the unified physical register space,
 * so integer and floating-point registers are not distinguished.  For
 * convenience, it also accepts operations on the physical-space
 * mapping of misc registers, which are numbered starting after the
 * end of the actual physical register file.  However, there is no
 * actual scoreboard for misc registers, and they are always
 * considered ready.
 */
class Scoreboard
{
  private:
    /** The object name, for DPRINTF.  We have to declare this
     *  explicitly because Scoreboard is not a SimObject. */
    const std::string _name;

    /** Scoreboard of physical integer registers, saying whether or not they
     *  are ready. */
    std::vector<bool> regScoreBoard;

    /** The number of actual physical registers */
    unsigned numPhysRegs;

    /**
     * The total number of registers which can be indexed, including
     * the misc registers that come after the physical registers and
     * which are hardwired to be always considered ready.
     */
    unsigned M5_CLASS_VAR_USED numTotalRegs;

    /** The index of the zero register. */
    PhysRegIndex zeroRegIdx;

    /** The index of the FP zero register. */
    PhysRegIndex fpZeroRegIdx;

    bool isZeroReg(PhysRegIndex idx) const
    {
        return (idx == zeroRegIdx ||
                (THE_ISA == ALPHA_ISA && idx == fpZeroRegIdx));
    }

  public:
    /** Constructs a scoreboard.
     *  @param _numPhysicalRegs Number of physical registers.
     *  @param _numMiscRegs Number of miscellaneous registers.
     *  @param _zeroRegIdx Index of the zero register.
     *  @param _fpZeroRegIdx Index of the FP zero register (if any, currently
     *                       used only for Alpha).
     */
    Scoreboard(const std::string &_my_name,
               unsigned _numPhysicalRegs,
               unsigned _numMiscRegs,
               PhysRegIndex _zeroRegIdx,
               PhysRegIndex _fpZeroRegIdx);

    /** Destructor. */
    ~Scoreboard() {}

    /** Returns the name of the scoreboard. */
    std::string name() const { return _name; };

    /** Checks if the register is ready. */
    bool getReg(PhysRegIndex reg_idx) const
    {
        assert(reg_idx < numTotalRegs);

        if (reg_idx >= numPhysRegs) {
            // misc regs are always ready
            return true;
        }

        bool ready = regScoreBoard[reg_idx];

        if (isZeroReg(reg_idx))
            assert(ready);

        return ready;
    }

    /** Sets the register as ready. */
    void setReg(PhysRegIndex reg_idx)
    {
        assert(reg_idx < numTotalRegs);

        if (reg_idx >= numPhysRegs) {
            // misc regs are always ready, ignore attempts to change that
            return;
        }

        DPRINTF(Scoreboard, "Setting reg %i as ready\n", reg_idx);

        assert(reg_idx < numTotalRegs);
        regScoreBoard[reg_idx] = true;
    }

    /** Sets the register as not ready. */
    void unsetReg(PhysRegIndex reg_idx)
    {
        assert(reg_idx < numTotalRegs);

        if (reg_idx >= numPhysRegs) {
            // misc regs are always ready, ignore attempts to change that
            return;
        }

        // zero reg should never be marked unready
        if (isZeroReg(reg_idx))
            return;

        regScoreBoard[reg_idx] = false;
    }

};

#endif
