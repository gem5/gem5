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
 */

#ifndef __CPU_O3_SCOREBOARD_HH__
#define __CPU_O3_SCOREBOARD_HH__

#include <cassert>
#include <vector>

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/reg_class.hh"
#include "debug/Scoreboard.hh"

namespace gem5
{

namespace o3
{

/**
 * Implements a simple scoreboard to track which registers are
 * ready. This class operates on the unified physical register space,
 * because the different classes of registers do not need to be distinguished.
 * Registers being part of a fixed mapping are always considered ready.
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
    GEM5_CLASS_VAR_USED unsigned numPhysRegs;

  public:
    /** Constructs a scoreboard.
     *  @param _numPhysicalRegs Number of physical registers.
     *  @param _numMiscRegs Number of miscellaneous registers.
     */
    Scoreboard(const std::string &_my_name, unsigned _numPhysicalRegs);

    /** Destructor. */
    ~Scoreboard() {}

    /** Returns the name of the scoreboard. */
    std::string
    name() const
    {
        return _name;
    };

    /** Checks if the register is ready. */
    bool
    getReg(PhysRegIdPtr phys_reg) const
    {
        if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready
            return true;
        }

        assert(phys_reg->flatIndex() < numPhysRegs);

        return regScoreBoard[phys_reg->flatIndex()];
    }

    /** Sets the register as ready. */
    void
    setReg(PhysRegIdPtr phys_reg)
    {
        if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to change
            // that
            return;
        }

        assert(phys_reg->flatIndex() < numPhysRegs);

        DPRINTF(Scoreboard, "Setting reg %i (%s) as ready\n",
                phys_reg->index(), phys_reg->className());

        regScoreBoard[phys_reg->flatIndex()] = true;
    }

    /** Sets the register as not ready. */
    void
    unsetReg(PhysRegIdPtr phys_reg)
    {
        if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to
            // change that
            return;
        }

        assert(phys_reg->flatIndex() < numPhysRegs);

        regScoreBoard[phys_reg->flatIndex()] = false;
    }
};

} // namespace o3
} // namespace gem5

#endif
