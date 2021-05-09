/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/**
 * @file
 * Declaration of top level class for the Malta chipset. This class just
 * retains pointers to all its children so the children can communicate.
 */

#ifndef __DEV_MALTA_HH__
#define __DEV_MALTA_HH__

#include "dev/platform.hh"
#include "params/Malta.hh"

namespace gem5
{

class MaltaCChip;
class MaltaIO;

/**
  * Top level class for Malta Chipset emulation.
  * This structure just contains pointers to all the
  * children so the children can commnicate to do the
  * read work
  */

class Malta : public Platform
{
  public:
    /** Max number of CPUs in a Malta */
    static const int Max_CPUs = 64;

    /** Pointer to the MaltaIO device which has the RTC */
    MaltaIO *io;

    /** Pointer to the Malta CChip.
     * The chip contains some configuration information and
     * all the interrupt mask and status registers
     */
    MaltaCChip *cchip;

    int intr_sum_type[Malta::Max_CPUs];
    int ipi_pending[Malta::Max_CPUs];

  public:
    typedef MaltaParams Params;
    Malta(const Params &p);

    /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    void postConsoleInt() override;

    /**
     * Clear a posted CPU interrupt (id=55)
     */
    void clearConsoleInt() override;

    /**
     * Cause the chipset to post a cpi interrupt to the CPU.
     */
    void postPciInt(int line) override;

    /**
     * Clear a posted PCI->CPU interrupt
     */
    void clearPciInt(int line) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_MALTA_HH__
