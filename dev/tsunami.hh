/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Declaration of top level class for the Tsunami chipset. This class just
 * retains pointers to all its children so the children can communicate.
 */

#ifndef __TSUNAMI_HH__
#define __TSUNAMI_HH__

#include "dev/platform.hh"

class IdeController;
class TlaserClock;
class NSGigE;
class TsunamiCChip;
class TsunamiPChip;
class TsunamiIO;
class PciConfigAll;
class System;

/**
  * Top level class for Tsunami Chipset emulation.
  * This structure just contains pointers to all the
  * children so the children can commnicate to do the
  * read work
  */

class Tsunami : public Platform
{
  public:

    /** Max number of CPUs in a Tsunami */
    static const int Max_CPUs = 4;

    /** Pointer to the system */
    System *system;

    /** Pointer to the TsunamiIO device which has the RTC */
    TsunamiIO *io;

    /** Pointer to the Tsunami CChip.
      * The chip contains some configuration information and
      * all the interrupt mask and status registers
      */
    TsunamiCChip *cchip;

    /** Pointer to the Tsunami PChip.
      * The pchip is the interface to the PCI bus, in our case
      * it does not have to do much.
      */
    TsunamiPChip *pchip;

    int intr_sum_type[Tsunami::Max_CPUs];
    int ipi_pending[Tsunami::Max_CPUs];

  public:
    /**
      * Constructor for the Tsunami Class.
      * @param name name of the object
      * @param con pointer to the console
      * @param intrcontrol pointer to the interrupt controller
      * @param intrFreq frequency that interrupts happen
      */
    Tsunami(const std::string &name, System *s, IntrControl *intctrl,
            PciConfigAll *pci, int intrFreq);

    /**
     * Return the interrupting frequency to AlphaAccess
     * @return frequency of RTC interrupts
     */
     virtual Tick intrFrequency();

    /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    virtual void postConsoleInt();

    /**
     * Clear a posted CPU interrupt (id=55)
     */
    virtual void clearConsoleInt();

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    virtual void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __TSUNAMI_HH__
