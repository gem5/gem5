/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 * Generic interface for platforms
 */

#ifndef __PLATFORM_HH_
#define __PLATFORM_HH_

#include "sim/sim_object.hh"

class PciConfigAll;
class IntrControl;
class SimConsole;

class Platform : public SimObject
{
  public:
    /** Pointer to the interrupt controller */
    IntrControl *intrctrl;
    /** Pointer to the simulation console */
    SimConsole *cons;
    /** Pointer to the PCI configuration space */
    PciConfigAll *pciconfig;

    int interrupt_frequency;

  public:
    Platform(const std::string &name, IntrControl *intctrl,
             PciConfigAll *pci, int intrFreq)
        : SimObject(name), intrctrl(intctrl), pciconfig(pci),
          interrupt_frequency(intrFreq) {}
    virtual ~Platform() {}
    virtual void postConsoleInt() = 0;
    virtual void clearConsoleInt() = 0;
    virtual Tick intrFrequency() = 0;
};

#endif // __PLATFORM_HH_
