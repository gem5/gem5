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
 *
 * Authors: Andrew Schultz
 *          Nathan Binkert
 */

/**
 * @file
 * Generic interface for platforms
 */

#ifndef __DEV_PLATFORM_HH__
#define __DEV_PLATFORM_HH__

#include "sim/sim_object.hh"
#include "arch/isa_traits.hh"

class PciConfigAll;
class IntrControl;
class SimConsole;
class Uart;
class System;

class Platform : public SimObject
{
  public:
    /** Pointer to the interrupt controller */
    IntrControl *intrctrl;

    /** Pointer to the PCI configuration space */
    PciConfigAll *pciconfig;

    /** Pointer to the UART, set by the uart */
    Uart *uart;

    /** Pointer to the system for info about the memory system. */
    System *system;

  public:
    Platform(const std::string &name, IntrControl *intctrl);
    virtual ~Platform();
    virtual void init() { if (pciconfig == NULL) panic("PCI Config not set"); }
    virtual void postConsoleInt() = 0;
    virtual void clearConsoleInt() = 0;
    virtual Tick intrFrequency() = 0;
    virtual void postPciInt(int line);
    virtual void clearPciInt(int line);
    virtual Addr pciToDma(Addr pciAddr) const;
};

#endif // __DEV_PLATFORM_HH__
