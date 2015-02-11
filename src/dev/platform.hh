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

#include <bitset>
#include <set>

#include "params/Platform.hh"
#include "sim/sim_object.hh"

class PciConfigAll;
class IntrControl;
class Terminal;
class Uart;
class System;

class Platform : public SimObject
{
  public:
    /** Pointer to the interrupt controller */
    IntrControl *intrctrl;

  public:
    typedef PlatformParams Params;
    Platform(const Params *p);
    virtual ~Platform();
    virtual void postConsoleInt() = 0;
    virtual void clearConsoleInt() = 0;
    virtual void postPciInt(int line);
    virtual void clearPciInt(int line);
    virtual Addr pciToDma(Addr pciAddr) const;
    virtual Addr calcPciConfigAddr(int bus, int dev, int func) = 0;
    virtual Addr calcPciIOAddr(Addr addr) = 0;
    virtual Addr calcPciMemAddr(Addr addr) = 0;
    virtual void registerPciDevice(uint8_t bus, uint8_t dev, uint8_t func,
            uint8_t intr);

  private:
    std::bitset<256> intLines;
    std::set<uint32_t> pciDevices;

};

#endif // __DEV_PLATFORM_HH__
