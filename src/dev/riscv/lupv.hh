/*
 * Copyright (c) 2021 The Regents of the University of California
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

#ifndef __DEV_RISCV_LUPV_HH__
#define __DEV_RISCV_LUPV_HH__

#include "dev/lupio/lupio_pic.hh"
#include "dev/platform.hh"
#include "params/LupV.hh"

namespace gem5
{

using namespace RiscvISA;

/**
 * The LupV collection consists of a RISC-V processor, as well as the set of
 * LupiIO devices. This LupV platform allows for us to not only use these
 * devices, bu alsoseamlessly decide which interrupt controller we want to use.
 * For example, this platform has been tested to use both the LupioPIC for
 * interrupts, as well as the PLIC.
 **/

class LupV : public Platform
{
  public:
    LupioPIC *pic;
    int uartIntID;

  public:

    PARAMS(LupV);
    LupV(const Params &params);

    void postConsoleInt() override;

    void clearConsoleInt() override;

    void postPciInt(int line) override;

    void clearPciInt(int line) override;

    virtual Addr pciToDma(Addr pciAddr) const;

    void serialize(CheckpointOut &cp) const override;

    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif  // __DEV_RISCV_LUPV_HH__
