/*
 * Copyright (c) 2012 Google
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_SPARC_DECODER_HH__
#define __ARCH_SPARC_DECODER_HH__

#include "arch/generic/decode_cache.hh"
#include "arch/sparc/registers.hh"
#include "arch/types.hh"
#include "cpu/static_inst.hh"

namespace SparcISA
{

class ISA;
class Decoder
{
  protected:
    // The extended machine instruction being generated
    ExtMachInst emi;
    bool instDone;
    MiscReg asi;

  public:
    Decoder(ISA* isa = nullptr) : instDone(false), asi(0)
    {}

    void process() {}

    void
    reset()
    {
        instDone = false;
    }

    // Use this to give data to the predecoder. This should be used
    // when there is control flow.
    void
    moreBytes(const PCState &pc, Addr fetchPC, MachInst inst)
    {
        emi = inst;
        // The I bit, bit 13, is used to figure out where the ASI
        // should come from. Use that in the ExtMachInst. This is
        // slightly redundant, but it removes the need to put a condition
        // into all the execute functions
        if (inst & (1 << 13)) {
            emi |= (static_cast<ExtMachInst>(
                        asi << (sizeof(MachInst) * 8)));
        } else {
            emi |= (static_cast<ExtMachInst>(bits(inst, 12, 5))
                    << (sizeof(MachInst) * 8));
        }
        instDone = true;
    }

    bool
    needMoreBytes()
    {
        return true;
    }

    bool
    instReady()
    {
        return instDone;
    }

    void
    setContext(MiscReg _asi)
    {
        asi = _asi;
    }

    void takeOverFrom(Decoder *old) {}

  protected:
    /// A cache of decoded instruction objects.
    static GenericISA::BasicDecodeCache defaultCache;

  public:
    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr
    decode(ExtMachInst mach_inst, Addr addr)
    {
        return defaultCache.decode(this, mach_inst, addr);
    }

    StaticInstPtr
    decode(SparcISA::PCState &nextPC)
    {
        if (!instDone)
            return NULL;
        instDone = false;
        return decode(emi, nextPC.instAddr());
    }
};

} // namespace SparcISA

#endif // __ARCH_SPARC_DECODER_HH__
