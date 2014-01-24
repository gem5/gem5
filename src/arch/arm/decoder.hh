/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __ARCH_ARM_DECODER_HH__
#define __ARCH_ARM_DECODER_HH__

#include <cassert>

#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "arch/generic/decode_cache.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"

namespace ArmISA
{

class Decoder
{
  protected:
    //The extended machine instruction being generated
    ExtMachInst emi;
    MachInst data;
    bool bigThumb;
    bool instDone;
    bool outOfBytes;
    int offset;
    bool foundIt;
    ITSTATE itBits;

    int fpscrLen;
    int fpscrStride;

  public:
    void reset()
    {
        bigThumb = false;
        offset = 0;
        emi = 0;
        instDone = false;
        outOfBytes = true;
        foundIt = false;
    }

    Decoder() : data(0), fpscrLen(0), fpscrStride(0)
    {
        reset();
    }

    void process();

    //Use this to give data to the decoder. This should be used
    //when there is control flow.
    void moreBytes(const PCState &pc, Addr fetchPC, MachInst inst);

    //Use this to give data to the decoder. This should be used
    //when instructions are executed in order.
    void moreBytes(MachInst machInst)
    {
        moreBytes(0, 0, machInst);
    }

    inline void consumeBytes(int numBytes)
    {
        offset += numBytes;
        assert(offset <= sizeof(MachInst));
        if (offset == sizeof(MachInst))
            outOfBytes = true;
    }

    bool needMoreBytes() const
    {
        return outOfBytes;
    }

    bool instReady() const
    {
        return instDone;
    }

    int getInstSize() const
    {
        return (!emi.thumb || emi.bigThumb) ? 4 : 2;
    }

    void setContext(FPSCR fpscr)
    {
        fpscrLen = fpscr.len;
        fpscrStride = fpscr.stride;
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
    decode(ArmISA::PCState &nextPC)
    {
        if (!instDone)
            return NULL;

        assert(instDone);
        ExtMachInst thisEmi = emi;
        nextPC.npc(nextPC.pc() + getInstSize());
        if (foundIt)
            nextPC.nextItstate(itBits);
        thisEmi.itstate = nextPC.itstate();
        nextPC.size(getInstSize());
        emi = 0;
        instDone = false;
        foundIt = false;
        return decode(thisEmi, nextPC.instAddr());
    }
};

} // namespace ArmISA

#endif // __ARCH_ARM_DECODER_HH__
