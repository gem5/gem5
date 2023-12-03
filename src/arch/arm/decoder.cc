/*
 * Copyright (c) 2012-2014,2018, 2021 Arm Limited
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
 */

#include "arch/arm/decoder.hh"

#include "arch/arm/isa.hh"
#include "arch/arm/utility.hh"
#include "base/cast.hh"
#include "base/trace.hh"
#include "debug/Decoder.hh"
#include "sim/full_system.hh"

namespace gem5
{

namespace ArmISA
{

GenericISA::BasicDecodeCache<Decoder, ExtMachInst> Decoder::defaultCache;

Decoder::Decoder(const ArmDecoderParams &params)
    : InstDecoder(params, &data),
      dvmEnabled(params.dvm_enabled),
      data(0),
      fpscrLen(0),
      fpscrStride(0),
      decoderFlavor(safe_cast<ISA *>(params.isa)->decoderFlavor())
{
    reset();

    // Initialize SVE vector length
    sveLen =
        (safe_cast<ISA *>(params.isa)->getCurSveVecLenInBitsAtReset() >> 7) -
        1;

    // Initialize SME vector length
    smeLen =
        (safe_cast<ISA *>(params.isa)->getCurSmeVecLenInBitsAtReset() >> 7) -
        1;

    if (dvmEnabled) {
        warn_once("DVM Ops instructions are micro-architecturally "
                  "modelled as loads. This will tamper the effective "
                  "number of loads stat\n");
    }
}

void
Decoder::reset()
{
    InstDecoder::reset();
    bigThumb = false;
    offset = 0;
    emi = 0;
    foundIt = false;
}

void
Decoder::process()
{
    // emi is typically ready, with some caveats below...
    instDone = true;

    if (!emi.thumb) {
        emi.instBits = data;
        if (!emi.aarch64) {
            emi.sevenAndFour = bits(data, 7) && bits(data, 4);
            emi.isMisc = (bits(data, 24, 23) == 0x2 && bits(data, 20) == 0);
        }
        consumeBytes(4);
        DPRINTF(Decoder, "Arm inst: %#x.\n", (uint64_t)emi);
    } else {
        uint16_t word = (data >> (offset * 8));
        if (bigThumb) {
            // A 32 bit thumb inst is half collected.
            emi.instBits = emi.instBits | word;
            bigThumb = false;
            consumeBytes(2);
            DPRINTF(Decoder, "Second half of 32 bit Thumb: %#x.\n",
                    emi.instBits);
        } else {
            uint16_t highBits = word & 0xF800;
            if (highBits == 0xE800 || highBits == 0xF000 ||
                highBits == 0xF800) {
                // The start of a 32 bit thumb inst.
                emi.bigThumb = 1;
                if (offset == 0) {
                    // We've got the whole thing.
                    emi.instBits = (data >> 16) | (data << 16);
                    DPRINTF(Decoder, "All of 32 bit Thumb: %#x.\n",
                            emi.instBits);
                    consumeBytes(4);
                } else {
                    // We only have the first half word.
                    DPRINTF(Decoder, "First half of 32 bit Thumb.\n");
                    emi.instBits = (uint32_t)word << 16;
                    bigThumb = true;
                    consumeBytes(2);
                    // emi not ready yet.
                    instDone = false;
                }
            } else {
                // A 16 bit thumb inst.
                consumeBytes(2);
                emi.instBits = word;
                // Set the condition code field artificially.
                emi.condCode = COND_UC;
                DPRINTF(Decoder, "16 bit Thumb: %#x.\n", emi.instBits);
                if (bits(word, 15, 8) == 0xbf && bits(word, 3, 0) != 0x0) {
                    foundIt = true;
                    itBits = bits(word, 7, 0);
                    DPRINTF(Decoder, "IT detected, cond = %#x, mask = %#x\n",
                            itBits.cond, itBits.mask);
                }
            }
        }
    }
}

void
Decoder::consumeBytes(int numBytes)
{
    offset += numBytes;
    assert(offset <= sizeof(data) || emi.decoderFault);
    if (offset == sizeof(data))
        outOfBytes = true;
}

void
Decoder::moreBytes(const PCStateBase &_pc, Addr fetchPC)
{
    auto &pc = _pc.as<PCState>();
    data = letoh(data);
    offset = (fetchPC >= pc.instAddr()) ? 0 : pc.instAddr() - fetchPC;
    emi.thumb = pc.thumb();
    emi.aarch64 = pc.aarch64();
    emi.fpscrLen = fpscrLen;
    emi.fpscrStride = fpscrStride;
    emi.sveLen = sveLen;

    const Addr alignment(pc.thumb() ? 0x1 : 0x3);
    emi.decoderFault = static_cast<uint8_t>(pc.instAddr() & alignment ?
                                                DecoderFault::UNALIGNED :
                                                DecoderFault::OK);

    outOfBytes = false;
    process();
}

StaticInstPtr
Decoder::decode(PCStateBase &_pc)
{
    if (!instDone)
        return NULL;

    auto &pc = _pc.as<PCState>();

    const int inst_size((!emi.thumb || emi.bigThumb) ? 4 : 2);
    ExtMachInst this_emi(emi);

    pc.npc(pc.pc() + inst_size);
    if (foundIt)
        pc.nextItstate(itBits);
    this_emi.itstate = pc.itstate();
    this_emi.illegalExecution = pc.illegalExec() ? 1 : 0;
    this_emi.debugStep = pc.debugStep() ? 1 : 0;
    pc.size(inst_size);

    emi = 0;
    instDone = false;
    foundIt = false;

    return decode(this_emi, pc.instAddr());
}

} // namespace ArmISA
} // namespace gem5
