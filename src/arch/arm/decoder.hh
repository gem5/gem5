/*
 * Copyright (c) 2013-2014 ARM Limited
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

#ifndef __ARCH_ARM_DECODER_HH__
#define __ARCH_ARM_DECODER_HH__

#include <cassert>

#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "arch/generic/decode_cache.hh"
#include "arch/generic/decoder.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "enums/DecoderFlavor.hh"

namespace ArmISA
{

class ISA;
class Decoder : public InstDecoder
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

    /**
     * SVE vector length, encoded in the same format as the ZCR_EL<x>.LEN
     * bitfields.
     */
    int sveLen;

    Enums::DecoderFlavor decoderFlavor;

    /// A cache of decoded instruction objects.
    static GenericISA::BasicDecodeCache defaultCache;

    /**
     * Pre-decode an instruction from the current state of the
     * decoder.
     */
    void process();

    /**
     * Consume bytes by moving the offset into the data word and
     * sanity check the results.
     */
    void consumeBytes(int numBytes);

  public: // Decoder API
    Decoder(ISA* isa = nullptr);

    /** Reset the decoders internal state. */
    void reset();

    /**
     * Can the decoder accept more data?
     *
     * A CPU model uses this method to determine if the decoder can
     * accept more data. Note that an instruction can be ready (see
     * instReady() even if this method returns true.
     */
    bool needMoreBytes() const { return outOfBytes; }

    /**
     * Is an instruction ready to be decoded?
     *
     * CPU models call this method to determine if decode() will
     * return a new instruction on the next call. It typically only
     * returns false if the decoder hasn't received enough data to
     * decode a full instruction.
     */
    bool instReady() const { return instDone; }

    /**
     * Feed data to the decoder.
     *
     * A CPU model uses this interface to load instruction data into
     * the decoder. Once enough data has been loaded (check with
     * instReady()), a decoded instruction can be retrieved using
     * decode(ArmISA::PCState).
     *
     * This method is intended to support both fixed-length and
     * variable-length instructions. Instruction data is fetch in
     * MachInst blocks (which correspond to the size of a typical
     * insturction). The method might need to be called multiple times
     * if the instruction spans multiple blocks, in that case
     * needMoreBytes() will return true and instReady() will return
     * false.
     *
     * The fetchPC parameter is used to indicate where in memory the
     * instruction was fetched from. This is should be the same
     * address as the pc. If fetching multiple blocks, it indicates
     * where subsequent blocks are fetched from (pc + n *
     * sizeof(MachInst)).
     *
     * @param pc Instruction pointer that we are decoding.
     * @param fetchPC The address this chunk was fetched from.
     * @param inst Raw instruction data.
     */
    void moreBytes(const PCState &pc, Addr fetchPC, MachInst inst);

    /**
     * Decode an instruction or fetch it from the code cache.
     *
     * This method decodes the currently pending pre-decoded
     * instruction. Data must be fed to the decoder using moreBytes()
     * until instReady() is true before calling this method.
     *
     * @param pc Instruction pointer that we are decoding.
     * @return A pointer to a static instruction or NULL if the
     * decoder isn't ready (see instReady()).
     */
    StaticInstPtr decode(ArmISA::PCState &pc);

    /**
     * Decode a pre-decoded machine instruction.
     *
     * @warn This method takes a pre-decoded instruction as its
     * argument. It should typically not be called directly.
     *
     * @param mach_inst A pre-decoded instruction
     * @retval A pointer to the corresponding StaticInst object.
     */
    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr)
    {
        return defaultCache.decode(this, mach_inst, addr);
    }

    /**
     * Decode a machine instruction without calling the cache.
     *
     * @note The implementation of this method is generated by the ISA
     * parser script.
     *
     * @warn This method takes a pre-decoded instruction as its
     * argument. It should typically not be called directly.
     *
     * @param mach_inst The binary instruction to decode.
     * @retval A pointer to the corresponding StaticInst object.
     */
    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /**
     * Take over the state from an old decoder when switching CPUs.
     *
     * @param old Decoder used in old CPU
     */
    void takeOverFrom(Decoder *old) {}


  public: // ARM-specific decoder state manipulation
    void setContext(FPSCR fpscr)
    {
        fpscrLen = fpscr.len;
        fpscrStride = fpscr.stride;
    }

    void setSveLen(uint8_t len)
    {
        sveLen = len;
    }
};

} // namespace ArmISA

#endif // __ARCH_ARM_DECODER_HH__
