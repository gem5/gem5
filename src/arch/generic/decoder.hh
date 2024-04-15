/*
 * Copyright 2020 Google, Inc.
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

#ifndef __ARCH_GENERIC_DECODER_HH__
#define __ARCH_GENERIC_DECODER_HH__

#include "arch/generic/pcstate.hh"
#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/types.hh"
#include "cpu/static_inst_fwd.hh"
#include "params/InstDecoder.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class InstDecoder : public SimObject
{
  protected:
    void *_moreBytesPtr;
    size_t _moreBytesSize;
    Addr _pcMask;

    bool instDone = false;
    bool outOfBytes = true;

  public:
    template <typename MoreBytesType>
    InstDecoder(const InstDecoderParams &params, MoreBytesType *mb_buf)
        : SimObject(params),
          _moreBytesPtr(mb_buf),
          _moreBytesSize(sizeof(MoreBytesType)),
          _pcMask(~mask(floorLog2(_moreBytesSize)))
    {}

    virtual StaticInstPtr fetchRomMicroop(MicroPC micropc,
                                          StaticInstPtr curMacroop);

    virtual void
    reset()
    {
        instDone = false;
        outOfBytes = true;
    }

    template <class Type>
    Type &
    as()
    {
        return *static_cast<Type *>(this);
    }

    template <class Type>
    const Type &
    as() const
    {
        return *static_cast<const Type *>(this);
    }

    /**
     * Take over the state from an old decoder when switching CPUs.
     *
     * @param old Decoder used in old CPU
     */
    virtual void
    takeOverFrom(InstDecoder *old)
    {
        instDone = old->instDone;
        outOfBytes = old->outOfBytes;
    }

    void *
    moreBytesPtr() const
    {
        return _moreBytesPtr;
    }

    size_t
    moreBytesSize() const
    {
        return _moreBytesSize;
    }

    Addr
    pcMask() const
    {
        return _pcMask;
    }

    /**
     * Is an instruction ready to be decoded?
     *
     * CPU models call this method to determine if decode() will
     * return a new instruction on the next call. It typically only
     * returns false if the decoder hasn't received enough data to
     * decode a full instruction.
     */
    bool
    instReady() const
    {
        return instDone;
    }

    /**
     * Can the decoder accept more data?
     *
     * A CPU model uses this method to determine if the decoder can
     * accept more data. Note that an instruction can be ready (see
     * instReady() even if this method returns true.
     */
    bool
    needMoreBytes() const
    {
        return outOfBytes;
    }

    /**
     * Feed data to the decoder.
     *
     * A CPU model uses this interface to load instruction data into
     * the decoder. Once enough data has been loaded (check with
     * instReady()), a decoded instruction can be retrieved using
     * decode(PCStateBase &).
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
     */
    virtual void moreBytes(const PCStateBase &pc, Addr fetchPC) = 0;

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
    virtual StaticInstPtr decode(PCStateBase &pc) = 0;
};

} // namespace gem5

#endif // __ARCH_DECODER_GENERIC_HH__
