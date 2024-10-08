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
 */

#ifndef __ARCH_X86_DECODER_HH__
#define __ARCH_X86_DECODER_HH__

#include <cassert>
#include <unordered_map>
#include <vector>

#include "arch/generic/decoder.hh"
#include "arch/x86/microcode_rom.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/types.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/decode_cache.hh"
#include "cpu/static_inst.hh"
#include "debug/Decoder.hh"
#include "params/X86Decoder.hh"

namespace gem5
{

class BaseISA;

namespace X86ISA
{

class Decoder : public InstDecoder
{
  private:
    // These are defined and documented in decoder_tables.cc
    static const uint8_t SizeTypeToSize[3][10];
    typedef const uint8_t ByteTable[256];
    static const ByteTable Prefixes[2];

    static const ByteTable UsesModRMOneByte;
    static const ByteTable UsesModRMTwoByte;
    static const ByteTable UsesModRMThreeByte0F38;
    static const ByteTable UsesModRMThreeByte0F3A;

    static const ByteTable ImmediateTypeOneByte;
    static const ByteTable ImmediateTypeTwoByte;
    static const ByteTable ImmediateTypeThreeByte0F38;
    static const ByteTable ImmediateTypeThreeByte0F3A;

    X86ISAInst::MicrocodeRom microcodeRom;

  protected:
    using MachInst = uint64_t;

    struct InstBytes
    {
        StaticInstPtr si;
        std::vector<MachInst> chunks;
        std::vector<MachInst> masks;
        int lastOffset;

        InstBytes() : lastOffset(0)
        {}
    };

    InstBytes dummy;

    // The bytes to be predecoded.
    MachInst fetchChunk;
    InstBytes *instBytes = &dummy;
    int chunkIdx;
    // The pc of the start of fetchChunk.
    Addr basePC = 0;
    // The pc the current instruction started at.
    Addr origPC = 0;
    // The offset into fetchChunk of current processing.
    int offset = 0;
    // The extended machine instruction being generated.
    ExtMachInst emi;
    // Predecoding state.
    X86Mode mode = LongMode;
    X86SubMode submode = SixtyFourBitMode;
    uint8_t altOp = 0;
    uint8_t defOp = 0;
    uint8_t altAddr = 0;
    uint8_t defAddr = 0;
    uint8_t stack = 0;

    uint8_t cpl = 0;

    uint8_t
    getNextByte()
    {
        return ((uint8_t *)&fetchChunk)[offset];
    }

    void
    getImmediate(int &collected, uint64_t &current, int size)
    {
        // Figure out how many bytes we still need to get for the
        // immediate.
        int toGet = size - collected;
        // Figure out how many bytes are left in our "buffer".
        int remaining = sizeof(MachInst) - offset;
        // Get as much as we need, up to the amount available.
        toGet = toGet > remaining ? remaining : toGet;

        // Shift the bytes we want to be all the way to the right
        uint64_t partialImm = fetchChunk >> (offset * 8);
        // Mask off what we don't want.
        partialImm &= mask(toGet * 8);
        // Shift it over to overlay with our displacement.
        partialImm <<= (immediateCollected * 8);
        // Put it into our displacement.
        current |= partialImm;
        // Update how many bytes we've collected.
        collected += toGet;
        consumeBytes(toGet);
    }

    void
    updateOffsetState()
    {
        assert(offset <= sizeof(MachInst));
        if (offset == sizeof(MachInst)) {
            DPRINTF(Decoder, "At the end of a chunk, idx = %d, chunks = %d.\n",
                    chunkIdx, instBytes->chunks.size());
            chunkIdx++;
            if (chunkIdx == instBytes->chunks.size()) {
                outOfBytes = true;
            } else {
                offset = 0;
                fetchChunk = instBytes->chunks[chunkIdx];
                basePC += sizeof(MachInst);
            }
        }
    }

    void
    consumeByte()
    {
        offset++;
        updateOffsetState();
    }

    void
    consumeBytes(int numBytes)
    {
        offset += numBytes;
        updateOffsetState();
    }

    // State machine state.
  protected:
    // The size of the displacement value.
    int displacementSize;
    // The size of the immediate value.
    int immediateSize;
    // This is how much of any immediate value we've gotten. This is used
    // for both the actual immediate and the displacement.
    int immediateCollected;

    enum State
    {
        ResetState,
        FromCacheState,
        PrefixState,
        Vex2Of2State,
        Vex2Of3State,
        Vex3Of3State,
        VexOpcodeState,
        OneByteOpcodeState,
        TwoByteOpcodeState,
        ThreeByte0F38OpcodeState,
        ThreeByte0F3AOpcodeState,
        ModRMState,
        SIBState,
        DisplacementState,
        ImmediateState,
        // We should never get to this state. Getting here is an error.
        ErrorState
    };

    State state = ResetState;

    // Functions to handle each of the states
    State doResetState();
    State doFromCacheState();
    State doPrefixState(uint8_t);
    State doVex2Of2State(uint8_t);
    State doVex2Of3State(uint8_t);
    State doVex3Of3State(uint8_t);
    State doVexOpcodeState(uint8_t);
    State doOneByteOpcodeState(uint8_t);
    State doTwoByteOpcodeState(uint8_t);
    State doThreeByte0F38OpcodeState(uint8_t);
    State doThreeByte0F3AOpcodeState(uint8_t);
    State doModRMState(uint8_t);
    State doSIBState(uint8_t);
    State doDisplacementState();
    State doImmediateState();

    // Process the actual opcode found earlier, using the supplied tables.
    State processOpcode(ByteTable &immTable, ByteTable &modrmTable,
                        bool addrSizedImm = false);
    // Process the opcode found with VEX / XOP prefix.
    State processExtendedOpcode(ByteTable &immTable);

  protected:
    /// Caching for decoded instruction objects.

    typedef RegVal CacheKey;

    typedef decode_cache::AddrMap<Decoder::InstBytes> DecodePages;
    DecodePages *decodePages = nullptr;
    typedef std::unordered_map<CacheKey, DecodePages *> AddrCacheMap;
    AddrCacheMap addrCacheMap;

    decode_cache::InstMap<ExtMachInst> *instMap = nullptr;
    typedef std::unordered_map<
            CacheKey, decode_cache::InstMap<ExtMachInst> *> InstCacheMap;
    InstCacheMap instCacheMap;

    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr);

    void process();

  public:
    Decoder(const X86DecoderParams &p) : InstDecoder(p, &fetchChunk)
    {
        emi.reset();
        emi.mode.cpl = cpl;
        emi.mode.mode = mode;
        emi.mode.submode = submode;
    }

    void
    setM5Reg(HandyM5Reg m5Reg)
    {
        cpl = m5Reg.cpl;
        mode = (X86Mode)(uint64_t)m5Reg.mode;
        submode = (X86SubMode)(uint64_t)m5Reg.submode;
        emi.mode.cpl = cpl;
        emi.mode.mode = mode;
        emi.mode.submode = submode;
        altOp = m5Reg.altOp;
        defOp = m5Reg.defOp;
        altAddr = m5Reg.altAddr;
        defAddr = m5Reg.defAddr;
        stack = m5Reg.stack;

        AddrCacheMap::iterator amIter = addrCacheMap.find(m5Reg);
        if (amIter != addrCacheMap.end()) {
            decodePages = amIter->second;
        } else {
            decodePages = new DecodePages;
            addrCacheMap[m5Reg] = decodePages;
        }

        InstCacheMap::iterator imIter = instCacheMap.find(m5Reg);
        if (imIter != instCacheMap.end()) {
            instMap = imIter->second;
        } else {
            instMap = new decode_cache::InstMap<ExtMachInst>;
            instCacheMap[m5Reg] = instMap;
        }
    }

    void
    takeOverFrom(InstDecoder *old) override
    {
        InstDecoder::takeOverFrom(old);

        Decoder *dec = dynamic_cast<Decoder *>(old);
        assert(dec);

        cpl = dec->cpl;
        mode = dec->mode;
        submode = dec->submode;
        emi.mode.cpl = cpl;
        emi.mode.mode = mode;
        emi.mode.submode = submode;
        altOp = dec->altOp;
        defOp = dec->defOp;
        altAddr = dec->altAddr;
        defAddr = dec->defAddr;
        stack = dec->stack;
    }

    void
    reset() override
    {
        InstDecoder::reset();
        state = ResetState;
    }

    // Use this to give data to the decoder. This should be used
    // when there is control flow.
    void
    moreBytes(const PCStateBase &pc, Addr fetchPC) override
    {
        DPRINTF(Decoder, "Getting more bytes.\n");
        basePC = fetchPC;
        offset = (fetchPC >= pc.instAddr()) ? 0 : pc.instAddr() - fetchPC;
        fetchChunk = letoh(fetchChunk);
        outOfBytes = false;
        process();
    }

    void
    updateNPC(X86ISA::PCState &nextPC)
    {
        if (!nextPC.size()) {
            int size = basePC + offset - origPC;
            DPRINTF(Decoder,
                    "Calculating the instruction size: "
                    "basePC: %#x offset: %#x origPC: %#x size: %d\n",
                    basePC, offset, origPC, size);
            nextPC.size(size);
            nextPC.npc(nextPC.pc() + size);
        }
    }

  public:
    StaticInstPtr decode(PCStateBase &next_pc) override;

    StaticInstPtr fetchRomMicroop(
            MicroPC micropc, StaticInstPtr curMacroop) override;
};

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_DECODER_HH__
