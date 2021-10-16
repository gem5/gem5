/*
 * Copyright (c) 2010, 2012-2013, 2017-2018 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARCH_ARM_PCSTATE_HH__
#define __ARCH_ARM_PCSTATE_HH__

#include "arch/generic/pcstate.hh"
#include "base/bitunion.hh"
#include "base/types.hh"
#include "debug/Decoder.hh"

namespace gem5
{

namespace ArmISA
{

BitUnion8(ITSTATE)
    /* Note that the split (cond, mask) below is not as in ARM ARM.
     * But it is more convenient for simulation. The condition
     * is always the concatenation of the top 3 bits and the next bit,
     * which applies when one of the bottom 4 bits is set.
     * Refer to predecoder.cc for the use case.
     */
    Bitfield<7, 4> cond;
    Bitfield<3, 0> mask;
    // Bitfields for moving to/from CPSR
    Bitfield<7, 2> top6;
    Bitfield<1, 0> bottom2;
EndBitUnion(ITSTATE)

class PCState : public GenericISA::UPCState<4>
{
  protected:

    typedef GenericISA::UPCState<4> Base;

    enum FlagBits
    {
        ThumbBit = (1 << 0),
        JazelleBit = (1 << 1),
        AArch64Bit = (1 << 2)
    };

    uint8_t flags = 0;
    uint8_t nextFlags = 0;
    uint8_t _itstate = 0;
    uint8_t _nextItstate = 0;
    uint8_t _size = 0;
    bool _illegalExec = false;

    // Software Step flags
    bool _debugStep = false;
    bool _stepped = false;

  public:
    void
    set(Addr val)
    {
        Base::set(val);
        npc(val + (thumb() ? 2 : 4));
    }

    PCState(const PCState &other) : Base(other),
        flags(other.flags), nextFlags(other.nextFlags),
        _itstate(other._itstate), _nextItstate(other._nextItstate),
        _size(other._size), _illegalExec(other._illegalExec),
        _debugStep(other._debugStep), _stepped(other._stepped)
    {}
    PCState &operator=(const PCState &other) = default;

    PCState() {}
    explicit PCState(Addr val) { set(val); }

    PCStateBase *clone() const override { return new PCState(*this); }

    void
    update(const PCStateBase &other) override
    {
        Base::update(other);
        auto &pcstate = other.as<PCState>();
        flags = pcstate.flags;
        nextFlags = pcstate.nextFlags;
        _itstate = pcstate._itstate;
        _nextItstate = pcstate._nextItstate;
        _size = pcstate._size;
        _illegalExec = pcstate._illegalExec;
        _debugStep = pcstate._debugStep;
        _stepped = pcstate._stepped;
    }

    bool
    illegalExec() const
    {
        return _illegalExec;
    }

    void
    illegalExec(bool val)
    {
        _illegalExec = val;
    }

    bool
    debugStep() const
    {
        return _debugStep;
    }

    void
    debugStep(bool val)
    {
        _debugStep = val;
    }

    bool
    stepped() const
    {
        return _stepped;
    }

    void
    stepped(bool val)
    {
        _stepped = val;
    }

    bool
    thumb() const
    {
        return flags & ThumbBit;
    }

    void
    thumb(bool val)
    {
        if (val)
            flags |= ThumbBit;
        else
            flags &= ~ThumbBit;
    }

    bool
    nextThumb() const
    {
        return nextFlags & ThumbBit;
    }

    void
    nextThumb(bool val)
    {
        if (val)
            nextFlags |= ThumbBit;
        else
            nextFlags &= ~ThumbBit;
    }

    void size(uint8_t s) { _size = s; }
    uint8_t size() const { return _size; }

    bool
    branching() const override
    {
        return ((this->pc() + this->size()) != this->npc());
    }


    bool
    jazelle() const
    {
        return flags & JazelleBit;
    }

    void
    jazelle(bool val)
    {
        if (val)
            flags |= JazelleBit;
        else
            flags &= ~JazelleBit;
    }

    bool
    nextJazelle() const
    {
        return nextFlags & JazelleBit;
    }

    void
    nextJazelle(bool val)
    {
        if (val)
            nextFlags |= JazelleBit;
        else
            nextFlags &= ~JazelleBit;
    }

    bool
    aarch64() const
    {
        return flags & AArch64Bit;
    }

    void
    aarch64(bool val)
    {
        if (val)
            flags |= AArch64Bit;
        else
            flags &= ~AArch64Bit;
    }

    bool
    nextAArch64() const
    {
        return nextFlags & AArch64Bit;
    }

    void
    nextAArch64(bool val)
    {
        if (val)
            nextFlags |= AArch64Bit;
        else
            nextFlags &= ~AArch64Bit;
    }


    uint8_t
    itstate() const
    {
        return _itstate;
    }

    void
    itstate(uint8_t value)
    {
        _itstate = value;
    }

    uint8_t
    nextItstate() const
    {
        return _nextItstate;
    }

    void
    nextItstate(uint8_t value)
    {
        _nextItstate = value;
    }

    void
    advance() override
    {
        Base::advance();
        flags = nextFlags;
        npc(pc() + (thumb() ? 2 : 4));

        if (_nextItstate) {
            _itstate = _nextItstate;
            _nextItstate = 0;
        } else if (_itstate) {
            ITSTATE it = _itstate;
            uint8_t cond_mask = it.mask;
            uint8_t thumb_cond = it.cond;
            DPRINTF(Decoder, "Advancing ITSTATE from %#x,%#x.\n",
                    thumb_cond, cond_mask);
            cond_mask <<= 1;
            uint8_t new_bit = bits(cond_mask, 4);
            cond_mask &= mask(4);
            if (cond_mask == 0)
                thumb_cond = 0;
            else
                replaceBits(thumb_cond, 0, new_bit);
            DPRINTF(Decoder, "Advancing ITSTATE to %#x,%#x.\n",
                    thumb_cond, cond_mask);
            it.mask = cond_mask;
            it.cond = thumb_cond;
            _itstate = it;
        }
    }

    void
    uEnd()
    {
        advance();
        upc(0);
        nupc(1);
    }

    Addr
    instPC() const
    {
        return pc() + (thumb() ? 4 : 8);
    }

    void
    instNPC(Addr val)
    {
        // @todo: review this when AArch32/64 interprocessing is
        // supported
        if (aarch64())
            npc(val);  // AArch64 doesn't force PC alignment, a PC
                       // Alignment Fault can be raised instead
        else
            npc(val &~ mask(nextThumb() ? 1 : 2));
    }

    Addr
    instNPC() const
    {
        return npc();
    }

    // Perform an interworking branch.
    void
    instIWNPC(Addr val)
    {
        bool thumbEE = (thumb() && jazelle());

        Addr newPC = val;
        if (thumbEE) {
            if (bits(newPC, 0)) {
                newPC = newPC & ~mask(1);
            }  // else we have a bad interworking address; do not call
               // panic() since the instruction could be executed
               // speculatively
        } else {
            if (bits(newPC, 0)) {
                nextThumb(true);
                newPC = newPC & ~mask(1);
            } else if (!bits(newPC, 1)) {
                nextThumb(false);
            } else {
                // This state is UNPREDICTABLE in the ARM architecture
                // The easy thing to do is just mask off the bit and
                // stay in the current mode, so we'll do that.
                newPC &= ~mask(2);
            }
        }
        npc(newPC);
    }

    // Perform an interworking branch in ARM mode, a regular branch
    // otherwise.
    void
    instAIWNPC(Addr val)
    {
        if (!thumb() && !jazelle())
            instIWNPC(val);
        else
            instNPC(val);
    }

    bool
    equals(const PCStateBase &other) const override
    {
        auto &opc = other.as<PCState>();
        return Base::equals(other) &&
            flags == opc.flags && nextFlags == opc.nextFlags &&
            _itstate == opc._itstate &&
            _nextItstate == opc._nextItstate &&
            _illegalExec == opc._illegalExec &&
            _debugStep == opc._debugStep &&
            _stepped == opc._stepped;
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        Base::serialize(cp);
        SERIALIZE_SCALAR(flags);
        SERIALIZE_SCALAR(_size);
        SERIALIZE_SCALAR(nextFlags);
        SERIALIZE_SCALAR(_itstate);
        SERIALIZE_SCALAR(_nextItstate);
        SERIALIZE_SCALAR(_illegalExec);
        SERIALIZE_SCALAR(_debugStep);
        SERIALIZE_SCALAR(_stepped);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        Base::unserialize(cp);
        UNSERIALIZE_SCALAR(flags);
        UNSERIALIZE_SCALAR(_size);
        UNSERIALIZE_SCALAR(nextFlags);
        UNSERIALIZE_SCALAR(_itstate);
        UNSERIALIZE_SCALAR(_nextItstate);
        UNSERIALIZE_SCALAR(_illegalExec);
        UNSERIALIZE_SCALAR(_debugStep);
        UNSERIALIZE_SCALAR(_stepped);
    }
};

} // namespace ArmISA
} // namespace gem5

#endif
