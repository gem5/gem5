/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_TYPES_HH__
#define __ARCH_X86_TYPES_HH__

#include <iostream>

#include "arch/generic/types.hh"
#include "base/bitunion.hh"
#include "base/cprintf.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace X86ISA
{
    //This really determines how many bytes are passed to the decoder.
    typedef uint64_t MachInst;

    enum Prefixes {
        NoOverride,
        ESOverride,
        CSOverride,
        SSOverride,
        DSOverride,
        FSOverride,
        GSOverride,
        RexPrefix,
        OperandSizeOverride,
        AddressSizeOverride,
        Lock,
        Rep,
        Repne,
        Vex2Prefix,
        Vex3Prefix,
        XopPrefix,
    };

    BitUnion8(LegacyPrefixVector)
        Bitfield<7, 4> decodeVal;
        Bitfield<7> repne;
        Bitfield<6> rep;
        Bitfield<5> lock;
        Bitfield<4> op;
        Bitfield<3> addr;
        //There can be only one segment override, so they share the
        //first 3 bits in the legacyPrefixes bitfield.
        Bitfield<2,0> seg;
    EndBitUnion(LegacyPrefixVector)

    BitUnion8(ModRM)
        Bitfield<7,6> mod;
        Bitfield<5,3> reg;
        Bitfield<2,0> rm;
    EndBitUnion(ModRM)

    BitUnion8(Sib)
        Bitfield<7,6> scale;
        Bitfield<5,3> index;
        Bitfield<2,0> base;
    EndBitUnion(Sib)

    BitUnion8(Rex)
        //This bit doesn't mean anything according to the ISA, but in
        //this implementation, it being set means an REX prefix was present.
        Bitfield<6> present;
        Bitfield<3> w;
        Bitfield<2> r;
        Bitfield<1> x;
        Bitfield<0> b;
    EndBitUnion(Rex)

    BitUnion(uint32_t, ThreeByteVex)
        Bitfield<7,0> zero;
        SubBitUnion(first, 15, 8)
            // Inverted one-bit extension of ModRM reg field
            Bitfield<15> r;
            // Inverted one-bit extension of SIB index field
            Bitfield<14> x;
            // Inverted one-bit extension, r/m field or SIB base field
            Bitfield<13> b;
            // Opcode map select
            Bitfield<12, 8> map_select;
        EndSubBitUnion(first)
        SubBitUnion(second, 23, 16)
            // Default operand size override for a general purpose register to
            // 64-bit size in 64-bit mode; operand configuration specifier for
            // certain YMM/XMM-based operations.
            Bitfield<23> w;
            // Source or destination register selector, in ones' complement
            // format
            Bitfield<22, 19>  vvvv;
            // Vector length specifier
            Bitfield<18> l;
            // Implied 66, F2, or F3 opcode extension
            Bitfield<17, 16> pp;
        EndSubBitUnion(second)
    EndBitUnion(ThreeByteVex)

    BitUnion16(TwoByteVex)
        Bitfield<7,0> zero;
        SubBitUnion(first, 15, 8)
            // Inverted one-bit extension of ModRM reg field
            Bitfield<15> r;
            // Source or destination register selector, in ones' complement
            // format
            Bitfield<14, 11>  vvvv;
            // Vector length specifier
            Bitfield<10> l;
            // Implied 66, F2, or F3 opcode extension
            Bitfield<9, 8> pp;
        EndSubBitUnion(first)
    EndBitUnion(TwoByteVex)

    enum OpcodeType {
        BadOpcode,
        OneByteOpcode,
        TwoByteOpcode,
        ThreeByte0F38Opcode,
        ThreeByte0F3AOpcode,
        Vex,
    };

    static inline const char *
    opcodeTypeToStr(OpcodeType type)
    {
        switch (type) {
          case BadOpcode:
            return "bad";
          case OneByteOpcode:
            return "one byte";
          case TwoByteOpcode:
            return "two byte";
          case ThreeByte0F38Opcode:
            return "three byte 0f38";
          case ThreeByte0F3AOpcode:
            return "three byte 0f3a";
          case Vex:
            return "vex";
          default:
            return "unrecognized!";
        }
    }

    BitUnion8(Opcode)
        Bitfield<7,3> top5;
        Bitfield<2,0> bottom3;
    EndBitUnion(Opcode)

    BitUnion8(OperatingMode)
        Bitfield<3> mode;
        Bitfield<2,0> submode;
    EndBitUnion(OperatingMode)

    enum X86Mode {
        LongMode,
        LegacyMode
    };

    enum X86SubMode {
        SixtyFourBitMode,
        CompatabilityMode,
        ProtectedMode,
        Virtual8086Mode,
        RealMode
    };

    //The intermediate structure used by the x86 decoder.
    struct ExtMachInst
    {
        //Prefixes
        LegacyPrefixVector legacy;
        Rex rex;
        // We use the following field for encoding both two byte and three byte
        // escape sequences
        ThreeByteVex vex;

        //This holds all of the bytes of the opcode
        struct
        {
            OpcodeType type;
            //The main opcode byte. The highest addressed byte in the opcode.
            Opcode op;
        } opcode;
        //Modifier bytes
        ModRM modRM;
        Sib sib;
        //Immediate fields
        uint64_t immediate;
        uint64_t displacement;

        //The effective operand size.
        uint8_t opSize;
        //The effective address size.
        uint8_t addrSize;
        //The effective stack size.
        uint8_t stackSize;
        //The size of the displacement
        uint8_t dispSize;

        //Mode information
        OperatingMode mode;
    };

    inline static std::ostream &
        operator << (std::ostream & os, const ExtMachInst & emi)
    {
        ccprintf(os, "\n{\n\tleg = %#x,\n\trex = %#x,\n\t"
                     "vex/xop = %#x,\n\t"
                     "op = {\n\t\ttype = %s,\n\t\top = %#x,\n\t\t},\n\t"
                     "modRM = %#x,\n\tsib = %#x,\n\t"
                     "immediate = %#x,\n\tdisplacement = %#x\n\t"
                     "dispSize = %d}\n",
                     (uint8_t)emi.legacy, (uint8_t)emi.rex,
                     (uint32_t)emi.vex,
                     opcodeTypeToStr(emi.opcode.type), (uint8_t)emi.opcode.op,
                     (uint8_t)emi.modRM, (uint8_t)emi.sib,
                     emi.immediate, emi.displacement, emi.dispSize);
        return os;
    }

    inline static bool
        operator == (const ExtMachInst &emi1, const ExtMachInst &emi2)
    {
        if (emi1.legacy != emi2.legacy)
            return false;
        if (emi1.rex != emi2.rex)
            return false;
        if (emi1.opcode.type != emi2.opcode.type)
            return false;
        if (emi1.opcode.op != emi2.opcode.op)
            return false;
        if (emi1.modRM != emi2.modRM)
            return false;
        if (emi1.sib != emi2.sib)
            return false;
        if (emi1.immediate != emi2.immediate)
            return false;
        if (emi1.displacement != emi2.displacement)
            return false;
        if (emi1.mode != emi2.mode)
            return false;
        if (emi1.opSize != emi2.opSize)
            return false;
        if (emi1.addrSize != emi2.addrSize)
            return false;
        if (emi1.stackSize != emi2.stackSize)
            return false;
        if (emi1.dispSize != emi2.dispSize)
            return false;
        return true;
    }

    class PCState : public GenericISA::UPCState<MachInst>
    {
      protected:
        typedef GenericISA::UPCState<MachInst> Base;

        uint8_t _size;

      public:
        void
        set(Addr val)
        {
            Base::set(val);
            _size = 0;
        }

        PCState() {}
        PCState(Addr val) { set(val); }

        uint8_t size() const { return _size; }
        void size(uint8_t newSize) { _size = newSize; }

        bool
        branching() const
        {
            return this->npc() != this->pc() + size();
        }

        void
        advance()
        {
            Base::advance();
            _size = 0;
        }

        void
        uEnd()
        {
            Base::uEnd();
            _size = 0;
        }

        void
        serialize(CheckpointOut &cp) const
        {
            Base::serialize(cp);
            SERIALIZE_SCALAR(_size);
        }

        void
        unserialize(CheckpointIn &cp)
        {
            Base::unserialize(cp);
            UNSERIALIZE_SCALAR(_size);
        }
    };

}

namespace std {
    template<>
    struct hash<X86ISA::ExtMachInst> {
        size_t operator()(const X86ISA::ExtMachInst &emi) const {
            return (((uint64_t)emi.legacy << 40) |
                    ((uint64_t)emi.rex  << 32) |
                    ((uint64_t)emi.modRM << 24) |
                    ((uint64_t)emi.sib << 16) |
                    ((uint64_t)emi.opcode.type << 8) |
                    ((uint64_t)emi.opcode.op)) ^
                    emi.immediate ^ emi.displacement ^
                    emi.mode ^
                    emi.opSize ^ emi.addrSize ^
                    emi.stackSize ^ emi.dispSize;
        };
    };
}

// These two functions allow ExtMachInst to be used with SERIALIZE_SCALAR
// and UNSERIALIZE_SCALAR.
template <>
void
paramOut(CheckpointOut &cp, const std::string &name,
         const X86ISA::ExtMachInst &machInst);
template <>
void
paramIn(CheckpointIn &cp, const std::string &name,
        X86ISA::ExtMachInst &machInst);

#endif // __ARCH_X86_TYPES_HH__
