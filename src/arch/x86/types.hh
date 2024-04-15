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
 */

#ifndef __ARCH_X86_TYPES_HH__
#define __ARCH_X86_TYPES_HH__

#include <cstdint>
#include <functional>
#include <iostream>

#include "arch/x86/pcstate.hh"
#include "base/bitunion.hh"
#include "base/cprintf.hh"

namespace gem5
{

namespace X86ISA
{

// This really determines how many bytes are passed to the decoder.
typedef uint64_t MachInst;

enum Prefixes
{
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
    // There can be only one segment override, so they share the
    // first 3 bits in the legacyPrefixes bitfield.
    Bitfield<2, 0> seg;
EndBitUnion(LegacyPrefixVector)

BitUnion8(ModRM)
    Bitfield<7, 6> mod;
    Bitfield<5, 3> reg;
    Bitfield<2, 0> rm;
EndBitUnion(ModRM)

BitUnion8(Sib)
    Bitfield<7, 6> scale;
    Bitfield<5, 3> index;
    Bitfield<2, 0> base;
EndBitUnion(Sib)

BitUnion8(Rex)
    // This bit doesn't mean anything according to the ISA, but in
    // this implementation, it being set means an REX prefix was present.
    Bitfield<6> present;
    Bitfield<3> w;
    Bitfield<2> r;
    Bitfield<1> x;
    Bitfield<0> b;
EndBitUnion(Rex)

BitUnion8(Vex2Of3)
    // Inverted bits from the REX prefix.
    Bitfield<7> r;
    Bitfield<6> x;
    Bitfield<5> b;
    // Selector for what would be two or three byte opcode types.
    Bitfield<4, 0> m;
EndBitUnion(Vex2Of3)

BitUnion8(Vex3Of3)
    // Bit from the REX prefix.
    Bitfield<7> w;
    // Inverted extra register index.
    Bitfield<6, 3> v;
    // Vector length specifier.
    Bitfield<2> l;
    // Implied 66, F2, or F3 opcode prefix.
    Bitfield<1, 0> p;
EndBitUnion(Vex3Of3)

BitUnion8(Vex2Of2)
    // Inverted bit from the REX prefix.
    Bitfield<7> r;
    // Inverted extra register index.
    Bitfield<6, 3> v;
    // Vector length specifier
    Bitfield<2> l;
    // Implied 66, F2, or F3 opcode prefix.
    Bitfield<1, 0> p;
EndBitUnion(Vex2Of2)

BitUnion8(VexInfo)
    // Extra register index.
    Bitfield<6, 3> v;
    // Vector length specifier.
    Bitfield<2> l;
    // Whether the VEX prefix was used.
    Bitfield<0> present;
EndBitUnion(VexInfo)

enum OpcodeType
{
    BadOpcode,
    OneByteOpcode,
    TwoByteOpcode,
    ThreeByte0F38Opcode,
    ThreeByte0F3AOpcode,
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
    default:
        return "unrecognized!";
    }
}

BitUnion8(Opcode)
    Bitfield<7, 3> top5;
    Bitfield<2, 0> bottom3;
EndBitUnion(Opcode)

BitUnion8(OperatingMode)
    Bitfield<3> mode;
    Bitfield<2, 0> submode;
EndBitUnion(OperatingMode)

BitUnion8(OperatingModeAndCPL)
    Bitfield<5, 4> cpl;
    Bitfield<3> mode;
    Bitfield<2, 0> submode;
EndBitUnion(OperatingModeAndCPL)

enum X86Mode
{
    LongMode,
    LegacyMode
};

enum X86SubMode
{
    SixtyFourBitMode,
    CompatabilityMode,
    ProtectedMode,
    Virtual8086Mode,
    RealMode
};

// The intermediate structure used by the x86 decoder.
struct ExtMachInst
{
    void
    reset()
    {
        memset(static_cast<void *>(this), 0, sizeof(*this));
    }

    // Prefixes
    LegacyPrefixVector legacy;
    Rex rex;
    VexInfo vex;

    // This holds all of the bytes of the opcode
    struct
    {
        OpcodeType type;
        // The main opcode byte. The highest addressed byte in the opcode.
        Opcode op;
    } opcode;

    // Modifier bytes
    ModRM modRM;
    Sib sib;
    // Immediate fields
    uint64_t immediate;
    uint64_t displacement;

    // The effective operand size.
    uint8_t opSize;
    // The effective address size.
    uint8_t addrSize;
    // The effective stack size.
    uint8_t stackSize;
    // The size of the displacement
    uint8_t dispSize;

    // Mode information
    OperatingModeAndCPL mode;
};

inline static std::ostream &
operator<<(std::ostream &os, const ExtMachInst &emi)
{
    ccprintf(os,
             "\n{\n\tleg = %#x,\n\trex = %#x,\n\t"
             "vex/xop = %#x,\n\t"
             "op = {\n\t\ttype = %s,\n\t\top = %#x,\n\t\t},\n\t"
             "modRM = %#x,\n\tsib = %#x,\n\t"
             "immediate = %#x,\n\tdisplacement = %#x\n\t"
             "dispSize = %d}\n",
             (uint8_t)emi.legacy, (uint8_t)emi.rex, (uint8_t)emi.vex,
             opcodeTypeToStr(emi.opcode.type), (uint8_t)emi.opcode.op,
             (uint8_t)emi.modRM, (uint8_t)emi.sib, emi.immediate,
             emi.displacement, emi.dispSize);
    return os;
}

inline static bool
operator==(const ExtMachInst &emi1, const ExtMachInst &emi2)
{
    if (emi1.legacy != emi2.legacy)
        return false;
    if (emi1.rex != emi2.rex)
        return false;
    if (emi1.vex != emi2.vex)
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

} // namespace X86ISA

// These two functions allow ExtMachInst to be used with SERIALIZE_SCALAR
// and UNSERIALIZE_SCALAR.
template <>
void paramOut(CheckpointOut &cp, const std::string &name,
              const X86ISA::ExtMachInst &machInst);
template <>
void paramIn(CheckpointIn &cp, const std::string &name,
             X86ISA::ExtMachInst &machInst);

} // namespace gem5

namespace std
{

template <>
struct hash<gem5::X86ISA::ExtMachInst>
{
    size_t
    operator()(const gem5::X86ISA::ExtMachInst &emi) const
    {
        return (((uint64_t)emi.legacy << 48) | ((uint64_t)emi.rex << 40) |
                ((uint64_t)emi.vex << 32) | ((uint64_t)emi.modRM << 24) |
                ((uint64_t)emi.sib << 16) | ((uint64_t)emi.opcode.type << 8) |
                ((uint64_t)emi.opcode.op)) ^
               emi.immediate ^ emi.displacement ^ emi.mode ^ emi.opSize ^
               emi.addrSize ^ emi.stackSize ^ emi.dispSize;
    };
};

} // namespace std

#endif // __ARCH_X86_TYPES_HH__
