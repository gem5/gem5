// University of Illinois/NCSA
// Open Source License
//
// Copyright (c) 2013-2015, Advanced Micro Devices, Inc.
// All rights reserved.
//
// Developed by:
//
//     HSA Team
//
//     Advanced Micro Devices, Inc
//
//     www.amd.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal with
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
//     * Redistributions of source code must retain the above copyright notice,
//       this list of conditions and the following disclaimers.
//
//     * Redistributions in binary form must reproduce the above copyright notice,
//       this list of conditions and the following disclaimers in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the names of the LLVM Team, University of Illinois at
//       Urbana-Champaign, nor the names of its contributors may be used to
//       endorse or promote products derived from this Software without specific
//       prior written permission.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE
// SOFTWARE.

//.ignore{

#ifndef INCLUDED_BRIG_H
#define INCLUDED_BRIG_H

#include <stdint.h>

enum BrigAuxDefs {
  MAX_OPERANDS_NUM = 6
};

//}

typedef uint32_t BrigVersion32_t;

enum BrigVersion {

    //.nowrap
    //.nodump
    //.nollvm

    BRIG_VERSION_HSAIL_MAJOR = 1,
    BRIG_VERSION_HSAIL_MINOR = 0,
    BRIG_VERSION_BRIG_MAJOR  = 1,
    BRIG_VERSION_BRIG_MINOR  = 0
};

typedef uint8_t BrigAlignment8_t;                           //.defValue=BRIG_ALIGNMENT_NONE

typedef uint8_t BrigAllocation8_t;                          //.defValue=BRIG_ALLOCATION_NONE

typedef uint8_t BrigAluModifier8_t;

typedef uint8_t BrigAtomicOperation8_t;

typedef uint32_t BrigCodeOffset32_t;                        //.defValue=0   //.wtype=ItemRef<Code>

typedef uint8_t BrigCompareOperation8_t;

typedef uint16_t BrigControlDirective16_t;

typedef uint32_t BrigDataOffset32_t;

typedef BrigDataOffset32_t BrigDataOffsetCodeList32_t;      //.wtype=ListRef<Code>      //.defValue=0

typedef BrigDataOffset32_t BrigDataOffsetOperandList32_t;   //.wtype=ListRef<Operand>   //.defValue=0

typedef BrigDataOffset32_t BrigDataOffsetString32_t;        //.wtype=StrRef             //.defValue=0

typedef uint8_t BrigExecutableModifier8_t;

typedef uint8_t BrigImageChannelOrder8_t;                   //.defValue=BRIG_CHANNEL_ORDER_UNKNOWN

typedef uint8_t BrigImageChannelType8_t;                    //.defValue=BRIG_CHANNEL_TYPE_UNKNOWN

typedef uint8_t BrigImageGeometry8_t;                       //.defValue=BRIG_GEOMETRY_UNKNOWN

typedef uint8_t BrigImageQuery8_t;

typedef uint16_t BrigKind16_t;

typedef uint8_t BrigLinkage8_t;                             //.defValue=BRIG_LINKAGE_NONE

typedef uint8_t BrigMachineModel8_t;                        //.defValue=BRIG_MACHINE_LARGE

typedef uint8_t BrigMemoryModifier8_t;

typedef uint8_t BrigMemoryOrder8_t;                         //.defValue=BRIG_MEMORY_ORDER_RELAXED

typedef uint8_t BrigMemoryScope8_t;                         //.defValue=BRIG_MEMORY_SCOPE_SYSTEM

typedef uint16_t BrigOpcode16_t;

typedef uint32_t BrigOperandOffset32_t;                     //.defValue=0 //.wtype=ItemRef<Operand>

typedef uint8_t BrigPack8_t;                                //.defValue=BRIG_PACK_NONE

typedef uint8_t BrigProfile8_t;                             //.defValue=BRIG_PROFILE_FULL

typedef uint16_t BrigRegisterKind16_t;

typedef uint8_t BrigRound8_t;                               //.defValue=BRIG_ROUND_NONE

typedef uint8_t BrigSamplerAddressing8_t;                   //.defValue=BRIG_ADDRESSING_CLAMP_TO_EDGE

typedef uint8_t BrigSamplerCoordNormalization8_t;

typedef uint8_t BrigSamplerFilter8_t;

typedef uint8_t BrigSamplerQuery8_t;

typedef uint32_t BrigSectionIndex32_t;

typedef uint8_t BrigSegCvtModifier8_t;

typedef uint8_t BrigSegment8_t;                             //.defValue=BRIG_SEGMENT_NONE

typedef uint32_t BrigStringOffset32_t;                      //.defValue=0       //.wtype=StrRef

typedef uint16_t BrigType16_t;

typedef uint8_t BrigVariableModifier8_t;

typedef uint8_t BrigWidth8_t;

typedef uint32_t BrigExceptions32_t;

enum BrigKind {

    //.nollvm
    //
    //.wname={ s/^BRIG_KIND//; MACRO2Name($_) }
    //.mnemo=$wname{ $wname }
    //
    //.sizeof=$wname{ "sizeof(".$structs->{"Brig".$wname}->{rawbrig}.")" }
    //.sizeof_switch //.sizeof_proto="int size_of_brig_record(unsigned arg)" //.sizeof_default="return -1"
    //
    //.isBodyOnly={ "false" }
    //.isBodyOnly_switch //.isBodyOnly_proto="bool isBodyOnly(Directive d)" //.isBodyOnly_arg="d.kind()"
    //.isBodyOnly_default="assert(false); return false"
    //
    //.isToplevelOnly={ "false" }
    //.isToplevelOnly_switch //.isToplevelOnly_proto="bool isToplevelOnly(Directive d)" //.isToplevelOnly_arg="d.kind()"
    //.isToplevelOnly_default="assert(false); return false"

    BRIG_KIND_NONE = 0x0000,                        //.skip

    BRIG_KIND_DIRECTIVE_BEGIN = 0x1000,             //.skip
    BRIG_KIND_DIRECTIVE_ARG_BLOCK_END = 0x1000,     //.isBodyOnly=true
    BRIG_KIND_DIRECTIVE_ARG_BLOCK_START = 0x1001,   //.isBodyOnly=true
    BRIG_KIND_DIRECTIVE_COMMENT = 0x1002,
    BRIG_KIND_DIRECTIVE_CONTROL = 0x1003,           //.isBodyOnly=true
    BRIG_KIND_DIRECTIVE_EXTENSION = 0x1004,         //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_FBARRIER = 0x1005,
    BRIG_KIND_DIRECTIVE_FUNCTION = 0x1006,          //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_INDIRECT_FUNCTION = 0x1007, //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_KERNEL = 0x1008,            //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_LABEL = 0x1009,             //.isBodyOnly=true
    BRIG_KIND_DIRECTIVE_LOC = 0x100a,
    BRIG_KIND_DIRECTIVE_MODULE = 0x100b,            //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_PRAGMA = 0x100c,
    BRIG_KIND_DIRECTIVE_SIGNATURE = 0x100d,         //.isToplevelOnly=true
    BRIG_KIND_DIRECTIVE_VARIABLE = 0x100e,
    BRIG_KIND_DIRECTIVE_END = 0x100f,               //.skip

    BRIG_KIND_INST_BEGIN = 0x2000,                  //.skip
    BRIG_KIND_INST_ADDR = 0x2000,
    BRIG_KIND_INST_ATOMIC = 0x2001,
    BRIG_KIND_INST_BASIC = 0x2002,
    BRIG_KIND_INST_BR = 0x2003,
    BRIG_KIND_INST_CMP = 0x2004,
    BRIG_KIND_INST_CVT = 0x2005,
    BRIG_KIND_INST_IMAGE = 0x2006,
    BRIG_KIND_INST_LANE = 0x2007,
    BRIG_KIND_INST_MEM = 0x2008,
    BRIG_KIND_INST_MEM_FENCE = 0x2009,
    BRIG_KIND_INST_MOD = 0x200a,
    BRIG_KIND_INST_QUERY_IMAGE = 0x200b,
    BRIG_KIND_INST_QUERY_SAMPLER = 0x200c,
    BRIG_KIND_INST_QUEUE = 0x200d,
    BRIG_KIND_INST_SEG = 0x200e,
    BRIG_KIND_INST_SEG_CVT = 0x200f,
    BRIG_KIND_INST_SIGNAL = 0x2010,
    BRIG_KIND_INST_SOURCE_TYPE = 0x2011,
    BRIG_KIND_INST_END = 0x2012,                    //.skip

    BRIG_KIND_OPERAND_BEGIN = 0x3000,               //.skip
    BRIG_KIND_OPERAND_ADDRESS = 0x3000,
    BRIG_KIND_OPERAND_ALIGN = 0x3001,
    BRIG_KIND_OPERAND_CODE_LIST = 0x3002,
    BRIG_KIND_OPERAND_CODE_REF = 0x3003,
    BRIG_KIND_OPERAND_CONSTANT_BYTES = 0x3004,
    BRIG_KIND_OPERAND_RESERVED = 0x3005, //.skip
    BRIG_KIND_OPERAND_CONSTANT_IMAGE = 0x3006,
    BRIG_KIND_OPERAND_CONSTANT_OPERAND_LIST = 0x3007,
    BRIG_KIND_OPERAND_CONSTANT_SAMPLER = 0x3008,
    BRIG_KIND_OPERAND_OPERAND_LIST = 0x3009,
    BRIG_KIND_OPERAND_REGISTER = 0x300a,
    BRIG_KIND_OPERAND_STRING = 0x300b,
    BRIG_KIND_OPERAND_WAVESIZE = 0x300c,
    BRIG_KIND_OPERAND_END = 0x300d                  //.skip
};

enum BrigAlignment {

    //.mnemo={ s/^BRIG_ALIGNMENT_//; lc }
    //.mnemo_proto="const char* align2str(unsigned arg)"
    //
    //.bytes={ /(\d+)/ ? $1 : undef }
    //.bytes_switch //.bytes_proto="unsigned align2num(unsigned arg)" //.bytes_default="assert(false); return -1"
    //
    //.rbytes=$bytes{ $bytes }
    //.rbytes_switch //.rbytes_reverse //.rbytes_proto="BrigAlignment num2align(uint64_t arg)"
    //.rbytes_default="return BRIG_ALIGNMENT_LAST"
    //
    //.print=$bytes{ $bytes>1 ? "_align($bytes)" : "" }

    BRIG_ALIGNMENT_NONE = 0,                        //.no_mnemo
    BRIG_ALIGNMENT_1 = 1,                           //.mnemo=""
    BRIG_ALIGNMENT_2 = 2,
    BRIG_ALIGNMENT_4 = 3,
    BRIG_ALIGNMENT_8 = 4,
    BRIG_ALIGNMENT_16 = 5,
    BRIG_ALIGNMENT_32 = 6,
    BRIG_ALIGNMENT_64 = 7,
    BRIG_ALIGNMENT_128 = 8,
    BRIG_ALIGNMENT_256 = 9,

    BRIG_ALIGNMENT_LAST,                            //.skip
    BRIG_ALIGNMENT_MAX = BRIG_ALIGNMENT_LAST - 1    //.skip
};

enum BrigAllocation {

    //.mnemo={ s/^BRIG_ALLOCATION_//;lc }
    //.mnemo_token=EAllocKind

    BRIG_ALLOCATION_NONE = 0,       //.mnemo=""
    BRIG_ALLOCATION_PROGRAM = 1,
    BRIG_ALLOCATION_AGENT = 2,
    BRIG_ALLOCATION_AUTOMATIC = 3
};

enum BrigAluModifierMask {
    BRIG_ALU_FTZ = 1
};

enum BrigAtomicOperation {

    //.tdcaption="Atomic Operations"
    //
    //.mnemo={ s/^BRIG_ATOMIC_//;lc }
    //.mnemo_token=_EMAtomicOp
    //.mnemo_context=EInstModifierInstAtomicContext
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_ATOMIC_ADD = 0,
    BRIG_ATOMIC_AND = 1,
    BRIG_ATOMIC_CAS = 2,
    BRIG_ATOMIC_EXCH = 3,
    BRIG_ATOMIC_LD = 4,
    BRIG_ATOMIC_MAX = 5,
    BRIG_ATOMIC_MIN = 6,
    BRIG_ATOMIC_OR = 7,
    BRIG_ATOMIC_ST = 8,
    BRIG_ATOMIC_SUB = 9,
    BRIG_ATOMIC_WRAPDEC = 10,
    BRIG_ATOMIC_WRAPINC = 11,
    BRIG_ATOMIC_XOR = 12,
    BRIG_ATOMIC_WAIT_EQ = 13,
    BRIG_ATOMIC_WAIT_NE = 14,
    BRIG_ATOMIC_WAIT_LT = 15,
    BRIG_ATOMIC_WAIT_GTE = 16,
    BRIG_ATOMIC_WAITTIMEOUT_EQ = 17,
    BRIG_ATOMIC_WAITTIMEOUT_NE = 18,
    BRIG_ATOMIC_WAITTIMEOUT_LT = 19,
    BRIG_ATOMIC_WAITTIMEOUT_GTE = 20
};

enum BrigCompareOperation {

    //.tdcaption="Comparison Operators"
    //
    //.mnemo={ s/^BRIG_COMPARE_//;lc }
    //.mnemo_token=_EMCompare
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_COMPARE_EQ = 0,
    BRIG_COMPARE_NE = 1,
    BRIG_COMPARE_LT = 2,
    BRIG_COMPARE_LE = 3,
    BRIG_COMPARE_GT = 4,
    BRIG_COMPARE_GE = 5,
    BRIG_COMPARE_EQU = 6,
    BRIG_COMPARE_NEU = 7,
    BRIG_COMPARE_LTU = 8,
    BRIG_COMPARE_LEU = 9,
    BRIG_COMPARE_GTU = 10,
    BRIG_COMPARE_GEU = 11,
    BRIG_COMPARE_NUM = 12,
    BRIG_COMPARE_NAN = 13,
    BRIG_COMPARE_SEQ = 14,
    BRIG_COMPARE_SNE = 15,
    BRIG_COMPARE_SLT = 16,
    BRIG_COMPARE_SLE = 17,
    BRIG_COMPARE_SGT = 18,
    BRIG_COMPARE_SGE = 19,
    BRIG_COMPARE_SGEU = 20,
    BRIG_COMPARE_SEQU = 21,
    BRIG_COMPARE_SNEU = 22,
    BRIG_COMPARE_SLTU = 23,
    BRIG_COMPARE_SLEU = 24,
    BRIG_COMPARE_SNUM = 25,
    BRIG_COMPARE_SNAN = 26,
    BRIG_COMPARE_SGTU = 27
};

enum BrigControlDirective {

    //.mnemo={ s/^BRIG_CONTROL_//;lc }
    //.mnemo_token=EControl
    //
    //.print=$mnemo{ $mnemo }

    BRIG_CONTROL_NONE = 0, //.skip
    BRIG_CONTROL_ENABLEBREAKEXCEPTIONS = 1,
    BRIG_CONTROL_ENABLEDETECTEXCEPTIONS = 2,
    BRIG_CONTROL_MAXDYNAMICGROUPSIZE = 3,
    BRIG_CONTROL_MAXFLATGRIDSIZE = 4,
    BRIG_CONTROL_MAXFLATWORKGROUPSIZE = 5,
    BRIG_CONTROL_REQUIREDDIM = 6,
    BRIG_CONTROL_REQUIREDGRIDSIZE = 7,
    BRIG_CONTROL_REQUIREDWORKGROUPSIZE = 8,
    BRIG_CONTROL_REQUIRENOPARTIALWORKGROUPS = 9
};

enum BrigExecutableModifierMask {
    //.nodump
    BRIG_EXECUTABLE_DEFINITION = 1
};

enum BrigImageChannelOrder {

    //.mnemo={ s/^BRIG_CHANNEL_ORDER_?//;lc }
    //.mnemo_token=EImageOrder
    //.mnemo_context=EImageOrderContext
    //
    //.print=$mnemo{ $mnemo }

    BRIG_CHANNEL_ORDER_A = 0,
    BRIG_CHANNEL_ORDER_R = 1,
    BRIG_CHANNEL_ORDER_RX = 2,
    BRIG_CHANNEL_ORDER_RG = 3,
    BRIG_CHANNEL_ORDER_RGX = 4,
    BRIG_CHANNEL_ORDER_RA = 5,
    BRIG_CHANNEL_ORDER_RGB = 6,
    BRIG_CHANNEL_ORDER_RGBX = 7,
    BRIG_CHANNEL_ORDER_RGBA = 8,
    BRIG_CHANNEL_ORDER_BGRA = 9,
    BRIG_CHANNEL_ORDER_ARGB = 10,
    BRIG_CHANNEL_ORDER_ABGR = 11,
    BRIG_CHANNEL_ORDER_SRGB = 12,
    BRIG_CHANNEL_ORDER_SRGBX = 13,
    BRIG_CHANNEL_ORDER_SRGBA = 14,
    BRIG_CHANNEL_ORDER_SBGRA = 15,
    BRIG_CHANNEL_ORDER_INTENSITY = 16,
    BRIG_CHANNEL_ORDER_LUMINANCE = 17,
    BRIG_CHANNEL_ORDER_DEPTH = 18,
    BRIG_CHANNEL_ORDER_DEPTH_STENCIL = 19,

    // used internally
    BRIG_CHANNEL_ORDER_UNKNOWN, //.mnemo="" // used when no order is specified

    BRIG_CHANNEL_ORDER_FIRST_USER_DEFINED = 128 //.skip

};

enum BrigImageChannelType {

    //.mnemo={ s/^BRIG_CHANNEL_TYPE_//;lc }
    //.mnemo_token=EImageFormat
    //
    //.print=$mnemo{ $mnemo }

    BRIG_CHANNEL_TYPE_SNORM_INT8 = 0,
    BRIG_CHANNEL_TYPE_SNORM_INT16 = 1,
    BRIG_CHANNEL_TYPE_UNORM_INT8 = 2,
    BRIG_CHANNEL_TYPE_UNORM_INT16 = 3,
    BRIG_CHANNEL_TYPE_UNORM_INT24 = 4,
    BRIG_CHANNEL_TYPE_UNORM_SHORT_555 = 5,
    BRIG_CHANNEL_TYPE_UNORM_SHORT_565 = 6,
    BRIG_CHANNEL_TYPE_UNORM_INT_101010 = 7,
    BRIG_CHANNEL_TYPE_SIGNED_INT8 = 8,
    BRIG_CHANNEL_TYPE_SIGNED_INT16 = 9,
    BRIG_CHANNEL_TYPE_SIGNED_INT32 = 10,
    BRIG_CHANNEL_TYPE_UNSIGNED_INT8 = 11,
    BRIG_CHANNEL_TYPE_UNSIGNED_INT16 = 12,
    BRIG_CHANNEL_TYPE_UNSIGNED_INT32 = 13,
    BRIG_CHANNEL_TYPE_HALF_FLOAT = 14,
    BRIG_CHANNEL_TYPE_FLOAT = 15,

    // used internally
    BRIG_CHANNEL_TYPE_UNKNOWN, //.mnemo=""

    BRIG_CHANNEL_TYPE_FIRST_USER_DEFINED = 128 //.skip
};

enum BrigImageGeometry {

    //.tdcaption="Geometry"
    //
    //.mnemo={ s/^BRIG_GEOMETRY_//;lc }
    //.mnemo_token=EImageGeometry
    //
    //.dim={/_([0-9]+D)(A)?/ ? $1+(defined $2?1:0) : undef}
    //.dim_switch //.dim_proto="unsigned getBrigGeometryDim(unsigned geo)" //.dim_arg="geo"
    //.dim_default="assert(0); return 0"
    //
    //.depth={/DEPTH$/?"true":"false"}
    //.depth_switch //.depth_proto="bool isBrigGeometryDepth(unsigned geo)" //.depth_arg="geo"
    //.depth_default="return false"

    BRIG_GEOMETRY_1D = 0,
    BRIG_GEOMETRY_2D = 1,
    BRIG_GEOMETRY_3D = 2,
    BRIG_GEOMETRY_1DA = 3,
    BRIG_GEOMETRY_2DA = 4,
    BRIG_GEOMETRY_1DB = 5,
    BRIG_GEOMETRY_2DDEPTH = 6,
    BRIG_GEOMETRY_2DADEPTH = 7,

    // used internally
    BRIG_GEOMETRY_UNKNOWN, //.mnemo=""

    BRIG_GEOMETRY_FIRST_USER_DEFINED = 128 //.skip
};

enum BrigImageQuery {

    //.mnemo={ s/^BRIG_IMAGE_QUERY_//;lc }
    //
    //.print=$mnemo{ $mnemo }

    BRIG_IMAGE_QUERY_WIDTH = 0,
    BRIG_IMAGE_QUERY_HEIGHT = 1,
    BRIG_IMAGE_QUERY_DEPTH = 2,
    BRIG_IMAGE_QUERY_ARRAY = 3,
    BRIG_IMAGE_QUERY_CHANNELORDER = 4,
    BRIG_IMAGE_QUERY_CHANNELTYPE = 5,
    BRIG_IMAGE_QUERY_NUMMIPLEVELS = 6
};

enum BrigLinkage {

    //.mnemo={ s/^BRIG_LINKAGE_//;s/NONE//;lc }

    BRIG_LINKAGE_NONE = 0,
    BRIG_LINKAGE_PROGRAM = 1,
    BRIG_LINKAGE_MODULE = 2,
    BRIG_LINKAGE_FUNCTION = 3,
    BRIG_LINKAGE_ARG = 4
};

enum BrigMachineModel {

    //.mnemo={ s/^BRIG_MACHINE_//; '$'.lc }
    //.mnemo_token=ETargetMachine
    //
    //.print=$mnemo{ $mnemo }

    BRIG_MACHINE_SMALL = 0,
    BRIG_MACHINE_LARGE = 1,

    BRIG_MACHINE_UNDEF = 2 //.skip
};

enum BrigMemoryModifierMask { //.tddef=0
    BRIG_MEMORY_CONST = 1
};

enum BrigMemoryOrder {

    //.mnemo={ s/^BRIG_MEMORY_ORDER_//; lc }
    //.mnemo_token=_EMMemoryOrder
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_MEMORY_ORDER_NONE = 0,                 //.mnemo=""
    BRIG_MEMORY_ORDER_RELAXED = 1,              //.mnemo=rlx
    BRIG_MEMORY_ORDER_SC_ACQUIRE = 2,           //.mnemo=scacq
    BRIG_MEMORY_ORDER_SC_RELEASE = 3,           //.mnemo=screl
    BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE = 4,   //.mnemo=scar

    BRIG_MEMORY_ORDER_LAST = 5 //.skip
};

enum BrigMemoryScope {

    //.mnemo={ s/^BRIG_MEMORY_SCOPE_//; lc }
    //.mnemo_token=_EMMemoryScope
    //
    //.print=$mnemo{ $mnemo }

    BRIG_MEMORY_SCOPE_NONE = 0,         //.mnemo=""
    BRIG_MEMORY_SCOPE_WORKITEM = 1,     //.mnemo=""
    BRIG_MEMORY_SCOPE_WAVEFRONT = 2,    //.mnemo=wave
    BRIG_MEMORY_SCOPE_WORKGROUP = 3,    //.mnemo=wg
    BRIG_MEMORY_SCOPE_AGENT = 4,        //.mnemo=agent
    BRIG_MEMORY_SCOPE_SYSTEM = 5,       //.mnemo=system

    BRIG_MEMORY_SCOPE_LAST = 6 //.skip
};

enum BrigOpcode {

    //.tdcaption="Instruction Opcodes"
    //
    //.k={ "BASIC" }
    //.pscode=$k{ MACRO2Name("_".$k) }
    //.opcodeparser=$pscode{ return $pscode && "parseMnemo$pscode" }
    //.opcodeparser_incfile=ParserUtilities
    //.opcodeparser_switch //.opcodeparser_proto="OpcodeParser getOpcodeParser(BrigOpcode16_t arg)" //.opcodeparser_default="return parseMnemoBasic"
    //
    //.psopnd={undef}
    //.opndparser=$psopnd{ return $psopnd && "&Parser::parse$psopnd" }
    //.opndparser_incfile=ParserUtilities
    //.opndparser_switch //.opndparser_proto="Parser::OperandParser Parser::getOperandParser(BrigOpcode16_t arg)" //.opndparser_default="return &Parser::parseOperands"
    //
    //.mnemo={ s/^BRIG_OPCODE_//; s/GCN([^_])/GCN_$1/; lc }
    //.mnemo_scanner=Instructions //.mnemo_token=EInstruction
    //.mnemo_context=EDefaultContext
    //
    //.has_memory_order={undef}
    //.semsupport=$has_memory_order{ return $has_memory_order && "true" }
    //
    //.hasType=$k{ return ($k and $k eq "BASIC_NO_TYPE") ? "false" : undef; }
    //.hasType_switch //.hasType_proto="bool instHasType(BrigOpcode16_t arg)" //.hasType_default="return true"
    //
    //.opcodevis=$pscode{ s/^BRIG_OPCODE_//; sprintf("%-47s(","vis.visitOpcode_".$_) . ($pscode =~m/^(BasicOrMod|Nop)$/? "inst" : "HSAIL_ASM::Inst". ($pscode=~m/BasicNoType/? "Basic":$pscode) ."(inst)").")" }
    //.opcodevis_switch //.opcodevis_proto="template <typename RetType, typename Visitor> RetType visitOpcode_gen(HSAIL_ASM::Inst inst, Visitor& vis)"
    //.opcodevis_arg="inst.opcode()" //.opcodevis_default="return RetType()"
    //.opcodevis_incfile=ItemUtils
    //
    //.ftz=$k{ return ($k eq "BASIC_OR_MOD" or $k eq "CMP" or $k eq "CVT") ? "true" : undef }
    //.ftz_incfile=ItemUtils //.ftz_switch //.ftz_proto="inline bool instSupportsFtz(BrigOpcode16_t arg)" //.ftz_default="return false"
    //
    //.vecOpndIndex={undef}
    //.vecOpndIndex_switch  //.vecOpndIndex_proto="int vecOpndIndex(BrigOpcode16_t arg)" //.vecOpndIndex_default="return -1"
    //.vecOpndIndex_incfile=ParserUtilities
    //
    //.numdst={undef}
    //.numdst_switch //.numdst_proto="int instNumDstOperands(BrigOpcode16_t arg)" //.numdst_default="return 1"
    //
    //.print=$mnemo{ $mnemo }

    BRIG_OPCODE_NOP = 0,                    //.k=NOP            //.hasType=false
    BRIG_OPCODE_ABS = 1,                    //.k=BASIC_OR_MOD
    BRIG_OPCODE_ADD = 2,                    //.k=BASIC_OR_MOD
    BRIG_OPCODE_BORROW = 3,
    BRIG_OPCODE_CARRY = 4,
    BRIG_OPCODE_CEIL = 5,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_COPYSIGN = 6,               //.k=BASIC_OR_MOD
    BRIG_OPCODE_DIV = 7,                    //.k=BASIC_OR_MOD
    BRIG_OPCODE_FLOOR = 8,                  //.k=BASIC_OR_MOD
    BRIG_OPCODE_FMA = 9,                    //.k=BASIC_OR_MOD
    BRIG_OPCODE_FRACT = 10,                 //.k=BASIC_OR_MOD
    BRIG_OPCODE_MAD = 11,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_MAX = 12,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_MIN = 13,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_MUL = 14,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_MULHI = 15,                 //.k=BASIC_OR_MOD
    BRIG_OPCODE_NEG = 16,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_REM = 17,
    BRIG_OPCODE_RINT = 18,                  //.k=BASIC_OR_MOD
    BRIG_OPCODE_SQRT = 19,                  //.k=BASIC_OR_MOD
    BRIG_OPCODE_SUB = 20,                   //.k=BASIC_OR_MOD
    BRIG_OPCODE_TRUNC = 21,                 //.k=BASIC_OR_MOD
    BRIG_OPCODE_MAD24 = 22,
    BRIG_OPCODE_MAD24HI = 23,
    BRIG_OPCODE_MUL24 = 24,
    BRIG_OPCODE_MUL24HI = 25,
    BRIG_OPCODE_SHL = 26,
    BRIG_OPCODE_SHR = 27,
    BRIG_OPCODE_AND = 28,
    BRIG_OPCODE_NOT = 29,
    BRIG_OPCODE_OR = 30,
    BRIG_OPCODE_POPCOUNT = 31,              //.k=SOURCE_TYPE
    BRIG_OPCODE_XOR = 32,
    BRIG_OPCODE_BITEXTRACT = 33,
    BRIG_OPCODE_BITINSERT = 34,
    BRIG_OPCODE_BITMASK = 35,
    BRIG_OPCODE_BITREV = 36,
    BRIG_OPCODE_BITSELECT = 37,
    BRIG_OPCODE_FIRSTBIT = 38,              //.k=SOURCE_TYPE
    BRIG_OPCODE_LASTBIT = 39,               //.k=SOURCE_TYPE
    BRIG_OPCODE_COMBINE = 40,               //.k=SOURCE_TYPE    //.vecOpndIndex=1
    BRIG_OPCODE_EXPAND = 41,                //.k=SOURCE_TYPE    //.vecOpndIndex=0
    BRIG_OPCODE_LDA = 42,                   //.k=ADDR
    BRIG_OPCODE_MOV = 43,
    BRIG_OPCODE_SHUFFLE = 44,
    BRIG_OPCODE_UNPACKHI = 45,
    BRIG_OPCODE_UNPACKLO = 46,
    BRIG_OPCODE_PACK = 47,                  //.k=SOURCE_TYPE
    BRIG_OPCODE_UNPACK = 48,                //.k=SOURCE_TYPE
    BRIG_OPCODE_CMOV = 49,
    BRIG_OPCODE_CLASS = 50,                 //.k=SOURCE_TYPE
    BRIG_OPCODE_NCOS = 51,
    BRIG_OPCODE_NEXP2 = 52,
    BRIG_OPCODE_NFMA = 53,
    BRIG_OPCODE_NLOG2 = 54,
    BRIG_OPCODE_NRCP = 55,
    BRIG_OPCODE_NRSQRT = 56,
    BRIG_OPCODE_NSIN = 57,
    BRIG_OPCODE_NSQRT = 58,
    BRIG_OPCODE_BITALIGN = 59,
    BRIG_OPCODE_BYTEALIGN = 60,
    BRIG_OPCODE_PACKCVT = 61,               //.k=SOURCE_TYPE
    BRIG_OPCODE_UNPACKCVT = 62,             //.k=SOURCE_TYPE
    BRIG_OPCODE_LERP = 63,
    BRIG_OPCODE_SAD = 64,                   //.k=SOURCE_TYPE
    BRIG_OPCODE_SADHI = 65,                 //.k=SOURCE_TYPE
    BRIG_OPCODE_SEGMENTP = 66,              //.k=SEG_CVT
    BRIG_OPCODE_FTOS = 67,                  //.k=SEG_CVT
    BRIG_OPCODE_STOF = 68,                  //.k=SEG_CVT
    BRIG_OPCODE_CMP = 69,                   //.k=CMP
    BRIG_OPCODE_CVT = 70,                   //.k=CVT
    BRIG_OPCODE_LD = 71,                    //.k=MEM            //.has_memory_order //.vecOpndIndex=0
    BRIG_OPCODE_ST = 72,                    //.k=MEM            //.has_memory_order //.vecOpndIndex=0 //.numdst=0
    BRIG_OPCODE_ATOMIC = 73,                //.k=ATOMIC
    BRIG_OPCODE_ATOMICNORET = 74,           //.k=ATOMIC         //.numdst=0
    BRIG_OPCODE_SIGNAL = 75,                //.k=SIGNAL
    BRIG_OPCODE_SIGNALNORET = 76,           //.k=SIGNAL         //.numdst=0
    BRIG_OPCODE_MEMFENCE = 77,              //.k=MEM_FENCE      //.numdst=0
    BRIG_OPCODE_RDIMAGE = 78,               //.k=IMAGE          //.vecOpndIndex=0
    BRIG_OPCODE_LDIMAGE = 79,               //.k=IMAGE          //.vecOpndIndex=0
    BRIG_OPCODE_STIMAGE = 80,               //.k=IMAGE          //.vecOpndIndex=0 //.numdst=0
    BRIG_OPCODE_IMAGEFENCE = 81,            //.k=BASIC_NO_TYPE
    BRIG_OPCODE_QUERYIMAGE = 82,            //.k=QUERY_IMAGE
    BRIG_OPCODE_QUERYSAMPLER = 83,          //.k=QUERY_SAMPLER
    BRIG_OPCODE_CBR = 84,                   //.k=BR             //.numdst=0
    BRIG_OPCODE_BR = 85,                    //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_SBR = 86,                   //.k=BR             //.numdst=0     //.psopnd=SbrOperands
    BRIG_OPCODE_BARRIER = 87,               //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_WAVEBARRIER = 88,           //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_ARRIVEFBAR = 89,            //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_INITFBAR = 90,              //.k=BASIC_NO_TYPE  //.numdst=0     //.hasType=false
    BRIG_OPCODE_JOINFBAR = 91,              //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_LEAVEFBAR = 92,             //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_RELEASEFBAR = 93,           //.k=BASIC_NO_TYPE  //.numdst=0
    BRIG_OPCODE_WAITFBAR = 94,              //.k=BR             //.numdst=0     //.hasType=false
    BRIG_OPCODE_LDF = 95,
    BRIG_OPCODE_ACTIVELANECOUNT = 96,       //.k=LANE
    BRIG_OPCODE_ACTIVELANEID = 97,          //.k=LANE
    BRIG_OPCODE_ACTIVELANEMASK = 98,        //.k=LANE           //.vecOpndIndex=0
    BRIG_OPCODE_ACTIVELANEPERMUTE = 99,     //.k=LANE
    BRIG_OPCODE_CALL = 100,                 //.k=BR             //.psopnd=CallOperands //.numdst=0 //.hasType=false
    BRIG_OPCODE_SCALL = 101,                //.k=BR             //.psopnd=CallOperands //.numdst=0
    BRIG_OPCODE_ICALL = 102,                //.k=BR             //.psopnd=CallOperands //.numdst=0
    BRIG_OPCODE_RET = 103,                  //.k=BASIC_NO_TYPE
    BRIG_OPCODE_ALLOCA = 104,               //.k=MEM
    BRIG_OPCODE_CURRENTWORKGROUPSIZE = 105,
    BRIG_OPCODE_CURRENTWORKITEMFLATID = 106,
    BRIG_OPCODE_DIM = 107,
    BRIG_OPCODE_GRIDGROUPS = 108,
    BRIG_OPCODE_GRIDSIZE = 109,
    BRIG_OPCODE_PACKETCOMPLETIONSIG = 110,
    BRIG_OPCODE_PACKETID = 111,
    BRIG_OPCODE_WORKGROUPID = 112,
    BRIG_OPCODE_WORKGROUPSIZE = 113,
    BRIG_OPCODE_WORKITEMABSID = 114,
    BRIG_OPCODE_WORKITEMFLATABSID = 115,
    BRIG_OPCODE_WORKITEMFLATID = 116,
    BRIG_OPCODE_WORKITEMID = 117,
    BRIG_OPCODE_CLEARDETECTEXCEPT = 118,    //.numdst=0
    BRIG_OPCODE_GETDETECTEXCEPT = 119,
    BRIG_OPCODE_SETDETECTEXCEPT = 120,      //.numdst=0
    BRIG_OPCODE_ADDQUEUEWRITEINDEX = 121,   //.k=QUEUE
    BRIG_OPCODE_CASQUEUEWRITEINDEX = 122,   //.k=QUEUE
    BRIG_OPCODE_LDQUEUEREADINDEX = 123,     //.k=QUEUE
    BRIG_OPCODE_LDQUEUEWRITEINDEX = 124,    //.k=QUEUE
    BRIG_OPCODE_STQUEUEREADINDEX = 125,     //.k=QUEUE      //.numdst=0
    BRIG_OPCODE_STQUEUEWRITEINDEX = 126,    //.k=QUEUE      //.numdst=0
    BRIG_OPCODE_CLOCK = 127,
    BRIG_OPCODE_CUID = 128,
    BRIG_OPCODE_DEBUGTRAP = 129,            //.numdst=0
    BRIG_OPCODE_GROUPBASEPTR = 130,
    BRIG_OPCODE_KERNARGBASEPTR = 131,
    BRIG_OPCODE_LANEID = 132,
    BRIG_OPCODE_MAXCUID = 133,
    BRIG_OPCODE_MAXWAVEID = 134,
    BRIG_OPCODE_NULLPTR = 135,              //.k=SEG
    BRIG_OPCODE_WAVEID = 136,
    BRIG_OPCODE_FIRST_USER_DEFINED = 32768, //.skip

    BRIG_OPCODE_GCNMADU = (1u << 15) | 0,           //.k=BASIC_NO_TYPE
    BRIG_OPCODE_GCNMADS = (1u << 15) | 1,           //.k=BASIC_NO_TYPE
    BRIG_OPCODE_GCNMAX3 = (1u << 15) | 2,
    BRIG_OPCODE_GCNMIN3 = (1u << 15) | 3,
    BRIG_OPCODE_GCNMED3 = (1u << 15) | 4,
    BRIG_OPCODE_GCNFLDEXP = (1u << 15) | 5,         //.k=BASIC_OR_MOD
    BRIG_OPCODE_GCNFREXP_EXP = (1u << 15) | 6,      //.k=BASIC_OR_MOD
    BRIG_OPCODE_GCNFREXP_MANT = (1u << 15) | 7,     //.k=BASIC_OR_MOD
    BRIG_OPCODE_GCNTRIG_PREOP = (1u << 15) | 8,     //.k=BASIC_OR_MOD
    BRIG_OPCODE_GCNBFM = (1u << 15) | 9,
    BRIG_OPCODE_GCNLD = (1u << 15) | 10,            //.k=MEM            //.has_memory_order //.vecOpndIndex=0
    BRIG_OPCODE_GCNST = (1u << 15) | 11,            //.k=MEM            //.has_memory_order //.vecOpndIndex=0
    BRIG_OPCODE_GCNATOMIC = (1u << 15) | 12,        //.k=ATOMIC
    BRIG_OPCODE_GCNATOMICNORET = (1u << 15) | 13,   //.k=ATOMIC         //.mnemo=gcn_atomicNoRet
    BRIG_OPCODE_GCNSLEEP = (1u << 15) | 14,
    BRIG_OPCODE_GCNPRIORITY = (1u << 15) | 15,
    BRIG_OPCODE_GCNREGIONALLOC = (1u << 15) | 16,   //.k=BASIC_NO_TYPE //.mnemo=gcn_region_alloc
    BRIG_OPCODE_GCNMSAD = (1u << 15) | 17,
    BRIG_OPCODE_GCNQSAD = (1u << 15) | 18,
    BRIG_OPCODE_GCNMQSAD = (1u << 15) | 19,
    BRIG_OPCODE_GCNMQSAD4 = (1u << 15) | 20,        //.k=BASIC_NO_TYPE
    BRIG_OPCODE_GCNSADW = (1u << 15) | 21,
    BRIG_OPCODE_GCNSADD = (1u << 15) | 22,
    BRIG_OPCODE_GCNCONSUME = (1u << 15) | 23,       //.k=ADDR           //.mnemo=gcn_atomic_consume
    BRIG_OPCODE_GCNAPPEND = (1u << 15) | 24,        //.k=ADDR           //.mnemo=gcn_atomic_append
    BRIG_OPCODE_GCNB4XCHG = (1u << 15) | 25,        //.mnemo=gcn_b4xchg
    BRIG_OPCODE_GCNB32XCHG = (1u << 15) | 26,       //.mnemo=gcn_b32xchg
    BRIG_OPCODE_GCNMAX = (1u << 15) | 27,
    BRIG_OPCODE_GCNMIN = (1u << 15) | 28,
    BRIG_OPCODE_GCNDIVRELAXED = (1u << 15) | 29,    //.k=BASIC_OR_MOD
    BRIG_OPCODE_GCNDIVRELAXEDNARROW = (1u << 15) | 30,

    BRIG_OPCODE_AMDRDIMAGELOD  = (1u << 15) | 31,    //.k=IMAGE //.mnemo=amd_rdimagelod  //.vecOpndIndex=0
    BRIG_OPCODE_AMDRDIMAGEGRAD = (1u << 15) | 32,    //.k=IMAGE //.mnemo=amd_rdimagegrad //.vecOpndIndex=0
    BRIG_OPCODE_AMDLDIMAGEMIP  = (1u << 15) | 33,    //.k=IMAGE //.mnemo=amd_ldimagemip //.vecOpndIndex=0
    BRIG_OPCODE_AMDSTIMAGEMIP  = (1u << 15) | 34,    //.k=IMAGE //.mnemo=amd_stimagemip //.vecOpndIndex=0 //.numdst=0
    BRIG_OPCODE_AMDQUERYIMAGE  = (1u << 15) | 35     //.k=QUERY_IMAGE //.mnemo=amd_queryimage
};

enum BrigPack {

    //.tdcaption="Packing"
    //
    //.mnemo={ s/^BRIG_PACK_//;s/SAT$/_sat/;lc }
    //.mnemo_token=_EMPacking
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_PACK_NONE = 0, //.mnemo=""
    BRIG_PACK_PP = 1,
    BRIG_PACK_PS = 2,
    BRIG_PACK_SP = 3,
    BRIG_PACK_SS = 4,
    BRIG_PACK_S = 5,
    BRIG_PACK_P = 6,
    BRIG_PACK_PPSAT = 7,
    BRIG_PACK_PSSAT = 8,
    BRIG_PACK_SPSAT = 9,
    BRIG_PACK_SSSAT = 10,
    BRIG_PACK_SSAT = 11,
    BRIG_PACK_PSAT = 12
};

enum BrigProfile {

    //.mnemo={ s/^BRIG_PROFILE_//;'$'.lc }
    //.mnemo_token=ETargetProfile
    //
    //.print=$mnemo{ $mnemo }

    BRIG_PROFILE_BASE = 0,
    BRIG_PROFILE_FULL = 1,

    BRIG_PROFILE_UNDEF = 2 //.skip
};

enum BrigRegisterKind {

    //.mnemo={ s/^BRIG_REGISTER_KIND_//;'$'.lc(substr($_,0,1)) }
    //
    //.bits={ }
    //.bits_switch //.bits_proto="unsigned getRegBits(BrigRegisterKind16_t arg)" //.bits_default="return (unsigned)-1"
    //
    //.nollvm

    BRIG_REGISTER_KIND_CONTROL = 0, //.bits=1
    BRIG_REGISTER_KIND_SINGLE = 1,  //.bits=32
    BRIG_REGISTER_KIND_DOUBLE = 2,  //.bits=64
    BRIG_REGISTER_KIND_QUAD = 3     //.bits=128
};

enum BrigRound {

    //.mnemo={}
    //.mnemo_fn=round2str //.mnemo_token=_EMRound
    //
    //.sat={/_SAT$/? "true" : "false"}
    //.sat_switch //.sat_proto="bool isSatRounding(unsigned rounding)" //.sat_arg="rounding"
    //.sat_default="return false"
    //
    //.sig={/_SIGNALING_/? "true" : "false"}
    //.sig_switch //.sig_proto="bool isSignalingRounding(unsigned rounding)" //.sig_arg="rounding"
    //.sig_default="return false"
    //
    //.int={/_INTEGER_/? "true" : "false"}
    //.int_switch //.int_proto="bool isIntRounding(unsigned rounding)" //.int_arg="rounding"
    //.int_default="return false"
    //
    //.flt={/_FLOAT_/? "true" : "false"}
    //.flt_switch //.flt_proto="bool isFloatRounding(unsigned rounding)" //.flt_arg="rounding"
    //.flt_default="return false"
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_ROUND_NONE = 0,                                    //.no_mnemo
    BRIG_ROUND_FLOAT_DEFAULT = 1,                           //.no_mnemo
    BRIG_ROUND_FLOAT_NEAR_EVEN = 2,                         //.mnemo=near
    BRIG_ROUND_FLOAT_ZERO = 3,                              //.mnemo=zero
    BRIG_ROUND_FLOAT_PLUS_INFINITY = 4,                     //.mnemo=up
    BRIG_ROUND_FLOAT_MINUS_INFINITY = 5,                    //.mnemo=down
    BRIG_ROUND_INTEGER_NEAR_EVEN = 6,                       //.mnemo=neari
    BRIG_ROUND_INTEGER_ZERO = 7,                            //.mnemo=zeroi
    BRIG_ROUND_INTEGER_PLUS_INFINITY = 8,                   //.mnemo=upi
    BRIG_ROUND_INTEGER_MINUS_INFINITY = 9,                  //.mnemo=downi
    BRIG_ROUND_INTEGER_NEAR_EVEN_SAT = 10,                  //.mnemo=neari_sat
    BRIG_ROUND_INTEGER_ZERO_SAT = 11,                       //.mnemo=zeroi_sat
    BRIG_ROUND_INTEGER_PLUS_INFINITY_SAT = 12,              //.mnemo=upi_sat
    BRIG_ROUND_INTEGER_MINUS_INFINITY_SAT = 13,             //.mnemo=downi_sat
    BRIG_ROUND_INTEGER_SIGNALING_NEAR_EVEN = 14,            //.mnemo=sneari
    BRIG_ROUND_INTEGER_SIGNALING_ZERO = 15,                 //.mnemo=szeroi
    BRIG_ROUND_INTEGER_SIGNALING_PLUS_INFINITY = 16,        //.mnemo=supi
    BRIG_ROUND_INTEGER_SIGNALING_MINUS_INFINITY = 17,       //.mnemo=sdowni
    BRIG_ROUND_INTEGER_SIGNALING_NEAR_EVEN_SAT = 18,        //.mnemo=sneari_sat
    BRIG_ROUND_INTEGER_SIGNALING_ZERO_SAT = 19,             //.mnemo=szeroi_sat
    BRIG_ROUND_INTEGER_SIGNALING_PLUS_INFINITY_SAT = 20,    //.mnemo=supi_sat
    BRIG_ROUND_INTEGER_SIGNALING_MINUS_INFINITY_SAT = 21    //.mnemo=sdowni_sat
};

enum BrigSamplerAddressing {

    //.mnemo={ s/^BRIG_ADDRESSING_//;lc }
    //.mnemo_token=ESamplerAddressingMode

    BRIG_ADDRESSING_UNDEFINED = 0,
    BRIG_ADDRESSING_CLAMP_TO_EDGE = 1,
    BRIG_ADDRESSING_CLAMP_TO_BORDER = 2,
    BRIG_ADDRESSING_REPEAT = 3,
    BRIG_ADDRESSING_MIRRORED_REPEAT = 4,

    BRIG_ADDRESSING_FIRST_USER_DEFINED = 128 //.skip
};

enum BrigSamplerCoordNormalization {

    //.mnemo={ s/^BRIG_COORD_//;lc }
    //.mnemo_token=ESamplerCoord
    //
    //.print=$mnemo{ $mnemo }

    BRIG_COORD_UNNORMALIZED = 0,
    BRIG_COORD_NORMALIZED = 1
};

enum BrigSamplerFilter {

    //.mnemo={ s/^BRIG_FILTER_//;lc }
    //
    //.print=$mnemo{ $mnemo }

    BRIG_FILTER_NEAREST = 0,
    BRIG_FILTER_LINEAR = 1,

    BRIG_FILTER_FIRST_USER_DEFINED = 128 //.skip
};

enum BrigSamplerQuery {

    //.mnemo={ s/^BRIG_SAMPLER_QUERY_//;lc }
    //.mnemo_token=_EMSamplerQuery
    //
    //.print=$mnemo{ $mnemo }

    BRIG_SAMPLER_QUERY_ADDRESSING = 0,
    BRIG_SAMPLER_QUERY_COORD = 1,
    BRIG_SAMPLER_QUERY_FILTER = 2
};

enum BrigSectionIndex {

    //.nollvm
    //
    //.mnemo={ s/^BRIG_SECTION_INDEX_/HSA_/;lc }

    BRIG_SECTION_INDEX_DATA = 0,
    BRIG_SECTION_INDEX_CODE = 1,
    BRIG_SECTION_INDEX_OPERAND = 2,
    BRIG_SECTION_INDEX_BEGIN_IMPLEMENTATION_DEFINED = 3,

    // used internally
    BRIG_SECTION_INDEX_IMPLEMENTATION_DEFINED = BRIG_SECTION_INDEX_BEGIN_IMPLEMENTATION_DEFINED //.skip
};

enum BrigSegCvtModifierMask {
    BRIG_SEG_CVT_NONULL = 1         //.mnemo="nonull" //.print="_nonull"
};

enum BrigSegment {

    //.mnemo={ s/^BRIG_SEGMENT_//;lc}
    //.mnemo_token=_EMSegment
    //.mnemo_context=EInstModifierContext
    //
    //.print=$mnemo{ $mnemo ? "_$mnemo" : "" }

    BRIG_SEGMENT_NONE = 0, //.mnemo=""
    BRIG_SEGMENT_FLAT = 1, //.mnemo=""
    BRIG_SEGMENT_GLOBAL = 2,
    BRIG_SEGMENT_READONLY = 3,
    BRIG_SEGMENT_KERNARG = 4,
    BRIG_SEGMENT_GROUP = 5,
    BRIG_SEGMENT_PRIVATE = 6,
    BRIG_SEGMENT_SPILL = 7,
    BRIG_SEGMENT_ARG = 8,

    BRIG_SEGMENT_FIRST_USER_DEFINED = 128, //.skip

    BRIG_SEGMENT_AMD_GCN = 9, //.mnemo="region"
};

enum BrigPackedTypeBits {

    //.nodump
    //
    //.nollvm

    BRIG_TYPE_BASE_SIZE  = 5,
    BRIG_TYPE_PACK_SIZE  = 2,
    BRIG_TYPE_ARRAY_SIZE = 1,

    BRIG_TYPE_BASE_SHIFT  = 0,
    BRIG_TYPE_PACK_SHIFT  = BRIG_TYPE_BASE_SHIFT + BRIG_TYPE_BASE_SIZE,
    BRIG_TYPE_ARRAY_SHIFT = BRIG_TYPE_PACK_SHIFT + BRIG_TYPE_PACK_SIZE,

    BRIG_TYPE_BASE_MASK  = ((1 << BRIG_TYPE_BASE_SIZE)  - 1) << BRIG_TYPE_BASE_SHIFT,
    BRIG_TYPE_PACK_MASK  = ((1 << BRIG_TYPE_PACK_SIZE)  - 1) << BRIG_TYPE_PACK_SHIFT,
    BRIG_TYPE_ARRAY_MASK = ((1 << BRIG_TYPE_ARRAY_SIZE) - 1) << BRIG_TYPE_ARRAY_SHIFT,

    BRIG_TYPE_PACK_NONE = 0 << BRIG_TYPE_PACK_SHIFT,
    BRIG_TYPE_PACK_32   = 1 << BRIG_TYPE_PACK_SHIFT,
    BRIG_TYPE_PACK_64   = 2 << BRIG_TYPE_PACK_SHIFT,
    BRIG_TYPE_PACK_128  = 3 << BRIG_TYPE_PACK_SHIFT,

    BRIG_TYPE_ARRAY     = 1 << BRIG_TYPE_ARRAY_SHIFT
};

enum BrigType {

    //.numBits={ /ARRAY$/ ? undef : /([0-9]+)X([0-9]+)/ ? $1*$2 : /([0-9]+)/ ? $1 : undef }
    //.numBits_switch //.numBits_proto="unsigned getBrigTypeNumBits(unsigned arg)" //.numBits_default="assert(0); return 0"
    //.numBytes=$numBits{ $numBits > 1 ? $numBits/8 : undef }
    //.numBytes_switch //.numBytes_proto="unsigned getBrigTypeNumBytes(unsigned arg)" //.numBytes_default="assert(0); return 0"
    //
    //.mnemo={ s/^BRIG_TYPE_//;lc }
    //.mnemo_token=_EMType
    //
    //.array={/ARRAY$/?"true":"false"}
    //.array_switch //.array_proto="bool isArrayType(unsigned type)" //.array_arg="type"
    //.array_default="return false"
    //
    //.a2e={/(.*)_ARRAY$/? $1 : "BRIG_TYPE_NONE"}
    //.a2e_switch //.a2e_proto="unsigned arrayType2elementType(unsigned type)" //.a2e_arg="type"
    //.a2e_default="return BRIG_TYPE_NONE"
    //
    //.e2a={/_ARRAY$/? "BRIG_TYPE_NONE" : /_NONE$/ ? "BRIG_TYPE_NONE" : /_B1$/ ? "BRIG_TYPE_NONE" : $_ . "_ARRAY"}
    //.e2a_switch //.e2a_proto="unsigned elementType2arrayType(unsigned type)" //.e2a_arg="type"
    //.e2a_default="return BRIG_TYPE_NONE"
    //
    //.t2s={s/^BRIG_TYPE_//;lc s/_ARRAY$/[]/;lc}
    //.t2s_switch //.t2s_proto="const char* type2name(unsigned type)" //.t2s_arg="type"
    //.t2s_default="return NULL"
    //
    //.dispatch_switch //.dispatch_incfile=TemplateUtilities
    //.dispatch_proto="template<typename RetType, typename Visitor>\nRetType dispatchByType_gen(unsigned type, Visitor& v)"
    //.dispatch={ /ARRAY$/ ? "v.visitNone(type)" : /^BRIG_TYPE_([BUSF]|SIG)[0-9]+/ ? "v.template visit< BrigTypeTraits<$_> >()" : "v.visitNone(type)" }
    //.dispatch_arg="type" //.dispatch_default="return v.visitNone(type)"
    //
    //- .tdname=BrigType
    //
    //.print=$mnemo{ "_$mnemo" }

    BRIG_TYPE_NONE  = 0,  //.mnemo=""       //.print=""
    BRIG_TYPE_U8    = 1,  //.ctype=uint8_t
    BRIG_TYPE_U16   = 2,  //.ctype=uint16_t
    BRIG_TYPE_U32   = 3,  //.ctype=uint32_t
    BRIG_TYPE_U64   = 4,  //.ctype=uint64_t
    BRIG_TYPE_S8    = 5,  //.ctype=int8_t
    BRIG_TYPE_S16   = 6,  //.ctype=int16_t
    BRIG_TYPE_S32   = 7,  //.ctype=int32_t
    BRIG_TYPE_S64   = 8,  //.ctype=int64_t
    BRIG_TYPE_F16   = 9,  //.ctype=f16_t
    BRIG_TYPE_F32   = 10, //.ctype=float
    BRIG_TYPE_F64   = 11, //.ctype=double
    BRIG_TYPE_B1    = 12, //.ctype=bool     //.numBytes=1
    BRIG_TYPE_B8    = 13, //.ctype=uint8_t
    BRIG_TYPE_B16   = 14, //.ctype=uint16_t
    BRIG_TYPE_B32   = 15, //.ctype=uint32_t
    BRIG_TYPE_B64   = 16, //.ctype=uint64_t
    BRIG_TYPE_B128  = 17, //.ctype=b128_t
    BRIG_TYPE_SAMP  = 18, //.mnemo=samp     //.numBits=64
    BRIG_TYPE_ROIMG = 19, //.mnemo=roimg    //.numBits=64
    BRIG_TYPE_WOIMG = 20, //.mnemo=woimg    //.numBits=64
    BRIG_TYPE_RWIMG = 21, //.mnemo=rwimg    //.numBits=64
    BRIG_TYPE_SIG32 = 22, //.mnemo=sig32    //.numBits=64
    BRIG_TYPE_SIG64 = 23, //.mnemo=sig64    //.numBits=64

    BRIG_TYPE_U8X4  = BRIG_TYPE_U8  | BRIG_TYPE_PACK_32,  //.ctype=uint8_t
    BRIG_TYPE_U8X8  = BRIG_TYPE_U8  | BRIG_TYPE_PACK_64,  //.ctype=uint8_t
    BRIG_TYPE_U8X16 = BRIG_TYPE_U8  | BRIG_TYPE_PACK_128, //.ctype=uint8_t
    BRIG_TYPE_U16X2 = BRIG_TYPE_U16 | BRIG_TYPE_PACK_32,  //.ctype=uint16_t
    BRIG_TYPE_U16X4 = BRIG_TYPE_U16 | BRIG_TYPE_PACK_64,  //.ctype=uint16_t
    BRIG_TYPE_U16X8 = BRIG_TYPE_U16 | BRIG_TYPE_PACK_128, //.ctype=uint16_t
    BRIG_TYPE_U32X2 = BRIG_TYPE_U32 | BRIG_TYPE_PACK_64,  //.ctype=uint32_t
    BRIG_TYPE_U32X4 = BRIG_TYPE_U32 | BRIG_TYPE_PACK_128, //.ctype=uint32_t
    BRIG_TYPE_U64X2 = BRIG_TYPE_U64 | BRIG_TYPE_PACK_128, //.ctype=uint64_t
    BRIG_TYPE_S8X4  = BRIG_TYPE_S8  | BRIG_TYPE_PACK_32,  //.ctype=int8_t
    BRIG_TYPE_S8X8  = BRIG_TYPE_S8  | BRIG_TYPE_PACK_64,  //.ctype=int8_t
    BRIG_TYPE_S8X16 = BRIG_TYPE_S8  | BRIG_TYPE_PACK_128, //.ctype=int8_t
    BRIG_TYPE_S16X2 = BRIG_TYPE_S16 | BRIG_TYPE_PACK_32,  //.ctype=int16_t
    BRIG_TYPE_S16X4 = BRIG_TYPE_S16 | BRIG_TYPE_PACK_64,  //.ctype=int16_t
    BRIG_TYPE_S16X8 = BRIG_TYPE_S16 | BRIG_TYPE_PACK_128, //.ctype=int16_t
    BRIG_TYPE_S32X2 = BRIG_TYPE_S32 | BRIG_TYPE_PACK_64,  //.ctype=int32_t
    BRIG_TYPE_S32X4 = BRIG_TYPE_S32 | BRIG_TYPE_PACK_128, //.ctype=int32_t
    BRIG_TYPE_S64X2 = BRIG_TYPE_S64 | BRIG_TYPE_PACK_128, //.ctype=int64_t
    BRIG_TYPE_F16X2 = BRIG_TYPE_F16 | BRIG_TYPE_PACK_32,  //.ctype=f16_t
    BRIG_TYPE_F16X4 = BRIG_TYPE_F16 | BRIG_TYPE_PACK_64,  //.ctype=f16_t
    BRIG_TYPE_F16X8 = BRIG_TYPE_F16 | BRIG_TYPE_PACK_128, //.ctype=f16_t
    BRIG_TYPE_F32X2 = BRIG_TYPE_F32 | BRIG_TYPE_PACK_64,  //.ctype=float
    BRIG_TYPE_F32X4 = BRIG_TYPE_F32 | BRIG_TYPE_PACK_128, //.ctype=float
    BRIG_TYPE_F64X2 = BRIG_TYPE_F64 | BRIG_TYPE_PACK_128, //.ctype=double

    BRIG_TYPE_U8_ARRAY    = BRIG_TYPE_U8    | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U16_ARRAY   = BRIG_TYPE_U16   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U32_ARRAY   = BRIG_TYPE_U32   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U64_ARRAY   = BRIG_TYPE_U64   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S8_ARRAY    = BRIG_TYPE_S8    | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S16_ARRAY   = BRIG_TYPE_S16   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S32_ARRAY   = BRIG_TYPE_S32   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S64_ARRAY   = BRIG_TYPE_S64   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F16_ARRAY   = BRIG_TYPE_F16   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F32_ARRAY   = BRIG_TYPE_F32   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F64_ARRAY   = BRIG_TYPE_F64   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_B8_ARRAY    = BRIG_TYPE_B8    | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_B16_ARRAY   = BRIG_TYPE_B16   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_B32_ARRAY   = BRIG_TYPE_B32   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_B64_ARRAY   = BRIG_TYPE_B64   | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_B128_ARRAY  = BRIG_TYPE_B128  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_SAMP_ARRAY  = BRIG_TYPE_SAMP  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_ROIMG_ARRAY = BRIG_TYPE_ROIMG | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_WOIMG_ARRAY = BRIG_TYPE_WOIMG | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_RWIMG_ARRAY = BRIG_TYPE_RWIMG | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_SIG32_ARRAY = BRIG_TYPE_SIG32 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_SIG64_ARRAY = BRIG_TYPE_SIG64 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U8X4_ARRAY  = BRIG_TYPE_U8X4  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U8X8_ARRAY  = BRIG_TYPE_U8X8  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U8X16_ARRAY = BRIG_TYPE_U8X16 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U16X2_ARRAY = BRIG_TYPE_U16X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U16X4_ARRAY = BRIG_TYPE_U16X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U16X8_ARRAY = BRIG_TYPE_U16X8 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U32X2_ARRAY = BRIG_TYPE_U32X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U32X4_ARRAY = BRIG_TYPE_U32X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_U64X2_ARRAY = BRIG_TYPE_U64X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S8X4_ARRAY  = BRIG_TYPE_S8X4  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S8X8_ARRAY  = BRIG_TYPE_S8X8  | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S8X16_ARRAY = BRIG_TYPE_S8X16 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S16X2_ARRAY = BRIG_TYPE_S16X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S16X4_ARRAY = BRIG_TYPE_S16X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S16X8_ARRAY = BRIG_TYPE_S16X8 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S32X2_ARRAY = BRIG_TYPE_S32X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S32X4_ARRAY = BRIG_TYPE_S32X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_S64X2_ARRAY = BRIG_TYPE_S64X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F16X2_ARRAY = BRIG_TYPE_F16X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F16X4_ARRAY = BRIG_TYPE_F16X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F16X8_ARRAY = BRIG_TYPE_F16X8 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F32X2_ARRAY = BRIG_TYPE_F32X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F32X4_ARRAY = BRIG_TYPE_F32X4 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""
    BRIG_TYPE_F64X2_ARRAY = BRIG_TYPE_F64X2 | BRIG_TYPE_ARRAY,  //.mnemo=""     //.print=""

    // Used internally
    BRIG_TYPE_INVALID = (unsigned) -1 //.skip
};

enum BrigVariableModifierMask {

    //.nodump

    BRIG_VARIABLE_DEFINITION = 1,
    BRIG_VARIABLE_CONST = 2
};

enum BrigWidth {

    //.tddef=1
    //
    //.print={ s/^BRIG_WIDTH_//; "_width($_)" }

    BRIG_WIDTH_NONE = 0,
    BRIG_WIDTH_1 = 1,
    BRIG_WIDTH_2 = 2,
    BRIG_WIDTH_4 = 3,
    BRIG_WIDTH_8 = 4,
    BRIG_WIDTH_16 = 5,
    BRIG_WIDTH_32 = 6,
    BRIG_WIDTH_64 = 7,
    BRIG_WIDTH_128 = 8,
    BRIG_WIDTH_256 = 9,
    BRIG_WIDTH_512 = 10,
    BRIG_WIDTH_1024 = 11,
    BRIG_WIDTH_2048 = 12,
    BRIG_WIDTH_4096 = 13,
    BRIG_WIDTH_8192 = 14,
    BRIG_WIDTH_16384 = 15,
    BRIG_WIDTH_32768 = 16,
    BRIG_WIDTH_65536 = 17,
    BRIG_WIDTH_131072 = 18,
    BRIG_WIDTH_262144 = 19,
    BRIG_WIDTH_524288 = 20,
    BRIG_WIDTH_1048576 = 21,
    BRIG_WIDTH_2097152 = 22,
    BRIG_WIDTH_4194304 = 23,
    BRIG_WIDTH_8388608 = 24,
    BRIG_WIDTH_16777216 = 25,
    BRIG_WIDTH_33554432 = 26,
    BRIG_WIDTH_67108864 = 27,
    BRIG_WIDTH_134217728 = 28,
    BRIG_WIDTH_268435456 = 29,
    BRIG_WIDTH_536870912 = 30,
    BRIG_WIDTH_1073741824 = 31,
    BRIG_WIDTH_2147483648 = 32,
    BRIG_WIDTH_WAVESIZE = 33,
    BRIG_WIDTH_ALL = 34,

    BRIG_WIDTH_LAST //.skip
};

struct BrigUInt64 { //.isroot //.standalone
    uint32_t lo;     //.defValue=0
    uint32_t hi;     //.defValue=0

    //+hcode KLASS& operator=(uint64_t rhs);
    //+hcode operator uint64_t();
    //+implcode inline KLASS& KLASS::operator=(uint64_t rhs) { lo() = (uint32_t)rhs; hi() = (uint32_t)(rhs >> 32); return *this; }
    //+implcode inline KLASS::operator uint64_t() { return ((uint64_t)hi()) << 32 | lo(); }
};

struct BrigAluModifier { //.isroot //.standalone
    BrigAluModifier8_t allBits; //.defValue=0
    //^^ bool ftz; //.wtype=BitValRef<0>
};

struct BrigBase { //.nowrap
    uint16_t byteCount;
    BrigKind16_t kind;
};

//.alias Code:Base { //.generic //.isroot //.section=BRIG_SECTION_INDEX_CODE };
//.alias Directive:Code { //.generic };
//.alias Operand:Base { //.generic //.isroot //.section=BRIG_SECTION_INDEX_OPERAND };

struct BrigData {
    //.nowrap
    uint32_t byteCount;
    uint8_t bytes[1];
};

struct BrigExecutableModifier { //.isroot //.standalone
    BrigExecutableModifier8_t allBits; //.defValue=0
    //^^ bool isDefinition; //.wtype=BitValRef<0>
};

struct BrigMemoryModifier { //.isroot //.standalone
    BrigMemoryModifier8_t allBits; //.defValue=0
    //^^ bool isConst; //.wtype=BitValRef<0>
};

struct BrigSegCvtModifier { //.isroot //.standalone
    BrigSegCvtModifier8_t allBits; //.defValue=0
    //^^ bool isNoNull; //.wtype=BitValRef<0>
};

struct BrigVariableModifier { //.isroot //.standalone
    BrigVariableModifier8_t allBits;    //.defValue=0

    //^^ bool isDefinition;     //.wtype=BitValRef<0>
    //^^ bool isConst;          //.wtype=BitValRef<1>
};

struct BrigDirectiveArgBlockEnd {
    BrigBase base;
};

struct BrigDirectiveArgBlockStart {
    BrigBase base;
};

struct BrigDirectiveComment {
    BrigBase base;
    BrigDataOffsetString32_t name;
};

struct BrigDirectiveControl {
    BrigBase base;
    BrigControlDirective16_t control;
    uint16_t reserved; //.defValue=0
    BrigDataOffsetOperandList32_t operands;
};

struct BrigDirectiveExecutable { //.generic
    BrigBase base;
    BrigDataOffsetString32_t name;
    uint16_t outArgCount; //.defValue=0
    uint16_t inArgCount;  //.defValue=0
    BrigCodeOffset32_t firstInArg;
    BrigCodeOffset32_t firstCodeBlockEntry;
    BrigCodeOffset32_t nextModuleEntry;
    BrigExecutableModifier modifier; //.acc=subItem<ExecutableModifier> //.wtype=ExecutableModifier
    BrigLinkage8_t linkage;
    uint16_t reserved; //.defValue=0
};

//.alias DirectiveKernel:DirectiveExecutable { };
//.alias DirectiveFunction:DirectiveExecutable { };
//.alias DirectiveSignature:DirectiveExecutable { };
//.alias DirectiveIndirectFunction:DirectiveExecutable { };

struct BrigDirectiveExtension {
    BrigBase base;
    BrigDataOffsetString32_t name;
};

struct BrigDirectiveFbarrier {
    BrigBase base;
    BrigDataOffsetString32_t name;
    BrigVariableModifier modifier; //.acc=subItem<VariableModifier> //.wtype=VariableModifier
    BrigLinkage8_t linkage;
    uint16_t reserved; //.defValue=0
};

struct BrigDirectiveLabel {
    BrigBase base;
    BrigDataOffsetString32_t name;
};

struct BrigDirectiveLoc {
    BrigBase base;
    BrigDataOffsetString32_t filename;
    uint32_t line;
    uint32_t column; //.defValue=1
};

struct BrigDirectiveNone { //.enum=BRIG_KIND_NONE
    BrigBase base;
};

struct BrigDirectivePragma {
    BrigBase base;
    BrigDataOffsetOperandList32_t operands;
};

struct BrigDirectiveVariable {
    BrigBase base;
    BrigDataOffsetString32_t name;
    BrigOperandOffset32_t init;
    BrigType16_t type;

    //+hcode bool isArray();
    //+implcode inline bool KLASS::isArray() { return isArrayType(type()); }

    //+hcode unsigned elementType();
    //+implcode inline unsigned KLASS::elementType() { return isArray()? arrayType2elementType(type()) : type(); }

    BrigSegment8_t segment;
    BrigAlignment8_t align;
    BrigUInt64 dim; //.acc=subItem<UInt64> //.wtype=UInt64
    BrigVariableModifier modifier; //.acc=subItem<VariableModifier> //.wtype=VariableModifier
    BrigLinkage8_t linkage;
    BrigAllocation8_t allocation;
    uint8_t reserved; //.defValue=0
};

struct BrigDirectiveModule {
    BrigBase base;
    BrigDataOffsetString32_t name;
    BrigVersion32_t hsailMajor;         //.wtype=ValRef<uint32_t>
    BrigVersion32_t hsailMinor;         //.wtype=ValRef<uint32_t>
    BrigProfile8_t profile;
    BrigMachineModel8_t machineModel;
    BrigRound8_t defaultFloatRound;
    uint8_t reserved;                   //.defValue=0
};

struct BrigInstBase { //.wname=Inst //.generic //.parent=BrigCode
    BrigBase base;
    BrigOpcode16_t opcode;
    BrigType16_t type;
    BrigDataOffsetOperandList32_t operands;

    //+hcode Operand operand(int index);
    //+implcode inline Operand KLASS::operand(int index) { return operands()[index]; }
};

struct BrigInstAddr {
    BrigInstBase base;
    BrigSegment8_t segment;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstAtomic {
    BrigInstBase base;
    BrigSegment8_t segment;
    BrigMemoryOrder8_t memoryOrder;
    BrigMemoryScope8_t memoryScope;
    BrigAtomicOperation8_t atomicOperation;
    uint8_t equivClass;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstBasic {
    BrigInstBase base;
};

struct BrigInstBr {
    BrigInstBase base;
    BrigWidth8_t width;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstCmp {
    BrigInstBase base;
    BrigType16_t sourceType;
    BrigAluModifier modifier; //.acc=subItem<AluModifier> //.wtype=AluModifier
    BrigCompareOperation8_t compare;
    BrigPack8_t pack;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstCvt {
    BrigInstBase base;
    BrigType16_t sourceType;
    BrigAluModifier modifier; //.acc=subItem<AluModifier> //.wtype=AluModifier
    BrigRound8_t round;
};

struct BrigInstImage {
    BrigInstBase base;
    BrigType16_t imageType;
    BrigType16_t coordType;
    BrigImageGeometry8_t geometry;
    uint8_t equivClass;
    uint16_t reserved; //.defValue=0
};

struct BrigInstLane {
    BrigInstBase base;
    BrigType16_t sourceType;
    BrigWidth8_t width;
    uint8_t reserved; //.defValue=0
};

struct BrigInstMem {
    BrigInstBase base;
    BrigSegment8_t segment;
    BrigAlignment8_t align;
    uint8_t equivClass;
    BrigWidth8_t width;
    BrigMemoryModifier modifier; //.acc=subItem<MemoryModifier> //.wtype=MemoryModifier
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstMemFence {
    BrigInstBase base;
    BrigMemoryOrder8_t memoryOrder;
    BrigMemoryScope8_t globalSegmentMemoryScope;
    BrigMemoryScope8_t groupSegmentMemoryScope;
    BrigMemoryScope8_t imageSegmentMemoryScope;
};

struct BrigInstMod {
    BrigInstBase base;
    BrigAluModifier modifier; //.acc=subItem<AluModifier> //.wtype=AluModifier
    BrigRound8_t round;
    BrigPack8_t pack;
    uint8_t reserved; //.defValue=0
};

struct BrigInstQueryImage {
    BrigInstBase base;
    BrigType16_t imageType;
    BrigImageGeometry8_t geometry;
    BrigImageQuery8_t imageQuery;
};

struct BrigInstQuerySampler {
    BrigInstBase base;
    BrigSamplerQuery8_t samplerQuery;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstQueue {
    BrigInstBase base;
    BrigSegment8_t segment;
    BrigMemoryOrder8_t memoryOrder;
    uint16_t reserved; //.defValue=0
};

struct BrigInstSeg {
    BrigInstBase base;
    BrigSegment8_t segment;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigInstSegCvt {
    BrigInstBase base;
    BrigType16_t sourceType;
    BrigSegment8_t segment;
    BrigSegCvtModifier modifier; //.acc=subItem<SegCvtModifier> //.wtype=SegCvtModifier
};

struct BrigInstSignal {
    BrigInstBase base;
    BrigType16_t signalType;
    BrigMemoryOrder8_t memoryOrder;
    BrigAtomicOperation8_t signalOperation;
};

struct BrigInstSourceType {
    BrigInstBase base;
    BrigType16_t sourceType;
    uint16_t reserved; //.defValue=0
};

struct BrigOperandAddress {
    BrigBase base;
    BrigCodeOffset32_t symbol; //.wtype=ItemRef<DirectiveVariable>
    BrigOperandOffset32_t reg; //.wtype=ItemRef<OperandRegister>
    BrigUInt64 offset; //.acc=subItem<UInt64> //.wtype=UInt64
};

struct BrigOperandAlign {
    BrigBase base;
    BrigAlignment8_t align;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigOperandCodeList {
    BrigBase base;
    BrigDataOffsetCodeList32_t elements;

    //+hcode unsigned elementCount();
    //+implcode inline unsigned KLASS::elementCount() { return elements().size(); }
    //+hcode Code elements(int index);
    //+implcode inline Code KLASS::elements(int index) { return elements()[index]; }
};

struct BrigOperandCodeRef {
    BrigBase base;
    BrigCodeOffset32_t ref;
};

struct BrigOperandConstantBytes {
    BrigBase base;
    BrigType16_t type; //.defValue=0
    uint16_t reserved; //.defValue=0
    BrigDataOffsetString32_t bytes;
};

struct BrigOperandConstantOperandList {
    BrigBase base;
    BrigType16_t type;
    uint16_t reserved; //.defValue=0
    BrigDataOffsetOperandList32_t elements;

    //+hcode unsigned elementCount();
    //+implcode inline unsigned KLASS::elementCount() { return elements().size(); }
    //+hcode Operand elements(int index);
    //+implcode inline Operand KLASS::elements(int index) { return elements()[index]; }
};

struct BrigOperandConstantImage {
    BrigBase base;
    BrigType16_t type;
    BrigImageGeometry8_t geometry;
    BrigImageChannelOrder8_t channelOrder;
    BrigImageChannelType8_t channelType;
    uint8_t reserved[3]; //.defValue=0
    BrigUInt64 width;    //.acc=subItem<UInt64> //.wtype=UInt64
    BrigUInt64 height;   //.acc=subItem<UInt64> //.wtype=UInt64
    BrigUInt64 depth;    //.acc=subItem<UInt64> //.wtype=UInt64
    BrigUInt64 array;    //.acc=subItem<UInt64> //.wtype=UInt64
};

struct BrigOperandOperandList {
    BrigBase base;
    BrigDataOffsetOperandList32_t elements;

    //+hcode unsigned elementCount();
    //+implcode inline unsigned KLASS::elementCount() { return elements().size(); }
    //+hcode Operand elements(int index);
    //+implcode inline Operand KLASS::elements(int index) { return elements()[index]; }
};

struct BrigOperandRegister {
    BrigBase base;
    BrigRegisterKind16_t regKind;
    uint16_t regNum;
};

struct BrigOperandConstantSampler {
    BrigBase base;
    BrigType16_t type;
    BrigSamplerCoordNormalization8_t coord;
    BrigSamplerFilter8_t filter;
    BrigSamplerAddressing8_t addressing;
    uint8_t reserved[3]; //.defValue=0
};

struct BrigOperandString {
    BrigBase base;
    BrigDataOffsetString32_t string;
};

struct BrigOperandWavesize {
    BrigBase base;
};

//.ignore{

enum BrigExceptionsMask {
    BRIG_EXCEPTIONS_INVALID_OPERATION = 1 << 0,
    BRIG_EXCEPTIONS_DIVIDE_BY_ZERO = 1 << 1,
    BRIG_EXCEPTIONS_OVERFLOW = 1 << 2,
    BRIG_EXCEPTIONS_UNDERFLOW = 1 << 3,
    BRIG_EXCEPTIONS_INEXACT = 1 << 4,

    BRIG_EXCEPTIONS_FIRST_USER_DEFINED = 1 << 16
};

struct BrigSectionHeader {
    uint64_t byteCount;
    uint32_t headerByteCount;
    uint32_t nameLength;
    uint8_t name[1];
};

#define MODULE_IDENTIFICATION_LENGTH (8)

struct BrigModuleHeader {
    char identification[MODULE_IDENTIFICATION_LENGTH];
    BrigVersion32_t brigMajor;
    BrigVersion32_t brigMinor;
    uint64_t byteCount;
    uint8_t hash[64];
    uint32_t reserved;
    uint32_t sectionCount;
    uint64_t sectionIndex;
};

typedef BrigModuleHeader* BrigModule_t;

#endif // defined(INCLUDED_BRIG_H)
//}
