/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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

#ifndef __ARCH_MIPS_TYPES_HH__
#define __ARCH_MIPS_TYPES_HH__

#include "arch/generic/types.hh"
#include "base/types.hh"

namespace MipsISA
{

typedef uint32_t MachInst;
typedef uint64_t ExtMachInst;

typedef GenericISA::DelaySlotPCState<MachInst> PCState;

//used in FP convert & round function
enum ConvertType{
    SINGLE_TO_DOUBLE,
    SINGLE_TO_WORD,
    SINGLE_TO_LONG,

    DOUBLE_TO_SINGLE,
    DOUBLE_TO_WORD,
    DOUBLE_TO_LONG,

    LONG_TO_SINGLE,
    LONG_TO_DOUBLE,
    LONG_TO_WORD,
    LONG_TO_PS,

    WORD_TO_SINGLE,
    WORD_TO_DOUBLE,
    WORD_TO_LONG,
    WORD_TO_PS,

    PL_TO_SINGLE,
    PU_TO_SINGLE
};

//used in FP convert & round function
enum RoundMode{
    RND_ZERO,
    RND_DOWN,
    RND_UP,
    RND_NEAREST
};

struct CoreSpecific {
    CoreSpecific()
        : CP0_IntCtl_IPTI(0), CP0_IntCtl_IPPCI(0), CP0_SrsCtl_HSS(0),
          CP0_PRId_CompanyOptions(0), CP0_PRId_CompanyID(0),
          CP0_PRId_ProcessorID(0), CP0_PRId_Revision(0),
          CP0_EBase_CPUNum(0), CP0_Config_BE(0), CP0_Config_AT(0),
          CP0_Config_AR(0), CP0_Config_MT(0), CP0_Config_VI(0),
          CP0_Config1_M(0), CP0_Config1_MMU(0), CP0_Config1_IS(0),
          CP0_Config1_IL(0), CP0_Config1_IA(0), CP0_Config1_DS(0),
          CP0_Config1_DL(0), CP0_Config1_DA(0), CP0_Config1_C2(false),
          CP0_Config1_MD(false), CP0_Config1_PC(false), CP0_Config1_WR(false),
          CP0_Config1_CA(false), CP0_Config1_EP(false), CP0_Config1_FP(false),
          CP0_Config2_M(false), CP0_Config2_TU(0), CP0_Config2_TS(0),
          CP0_Config2_TL(0), CP0_Config2_TA(0), CP0_Config2_SU(0),
          CP0_Config2_SS(0), CP0_Config2_SL(0), CP0_Config2_SA(0),
          CP0_Config3_M(false), CP0_Config3_DSPP(false), CP0_Config3_LPA(false),
          CP0_Config3_VEIC(false), CP0_Config3_VInt(false),
          CP0_Config3_SP(false), CP0_Config3_MT(false), CP0_Config3_SM(false),
          CP0_Config3_TL(false), CP0_WatchHi_M(false), CP0_PerfCtr_M(false),
          CP0_PerfCtr_W(false), CP0_PRId(0), CP0_Config(0), CP0_Config1(0),
          CP0_Config2(0), CP0_Config3(0)
    { }

      // MIPS CP0 State - First individual variables
      // Page numbers refer to revision 2.50 (July 2005) of the MIPS32 ARM,
      // Volume III (PRA)
      unsigned CP0_IntCtl_IPTI; // Page 93, IP Timer Interrupt
      unsigned CP0_IntCtl_IPPCI; // Page 94, IP Performance Counter Interrupt
      unsigned CP0_SrsCtl_HSS; // Page 95, Highest Implemented Shadow Set
      unsigned CP0_PRId_CompanyOptions; // Page 105, Manufacture options
      unsigned CP0_PRId_CompanyID; // Page 105, Company ID - (0-255, 1=>MIPS)
      unsigned CP0_PRId_ProcessorID; // Page 105
      unsigned CP0_PRId_Revision; // Page 105
      unsigned CP0_EBase_CPUNum; // Page 106, CPU Number in a multiprocessor
                                 //system
      unsigned CP0_Config_BE; // Page 108, Big/Little Endian mode
      unsigned CP0_Config_AT; //Page 109
      unsigned CP0_Config_AR; //Page 109
      unsigned CP0_Config_MT; //Page 109
      unsigned CP0_Config_VI; //Page 109
      unsigned CP0_Config1_M; // Page 110
      unsigned CP0_Config1_MMU; // Page 110
      unsigned CP0_Config1_IS; // Page 110
      unsigned CP0_Config1_IL; // Page 111
      unsigned CP0_Config1_IA; // Page 111
      unsigned CP0_Config1_DS; // Page 111
      unsigned CP0_Config1_DL; // Page 112
      unsigned CP0_Config1_DA; // Page 112
      bool CP0_Config1_C2; // Page 112
      bool CP0_Config1_MD;// Page 112 - Technically not used in MIPS32
      bool CP0_Config1_PC;// Page 112
      bool CP0_Config1_WR;// Page 113
      bool CP0_Config1_CA;// Page 113
      bool CP0_Config1_EP;// Page 113
      bool CP0_Config1_FP;// Page 113
      bool CP0_Config2_M; // Page 114
      unsigned CP0_Config2_TU;// Page 114
      unsigned CP0_Config2_TS;// Page 114
      unsigned CP0_Config2_TL;// Page 115
      unsigned CP0_Config2_TA;// Page 115
      unsigned CP0_Config2_SU;// Page 115
      unsigned CP0_Config2_SS;// Page 115
      unsigned CP0_Config2_SL;// Page 116
      unsigned CP0_Config2_SA;// Page 116
      bool CP0_Config3_M; //// Page 117
      bool CP0_Config3_DSPP;// Page 117
      bool CP0_Config3_LPA;// Page 117
      bool CP0_Config3_VEIC;// Page 118
      bool CP0_Config3_VInt; // Page 118
      bool CP0_Config3_SP;// Page 118
      bool CP0_Config3_MT;// Page 119
      bool CP0_Config3_SM;// Page 119
      bool CP0_Config3_TL;// Page 119

      bool CP0_WatchHi_M; // Page 124
      bool CP0_PerfCtr_M; // Page 130
      bool CP0_PerfCtr_W; // Page 130


      // Then, whole registers
      unsigned CP0_PRId;
      unsigned CP0_Config;
      unsigned CP0_Config1;
      unsigned CP0_Config2;
      unsigned CP0_Config3;
};

} // namespace MipsISA
#endif
