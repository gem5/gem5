/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_MT_CONSTANTS_HH__
#define __ARCH_MIPS_MT_CONSTANTS_HH__

#include "arch/mips/types.hh"
//#include "config/full_system.hh"

namespace MipsISA
{
// MVPControl
const unsigned MVPC_EVP = 0;
const unsigned MVPC_CUR_VPE_HI = 3;
const unsigned MVPC_CUR_VPE_LO = 0;

// MVPConf0
const unsigned MVPC0_TCA = 15;
const unsigned MVPC0_PVPE_HI = 13;
const unsigned MVPC0_PVPE_LO = 10;
const unsigned MVPC0_PTC_HI = 7;
const unsigned MVPC0_PTC_LO = 0;

//VPEControl
const unsigned VPEC_YSI = 21;
const unsigned VPEC_EXCPT_HI = 18;
const unsigned VPEC_EXCPT_LO = 16;
const unsigned VPEC_TE = 15;
const unsigned VPEC_TARG_TC_HI = 7;
const unsigned VPEC_TARG_TC_LO = 0;

//VPEConf0
const unsigned VPEC0_MVP = 1;

//TCBind
const unsigned TCB_CUR_VPE_HI = 3;
const unsigned TCB_CUR_VPE_LO = 0;
const unsigned TCB_CUR_TC_HI = 28;
const unsigned TCB_CUR_TC_LO = 21;


//TCStatus
const unsigned TCS_TCU_HI = 31;
const unsigned TCS_TCU_LO = 28;
const unsigned TCS_TMX = 27;
const unsigned TCS_DT = 20;
const unsigned TCS_DA = 15;
const unsigned TCS_A = 13;
const unsigned TCS_TKSU_HI = 12;
const unsigned TCS_TKSU_LO = 11;
const unsigned TCS_IXMT = 7;
const unsigned TCS_ASID_HI = 7;
const unsigned TCS_ASID_LO = 7;

const unsigned TCSTATUS_TCU_HI = 31;
const unsigned TCSTATUS_TCU_LO = 28;
const unsigned TCSTATUS_TMX = 27;
const unsigned TCSTATUS_RNST_HI = 24;
const unsigned TCSTATUS_RNST_LO = 23;
const unsigned TCSTATUS_TDS = 21;
const unsigned TCSTATUS_DT = 20;
const unsigned TCSTATUS_DA = 15;
const unsigned TCSTATUS_A = 13;
const unsigned TCSTATUS_TKSU_HI = 12;
const unsigned TCSTATUS_TKSU_LO = 11;
const unsigned TCSTATUS_IXMT = 7;
const unsigned TCSTATUS_ASID_HI = 7;
const unsigned TCSTATUS_ASID_LO = 7;

//TCHalt
const unsigned TCH_H = 0;

//Status
const unsigned S_CU_HI = 31;
const unsigned S_CU_LO = 28;
const unsigned S_MX = 24;
const unsigned S_KSU_HI = 4;
const unsigned S_KSU_LO = 3;

// Config0
const unsigned CFG_M = 31;

// Config1
const unsigned CFG1_M = 31;

// Config2
const unsigned CFG2_M = 31;

// Config3
const unsigned CFG3_M = 31;
const unsigned CFG3_MT = 2;

} // namespace MipsISA

#endif
