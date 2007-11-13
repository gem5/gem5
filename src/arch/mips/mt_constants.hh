/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Jaidev Patwardhan
 *
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
