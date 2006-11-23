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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/sparc/asi.hh"

namespace SparcISA
{
    bool AsiIsBlock(ASI asi)
    {
        return
            (asi == ASI_BLK_AIUP) ||
            (asi == ASI_BLK_AIUS) ||
            (asi == ASI_BLK_AIUP_L) ||
            (asi == ASI_BLK_AIUS_L) ||
            (asi == ASI_BLK_P) ||
            (asi == ASI_BLK_S) ||
            (asi == ASI_BLK_PL) ||
            (asi == ASI_BLK_SL);
    }

    bool AsiIsPrimary(ASI asi)
    {
        return
            (asi == ASI_AIUP) ||
            (asi == ASI_BLK_AIUP) ||
            (asi == ASI_AIUP_L) ||
            (asi == ASI_BLK_AIUP_L) ||
            (asi == ASI_LDTX_AIUP) ||
            (asi == ASI_LDTX_AIUP_L) ||
            (asi == ASI_P) ||
            (asi == ASI_PNF) ||
            (asi == ASI_PL) ||
            (asi == ASI_PNFL) ||
            (asi == ASI_PST8_P) ||
            (asi == ASI_PST16_P) ||
            (asi == ASI_PST32_P) ||
            (asi == ASI_PST8_PL) ||
            (asi == ASI_PST16_PL) ||
            (asi == ASI_PST32_PL) ||
            (asi == ASI_FL8_P) ||
            (asi == ASI_FL16_P) ||
            (asi == ASI_FL8_PL) ||
            (asi == ASI_FL16_PL) ||
            (asi == ASI_LDTX_P) ||
            (asi == ASI_LDTX_PL) ||
            (asi == ASI_BLK_P) ||
            (asi == ASI_BLK_PL);
    }

    bool AsiIsSecondary(ASI asi)
    {
        return
            (asi == ASI_AIUS) ||
            (asi == ASI_BLK_AIUS) ||
            (asi == ASI_AIUS_L) ||
            (asi == ASI_BLK_AIUS_L) ||
            (asi == ASI_LDTX_AIUS) ||
            (asi == ASI_LDTX_AIUS_L) ||
            (asi == ASI_S) ||
            (asi == ASI_SNF) ||
            (asi == ASI_SL) ||
            (asi == ASI_SNFL) ||
            (asi == ASI_PST8_S) ||
            (asi == ASI_PST16_S) ||
            (asi == ASI_PST32_S) ||
            (asi == ASI_PST8_SL) ||
            (asi == ASI_PST16_SL) ||
            (asi == ASI_PST32_SL) ||
            (asi == ASI_FL8_S) ||
            (asi == ASI_FL16_S) ||
            (asi == ASI_FL8_SL) ||
            (asi == ASI_FL16_SL) ||
            (asi == ASI_LDTX_S) ||
            (asi == ASI_LDTX_SL) ||
            (asi == ASI_BLK_S) ||
            (asi == ASI_BLK_SL);
    }

    bool AsiNucleus(ASI asi)
    {
        return
            (asi == ASI_N) ||
            (asi == ASI_NL) ||
            (asi == ASI_LDTX_N) ||
            (asi == ASI_LDTX_NL);
    }

    bool AsiIsAsIfUser(ASI asi)
    {
        return
            (asi == ASI_AIUP) ||
            (asi == ASI_AIUS) ||
            (asi == ASI_BLK_AIUP) ||
            (asi == ASI_BLK_AIUS) ||
            (asi == ASI_AIUP_L) ||
            (asi == ASI_AIUS_L) ||
            (asi == ASI_BLK_AIUP_L) ||
            (asi == ASI_BLK_AIUS_L) ||
            (asi == ASI_LDTX_AIUP) ||
            (asi == ASI_LDTX_AIUS) ||
            (asi == ASI_LDTX_AIUP_L) ||
            (asi == ASI_LDTX_AIUS_L);
    }

    bool AsiIsIO(ASI asi)
    {
        return
            (asi == ASI_REAL_IO) ||
            (asi == ASI_REAL_IO_L);
    }

    bool AsiIsReal(ASI asi)
    {
        return
            (asi == ASI_REAL) ||
            (asi == ASI_REAL_IO) ||
            (asi == ASI_REAL_L) ||
            (asi == ASI_REAL_IO_L) ||
            (asi == ASI_LDTX_REAL) ||
            (asi == ASI_LDTX_REAL_L);
    }

    bool AsiIsLittle(ASI asi)
    {
        return
            (asi == ASI_NL) ||
            (asi == ASI_AIUP_L) ||
            (asi == ASI_AIUS_L) ||
            (asi == ASI_REAL_L) ||
            (asi == ASI_REAL_IO_L) ||
            (asi == ASI_BLK_AIUP_L) ||
            (asi == ASI_BLK_AIUS_L) ||
            (asi == ASI_LDTX_AIUP_L) ||
            (asi == ASI_LDTX_AIUS_L) ||
            (asi == ASI_LDTX_REAL_L) ||
            (asi == ASI_LDTX_NL) ||
            (asi == ASI_PL) ||
            (asi == ASI_SL) ||
            (asi == ASI_PNFL) ||
            (asi == ASI_SNFL) ||
            (asi == ASI_PST8_PL) ||
            (asi == ASI_PST8_SL) ||
            (asi == ASI_PST16_PL) ||
            (asi == ASI_PST16_SL) ||
            (asi == ASI_PST32_PL) ||
            (asi == ASI_PST32_SL) ||
            (asi == ASI_FL8_PL) ||
            (asi == ASI_FL8_SL) ||
            (asi == ASI_FL16_PL) ||
            (asi == ASI_FL16_SL) ||
            (asi == ASI_LDTX_PL) ||
            (asi == ASI_LDTX_SL) ||
            (asi == ASI_BLK_PL) ||
            (asi == ASI_BLK_SL);
    }

    bool AsiIsTwin(ASI asi)
    {
        return
            (asi == ASI_LDTX_AIUP) ||
            (asi == ASI_LDTX_AIUS) ||
            (asi == ASI_LDTX_REAL) ||
            (asi == ASI_LDTX_N) ||
            (asi == ASI_LDTX_AIUP_L) ||
            (asi == ASI_LDTX_AIUS_L) ||
            (asi == ASI_LDTX_REAL_L) ||
            (asi == ASI_LDTX_NL) ||
            (asi == ASI_LDTX_P) ||
            (asi == ASI_LDTX_S) ||
            (asi == ASI_LDTX_PL) ||
            (asi == ASI_LDTX_SL);
    }

    bool AsiIsPartialStore(ASI asi)
    {
        return
            (asi == ASI_PST8_P) ||
            (asi == ASI_PST8_S) ||
            (asi == ASI_PST16_P) ||
            (asi == ASI_PST16_S) ||
            (asi == ASI_PST32_P) ||
            (asi == ASI_PST32_S) ||
            (asi == ASI_PST8_PL) ||
            (asi == ASI_PST8_SL) ||
            (asi == ASI_PST16_PL) ||
            (asi == ASI_PST16_SL) ||
            (asi == ASI_PST32_PL) ||
            (asi == ASI_PST32_SL);
    }

    bool AsiIsFloatingLoad(ASI asi)
    {
        return
            (asi == ASI_FL8_P) ||
            (asi == ASI_FL8_S) ||
            (asi == ASI_FL16_P) ||
            (asi == ASI_FL16_S) ||
            (asi == ASI_FL8_PL) ||
            (asi == ASI_FL8_SL) ||
            (asi == ASI_FL16_PL) ||
            (asi == ASI_FL16_SL);
    }

    bool AsiIsNoFault(ASI asi)
    {
        return
            (asi == ASI_PNF) ||
            (asi == ASI_SNF) ||
            (asi == ASI_PNFL) ||
            (asi == ASI_SNFL);
    }

    bool AsiIsScratchPad(ASI asi)
    {
        return
            (asi == ASI_SCRATCHPAD) ||
            (asi == ASI_HYP_SCRATCHPAD);
    }

    bool AsiIsCmt(ASI asi)
    {
        return
            (asi == ASI_CMT_PER_STRAND);
    }

    bool AsiIsQueue(ASI asi)
    {
        return asi == ASI_QUEUE;
    }

    bool AsiIsMmu(ASI asi)
    {
        return  asi == ASI_MMU ||
               (asi >= ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0 &&
                asi <= ASI_IMMU_CTXT_ZERO_CONFIG) ||
               (asi >= ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0 &&
                asi <= ASI_IMMU_CTXT_NONZERO_CONFIG) ||
               (asi >= ASI_IMMU &&
                asi <= ASI_IMMU_TSB_PS1_PTR_REG) ||
               (asi >= ASI_ITLB_DATA_IN_REG  &&
                asi <= ASI_TLB_INVALIDATE_ALL);
    }

    bool AsiIsUnPriv(ASI asi)
    {
        return asi >= 0x80;
    }

    bool AsiIsPriv(ASI asi)
    {
        return asi <= 0x2f;
    }


    bool AsiIsHPriv(ASI asi)
    {
        return asi >= 0x30 && asi <= 0x7f;
    }

    bool AsiIsReg(ASI asi)
    {
        return AsiIsMmu(asi) || AsiIsScratchPad(asi);
    }

}
