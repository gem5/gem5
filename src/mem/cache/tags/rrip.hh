/*
 * Copyright (c) 2015 Min Cai
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Authors: Min Cai
 */

/**
 * @file
 * Declaration of an RRIP tag store.
 */

#ifndef __MEM_CACHE_TAGS_RRIP_HH__
#define __MEM_CACHE_TAGS_RRIP_HH__

#include "mem/cache/tags/base_set_assoc.hh"
#include "params/RRIP.hh"

#include "utils.hh"
#include "dip.hh"

/********************************************************************************
** Parameter for Re-Reference Interval Prediction (RRIP) [ISCA'2010]:
** ===================================================================
**
** Change macro maxRPV below to simulate one of the following policies:
**
**     (a)  maxRPV=1 simulates Not Recently Used (NRU) replacement.
**          NRU is the policy commonly used in microprocessors today.
**
**     (b)  maxRPV=3 simulates Static RRIP (SRRIP) with max prediction of 3
**
**     (c)  maxRPV=7 simulates Static RRIP (SRRIP) with max prediction of 7
**
**
** Dynamic Insertion Policy (DIP) [ISCA'07 and PACT'08]:
** ======================================================
**
** Change macro USE_INSERTION_POLICY below to choose one of the following policies:
**
**     (a)  USE_INSERTION_POLICY=0 simulates baseline policy where all newly
**          inserted blocks are moved to head of RRIP chain (i.e. MRU position)
**
**     (b)  USE_INSERTION_POLICY=1 simulates DIP where newly inserted blocks
**          are inserted either at head of RRIP chain or tail of RRIP chain. In
**          a shared cache, the policy decision is thread-unaware [ISCA'07]
**
**     (c)  USE_INSERTION_POLICY=2 simulates TADIP where newly inserted blocks
**          are inserted either at head of RRIP chain or tail of RRIP chain on a
**          per-thread basis. The policy decisions are thread-aware [PACT'08]
**
** Other DIP Parameters:
** =====================
**
**     (a) USE_PSELwidth: The number of bits in the Policy Selection (PSEL) counter
**
**     (b) USE_SDMsize:   The number of sets sampled in a Set Dueling Monitor (SDM)
**
** IMPORTANT NOTES:
** =====================
**
** NOTE 1:  The DIP and TADIP papers illustrated a mechanism for using the
** cache set index to identify SDMs.  To provide a generalized framework
** where SDMs can be selected for any cache configuration, this source code release
** selects SDMs at random and store the SDM type in a separate data structure.
** However, note that this extra data structure is NOT required as described in the
** DIP and TADIP papers.
**
** NOTE 2:  DRRIP policy configuration is maxRPV > 1 and USE_INSERTION_POLICY=2 (TADIP)
**
********************************************************************************/

#define maxRPV                3     // Use SRRIP replacement

#define USE_INSERTION_POLICY  2     // Use TADIP
#define USE_PSELwidth         10    // Width of the PSEL counter
#define USE_SDMsize           32    // Use 32 sets per SDM

class RRIP : public BaseSetAssoc
{
  public:
    /** Convenience typedef. */
    typedef RRIPParams Params;

    /**
     * Construct and initialize this tag store.
     */
    RRIP(const Params *p);

    /**
     * Destructor
     */
    ~RRIP() {}

    CacheBlk* accessBlock(ThreadID threadId, Addr pc, Addr addr, bool is_secure, Cycles &lat,
                         int context_src);
    CacheBlk* findVictim(Addr pc, Addr addr);
    void insertBlock(PacketPtr pkt, BlkType *blk);
    void invalidate(CacheBlk *blk);

  private:
    INT32  Get_RRIP_Victim( UINT32 setIndex );

    void   UpdateRRIP( UINT32 tid, UINT32 setIndex, INT32 updateWayID, bool cacheHit );

    DIP *rrip;
};

#endif // __MEM_CACHE_TAGS_RRIP_HH__
