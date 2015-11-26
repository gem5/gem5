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
 * Definitions of a RECAP (Region-Aware Cache Partitioning) tag store.
 *
 * Each way contains an access permission register (APR) to control which cores
 * have access to it.
 *
 * Data that is private to a particular thread is kept left-justified in the cache
 * (i.e., in the low-numbered ways) whereas shared data is kept right-justified.
 */

#include "debug/CacheRepl.hh"
#include "mem/cache/tags/recap.hh"
#include "mem/cache/base.hh"

RECAP::RECAP(const Params *p)
    : BaseSetAssoc(p)
{
    for(uint32_t i = 0; i < numSets; i++)
    {
        CacheBlk *blk = findBlockBySetAndWay(i, 0);

        for(int c = 0; c < NUM_CORES; c++)
        {
            blk->apr_per_core.push_back(true);
        }
    }
}

CacheBlk*
RECAP::accessBlock(ThreadID threadId, Addr pc, Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(threadId, pc, addr, is_secure, lat, master_id);
    return blk;
}

CacheBlk*
RECAP::findVictim(Addr pc, Addr addr)
{
    CacheBlk *blk = BaseSetAssoc::findVictim(pc, addr);
    return blk;
}

void
RECAP::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);
}

void
RECAP::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);
}

RECAP*
RECAPParams::create()
{
    return new RECAP(this);
}

void
RECAP::determine_cache_requirements()
{
//    std::vector<int> blocks_req;

    for(int c = 0; c < NUM_CORES; c++)
    {

    }
}

double
RECAP::get_max_mu(uint32_t core_id)
{
    double max_mu = 0;

    for(uint32_t j = 1; j <= assoc; j++)
    {
        int utility = 0; //TODO
        double mu = utility / j;
        if(mu > max_mu)
        {
            max_mu = mu;
        }
    }

    return max_mu;
}
