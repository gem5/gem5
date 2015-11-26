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
 * Declaration of a DBRSP (Dead Block Replacement with a Sampling Predictor) tag store.
 */

#ifndef __MEM_CACHE_TAGS_DBRSP_HH__
#define __MEM_CACHE_TAGS_DBRSP_HH__

#include "mem/cache/tags/base_set_assoc.hh"
#include "params/DBRSP.hh"

#include "utils.hh"

struct sampler;

class DBRSP : public BaseSetAssoc
{
  private:
    sampler *samp;

  public:
    /** Convenience typedef. */
    typedef DBRSPParams Params;

    /**
     * Construct and initialize this tag store.
     */
    DBRSP(const Params *p);

    /**
     * Destructor
     */
    ~DBRSP() {}

    CacheBlk* accessBlock(ThreadID threadId, Addr pc, Addr addr, bool is_secure, Cycles &lat,
                         int context_src);
    CacheBlk* findVictim(Addr pc, Addr addr);
    void insertBlock(PacketPtr pkt, BlkType *blk);
    void invalidate(CacheBlk *blk);

    INT32 Get_Sampler_Victim ( UINT32 tid, UINT32 setIndex, UINT32 assoc, Addr_t PC, Addr_t paddr, UINT32 accessType );
    void   UpdateSampler ( UINT32 setIndex, Addr_t tag, UINT32 tid, Addr_t PC, INT32 updateWayID, bool hit);
};

struct sampler_entry {
	unsigned int
		lru_stack_position,
		tag,
		trace,
		prediction;

	bool
		valid;

	// constructor for sampler entry
	sampler_entry (void) {
		lru_stack_position = 0;
		valid = false;
		tag = 0;
		trace = 0;
		prediction = 0;
	};
};

// one sampler set (just a pointer to the entries)
struct sampler_set {
	sampler_entry *blocks;

	sampler_set (void);
};

// the dead block predictor
struct predictor {
	int **tables; 	// tables of two-bit counters

	predictor (void);
	unsigned int get_table_index (UINT32 tid, unsigned int, int t);
	bool get_prediction (UINT32 tid, unsigned int trace, int set);
	void block_is_dead (UINT32 tid, unsigned int, bool);
};

// the sampler
struct sampler {
	sampler_set *sets;
	int
		nsampler_sets,   // number of sampler sets
		sampler_modulus; // determines which LLC sets are sampler sets

	predictor *pred;
	sampler (int nsets, int assoc);
	void access (UINT32 tid, int set, Addr_t tag, Addr_t PC);
};

#endif // __MEM_CACHE_TAGS_DBRSP_HH__
