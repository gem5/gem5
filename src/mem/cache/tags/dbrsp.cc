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
 * Definitions of a DRPSP tag store.
 */

#include "debug/CacheRepl.hh"
#include "mem/cache/tags/dbrsp.hh"
#include "mem/cache/base.hh"

// some run-time constants
int
	// sampler associativity (changed for 4MB cache)
	dan_sampler_assoc = 12,

	// number of bits used to index predictor; determines number of
	// entries in prediction tables (changed for 4MB cache)
	dan_predictor_index_bits = 12,

	// number of prediction tables
	dan_predictor_tables = 3,

	// width of prediction saturating counters
	dan_counter_width = 2,

	// predictor must meet this threshold to predict a block is dead
	dan_threshold = 8,

	// number of partial tag bits kept per sampler entry
	dan_sampler_tag_bits = 16,

	// number of trace (partial PC) bits kept per sampler entry
	dan_sampler_trace_bits = 16,

	// number of entries in prediction table; derived from # of index bits
	dan_predictor_table_entries,

	// maximum value of saturating counter; derived from counter width
	dan_counter_max;

DBRSP::DBRSP(const Params *p)
    : BaseSetAssoc(p)
{
    samp = new sampler (numSets, assoc);
}

CacheBlk*
DBRSP::accessBlock(ThreadID threadId, Addr pc, Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(threadId, pc, addr, is_secure, lat, master_id);

    if (blk != NULL) {
		UpdateSampler (blk->set, blk->tag, threadId, pc, blk->way, true);
    }
    return blk;
}

CacheBlk*
DBRSP::findVictim(ThreadID threadId, Addr pc, Addr addr)
{
    // grab a replacement candidate
    BlkType *blk = BaseSetAssoc::findVictim(threadId, pc, addr);
    int set = extractSet(addr);

    // if all blocks are valid, pick a replacement at random
    if (blk && blk->isValid())
    {
        blk = findBlockBySetAndWay(set, Get_Sampler_Victim( threadId, set, assoc, pc, addr ));
    }
    return blk;
}

void
DBRSP::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);

    ThreadID threadId = pkt->req->hasThreadId() ? pkt->req->threadId() : 0;

    Addr pc = pkt->req->hasPC() ?
        pkt->req->getPC() : 0;

	UpdateSampler (blk->set, blk->tag, threadId, pc, blk->way, false);
}

void
DBRSP::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);

    // should be evicted before valid blocks
    int set = blk->set;
    sets[set].moveToTail(blk);
}

DBRSP*
DBRSPParams::create()
{
    return new DBRSP(this);
}

// make a trace from a PC (just extract some bits)
inline unsigned int make_trace (UINT32 tid, predictor *pred, Addr_t PC) {
	return PC & ((1<<dan_sampler_trace_bits)-1);
}

// called when there is an access to an LLC cache block
void
DBRSP::UpdateSampler (UINT32 setIndex, Addr_t tag, UINT32 tid, Addr_t PC, INT32 way, bool hit)
{
	// determine if this is a sampler set
	if (setIndex % samp->sampler_modulus == 0)
	{
		// this is a sampler set.  access the sampler.
		int set = setIndex / samp->sampler_modulus;
		if (set >= 0 && set < samp->nsampler_sets)
			samp->access (tid, set, tag, PC);
	}

	// update default replacement policy
    sets[setIndex].moveToHead(findBlockBySetAndWay(setIndex, way));

	// make the trace
	unsigned int trace = make_trace (tid, samp->pred, PC);

	// get the next prediction for this block using that trace
	findBlockBySetAndWay(setIndex, way)->prediction = samp->pred->get_prediction (tid, trace, setIndex);
}

// called to select a victim.  returns victim way, or -1 if the block should bypass
INT32
DBRSP::Get_Sampler_Victim ( UINT32 tid, UINT32 setIndex, UINT32 assoc, Addr_t PC, Addr_t paddr )
{
	// select a victim using default LRU policy
    BlkType *blk = NULL;
    for (int i = assoc - 1; i >= 0; i--)
    {
        BlkType *b = sets[setIndex].blks[i];
        if (b->way < allocAssoc) {
            blk = b;
            break;
        }
    }
    assert(!blk || blk->way < allocAssoc);

    int r = blk->way;

	// look for a predicted dead block
	for (unsigned int i=0; i<assoc; i++) {
		if (findBlockBySetAndWay(setIndex, i)->prediction)
		{
			// found a predicted dead block; this is our new victim
			r = i;
			break;
		}
	}

	// return the selected victim
	return r;
}

// constructor for a sampler set
sampler_set::sampler_set (void)
{
	// allocate some sampler entries
	blocks = new sampler_entry[dan_sampler_assoc];

	// initialize the LRU replacement algorithm for these entries
	for (int i=0; i<dan_sampler_assoc; i++)
		blocks[i].lru_stack_position = i;
}

// access the sampler with an LLC tag
void
sampler::access (UINT32 tid, int set, Addr_t tag, Addr_t PC)
 {
	// get a pointer to this set's sampler entries
	sampler_entry *blocks = &sets[set].blocks[0];

	// get a partial tag to search for
	unsigned int partial_tag = tag & ((1<<dan_sampler_tag_bits)-1);

	// this will be the way of the sampler entry we end up hitting or replacing
	int i;

	// search for a matching tag
	for (i=0; i<dan_sampler_assoc; i++)
	{
        if (blocks[i].valid && (blocks[i].tag == partial_tag))
        {
            // we know this block is not dead; inform the predictor
            pred->block_is_dead (tid, blocks[i].trace, false);
            break;
        }
	}

	// did we find a match?
	if (i == dan_sampler_assoc)
	{
		// look for an invalid block to replace
		for (i=0; i<dan_sampler_assoc; i++) if (!blocks[i].valid) break;

		// no invalid block?  look for a dead block.
		if (i == dan_sampler_assoc)
		{
			// find the LRU dead block
			for (i=0; i<dan_sampler_assoc; i++) if (blocks[i].prediction) break;
		}

		// no invalid or dead block?  use the LRU block
		if (i == dan_sampler_assoc)
		{
			int j;
			for (j=0; j<dan_sampler_assoc; j++)
				if (blocks[j].lru_stack_position == (unsigned int) (dan_sampler_assoc-1)) break;
			assert (j < dan_sampler_assoc);
			i = j;
		}

		// previous trace leads to block being dead; inform the predictor
		pred->block_is_dead (tid, blocks[i].trace, true);

		// fill the victim block
		blocks[i].tag = partial_tag;
		blocks[i].valid = true;
	}

	// record the trace
	blocks[i].trace = make_trace (tid, pred, PC);

	// get the next prediction for this entry
	blocks[i].prediction = pred->get_prediction (tid, blocks[i].trace, -1);

	// now the replaced entry should be moved to the MRU position
	unsigned int position = blocks[i].lru_stack_position;
	for(int way=0; way<dan_sampler_assoc; way++)
		if (blocks[way].lru_stack_position < position)
			blocks[way].lru_stack_position++;
	blocks[i].lru_stack_position = 0;
}

// constructor for sampler
sampler::sampler (int nsets, int assoc)
{
	// four-core version gets slightly different parameters
	if (nsets == 4096)
	{
		dan_sampler_assoc = 13;
		dan_predictor_index_bits = 14;
	}

	// here, we figure out the total number of bits used by the various
	// structures etc.  along the way we will figure out how many
	// sampler sets we have room for

	// figure out number of entries in each table
	dan_predictor_table_entries = 1 << dan_predictor_index_bits;

	nsampler_sets = 128; //TODO: should not be hardcoded

	// compute the maximum saturating counter value; predictor constructor
	// needs this so we do it here
	dan_counter_max = (1 << dan_counter_width) -1;

	// make a predictor
	pred = new predictor ();

	// make the sampler sets
	sets = new sampler_set [nsampler_sets];

	// figure out what should divide evenly into a set index to be
	// considered a sampler set
	sampler_modulus = nsets / nsampler_sets;

	assert(sampler_modulus>0);
}

// constructor for the predictor
predictor::predictor (void)
{
	// make the tables
	tables = new int* [dan_predictor_tables];

	// initialize each table to all 0s
	for (int i=0; i<dan_predictor_tables; i++)
	{
		tables[i] = new int[dan_predictor_table_entries];
		memset (tables[i], 0, sizeof (int) * dan_predictor_table_entries);
	}
}

// hash three numbers into one
inline unsigned int mix (unsigned int a, unsigned int b, unsigned int c)
{
	a=a-b;  a=a-c;  a=a^(c >> 13);
	b=b-c;  b=b-a;  b=b^(a << 8);
	c=c-a;  c=c-b;  c=c^(b >> 13);
	return c;
}

// first hash function
inline unsigned int f1 (unsigned int x)
{
	return mix (0xfeedface, 0xdeadb10c, x);
}

// second hash function
inline unsigned int f2 (unsigned int x)
{
	return mix (0xc001d00d, 0xfade2b1c, x);
}

// generalized hash function
inline unsigned int fi (unsigned int x, int i)
{
	return f1 (x) + (f2 (x) >> i);
}

// hash a trace, thread ID, and predictor table number into a predictor table index
unsigned int predictor::get_table_index (UINT32 tid, unsigned int trace, int t)
{
	unsigned int x = fi (trace ^ (tid << 2), t);
	return x & ((1<<dan_predictor_index_bits)-1);
}

// inform the predictor that a block is either dead or not dead
void
predictor::block_is_dead (UINT32 tid, unsigned int trace, bool d)
{
	// for each predictor table...
	for (int i=0; i<dan_predictor_tables; i++)
	{
		// ...get a pointer to the corresponding entry in that table
		int *c = &tables[i][get_table_index (tid, trace, i)];

		// if the block is dead, increment the counter
		if (d)
		{
			if (*c < dan_counter_max) (*c)++;
		}
		else
		{
			// otherwise, decrease the counter
			if (i & 1)
			{
				// odd numbered tables decrease exponentially
				(*c) >>= 1;
			} else
			{
				// even numbered tables decrease by one
				if (*c > 0) (*c)--;
			}
		}
	}
}

// get a prediction for a given trace
bool
predictor::get_prediction (UINT32 tid, unsigned int trace, int set)
{
	// start the confidence sum as 0
	int conf = 0;

	// for each table...
	for (int i=0; i<dan_predictor_tables; i++)
	{
		// ...get the counter value for that table...
		int val = tables[i][get_table_index (tid, trace, i)];

		// and add it to the running total
		conf += val;
	}

	// if the counter is at least the threshold, the block is predicted dead
	return conf >= dan_threshold;
}