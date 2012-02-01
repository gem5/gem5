/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 *          Steve Reinhardt
 */

/**
 * @file
 * Definitions of the Generational replacement policy.
 */

#include <string>

#include "base/misc.hh"
#include "base/types.hh"
#include "mem/cache/tags/iic_repl/gen.hh"
#include "mem/cache/tags/iic.hh"
#include "params/GenRepl.hh"

using namespace std;

GenRepl::GenRepl(const Params *p) // fix this, should be set by cache
    : Repl(p), num_pools(p->num_pools), fresh_res(p->fresh_res),
      pool_res(p->pool_res), num_entries(0), num_pool_entries(0), misses(0),
      pools(new GenPool[num_pools+1])
{
}

GenRepl::~GenRepl()
{
    delete [] pools;
}

unsigned long
GenRepl::getRepl()
{
    unsigned long tmp;
    GenReplEntry *re;
    int i;
    int num_seen = 0;
    if (!(num_pool_entries>0)) {
        fatal("No blks available to replace");
    }
    num_entries--;
    num_pool_entries--;
    for (i = 0; i < num_pools; i++) {
        while ((re = pools[i].pop())) {
            num_seen++;
            // Remove invalidated entries
            if (!re->valid) {
                delete re;
                continue;
            }
            if (iic->clearRef(re->tag_ptr)) {
                pools[(((i+1)== num_pools)? i :i+1)].push(re, misses);
            }
            else {
                tmp = re->tag_ptr;
                delete re;

                repl_pool.sample(i);

                return tmp;
            }
        }
    }
    fatal("No replacement found");
    return 0xffffffff;
}

unsigned long *
GenRepl::getNRepl(int n)
{
    unsigned long *tmp;
    GenReplEntry *re;
    int i;
    if (!(num_pool_entries>(n-1))) {
        fatal("Not enough blks available to replace");
    }
    num_entries -= n;
    num_pool_entries -= n;
    tmp = new unsigned long[n]; /* array of cache_blk pointers */
    int blk_index = 0;
    for (i = 0; i < num_pools && blk_index < n; i++) {
        while (blk_index < n && (re = pools[i].pop())) {
            // Remove invalidated entries
            if (!re->valid) {
                delete re;
                continue;
            }
            if (iic->clearRef(re->tag_ptr)) {
                pools[(((i+1)== num_pools)? i :i+1)].push(re, misses);
            }
            else {
                tmp[blk_index] = re->tag_ptr;
                blk_index++;
                delete re;
                repl_pool.sample(i);
            }
        }
    }
    if (blk_index >= n)
        return tmp;
    /* search the fresh pool */

    fatal("No N  replacements found");
    return NULL;
}

void
GenRepl::doAdvance(std::list<unsigned long> &demoted)
{
    int i;
    int num_seen = 0;
    GenReplEntry *re;
    misses++;
    for (i=0; i<num_pools; i++) {
        while (misses-pools[i].oldest > pool_res && (re = pools[i].pop())!=NULL) {
            if (iic->clearRef(re->tag_ptr)) {
                pools[(((i+1)== num_pools)? i :i+1)].push(re, misses);
                /** @todo Not really demoted, but use it for now. */
                demoted.push_back(re->tag_ptr);
                advance_pool.sample(i);
            }
            else {
                pools[(((i-1)<0)?i:i-1)].push(re, misses);
                demoted.push_back(re->tag_ptr);
                demote_pool.sample(i);
            }
        }
        num_seen += pools[i].size;
    }
    while (misses-pools[num_pools].oldest > fresh_res
          && (re = pools[num_pools].pop())!=NULL) {
        num_pool_entries++;
        if (iic->clearRef(re->tag_ptr)) {
            pools[num_pools/2].push(re, misses);
            /** @todo Not really demoted, but use it for now. */
            demoted.push_back(re->tag_ptr);
            advance_pool.sample(num_pools);
        }
        else {
            pools[num_pools/2-1].push(re, misses);
            demoted.push_back(re->tag_ptr);
            demote_pool.sample(num_pools);
        }
    }
}

void*
GenRepl::add(unsigned long tag_index)
{
    GenReplEntry *re = new GenReplEntry;
    re->tag_ptr = tag_index;
    re->valid = true;
    pools[num_pools].push(re, misses);
    num_entries++;
    return (void*)re;
}

void
GenRepl::regStatsWithSuffix(const string name)
{
    using namespace Stats;

    /** GEN statistics */
    repl_pool
        .init(0, 16, 1)
        .name(name + ".repl_pool_dist")
        .desc("Dist. of Repl. across pools")
        .flags(pdf)
        ;

    advance_pool
        .init(0, 16, 1)
        .name(name + ".advance_pool_dist")
        .desc("Dist. of Repl. across pools")
        .flags(pdf)
        ;

    demote_pool
        .init(0, 16, 1)
        .name(name + ".demote_pool_dist")
        .desc("Dist. of Repl. across pools")
        .flags(pdf)
        ;
}

int
GenRepl::fixTag(void* _re, unsigned long old_index, unsigned long new_index)
{
    GenReplEntry *re = (GenReplEntry*)_re;
    assert(re->valid);
    if (re->tag_ptr == old_index) {
        re->tag_ptr = new_index;
        return 1;
    }
    fatal("Repl entry: tag ptrs do not match");
    return 0;
}

bool
GenRepl::findTagPtr(unsigned long index)
{
    for (int i = 0; i < num_pools + 1; ++i) {
        list<GenReplEntry*>::const_iterator iter = pools[i].entries.begin();
        list<GenReplEntry*>::const_iterator end = pools[i].entries.end();
        for (; iter != end; ++iter) {
            if ((*iter)->valid && (*iter)->tag_ptr == index) {
                return true;
            }
        }
    }
    return false;
}

GenRepl *
GenReplParams::create()
{
    return new GenRepl(this);
}
