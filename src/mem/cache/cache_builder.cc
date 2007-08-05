/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Nathan Binkert
 */

/**
 * @file
 * Simobject instatiation of caches.
 */
#include <vector>

// Must be included first to determine which caches we want
#include "enums/Prefetch.hh"
#include "mem/config/cache.hh"
#include "mem/config/prefetch.hh"
#include "mem/cache/base_cache.hh"
#include "mem/cache/cache.hh"
#include "mem/bus.hh"
#include "params/BaseCache.hh"

// Tag Templates
#if defined(USE_CACHE_LRU)
#include "mem/cache/tags/lru.hh"
#endif

#if defined(USE_CACHE_FALRU)
#include "mem/cache/tags/fa_lru.hh"
#endif

#if defined(USE_CACHE_IIC)
#include "mem/cache/tags/iic.hh"
#endif

#if defined(USE_CACHE_SPLIT)
#include "mem/cache/tags/split.hh"
#endif

#if defined(USE_CACHE_SPLIT_LIFO)
#include "mem/cache/tags/split_lifo.hh"
#endif

//Prefetcher Headers
#if defined(USE_GHB)
#include "mem/cache/prefetch/ghb_prefetcher.hh"
#endif
#if defined(USE_TAGGED)
#include "mem/cache/prefetch/tagged_prefetcher.hh"
#endif
#if defined(USE_STRIDED)
#include "mem/cache/prefetch/stride_prefetcher.hh"
#endif


using namespace std;
using namespace TheISA;

#define BUILD_CACHE(TAGS, tags)                                      \
    do {                                                                \
        BasePrefetcher *pf;                                             \
        if (prefetch_policy == Enums::tagged) {                         \
            BUILD_TAGGED_PREFETCHER(TAGS);                              \
        }                                                               \
        else if (prefetch_policy == Enums::stride) {                    \
            BUILD_STRIDED_PREFETCHER(TAGS);                             \
        }                                                               \
        else if (prefetch_policy == Enums::ghb) {                       \
            BUILD_GHB_PREFETCHER(TAGS);                                 \
        }                                                               \
        else {                                                          \
            BUILD_NULL_PREFETCHER(TAGS);                                \
        }                                                               \
        Cache<TAGS>::Params params(tags, base_params,       \
                                   pf, prefetch_access, latency,        \
                                   true,                                \
                                   prefetch_miss);                      \
        Cache<TAGS> *retval =                                        \
            new Cache<TAGS>(name, params);                              \
        return retval;                                                  \
    } while (0)

#define BUILD_CACHE_PANIC(x) do {			\
        panic("%s not compiled into M5", x);		\
    } while (0)

#if defined(USE_CACHE_FALRU)
#define BUILD_FALRU_CACHE do {			    \
        FALRU *tags = new FALRU(block_size, size, latency); \
        BUILD_CACHE(FALRU, tags);		\
    } while (0)
#else
#define BUILD_FALRU_CACHE BUILD_CACHE_PANIC("falru cache")
#endif

#if defined(USE_CACHE_LRU)
#define BUILD_LRU_CACHE do {				\
        LRU *tags = new LRU(numSets, block_size, assoc, latency);	\
        BUILD_CACHE(LRU, tags);			\
    } while (0)
#else
#define BUILD_LRU_CACHE BUILD_CACHE_PANIC("lru cache")
#endif

#if defined(USE_CACHE_SPLIT)
#define BUILD_SPLIT_CACHE do {					\
        Split *tags = new Split(numSets, block_size, assoc, split_size, lifo, \
                                two_queue, latency);		\
        BUILD_CACHE(Split, tags);			\
    } while (0)
#else
#define BUILD_SPLIT_CACHE BUILD_CACHE_PANIC("split cache")
#endif

#if defined(USE_CACHE_SPLIT_LIFO)
#define BUILD_SPLIT_LIFO_CACHE do {				\
        SplitLIFO *tags = new SplitLIFO(block_size, size, assoc,        \
                                        latency, two_queue, -1);	\
        BUILD_CACHE(SplitLIFO, tags);			\
    } while (0)
#else
#define BUILD_SPLIT_LIFO_CACHE BUILD_CACHE_PANIC("lifo cache")
#endif

#if defined(USE_CACHE_IIC)
#define BUILD_IIC_CACHE do {			\
        IIC *tags = new IIC(iic_params);		\
        BUILD_CACHE(IIC, tags);	\
    } while (0)
#else
#define BUILD_IIC_CACHE BUILD_CACHE_PANIC("iic")
#endif

#define BUILD_CACHES do {				\
        if (repl == NULL) {				\
            if (numSets == 1) {				\
                BUILD_FALRU_CACHE;		\
            } else {					\
                if (split == true) {			\
                    BUILD_SPLIT_CACHE;		\
                } else if (lifo == true) {		\
                    BUILD_SPLIT_LIFO_CACHE;	\
                } else {				\
                    BUILD_LRU_CACHE;		\
                }					\
            }						\
        } else {					\
            BUILD_IIC_CACHE;			\
        }						\
    } while (0)

#define BUILD_COHERENCE(b) do {						\
    } while (0)

#if defined(USE_TAGGED)
#define BUILD_TAGGED_PREFETCHER(t)                              \
    pf = new TaggedPrefetcher(prefetcher_size,                  \
                              !prefetch_past_page,              \
                              prefetch_serial_squash,           \
                              prefetch_cache_check_push,        \
                              prefetch_data_accesses_only,      \
                              prefetch_latency,                 \
                              prefetch_degree)
#else
#define BUILD_TAGGED_PREFETCHER(t) BUILD_CACHE_PANIC("Tagged Prefetcher")
#endif

#if defined(USE_STRIDED)
#define BUILD_STRIDED_PREFETCHER(t)                             \
    pf = new StridePrefetcher(prefetcher_size,                  \
                              !prefetch_past_page,              \
                              prefetch_serial_squash,           \
                              prefetch_cache_check_push,        \
                              prefetch_data_accesses_only,      \
                              prefetch_latency,                 \
                              prefetch_degree,                  \
                              prefetch_use_cpu_id)
#else
#define BUILD_STRIDED_PREFETCHER(t) BUILD_CACHE_PANIC("Stride Prefetcher")
#endif

#if defined(USE_GHB)
#define BUILD_GHB_PREFETCHER(t)                         \
    pf = new GHBPrefetcher(prefetcher_size,             \
                           !prefetch_past_page,         \
                           prefetch_serial_squash,      \
                           prefetch_cache_check_push,   \
                           prefetch_data_accesses_only, \
                           prefetch_latency,            \
                           prefetch_degree,             \
                           prefetch_use_cpu_id)
#else
#define BUILD_GHB_PREFETCHER(t) BUILD_CACHE_PANIC("GHB Prefetcher")
#endif

#if defined(USE_TAGGED)
#define BUILD_NULL_PREFETCHER(t)                                \
    pf = new TaggedPrefetcher(prefetcher_size,                  \
                              !prefetch_past_page,              \
                              prefetch_serial_squash,           \
                              prefetch_cache_check_push,        \
                              prefetch_data_accesses_only,      \
                              prefetch_latency,                 \
                              prefetch_degree)
#else
#define BUILD_NULL_PREFETCHER(t) BUILD_CACHE_PANIC("NULL Prefetcher (uses Tagged)")
#endif

BaseCache *
BaseCacheParams::create()
{
    int numSets = size / (assoc * block_size);
    if (subblock_size == 0) {
        subblock_size = block_size;
    }

    // Build BaseCache param object
    BaseCache::Params base_params(latency, block_size,
                                  mshrs, tgts_per_mshr, write_buffers,
                                  max_miss_count);

    //Warnings about prefetcher policy
    if (prefetch_policy == Enums::none) {
        if (prefetch_miss || prefetch_access)
            panic("With no prefetcher, you shouldn't prefetch from"
                  " either miss or access stream\n");
    }

    if (prefetch_policy == Enums::tagged || prefetch_policy == Enums::stride ||
        prefetch_policy == Enums::ghb) {

        if (!prefetch_miss && !prefetch_access)
            warn("With this prefetcher you should chose a prefetch"
                 " stream (miss or access)\nNo Prefetching will occur\n");

        if (prefetch_miss && prefetch_access)
            panic("Can't do prefetches from both miss and access stream");
    }

#if defined(USE_CACHE_IIC)
    // Build IIC params
    IIC::Params iic_params;
    iic_params.size = size;
    iic_params.numSets = numSets;
    iic_params.blkSize = block_size;
    iic_params.assoc = assoc;
    iic_params.hashDelay = hash_delay;
    iic_params.hitLatency = latency;
    iic_params.rp = repl;
    iic_params.subblockSize = subblock_size;
#else
    const void *repl = NULL;
#endif

    BUILD_CACHES;
    return NULL;
}
