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
#include "mem/config/cache.hh"
#include "mem/config/prefetch.hh"

#include "mem/cache/base_cache.hh"
#include "mem/cache/cache.hh"
#include "mem/bus.hh"
#include "mem/cache/coherence/coherence_protocol.hh"
#include "sim/builder.hh"

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

// Compression Templates
#include "base/compression/null_compression.hh"
#include "base/compression/lzss_compression.hh"

// MissQueue Templates
#include "mem/cache/miss/miss_queue.hh"
#include "mem/cache/miss/blocking_buffer.hh"

// Coherence Templates
#include "mem/cache/coherence/simple_coherence.hh"

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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(BaseCache)

    Param<int> size;
    Param<int> assoc;
    Param<int> block_size;
    Param<int> latency;
    Param<int> mshrs;
    Param<int> tgts_per_mshr;
    Param<int> write_buffers;
    Param<bool> prioritizeRequests;
    SimObjectParam<CoherenceProtocol *> protocol;
    Param<Addr> trace_addr;
    Param<int> hash_delay;
#if defined(USE_CACHE_IIC)
    SimObjectParam<Repl *> repl;
#endif
    Param<bool> compressed_bus;
    Param<bool> store_compressed;
    Param<bool> adaptive_compression;
    Param<int> compression_latency;
    Param<int> subblock_size;
    Param<Counter> max_miss_count;
    VectorParam<Range<Addr> > addr_range;
//    SimObjectParam<MemTraceWriter *> mem_trace;
    Param<bool> split;
    Param<int> split_size;
    Param<bool> lifo;
    Param<bool> two_queue;
    Param<bool> prefetch_miss;
    Param<bool> prefetch_access;
    Param<int> prefetcher_size;
    Param<bool> prefetch_past_page;
    Param<bool> prefetch_serial_squash;
    Param<Tick> prefetch_latency;
    Param<int> prefetch_degree;
    Param<string> prefetch_policy;
    Param<bool> prefetch_cache_check_push;
    Param<bool> prefetch_use_cpu_id;
    Param<bool> prefetch_data_accesses_only;

END_DECLARE_SIM_OBJECT_PARAMS(BaseCache)


BEGIN_INIT_SIM_OBJECT_PARAMS(BaseCache)

    INIT_PARAM(size, "capacity in bytes"),
    INIT_PARAM(assoc, "associativity"),
    INIT_PARAM(block_size, "block size in bytes"),
    INIT_PARAM(latency, "hit latency in CPU cycles"),
    INIT_PARAM(mshrs, "number of MSHRs (max outstanding requests)"),
    INIT_PARAM(tgts_per_mshr, "max number of accesses per MSHR"),
    INIT_PARAM_DFLT(write_buffers, "number of write buffers", 8),
    INIT_PARAM_DFLT(prioritizeRequests, "always service demand misses first",
                    false),
    INIT_PARAM_DFLT(protocol, "coherence protocol to use in the cache", NULL),
    INIT_PARAM_DFLT(trace_addr, "address to trace", 0),

    INIT_PARAM_DFLT(hash_delay, "time in cycles of hash access",1),
#if defined(USE_CACHE_IIC)
    INIT_PARAM_DFLT(repl, "replacement policy",NULL),
#endif
    INIT_PARAM_DFLT(compressed_bus,
                    "This cache connects to a compressed memory",
                    false),
    INIT_PARAM_DFLT(store_compressed, "Store compressed data in the cache",
                    false),
    INIT_PARAM_DFLT(adaptive_compression, "Use an adaptive compression scheme",
                    false),
    INIT_PARAM_DFLT(compression_latency,
                    "Latency in cycles of compression algorithm",
                    0),
    INIT_PARAM_DFLT(subblock_size,
                    "Size of subblock in IIC used for compression",
                    0),
    INIT_PARAM_DFLT(max_miss_count,
                    "The number of misses to handle before calling exit",
                    0),
    INIT_PARAM_DFLT(addr_range, "The address range in bytes",
                    vector<Range<Addr> >(1,RangeIn((Addr)0, MaxAddr))),
//    INIT_PARAM_DFLT(mem_trace, "Memory trace to write accesses to", NULL),
    INIT_PARAM_DFLT(split, "Whether this is a partitioned cache", false),
    INIT_PARAM_DFLT(split_size, "the number of \"ways\" belonging to the LRU partition", 0),
    INIT_PARAM_DFLT(lifo, "whether you are using a LIFO repl. policy", false),
    INIT_PARAM_DFLT(two_queue, "whether the lifo should have two queue replacement", false),
    INIT_PARAM_DFLT(prefetch_miss, "wheter you are using the hardware prefetcher from Miss stream", false),
    INIT_PARAM_DFLT(prefetch_access, "wheter you are using the hardware prefetcher from Access stream", false),
    INIT_PARAM_DFLT(prefetcher_size, "Number of entries in the harware prefetch queue", 100),
    INIT_PARAM_DFLT(prefetch_past_page, "Allow prefetches to cross virtual page boundaries", false),
    INIT_PARAM_DFLT(prefetch_serial_squash, "Squash prefetches with a later time on a subsequent miss", false),
    INIT_PARAM_DFLT(prefetch_latency, "Latency of the prefetcher", 10),
    INIT_PARAM_DFLT(prefetch_degree, "Degree of the prefetch depth", 1),
    INIT_PARAM_DFLT(prefetch_policy, "Type of prefetcher to use", "none"),
    INIT_PARAM_DFLT(prefetch_cache_check_push, "Check if in cash on push or pop of prefetch queue", true),
    INIT_PARAM_DFLT(prefetch_use_cpu_id, "Use the CPU ID to seperate calculations of prefetches", true),
    INIT_PARAM_DFLT(prefetch_data_accesses_only, "Only prefetch on data not on instruction accesses", false)
END_INIT_SIM_OBJECT_PARAMS(BaseCache)


#define BUILD_CACHE(TAGS, tags, c)                                      \
    do {                                                                \
        BasePrefetcher *pf;                                           \
        if (pf_policy == "tagged") {                                    \
            BUILD_TAGGED_PREFETCHER(TAGS);                              \
        }                                                               \
        else if (pf_policy == "stride") {                               \
            BUILD_STRIDED_PREFETCHER(TAGS);                             \
        }                                                               \
        else if (pf_policy == "ghb") {                                  \
            BUILD_GHB_PREFETCHER(TAGS);                                 \
        }                                                               \
        else {                                                          \
            BUILD_NULL_PREFETCHER(TAGS);                                \
        }                                                               \
        Cache<TAGS, c>::Params params(tags, mq, coh, base_params,       \
                                      pf, prefetch_access, latency, \
                                      true,                             \
                                      store_compressed,                 \
                                      adaptive_compression,             \
                                      compressed_bus,                   \
                                      compAlg, compression_latency,     \
                                      prefetch_miss);                   \
        Cache<TAGS, c> *retval =                                        \
            new Cache<TAGS, c>(getInstanceName(), params);              \
        return retval;                                                  \
    } while (0)

#define BUILD_CACHE_PANIC(x) do {			\
        panic("%s not compiled into M5", x);		\
    } while (0)

#define BUILD_COMPRESSED_CACHE(TAGS, tags, c)           \
    do {                                                \
        CompressionAlgorithm *compAlg;                  \
        if (compressed_bus || store_compressed) {       \
            compAlg = new LZSSCompression();            \
        } else {                                        \
            compAlg = new NullCompression();            \
        }                                               \
        BUILD_CACHE(TAGS, tags, c);                     \
    } while (0)

#if defined(USE_CACHE_FALRU)
#define BUILD_FALRU_CACHE(c) do {			    \
        FALRU *tags = new FALRU(block_size, size, latency); \
        BUILD_COMPRESSED_CACHE(FALRU, tags, c);		\
    } while (0)
#else
#define BUILD_FALRU_CACHE(c) BUILD_CACHE_PANIC("falru cache")
#endif

#if defined(USE_CACHE_LRU)
#define BUILD_LRU_CACHE(c) do {				\
        LRU *tags = new LRU(numSets, block_size, assoc, latency);	\
        BUILD_COMPRESSED_CACHE(LRU, tags, c);			\
    } while (0)
#else
#define BUILD_LRU_CACHE(c) BUILD_CACHE_PANIC("lru cache")
#endif

#if defined(USE_CACHE_SPLIT)
#define BUILD_SPLIT_CACHE(c) do {					\
        Split *tags = new Split(numSets, block_size, assoc, split_size, lifo, \
                                two_queue, latency);		\
        BUILD_COMPRESSED_CACHE(Split, tags, c);			\
    } while (0)
#else
#define BUILD_SPLIT_CACHE(c) BUILD_CACHE_PANIC("split cache")
#endif

#if defined(USE_CACHE_SPLIT_LIFO)
#define BUILD_SPLIT_LIFO_CACHE(c) do {				\
        SplitLIFO *tags = new SplitLIFO(block_size, size, assoc,        \
                                        latency, two_queue, -1);	\
        BUILD_COMPRESSED_CACHE(SplitLIFO, tags, c);			\
    } while (0)
#else
#define BUILD_SPLIT_LIFO_CACHE(c) BUILD_CACHE_PANIC("lifo cache")
#endif

#if defined(USE_CACHE_IIC)
#define BUILD_IIC_CACHE(c) do {			\
        IIC *tags = new IIC(iic_params);		\
        BUILD_COMPRESSED_CACHE(IIC, tags, c);	\
    } while (0)
#else
#define BUILD_IIC_CACHE(c) BUILD_CACHE_PANIC("iic")
#endif

#define BUILD_CACHES(c) do {				\
        if (repl == NULL) {				\
            if (numSets == 1) {				\
                BUILD_FALRU_CACHE(c);		\
            } else {					\
                if (split == true) {			\
                    BUILD_SPLIT_CACHE(c);		\
                } else if (lifo == true) {		\
                    BUILD_SPLIT_LIFO_CACHE(c);	\
                } else {				\
                    BUILD_LRU_CACHE(c);		\
                }					\
            }						\
        } else {					\
            BUILD_IIC_CACHE(c);			\
        }						\
    } while (0)

#define BUILD_COHERENCE(b) do {						\
        SimpleCoherence *coh = new SimpleCoherence(protocol);           \
        BUILD_CACHES(SimpleCoherence);                                  \
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

CREATE_SIM_OBJECT(BaseCache)
{
    string name = getInstanceName();
    int numSets = size / (assoc * block_size);
    string pf_policy = prefetch_policy;
    if (subblock_size == 0) {
        subblock_size = block_size;
    }

    // Build BaseCache param object
    BaseCache::Params base_params(addr_range, latency,
                                  block_size, max_miss_count);

    //Warnings about prefetcher policy
    if (pf_policy == "none" && (prefetch_miss || prefetch_access)) {
        panic("With no prefetcher, you shouldn't prefetch from"
              " either miss or access stream\n");
    }
    if ((pf_policy == "tagged" || pf_policy == "stride" ||
         pf_policy == "ghb") && !(prefetch_miss || prefetch_access)) {
        warn("With this prefetcher you should chose a prefetch"
             " stream (miss or access)\nNo Prefetching will occur\n");
    }
    if ((pf_policy == "tagged" || pf_policy == "stride" ||
         pf_policy == "ghb") && prefetch_miss && prefetch_access) {
        panic("Can't do prefetches from both miss and access"
              " stream\n");
    }
    if (pf_policy != "tagged" && pf_policy != "stride" &&
        pf_policy != "ghb"    && pf_policy != "none") {
        panic("Unrecognized form of a prefetcher: %s, try using"
              "['none','stride','tagged','ghb']\n", pf_policy);
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

    if (mshrs == 1 /*|| out_bus->doEvents() == false*/) {
        BlockingBuffer *mq = new BlockingBuffer(true);
        BUILD_COHERENCE(BlockingBuffer);
    } else {
        MissQueue *mq = new MissQueue(mshrs, tgts_per_mshr, write_buffers,
                                      true, prefetch_miss);
        BUILD_COHERENCE(MissQueue);
    }
    return NULL;
}

REGISTER_SIM_OBJECT("BaseCache", BaseCache)


#endif //DOXYGEN_SHOULD_SKIP_THIS
