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
#include <list>
#include <vector>

#include "config/the_isa.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/config/cache.hh"
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


using namespace std;

#define BUILD_CACHE(TAGS, tags)                         \
    do {                                                \
        Cache<TAGS> *retval =                           \
            new Cache<TAGS>(this, tags);            \
        return retval;                                  \
    } while (0)

#define BUILD_CACHE_PANIC(x) do {                       \
        panic("%s not compiled into M5", x);            \
    } while (0)

#if defined(USE_CACHE_FALRU)
#define BUILD_FALRU_CACHE do {                              \
        FALRU *tags = new FALRU(block_size, size, latency); \
        BUILD_CACHE(FALRU, tags);                           \
    } while (0)
#else
#define BUILD_FALRU_CACHE BUILD_CACHE_PANIC("falru cache")
#endif

#if defined(USE_CACHE_LRU)
#define BUILD_LRU_CACHE do {                                            \
        LRU *tags = new LRU(numSets, block_size, assoc, latency);       \
        BUILD_CACHE(LRU, tags);                                         \
    } while (0)
#else
#define BUILD_LRU_CACHE BUILD_CACHE_PANIC("lru cache")
#endif

#if defined(USE_CACHE_IIC)
#define BUILD_IIC_CACHE do {                            \
        IIC *tags = new IIC(iic_params);                \
        BUILD_CACHE(IIC, tags);                         \
    } while (0)
#else
#define BUILD_IIC_CACHE BUILD_CACHE_PANIC("iic")
#endif

#define BUILD_CACHES do {                               \
        if (repl == NULL) {                             \
            if (numSets == 1) {                         \
                BUILD_FALRU_CACHE;                      \
            } else {                                    \
               BUILD_LRU_CACHE;                    \
            }                                           \
        } else {                                        \
            BUILD_IIC_CACHE;                            \
        }                                               \
    } while (0)

BaseCache *
BaseCacheParams::create()
{
    int numSets = size / (assoc * block_size);
    if (subblock_size == 0) {
        subblock_size = block_size;
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
