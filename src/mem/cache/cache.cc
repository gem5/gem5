/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 *          Lisa Hsu
 *          Kevin Lim
 */

/**
 * @file
 * Cache template instantiations.
 */

#include "mem/config/cache.hh"
#include "mem/config/compression.hh"

#include "mem/cache/tags/cache_tags.hh"

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

#include "base/compression/null_compression.hh"
#if defined(USE_LZSS_COMPRESSION)
#include "base/compression/lzss_compression.hh"
#endif

#include "mem/cache/miss/miss_queue.hh"
#include "mem/cache/miss/blocking_buffer.hh"

#include "mem/cache/coherence/uni_coherence.hh"
#include "mem/cache/coherence/simple_coherence.hh"

#include "mem/cache/cache_impl.hh"

// Template Instantiations
#ifndef DOXYGEN_SHOULD_SKIP_THIS


#if defined(USE_CACHE_FALRU)
template class Cache<CacheTags<FALRU,NullCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<FALRU,NullCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<FALRU,NullCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<FALRU,NullCompression>, MissQueue, UniCoherence>;
#if defined(USE_LZSS_COMPRESSION)
template class Cache<CacheTags<FALRU,LZSSCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<FALRU,LZSSCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<FALRU,LZSSCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<FALRU,LZSSCompression>, MissQueue, UniCoherence>;
#endif
#endif

#if defined(USE_CACHE_IIC)
template class Cache<CacheTags<IIC,NullCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<IIC,NullCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<IIC,NullCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<IIC,NullCompression>, MissQueue, UniCoherence>;
#if defined(USE_LZSS_COMPRESSION)
template class Cache<CacheTags<IIC,LZSSCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<IIC,LZSSCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<IIC,LZSSCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<IIC,LZSSCompression>, MissQueue, UniCoherence>;
#endif
#endif

#if defined(USE_CACHE_LRU)
template class Cache<CacheTags<LRU,NullCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<LRU,NullCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<LRU,NullCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<LRU,NullCompression>, MissQueue, UniCoherence>;
#if defined(USE_LZSS_COMPRESSION)
template class Cache<CacheTags<LRU,LZSSCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<LRU,LZSSCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<LRU,LZSSCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<LRU,LZSSCompression>, MissQueue, UniCoherence>;
#endif
#endif

#if defined(USE_CACHE_SPLIT)
template class Cache<CacheTags<Split,NullCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<Split,NullCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<Split,NullCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<Split,NullCompression>, MissQueue, UniCoherence>;
#if defined(USE_LZSS_COMPRESSION)
template class Cache<CacheTags<Split,LZSSCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<Split,LZSSCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<Split,LZSSCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<Split,LZSSCompression>, MissQueue, UniCoherence>;
#endif
#endif

#if defined(USE_CACHE_SPLIT_LIFO)
template class Cache<CacheTags<SplitLIFO,NullCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<SplitLIFO,NullCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<SplitLIFO,NullCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<SplitLIFO,NullCompression>, MissQueue, UniCoherence>;
#if defined(USE_LZSS_COMPRESSION)
template class Cache<CacheTags<SplitLIFO,LZSSCompression>, BlockingBuffer, SimpleCoherence>;
template class Cache<CacheTags<SplitLIFO,LZSSCompression>, BlockingBuffer, UniCoherence>;
template class Cache<CacheTags<SplitLIFO,LZSSCompression>, MissQueue, SimpleCoherence>;
template class Cache<CacheTags<SplitLIFO,LZSSCompression>, MissQueue, UniCoherence>;
#endif
#endif

#endif //DOXYGEN_SHOULD_SKIP_THIS
