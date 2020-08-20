/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 */

#ifndef __BASE__CHUNK_GENERATOR_HH__
#define __BASE__CHUNK_GENERATOR_HH__

/**
 * @file
 * Declaration and inline definition of ChunkGenerator object.
 */

#include <algorithm>

#include "base/intmath.hh"
#include "base/types.hh"

/**
 * This class takes an arbitrary memory region (address/length pair)
 * and generates a series of appropriately (e.g. block- or page-)
 * aligned chunks covering the same region.
 *
 * Example usage:

\code
    for (ChunkGenerator gen(addr, size, chunkSize); !gen.done(); gen.next()) {
        doSomethingChunky(gen.addr(), gen.size());
    }
\endcode
 */
class ChunkGenerator
{
  private:
    /** The starting address of the current chunk. */
    Addr curAddr;
    /** The starting address of the next chunk (after the current one). */
    Addr nextAddr;
    /** The size of the current chunk (in bytes). */
    unsigned  curSize;
    /** The number of bytes remaining in the region after the current chunk. */
    unsigned  sizeLeft;
    /** The start address so we can calculate offset in writing block. */
    const Addr startAddr;
    /** The maximum chunk size, e.g., the cache block size or page size. */
    const unsigned chunkSize;

  public:
    /**
     * Constructor.
     * @param _startAddr The starting address of the region.
     * @param totalSize The total size of the region.
     * @param _chunkSize The size/alignment of chunks into which
     *    the region should be decomposed.
     *
     * @ingroup api_chunk_generator
     */
    ChunkGenerator(Addr _startAddr, unsigned totalSize, unsigned _chunkSize)
        : startAddr(_startAddr), chunkSize(_chunkSize)
    {
        // chunkSize must be a power of two
        assert(chunkSize == 0 || isPowerOf2(chunkSize));

        // set up initial chunk.
        curAddr = startAddr;

        if (chunkSize == 0) //Special Case, if we see 0, assume no chuncking
        {
            nextAddr = startAddr + totalSize;
        }
        else
        {
            // nextAddr should be *next* chunk start
            nextAddr = roundUp(startAddr, chunkSize);
            if (curAddr == nextAddr) {
                // ... even if startAddr is already chunk-aligned
                nextAddr += chunkSize;
            }
        }

        // how many bytes are left between curAddr and the end of this chunk?
        unsigned left_in_chunk = nextAddr - curAddr;
        curSize = std::min(totalSize, left_in_chunk);
        sizeLeft = totalSize - curSize;
    }

    /**
     * Return starting address of current chunk.
     *
     * @ingroup api_chunk_generator
     */
    Addr addr() const { return curAddr; }
    /**
     * Return size in bytes of current chunk.
     *
     * @ingroup api_chunk_generator
     */
    unsigned size() const { return curSize; }

    /**
     * Number of bytes we have already chunked up.
     *
     * @ingroup api_chunk_generator
     */
    unsigned complete() const { return curAddr - startAddr; }

    /**
     * Are we done?  That is, did the last call to next() advance
     * past the end of the region?
     * @return True if yes, false if more to go.
     *
     * @ingroup api_chunk_generator
     */
    bool done() const { return (curSize == 0); }

    /**
     * Is this the last chunk?
     * @return True if yes, false if more to go.
     *
     * @ingroup api_chunk_generator
     */
    bool last() const { return (sizeLeft == 0); }

    /**
     * Advance generator to next chunk.
     * @return True if successful, false if unsuccessful
     * (because we were at the last chunk).
     *
     * @ingroup api_chunk_generator
     */
    bool
    next()
    {
        if (sizeLeft == 0) {
            curSize = 0;
            return false;
        }

        curAddr = nextAddr;
        curSize = std::min(sizeLeft, chunkSize);
        sizeLeft -= curSize;
        nextAddr += curSize;
        return true;
    }
};

#endif // __BASE__CHUNK_GENERATOR_HH__
