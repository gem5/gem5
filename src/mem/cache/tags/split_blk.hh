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
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of partitioned tag store cache block class.
 */

#ifndef __SPLIT_BLK_HH__
#define __SPLIT_BLK_HH__

#include "mem/cache/cache_blk.hh" // base class

/**
 * Split cache block.
 */
class SplitBlk : public CacheBlk {
  public:
    /** Has this block been touched? Used to aid calculation of warmup time. */
    bool isTouched;
    /** Has this block been used after being brought in? (for LIFO partition) */
    bool isUsed;
    /** is this blk a NIC block? (i.e. requested by the NIC) */
    bool isNIC;
    /** timestamp of the arrival of this block into the cache */
    Tick ts;
    /** the previous block in the LIFO partition (brought in before than me) */
    SplitBlk *prev;
    /** the next block in the LIFO partition (brought in later than me) */
    SplitBlk *next;
    /** which partition this block is in */
    int part;

    SplitBlk()
        : isTouched(false), isUsed(false), isNIC(false), ts(0), prev(NULL), next(NULL),
          part(0)
    {}
};

#endif

