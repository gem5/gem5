/**
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho
 */

/** @file
 * Definition of a simple superblock class. Each superblock consists of a
 * number of compressed cache blocks limited by the maximum compression
 * factor that may or may not be present in the cache.
 */

#ifndef __MEM_CACHE_TAGS_SUPER_BLK_HH__
#define __MEM_CACHE_TAGS_SUPER_BLK_HH__

#include "mem/cache/tags/sector_blk.hh"

class SuperBlk;

/**
 * A superblock is composed of sub-blocks, and each sub-block has information
 * regarding its superblock and a pointer to its superblock tag. A superblock
 * can be seen as a variation of a sector block, and therefore we use a sector
 * nomenclature.
 */
class CompressionBlk : public SectorSubBlk
{
  public:
    CompressionBlk();
    CompressionBlk(const CompressionBlk&) = delete;
    CompressionBlk& operator=(const CompressionBlk&) = delete;
    ~CompressionBlk() {};
};

/**
 * A basic compression superblock.
 * Contains the tag and a list of blocks associated to this superblock.
 */
class SuperBlk : public SectorBlk
{
  public:
    SuperBlk() : SectorBlk() {}
    SuperBlk(const SuperBlk&) = delete;
    SuperBlk& operator=(const SuperBlk&) = delete;
    ~SuperBlk() {};
};

#endif //__MEM_CACHE_TAGS_SUPER_BLK_HH__
