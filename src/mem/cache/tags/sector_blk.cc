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
 * Implementation of a simple sector block class. Each sector consists of a
 * sequence of cache blocks that may or may not be present in the cache.
 */

#include "mem/cache/tags/sector_blk.hh"

#include <cassert>

#include "base/cprintf.hh"
#include "base/logging.hh"

void
SectorSubBlk::setSectorBlock(SectorBlk* sector_blk)
{
    assert(sector_blk != nullptr);
    _sectorBlk = sector_blk;
}

const SectorBlk*
SectorSubBlk::getSectorBlock() const
{
    return _sectorBlk;
}

void
SectorSubBlk::setSectorOffset(const int sector_offset)
{
    _sectorOffset = sector_offset;
}

int
SectorSubBlk::getSectorOffset() const
{
    return _sectorOffset;
}

Addr
SectorSubBlk::getTag() const
{
    return _sectorBlk->getTag();
}

void
SectorSubBlk::setValid()
{
    CacheBlk::setValid();
    _sectorBlk->validateSubBlk();
}

void
SectorSubBlk::setSecure()
{
    CacheBlk::setSecure();
    _sectorBlk->setSecure();
}

void
SectorSubBlk::invalidate()
{
    CacheBlk::invalidate();
    _sectorBlk->invalidateSubBlk();
}

void
SectorSubBlk::insert(const Addr tag, const bool is_secure,
                     const int src_master_ID, const uint32_t task_ID)
{
    // Make sure it is not overwriting another sector
    panic_if((_sectorBlk && _sectorBlk->isValid()) &&
             ((_sectorBlk->getTag() != tag) ||
              (_sectorBlk->isSecure() != is_secure)),
              "Overwriting valid sector!");

    CacheBlk::insert(tag, is_secure, src_master_ID, task_ID);

    // Set sector tag
    _sectorBlk->setTag(tag);
}

std::string
SectorSubBlk::print() const
{
    return csprintf("%s sector offset: %#x", CacheBlk::print(),
                    getSectorOffset());
}

SectorBlk::SectorBlk()
    : ReplaceableEntry(), _tag(MaxAddr), _validCounter(0), _secureBit(false)
{
}

bool
SectorBlk::isValid() const
{
    // If any of the blocks in the sector is valid, so is the sector
    return _validCounter > 0;
}

bool
SectorBlk::isSecure() const
{
    // If any of the valid blocks in the sector is secure, so is the sector
    return _secureBit;
}

void
SectorBlk::setTag(const Addr tag)
{
    _tag = tag;
}

Addr
SectorBlk::getTag() const
{
    return _tag;
}

void
SectorBlk::validateSubBlk()
{
    _validCounter++;
}

void
SectorBlk::invalidateSubBlk()
{
    // If all sub-blocks have been invalidated, the sector becomes invalid,
    // so clear secure bit
    if (--_validCounter == 0) {
        _secureBit = false;
    }
}

void
SectorBlk::setSecure()
{
    _secureBit = true;
}
