/**
 * Copyright (c) 2018, 2020 Inria
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

/** @file
 * Implementation of a simple sector block class. Each sector consists of a
 * sequence of cache blocks that may or may not be present in the cache.
 */

#include "mem/cache/tags/sector_blk.hh"

#include <cassert>

#include "base/cprintf.hh"
#include "base/logging.hh"

namespace gem5
{

void
SectorSubBlk::setSectorBlock(SectorBlk* sector_blk)
{
    assert(sector_blk != nullptr);
    _sectorBlk = sector_blk;
}

SectorBlk*
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
    // If the sub-block is valid its tag must match its sector's
    const Addr tag = _sectorBlk->getTag();
    assert(!isValid() || (CacheBlk::getTag() == tag));
    return tag;
}

void
SectorSubBlk::setValid()
{
    CacheBlk::setValid();
    _sectorBlk->validateSubBlk();
}

void
SectorSubBlk::insert(const Addr tag, const bool is_secure)
{
    // Make sure it is not overwriting another sector
    panic_if(_sectorBlk && _sectorBlk->isValid() &&
        !_sectorBlk->matchTag(tag, is_secure), "Overwriting valid sector!");

    // If the sector is not valid, insert the new tag. The sector block
    // handles its own tag's invalidation, so do not attempt to insert MaxAddr.
    if ((_sectorBlk && !_sectorBlk->isValid()) && (tag != MaxAddr)) {
        _sectorBlk->insert(tag, is_secure);
    }
    CacheBlk::insert(tag, is_secure);
}

void
SectorSubBlk::invalidate()
{
    CacheBlk::invalidate();
    _sectorBlk->invalidateSubBlk();
}

std::string
SectorSubBlk::print() const
{
    return csprintf("%s sector offset: %#x", CacheBlk::print(),
                    getSectorOffset());
}

SectorBlk::SectorBlk()
    : TaggedEntry(), _validCounter(0)
{
}

bool
SectorBlk::isValid() const
{
    // If any of the blocks in the sector is valid, so is the sector
    return _validCounter > 0;
}

uint8_t
SectorBlk::getNumValid() const
{
    return _validCounter;
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
        invalidate();
    }
}

void
SectorBlk::setPosition(const uint32_t set, const uint32_t way)
{
    ReplaceableEntry::setPosition(set, way);
    for (auto& blk : blks) {
        blk->setPosition(set, way);
    }
}

std::string
SectorBlk::print() const
{
    std::string sub_blk_print;
    for (const auto& sub_blk : blks) {
        if (sub_blk->isValid()) {
            sub_blk_print += "\t[" + sub_blk->print() + "]\n";
        }
    }
    return csprintf("%s valid sub-blks (%d):\n%s",
        TaggedEntry::print(), getNumValid(), sub_blk_print);
}

} // namespace gem5
