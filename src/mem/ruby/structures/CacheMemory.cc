/*
 * Copyright (c) 2020-2021, 2026 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "mem/ruby/structures/CacheMemory.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HtmMem.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubyResourceStalls.hh"
#include "debug/RubyStats.hh"
#include "mem/cache/replacement_policies/weighted_lru_rp.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

std::ostream&
operator<<(std::ostream& out, const CacheMemory& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

CacheMemory::RubyCacheSlot *
CacheMemory::RubyCache::selectVictim(
    const std::vector<RubyCacheSlot *> &candidates) const
{
    ReplacementCandidates replacement_candidates(candidates.begin(),
                                                 candidates.end());
    return static_cast<RubyCacheSlot *>(
        replPolicy->getVictim(replacement_candidates));
}

void
CacheMemory::RubyCache::weightedTouch(RubyCacheSlot *entry,
                                      const int occupancy) const
{
    static_cast<replacement_policy::WeightedLRU *>(replPolicy)
        ->touch(entry->replacementData, occupancy);
}

CacheMemory::CacheMemory(const Params &p)
    : SimObject(p),
      dataArray(p.dataArrayBanks, p.dataAccessLatency, p.start_index_bit),
      tagArray(p.tagArrayBanks, p.tagAccessLatency, p.start_index_bit),
      rubyCache(name() + ".cache"),
      atomicALUArray(p.atomicALUs, p.atomicLatency),
      cacheMemoryStats(this)
{
    m_cache_size = p.size;
    m_cache_assoc = p.assoc;
    m_start_index_bit = p.start_index_bit;
    m_is_instruction_only_cache = p.is_icache;
    m_resource_stalls = p.resourceStalls;
    m_block_size = p.block_size;  // may be 0 at this point. Updated in init()
    fatal_if(!p.indexing_policy, "%s requires an indexing policy", name());
    m_use_occupancy =
        dynamic_cast<replacement_policy::WeightedLRU *>(p.replacement_policy)
            ? true
            : false;
}

void
CacheMemory::setRubySystem(RubySystem* rs)
{
    dataArray.setClockPeriod(rs->clockPeriod());
    tagArray.setClockPeriod(rs->clockPeriod());
    atomicALUArray.setClockPeriod(rs->clockPeriod());
    atomicALUArray.setBlockSize(rs->getBlockSizeBytes());

    if (m_block_size == 0) {
        m_block_size = rs->getBlockSizeBytes();
    }

    m_ruby_system = rs;
}

void
CacheMemory::init()
{
    assert(m_block_size != 0);
    m_cache_num_sets = (m_cache_size / m_cache_assoc) / m_block_size;
    assert(m_cache_num_sets > 1);
    m_cache_num_set_bits = floorLog2(m_cache_num_sets);

    if (m_cache_num_sets * m_cache_assoc * m_block_size != m_cache_size) {
        fatal("Cache size %s is not a multiple of associativity %d * "
              "block size %d\n",
              m_cache_size, m_cache_assoc, m_block_size);
    }

    if ((1 << m_cache_num_set_bits) == m_cache_num_sets) {
        m_is_power_of_2_sets = true;
        assert(m_cache_num_set_bits > 0);
    } else {
        m_is_power_of_2_sets = false;
    }

    const size_t num_entries = m_cache_size / m_block_size;
    const auto &p = params();
    rubyCache.init(num_entries, m_cache_assoc, p.replacement_policy,
                   p.indexing_policy,
                   RubyCacheSlot(genTagExtractor(p.indexing_policy)));
}

CacheMemory::~CacheMemory()
{
    flushEntries();
}

// convert an Address to its location in the cache
int64_t
CacheMemory::addressToCacheSet(Addr address) const
{
    // TODO: this is exclusively used for contention modelling now,
    //  this methodology doesnt hold for differing indexing policies
    //  e.g. skewed associative caches.
    assert(address == makeLineAddress(address));
    if (m_is_power_of_2_sets) {
        return bitSelect(address, m_start_index_bit,
                         m_start_index_bit + m_cache_num_set_bits - 1);
    } else {
        return (address >> m_start_index_bit) % m_cache_num_sets;
    }
}

// Given an unique cache block identifier (idx): return the valid address
// stored by the cache block.  If the block is invalid/notpresent, the
// function returns the 0 address
Addr
CacheMemory::getAddressAtIdx(int idx) const
{
    assert(idx >= 0);
    assert(idx < getNumBlocks());

    const auto &slot = *std::next(rubyCache.begin(), idx);

    if (!slot.isValid() ||
        slot.entry->m_Permission == AccessPermission_Invalid) {
        return 0;
    }

    return slot.entry->m_Address;
}

bool
CacheMemory::tryCacheAccess(Addr address, RubyRequestType type,
                            DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "trying to access address: %#x\n", address);
    AbstractCacheEntry *entry = lookup(address);

    if (entry != nullptr) {
        rubyCache.accessEntry(address);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        if (entry->m_Permission == AccessPermission_Read_Write) {
            DPRINTF(RubyCache, "Have permission to access address: %#x\n",
                    address);
            return true;
        }
        if ((entry->m_Permission == AccessPermission_Read_Only) &&
            (type == RubyRequestType_LD || type == RubyRequestType_IFETCH)) {
            DPRINTF(RubyCache, "Have permission to access address: %#x\n",
                    address);
            return true;
        }
        // The line must not be accessible
    }
    DPRINTF(RubyCache, "Do not have permission to access address: %#x\n",
                address);
    data_ptr = nullptr;
    return false;
}

bool
CacheMemory::testCacheAccess(Addr address, RubyRequestType type,
                             DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "testing address: %#x\n", address);
    AbstractCacheEntry *entry = lookup(address);

    if (entry != nullptr) {
        rubyCache.accessEntry(address);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        DPRINTF(RubyCache, "have permission for address %#x\n", address);
        return true;
    }

    DPRINTF(RubyCache, "do not have permission for address %#x\n",
                address);
    data_ptr = nullptr;
    return false;
}

// tests to see if an address is present in the cache
bool
CacheMemory::isTagPresent(Addr address) const
{
    const AbstractCacheEntry* const entry = lookup(address);
    if (entry == nullptr) {
        // We didn't find the tag
        DPRINTF(RubyCache, "No tag match for address: %#x\n", address);
        return false;
    }
    DPRINTF(RubyCache, "address: %#x found\n", address);
    return true;
}

// Returns true if there is:
//   a) a tag match on this address or there is
//   b) an unused line in the same cache "way"
bool
CacheMemory::cacheAvail(Addr address) const
{
    assert(address == makeLineAddress(address));

    auto entries = rubyCache.getPossibleEntries(address);

    for (auto *ruby_entry : entries) {
        if (!ruby_entry->isValid() || ruby_entry->match(address)) {
            // We found an empty entry
            return true;
        }
    }
    return false;
}

AbstractCacheEntry*
CacheMemory::allocate(Addr address, AbstractCacheEntry *entry)
{
    assert(address == makeLineAddress(address));
    assert(!isTagPresent(address));
    assert(cacheAvail(address));
    DPRINTF(RubyCache, "allocating address: %#x\n", address);

    entry->initBlockSize(m_block_size);
    entry->setRubySystem(m_ruby_system);

    // Find the first open slot
    auto entries = rubyCache.getPossibleEntries(address);

    for (auto *ruby_entry : entries) {
        if (ruby_entry->isValid()) {
            continue;
        }

        assert(ruby_entry->entry == nullptr);

        ruby_entry->entry = entry; // Init entry
        entry->m_Address = address;
        entry->m_Permission = AccessPermission_Invalid;
        DPRINTF(RubyCache, "Allocate clearing lock for addr: 0x%x\n", address);
        entry->m_locked = -1;

        entry->setLastAccess(curTick());
        entry->setPosition(ruby_entry->getSet(), ruby_entry->getWay());

        rubyCache.insertEntry(address, ruby_entry);
        return entry;
    }
    panic("Allocate didn't find an available entry");
}

void
CacheMemory::deallocate(Addr address)
{
    DPRINTF(RubyCache, "deallocating address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    deallocate(entry);
}

void
CacheMemory::deallocate(AbstractCacheEntry *entry)
{
    assert(entry != nullptr);

    RubyCacheSlot *slot = lookupSlot(entry->m_Address);

    assert(slot != nullptr);
    assert(slot->entry == entry);

    /*
     * Clears the slot’s tag and entry pointer and invalidates its
     * replacement-policy metadata.
     */
    rubyCache.invalidate(slot);

    delete entry;
}

void
CacheMemory::flushEntries()
{
    for (auto &slot : rubyCache) {
        if (slot.entry != nullptr) {
            AbstractCacheEntry *entry = slot.entry;
            rubyCache.invalidate(&slot);
            delete entry;
        }
    }
}

// Returns with the physical address of the conflicting cache line
Addr
CacheMemory::cacheProbe(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvail(address));

    const auto candidates = rubyCache.getPossibleEntries(address);

    const auto *victim = rubyCache.selectVictim(candidates);

    assert(victim != nullptr);
    assert(victim->entry != nullptr);

    return victim->entry->m_Address;
}

CacheMemory::RubyCacheSlot *
CacheMemory::lookupSlot(Addr address)
{
    assert(address == makeLineAddress(address));
    return rubyCache.findEntry(address);
}

const CacheMemory::RubyCacheSlot *
CacheMemory::lookupSlot(Addr address) const
{
    assert(address == makeLineAddress(address));
    return rubyCache.findEntry(address);
}

// looks an address up in the cache
AbstractCacheEntry*
CacheMemory::lookup(Addr address)
{
    RubyCacheSlot *slot = lookupSlot(address);

    if (slot == nullptr) {
        return nullptr;
    }

    return slot->entry;
}

// looks an address up in the cache
const AbstractCacheEntry*
CacheMemory::lookup(Addr address) const
{
    const RubyCacheSlot *slot = lookupSlot(address);

    if (slot == nullptr) {
        return nullptr;
    }

    return slot->entry;
}

// Sets the most recently used bit for a cache block
void
CacheMemory::setMRU(Addr address)
{
    RubyCacheSlot *ruby_entry = lookupSlot(makeLineAddress(address));
    if (ruby_entry != nullptr) {
        assert(ruby_entry->isValid());
        rubyCache.accessEntry(ruby_entry);
        ruby_entry->entry->setLastAccess(curTick());
    }
}

void
CacheMemory::setMRU(AbstractCacheEntry *entry)
{
    assert(entry != nullptr);

    RubyCacheSlot *ruby_entry = lookupSlot(entry->m_Address);

    rubyCache.accessEntry(ruby_entry);
    entry->setLastAccess(curTick());
}

void
CacheMemory::setMRU(Addr address, int occupancy)
{
    RubyCacheSlot *ruby_entry = lookupSlot(makeLineAddress(address));
    if (ruby_entry == nullptr) {
        return;
    }

    if (m_use_occupancy) {
        // If we need to support weights
        rubyCache.weightedTouch(ruby_entry, occupancy);
    } else {
        rubyCache.accessEntry(ruby_entry);
    }

    ruby_entry->entry->setLastAccess(curTick());
}

void
CacheMemory::recordCacheContents(int cntrl, CacheRecorder* tr) const
{
    uint64_t warmedUpBlocks = 0;
    [[maybe_unused]] uint64_t totalBlocks = (uint64_t)m_cache_num_sets *
                                         (uint64_t)m_cache_assoc;

    for (const auto &slot : rubyCache) {
        if (!slot.isValid()) {
            continue;
        }

        AbstractCacheEntry *entry = slot.entry;

        RubyRequestType request_type = RubyRequestType_NULL;
        if (entry->m_Permission == AccessPermission_Read_Only) {
            if (m_is_instruction_only_cache) {
                request_type = RubyRequestType_IFETCH;
            } else {
                request_type = RubyRequestType_LD;
            }
        } else if (entry->m_Permission == AccessPermission_Read_Write) {
            request_type = RubyRequestType_ST;
        }

        if (request_type != RubyRequestType_NULL) {
            Tick lastAccessTick;
            lastAccessTick = entry->getLastAccess();
            Addr address = entry->m_Address;

            tr->addRecord(cntrl, address, 0, request_type, lastAccessTick,
                          entry->getDataBlk());
            warmedUpBlocks++;
        }
    }

    DPRINTF(RubyCacheTrace, "%s: %lli blocks of %lli total blocks"
            "recorded %.2f%% \n", name().c_str(), warmedUpBlocks,
            totalBlocks, (float(warmedUpBlocks) / float(totalBlocks)) * 100.0);
}

void
CacheMemory::print(std::ostream& out) const
{
    out << "Cache dump: " << name() << std::endl;

    for (const auto &slot : rubyCache) {
        if (slot.isValid()) {
            out << "  Index: " << slot.getSet() << " way: " << slot.getWay()
                << " entry: " << *slot.entry << std::endl;
        } else {
            out << "  Index: " << slot.getSet() << " way: " << slot.getWay()
                << " entry: NULL" << std::endl;
        }
    }
}

void
CacheMemory::printData(std::ostream& out) const
{
    out << "printData() not supported" << std::endl;
}

void
CacheMemory::setLocked(Addr address, int context)
{
    DPRINTF(RubyCache, "Setting Lock for addr: %#x to %d\n", address, context);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->setLocked(context);
}

void
CacheMemory::clearLocked(Addr address)
{
    DPRINTF(RubyCache, "Clear Lock for addr: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->clearLocked();
}

void
CacheMemory::clearLockedAll(int context)
{
    for (const auto &slot : rubyCache) {
        if (slot.isValid()) {
            if (slot.entry->isLocked(context)) {
                DPRINTF(RubyCache, "Clear Lock for addr: %#x\n",
                        slot.entry->m_Address);
                slot.entry->clearLocked();
            }
        }
    }
}

bool
CacheMemory::isLocked(Addr address, int context)
{
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    DPRINTF(RubyCache, "Testing Lock for addr: %#llx cur %d con %d\n",
            address, entry->m_locked, context);
    return entry->isLocked(context);
}

CacheMemory::
CacheMemoryStats::CacheMemoryStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numDataArrayReads, "Number of data array reads"),
      ADD_STAT(numDataArrayWrites, "Number of data array writes"),
      ADD_STAT(numTagArrayReads, "Number of tag array reads"),
      ADD_STAT(numTagArrayWrites, "Number of tag array writes"),
      ADD_STAT(numTagArrayStalls, "Number of stalls caused by tag array"),
      ADD_STAT(numDataArrayStalls, "Number of stalls caused by data array"),
      ADD_STAT(numAtomicALUOperations, "Number of atomic ALU operations"),
      ADD_STAT(numAtomicALUArrayStalls, "Number of stalls caused by atomic ALU array"),
      ADD_STAT(htmTransCommitReadSet, "Read set size of a committed "
                                      "transaction"),
      ADD_STAT(htmTransCommitWriteSet, "Write set size of a committed "
                                       "transaction"),
      ADD_STAT(htmTransAbortReadSet, "Read set size of a aborted transaction"),
      ADD_STAT(htmTransAbortWriteSet, "Write set size of a aborted "
                                      "transaction"),
      ADD_STAT(m_demand_hits, "Number of cache demand hits"),
      ADD_STAT(m_demand_misses, "Number of cache demand misses"),
      ADD_STAT(m_demand_accesses, "Number of cache demand accesses",
               m_demand_hits + m_demand_misses),
      ADD_STAT(m_prefetch_hits, "Number of cache prefetch hits"),
      ADD_STAT(m_prefetch_misses, "Number of cache prefetch misses"),
      ADD_STAT(m_prefetch_accesses, "Number of cache prefetch accesses",
               m_prefetch_hits + m_prefetch_misses),
      ADD_STAT(m_accessModeType, "")
{
    numDataArrayReads
        .flags(statistics::nozero);

    numDataArrayWrites
        .flags(statistics::nozero);

    numTagArrayReads
        .flags(statistics::nozero);

    numTagArrayWrites
        .flags(statistics::nozero);

    numTagArrayStalls
        .flags(statistics::nozero);

    numDataArrayStalls
        .flags(statistics::nozero);

    numAtomicALUOperations
        .flags(statistics::nozero);

    numAtomicALUArrayStalls
        .flags(statistics::nozero);

    htmTransCommitReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransCommitWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    m_prefetch_hits
        .flags(statistics::nozero);

    m_prefetch_misses
        .flags(statistics::nozero);

    m_prefetch_accesses
        .flags(statistics::nozero);

    m_accessModeType
        .init(RubyRequestType_NUM)
        .flags(statistics::pdf | statistics::total);

    for (int i = 0; i < RubyAccessMode_NUM; i++) {
        m_accessModeType
            .subname(i, RubyAccessMode_to_string(RubyAccessMode(i)))
            .flags(statistics::nozero)
            ;
    }
}

// assumption: SLICC generated files will only call this function
// once **all** resources are granted
void
CacheMemory::recordRequestType(CacheRequestType requestType, Addr addr)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            CacheRequestType_to_string(requestType));
    switch(requestType) {
    case CacheRequestType_DataArrayRead:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayReads++;
        return;
    case CacheRequestType_DataArrayWrite:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayWrites++;
        return;
    case CacheRequestType_TagArrayRead:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayReads++;
        return;
    case CacheRequestType_TagArrayWrite:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayWrites++;
        return;
    case CacheRequestType_AtomicALUOperation:
        if (m_resource_stalls)
            atomicALUArray.reserve(addr);
        cacheMemoryStats.numAtomicALUOperations++;
        return;
    default:
        warn("CacheMemory access_type not found: %s",
             CacheRequestType_to_string(requestType));
    }
}

bool
CacheMemory::checkResourceAvailable(CacheResourceType res, Addr addr)
{
    if (!m_resource_stalls) {
        return true;
    }

    if (res == CacheResourceType_TagArray) {
        if (tagArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Tag array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numTagArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_DataArray) {
        if (dataArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Data array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numDataArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_AtomicALUArray) {
        if (atomicALUArray.tryAccess(addr)) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Atomic ALU array stall on addr %#x in line address %#x\n",
                    addr, makeLineAddress(addr));
            cacheMemoryStats.numAtomicALUArrayStalls++;
            return false;
        }
    } else {
        panic("Unrecognized cache resource type.");
    }
}

/* hardware transactional memory */

void
CacheMemory::htmAbortTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    for (const auto &slot : rubyCache) {
        if (slot.isValid()) {
            htmReadSetSize += (slot.entry->getInHtmReadSet() ? 1 : 0);
            htmWriteSetSize += (slot.entry->getInHtmWriteSet() ? 1 : 0);
            if (slot.entry->getInHtmWriteSet()) {
                slot.entry->invalidateEntry();
            }
            slot.entry->setInHtmWriteSet(false);
            slot.entry->setInHtmReadSet(false);
            slot.entry->clearLocked();
        }
    }

    cacheMemoryStats.htmTransAbortReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransAbortWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmAbortTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::htmCommitTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    for (const auto &slot : rubyCache) {
        if (slot.isValid()) {
            htmReadSetSize += (slot.entry->getInHtmReadSet() ? 1 : 0);
            htmWriteSetSize += (slot.entry->getInHtmWriteSet() ? 1 : 0);
            slot.entry->setInHtmWriteSet(false);
            slot.entry->setInHtmReadSet(false);
            slot.entry->clearLocked();
        }
    }

    cacheMemoryStats.htmTransCommitReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransCommitWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmCommitTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::profileDemandHit()
{
    cacheMemoryStats.m_demand_hits++;
}

void
CacheMemory::profileDemandMiss()
{
    cacheMemoryStats.m_demand_misses++;
}

void
CacheMemory::profilePrefetchHit()
{
    cacheMemoryStats.m_prefetch_hits++;
}

void
CacheMemory::profilePrefetchMiss()
{
    cacheMemoryStats.m_prefetch_misses++;
}

} // namespace ruby
} // namespace gem5
