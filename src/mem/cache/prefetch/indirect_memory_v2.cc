/**
 * Copyright (c) 2018 Metempsy Technology Consulting
 * Copyright (c) 2026 The Regents of The Xi'an Jiaotong University
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

#include "mem/cache/prefetch/indirect_memory_v2.hh"

#include <cstring>

#include "base/logging.hh"
#include "debug/HWPrefetch.hh"
#include "params/IndirectMemoryPrefetcherV2.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

namespace gem5
{

namespace prefetch
{

IndirectMemoryV2::IndirectMemoryV2(const IndirectMemoryPrefetcherV2Params &p)
    : Queued(p),
      maxPrefetchDistance(p.max_prefetch_distance),
      shiftValues(p.shift_values),
      prefetchThreshold(p.prefetch_threshold),
      streamCounterThreshold(p.stream_counter_threshold),
      streamingDistance(p.streaming_distance),
      use_multi_way(p.use_multi_way),
      use_multi_level(p.use_multi_level),
      maxIndirectWay(p.max_indirect_way),
      maxIndirectLevel(p.max_indirect_level),
      maxPending(p.max_pending_indices),
      maxReadyIndices(p.max_ready_indices),
      prefetchTable(
          (name() + ".PrefetchTable").c_str(), p.pt_table_entries,
          p.pt_table_assoc, p.pt_table_replacement_policy,
          p.pt_table_indexing_policy,
          PrefetchTableEntry(p.num_indirect_counter_bits,
                             genTagExtractor(p.pt_table_indexing_policy))),
      ipd((name() + ".IPD").c_str(), p.ipd_table_entries, p.ipd_table_assoc,
          p.ipd_table_replacement_policy, p.ipd_table_indexing_policy,
          IndirectPatternDetectorEntry(
              p.addr_array_len, shiftValues.size(),
              genTagExtractor(p.ipd_table_indexing_policy))),
      byteOrder(p.sys->getGuestByteOrder())
{}

Addr
IndirectMemoryV2::getBaseAddress(Addr address, int64_t index, int shift)
{
    if (shift >= 0) {
        return address - (static_cast<Addr>(index) << shift);
    }
    return address - ((static_cast<Addr>(index) >> (-shift)));
}

Addr
IndirectMemoryV2::getIndirectAddress(Addr base_address, int64_t index,
                                     int shift)
{
    if (shift >= 0) {
        return base_address + (static_cast<Addr>(index) << shift);
    }
    return base_address + ((static_cast<Addr>(index) >> (-shift)));
}

bool
IndirectMemoryV2::isIndexStreamAccess(unsigned size, int64_t delta)
{
    if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
        return false;
    }
    // Sniper: only ±data_length steps count as the index stream.
    const int64_t sz = static_cast<int64_t>(size);
    return delta == sz || delta == -sz;
}

void
IndirectMemoryV2::calculatePrefetch(const PrefetchInfo &pfi,
                                    std::vector<AddrPriority> &addresses,
                                    const CacheAccessor &cache)
{
    const bool is_secure = pfi.isSecure();
    for (auto it = readyIndirectPrefetches.begin();
         it != readyIndirectPrefetches.end();) {
        if (it->secure == is_secure) {
            addresses.emplace_back(it->target, 0);
            it = readyIndirectPrefetches.erase(it);
        } else {
            ++it;
        }
    }

    // 1. preprogress
    if (!pfi.hasPC()) {
        return;
    }

    Addr pc = pfi.getPC();
    Addr addr = pfi.getAddr();
    bool miss = pfi.isCacheMiss();

    checkAccessMatchOnActiveEntries(pfi, cache);

    // 2. ipd track misses
    if (miss) {
        bool pattern_matched = false;
        for (auto &entry : ipd) {
            if (pattern_matched) {
                break;
            }
            if (!entry.isValid()) {
                continue;
            }
            assert(entry.numIndices >= 1);

            if (entry.numIndices == 1) {
                trackMissIndex1(entry, addr);
            } else {
                if (trackMissIndex2(entry, addr)) {
                    pattern_matched = true;
                }
            }
        }
    }

    unsigned data_size = pfi.getSize();
    if (pfi.isWrite() ||
        (data_size != sizeof(uint32_t) && data_size != sizeof(uint64_t))) {
        return;
    }

    int64_t index = 0;
    bool read_index = true;
    uint8_t index_bytes[sizeof(uint64_t)] = {};
    if (cache.tryRead(addr, is_secure, data_size, index_bytes)) {
        switch (data_size) {
            case sizeof(uint32_t): {
                uint32_t v;
                std::memcpy(&v, index_bytes, sizeof(v));
                index = (byteOrder == ByteOrder::little) ? letoh(v) : betoh(v);
                break;
            }
            case sizeof(uint64_t): {
                uint64_t v;
                std::memcpy(&v, index_bytes, sizeof(v));
                index = (byteOrder == ByteOrder::little) ? letoh(v) : betoh(v);
                break;
            }
            default:
                read_index = false;
        }
    } else {
        read_index = false;
    }

    if (read_index && index > (2ULL << 35)) {
        return;
    }

    // 3. stream detect
    const PrefetchTableEntry::KeyType key{pc, is_secure};
    PrefetchTableEntry *pt_entry = prefetchTable.findEntry(key);
    bool stream_matched = false;

    if (pt_entry != nullptr) {
        prefetchTable.accessEntry(pt_entry);
        if (pt_entry->dataSize != data_size) {
            pt_entry->address = addr;
            pt_entry->secure = is_secure;
            pt_entry->dataSize = data_size;
            pt_entry->streamCounter = 0;
            pt_entry->lastDelta = 0;
        } else if (addr == pt_entry->address) {
            // ignore duplicated address
            return;
        } else if (addr == pt_entry->address + data_size) {
            pt_entry->lastDelta = data_size;
            stream_matched = true;
        } else if (addr == pt_entry->address - data_size) {
            pt_entry->lastDelta = -static_cast<int64_t>(data_size);
            stream_matched = true;
        } else if (pt_entry->prefetchOn && pt_entry->lastDelta != 0) {
            pt_entry->currPrefetchDistance = 1;
            stream_matched = true;
        } else {
            pt_entry->address = addr;
            pt_entry->secure = is_secure;
            pt_entry->dataSize = data_size;
            pt_entry->streamCounter = 0;
            pt_entry->lastDelta = 0;
        }

        if (stream_matched) {
            pt_entry->address = addr;
            pt_entry->secure = is_secure;
            if (pt_entry->streamCounter < streamCounterThreshold) {
                pt_entry->streamCounter++;
            } else {
                pt_entry->streamCounter = streamCounterThreshold;
            }

            // // stream prefetch
            // if (pt_entry->streamCounter >=
            //     static_cast<unsigned>(streamCounterThreshold)) {
            //     for (unsigned int i = 1; i <= streamingDistance; ++i) {
            //         addresses.push_back(AddrPriority(
            //             addr + pt_entry->lastDelta *
            //             static_cast<int64_t>(i), 0));
            //     }
            // }
        }
    } else {
        pt_entry = prefetchTable.findVictim(key);
        assert(pt_entry != nullptr);
        prefetchTable.insertEntry(key, pt_entry);
        pt_entry->address = addr;
        pt_entry->secure = is_secure;
        pt_entry->dataSize = data_size;
        pt_entry->streamCounter = 0;
        pt_entry->lastDelta = 0;
    }

    // 4. IPD / indirect — only on stable ±size index stream.
    const bool index_stream =
        isIndexStreamAccess(data_size, pt_entry->lastDelta);
    if (!index_stream) {
        return;
    }

    if (pt_entry->streamCounter >= streamCounterThreshold &&
        !pt_entry->enabled && read_index) {
        const IndirectPatternDetectorEntry::KeyType ipd_key{(Addr)pt_entry,
                                                            false};
        IndirectPatternDetectorEntry *ipd_entry = findIPDEntry(pt_entry, 0);
        if (ipd_entry != nullptr) {
            ipd.accessEntry(ipd_entry);
            if (ipd_entry->idx1 != index) {
                if (ipd_entry->numIndices == 1) {
                    ipd_entry->idx2 = index;
                    ipd_entry->numIndices = 2;
                } else {
                    ipd.invalidate(ipd_entry);
                    pt_entry->currHit = 0;
                    if (pt_entry->numHitNextDetect == 0) {
                        pt_entry->numHitNextDetect = 1;
                    } else if (pt_entry->numHitNextDetect < 128) {
                        pt_entry->numHitNextDetect *= 2;
                    }
                }
            }
        } else if (pt_entry->currHit >= pt_entry->numHitNextDetect) {
            if (!allocateEmptyIPDEntry(ipd, ipd_entry)) {
                DPRINTF(HWPrefetch,
                        "IndirectMemoryV2: IPD table full for pc=%#x\n", pc);
            } else {
                ipd.insertEntry(ipd_key, ipd_entry);
                ipd_entry->detect_type = 0;
                ipd_entry->idx1 = index;
                ipd_entry->idx2 = 0;
                ipd_entry->numIndices = 1;
                ipd_entry->numMisses = 0;
                ipd_entry->pattern_entry = pt_entry;
                pt_entry->currHit = 0;
            }
        } else {
            pt_entry->currHit++;
        }
    } else if (pt_entry->enabled) {
        if (read_index) {
            insertIndex(*pt_entry, index);
        }
        int64_t index_delta = pt_entry->lastDelta;
        while (pt_entry != nullptr) {
            if (pt_entry->prefetchOn) {
                // next_level.
                if (read_index && pt_entry->next_level) {
                    processNextLevel(*pt_entry, index, is_secure, cache, true,
                                     nullptr);
                }

                unsigned d = pt_entry->currPrefetchDistance;
                if (d < 1) {
                    d = 1;
                }
                const unsigned num_pf = (d < maxPrefetchDistance) ? 2 : 1;

                for (unsigned k = 1; k <= num_pf; k += 1) {
                    const unsigned depth = k + d;
                    const Addr future_idx_addr =
                        addr + static_cast<Addr>(index_delta * depth);
                    uint8_t future_bytes[sizeof(uint64_t)] = {};
                    if (!cache.tryRead(future_idx_addr, is_secure, data_size,
                                       future_bytes)) {
                        addresses.emplace_back(future_idx_addr, 0);
                        enqueuePendingIndex(PendingIndex{
                            future_idx_addr, pc, pt_entry->baseAddr,
                            pt_entry->shift, is_secure, data_size, depth});
                        DPRINTF(
                            HWPrefetch,
                            "IndirectMemoryV2: defer index %#x at depth %u\n",
                            future_idx_addr, depth);
                        continue;
                    }
                    int64_t future_index = 0;
                    switch (data_size) {
                        case sizeof(uint32_t): {
                            uint32_t v;
                            std::memcpy(&v, future_bytes, sizeof(v));
                            future_index = (byteOrder == ByteOrder::little)
                                               ? letoh(v)
                                               : betoh(v);
                            break;
                        }
                        case sizeof(uint64_t): {
                            uint64_t v;
                            std::memcpy(&v, future_bytes, sizeof(v));
                            future_index = (byteOrder == ByteOrder::little)
                                               ? letoh(v)
                                               : betoh(v);
                            break;
                        }
                        default:
                            continue;
                    }
                    if (future_index > static_cast<int64_t>(2ULL << 35)) {
                        continue;
                    }
                    Addr pf_addr = getIndirectAddress(
                        pt_entry->baseAddr, future_index, pt_entry->shift);
                    addresses.push_back(AddrPriority(pf_addr, 0));
                    // Emit C from future B; do not insertIndex again.
                    if (pt_entry->next_level) {
                        processNextLevel(*pt_entry, future_index, is_secure,
                                         cache, false, &addresses);
                    }
                    DPRINTF(HWPrefetch,
                            "IndirectMemoryV2: prefetch %#x (depth=%u "
                            "index=%lld base=%#x shift=%d)\n",
                            pf_addr, depth, (long long)future_index,
                            pt_entry->baseAddr, pt_entry->shift);
                }

                if (pt_entry->currPrefetchDistance < maxPrefetchDistance) {
                    pt_entry->currPrefetchDistance += 1;
                }

                if (read_index && pt_entry->next_way == nullptr &&
                    use_multi_way && pt_entry->way_depth < maxIndirectWay) {
                    const IndirectPatternDetectorEntry::KeyType
                        next_way_ipd_key{(Addr)pt_entry, false};
                    IndirectPatternDetectorEntry *next_way_ipd_entry =
                        findIPDEntry(pt_entry, 1);
                    if (next_way_ipd_entry != nullptr) {
                        ipd.accessEntry(next_way_ipd_entry);
                        if (next_way_ipd_entry->idx1 != index) {
                            if (next_way_ipd_entry->numIndices == 1) {
                                next_way_ipd_entry->idx2 = index;
                                next_way_ipd_entry->numIndices = 2;
                            } else {
                                ipd.invalidate(next_way_ipd_entry);
                                pt_entry->currHit = 0;
                                if (pt_entry->numHitNextDetect == 0) {
                                    pt_entry->numHitNextDetect = 1;
                                } else if (pt_entry->numHitNextDetect < 128) {
                                    pt_entry->numHitNextDetect *= 2;
                                }
                            }
                        }
                    } else if (pt_entry->currHit >=
                               pt_entry->numHitNextDetect) {
                        if (!allocateEmptyIPDEntry(ipd, next_way_ipd_entry)) {
                            DPRINTF(HWPrefetch,
                                    "IndirectMemoryV2: IPD table full for "
                                    "next-way pc=%#x\n",
                                    pc);
                        } else {
                            ipd.insertEntry(next_way_ipd_key,
                                            next_way_ipd_entry);
                            next_way_ipd_entry->detect_type = 1;
                            next_way_ipd_entry->idx1 = index;
                            next_way_ipd_entry->idx2 = 0;
                            next_way_ipd_entry->numIndices = 1;
                            next_way_ipd_entry->numMisses = 0;
                            next_way_ipd_entry->pattern_entry = pt_entry;
                        }
                    }
                }
            }
            pt_entry = pt_entry->next_way;
        }
    }
}

void
IndirectMemoryV2::trackMissIndex1(IndirectPatternDetectorEntry &entry,
                                  Addr addr)
{
    if (entry.numMisses >= entry.baseAddr.size()) {
        return;
    }
    std::vector<Addr> &ba_array = entry.baseAddr[entry.numMisses];
    int idx = 0;
    for (int shift : shiftValues) {
        ba_array[idx] = getBaseAddress(addr, entry.idx1, shift);
        idx += 1;
    }
    entry.numMisses += 1;
}

bool
IndirectMemoryV2::trackMissIndex2(IndirectPatternDetectorEntry &entry,
                                  Addr addr)
{
    bool matched_this = false;
    for (int midx = 0; midx < entry.numMisses; midx += 1) {
        std::vector<Addr> &ba_array = entry.baseAddr[midx];
        int idx = 0;
        for (int shift : shiftValues) {
            if (ba_array[idx] == getBaseAddress(addr, entry.idx2, shift)) {
                PrefetchTableEntry *pt_entry = entry.pattern_entry;
                assert(pt_entry != nullptr);

                if (entry.detect_type == 0) {
                    establishIndirectPattern(pt_entry, shift, ba_array[idx]);
                    ipd.invalidate(&entry);
                    matched_this = true;
                    break;
                } else if (entry.detect_type == 1) {
                    if (ba_array[idx] == pt_entry->baseAddr &&
                        shift == pt_entry->shift) {
                        continue;
                    }
                    PrefetchTableEntry *next_way;
                    if (allocateEmptyPrefetchTableEntry(prefetchTable,
                                                        next_way)) {
                        if (next_way == pt_entry) {
                            continue;
                        }
                        const PrefetchTableEntry::KeyType child_key{
                            (Addr)next_way, pt_entry->isSecure()};
                        prefetchTable.insertEntry(child_key, next_way);
                        establishIndirectPattern(next_way, shift,
                                                 ba_array[idx]);
                        next_way->secure = pt_entry->secure;
                        next_way->dataSize = pt_entry->dataSize;
                        next_way->type = 1;
                        next_way->prev = pt_entry;
                        pt_entry->next_way = next_way;
                        next_way->way_depth = pt_entry->way_depth + 1;
                        next_way->level_depth = pt_entry->level_depth;
                        ipd.invalidate(&entry);
                        matched_this = true;
                        break;
                    }
                } else if (entry.detect_type == 2) {
                    PrefetchTableEntry *next_level;
                    if (allocateEmptyPrefetchTableEntry(prefetchTable,
                                                        next_level)) {
                        if (pt_entry == next_level) {
                            continue;
                        }
                        const PrefetchTableEntry::KeyType child_key{
                            (Addr)next_level, pt_entry->isSecure()};
                        prefetchTable.insertEntry(child_key, next_level);
                        establishIndirectPattern(next_level, shift,
                                                 ba_array[idx]);
                        next_level->secure = pt_entry->secure;
                        next_level->type = 2;
                        next_level->prev = pt_entry;
                        pt_entry->next_level = next_level;
                        next_level->level_depth = pt_entry->level_depth + 1;
                        next_level->way_depth = 1;
                        ipd.invalidate(&entry);
                        matched_this = true;
                        break;
                    }
                }
            }
            idx += 1;
        }
        if (matched_this) {
            return true;
        }
    }
    return false;
}

void
IndirectMemoryV2::establishIndirectPattern(PrefetchTableEntry *pt_entry,
                                           int shift, Addr base)
{
    pt_entry->baseAddr = base;
    pt_entry->shift = shift;
    pt_entry->enabled = true;
    pt_entry->prefetchOn = false;
    pt_entry->indirectCounter.reset();
    pt_entry->unmatchedIndices.clear();
    pt_entry->currPrefetchDistance = 1;
    pt_entry->numHitNextDetect = 1;
    pt_entry->currHit = 0;
    DPRINTF(HWPrefetch,
            "IndirectMemoryV2: pattern established base=%#x shift=%d\n", base,
            shift);
}

void
IndirectMemoryV2::notifyFill(const CacheAccessProbeArg &acc)
{
    const Addr filled_line = blockAddress(acc.pkt->getAddr());
    auto range = pendingIndices.equal_range(filled_line);

    for (auto it = range.first; it != range.second;) {
        const PendingIndex pending = it->second;
        auto current = it++;
        pendingIndices.erase(current);

        const PrefetchTableEntry::KeyType key{pending.streamPc,
                                              pending.secure};
        PrefetchTableEntry *entry = prefetchTable.findEntry(key);
        if (entry == nullptr || !entry->enabled || !entry->prefetchOn ||
            entry->baseAddr != pending.baseAddr ||
            entry->shift != pending.shift) {
            continue;
        }

        uint8_t index_bytes[sizeof(uint64_t)] = {};
        if (!acc.cache.tryRead(pending.indexaddr, pending.secure,
                               pending.dataSize, index_bytes)) {
            continue;
        }

        int64_t index = 0;
        if (pending.dataSize == sizeof(uint32_t)) {
            uint32_t value;
            std::memcpy(&value, index_bytes, sizeof(value));
            index =
                (byteOrder == ByteOrder::little) ? letoh(value) : betoh(value);
        } else if (pending.dataSize == sizeof(uint64_t)) {
            uint64_t value;
            std::memcpy(&value, index_bytes, sizeof(value));
            index =
                (byteOrder == ByteOrder::little) ? letoh(value) : betoh(value);
        } else {
            continue;
        }

        if (index > (2ULL << 35)) {
            continue;
        }

        const Addr target =
            getIndirectAddress(entry->baseAddr, index, entry->shift);
        readyIndirectPrefetches.push_back(
            ReadyIndirectPrefetch{target, pending.secure});
        if (entry->next_way && entry->next_way->prefetchOn) {
            const Addr next_way_addr = getIndirectAddress(
                entry->next_way->baseAddr, index, entry->next_way->shift);
            readyIndirectPrefetches.push_back(
                ReadyIndirectPrefetch{next_way_addr, pending.secure});
        }
        if (entry->next_level && entry->next_level->prefetchOn) {
            PrefetchTableEntry *nl = entry->next_level;
            unsigned nl_size = nl->dataSize;
            if (nl_size != sizeof(uint32_t) && nl_size != sizeof(uint64_t)) {
                nl_size = pending.dataSize;
            }
            int64_t aval = 0;
            if (tryReadIndexValue(acc.cache, target, pending.secure, nl_size,
                                  aval) &&
                aval <= static_cast<int64_t>(2ULL << 35)) {
                const Addr c_addr =
                    getIndirectAddress(nl->baseAddr, aval, nl->shift);
                readyIndirectPrefetches.push_back(
                    ReadyIndirectPrefetch{c_addr, pending.secure});
            }
        }
        DPRINTF(
            HWPrefetch,
            "IndirectMemoryV2: deferred prefetch %#x (depth=%u index=%lld)\n",
            target, pending.depth, (long long)index);
    }
}

bool
IndirectMemoryV2::tryReadIndexValue(const CacheAccessor &cache, Addr addr,
                                    bool is_secure, unsigned data_size,
                                    int64_t &index) const
{
    if (data_size != sizeof(uint32_t) && data_size != sizeof(uint64_t)) {
        return false;
    }
    uint8_t bytes[sizeof(uint64_t)] = {};
    if (!cache.tryRead(addr, is_secure, data_size, bytes)) {
        return false;
    }
    if (data_size == sizeof(uint32_t)) {
        uint32_t v;
        std::memcpy(&v, bytes, sizeof(v));
        index = (byteOrder == ByteOrder::little) ? letoh(v) : betoh(v);
    } else {
        uint64_t v;
        std::memcpy(&v, bytes, sizeof(v));
        index = (byteOrder == ByteOrder::little) ? letoh(v) : betoh(v);
    }
    return true;
}

bool
IndirectMemoryV2::processNextLevel(PrefetchTableEntry &pt_entry,
                                   int64_t b_index, bool is_secure,
                                   const CacheAccessor &cache, bool do_insert,
                                   std::vector<AddrPriority> *addresses)
{
    PrefetchTableEntry *nl = pt_entry.next_level;
    if (nl == nullptr || !nl->enabled) {
        return false;
    }

    const Addr a_addr =
        getIndirectAddress(pt_entry.baseAddr, b_index, pt_entry.shift);
    unsigned nl_size = nl->dataSize;
    if (nl_size != sizeof(uint32_t) && nl_size != sizeof(uint64_t)) {
        nl_size = (pt_entry.dataSize == sizeof(uint32_t) ||
                   pt_entry.dataSize == sizeof(uint64_t))
                      ? pt_entry.dataSize
                      : sizeof(uint32_t);
    }

    int64_t aval = 0;
    if (!tryReadIndexValue(cache, a_addr, is_secure, nl_size, aval) ||
        aval == 0 || aval > static_cast<int64_t>(2ULL << 35)) {
        return false;
    }

    if (do_insert) {
        insertIndex(*nl, aval);
    }

    if (addresses != nullptr && nl->prefetchOn) {
        const Addr c_addr = getIndirectAddress(nl->baseAddr, aval, nl->shift);
        addresses->emplace_back(c_addr, 0);
        DPRINTF(HWPrefetch,
                "IndirectMemoryV2: next-level prefetch %#x (index=%lld)\n",
                c_addr, (long long)aval);
        return true;
    }
    return false;
}

// 管理 index 队列
void
IndirectMemoryV2::insertIndex(PrefetchTableEntry &entry, int64_t index)
{
    std::pair<int64_t, bool> new_index{index, false};
    entry.unmatchedIndices.push_back(new_index);
    bool overflowed = false;
    if (entry.unmatchedIndices.size() >
        PrefetchTableEntry::MaxUnmatchedIndices) {
        overflowed = true;
        const auto evicted = entry.unmatchedIndices.front();
        if (!evicted.second && entry.indirectCounter > 0) {
            entry.indirectCounter--;
        }
        entry.unmatchedIndices.pop_front();
    }

    if (overflowed && entry.prefetchOn &&
        entry.indirectCounter <= prefetchThreshold) {
        DPRINTF(HWPrefetch,
                "IndirectMemoryV2: prefetch disabled after unmatched overflow "
                "(hits=%u)\n",
                (unsigned)entry.indirectCounter);
        entry.prefetchOn = false;
        entry.currPrefetchDistance = 1;
    }

    if (entry.next_way != nullptr) {
        insertIndex(*entry.next_way, index);
    }
}

bool
IndirectMemoryV2::isIndirectHit(PrefetchTableEntry &entry, Addr addr)
{
    for (size_t i = 0; i < entry.unmatchedIndices.size(); ++i) {
        Addr curr_addr = addr;
        if (curr_addr == getIndirectAddress(entry.baseAddr,
                                            entry.unmatchedIndices[i].first,
                                            entry.shift)) {
            for (size_t skipped = 0; skipped < i; ++skipped) {
                if (!entry.unmatchedIndices[skipped].second &&
                    entry.indirectCounter > 0) {
                    entry.indirectCounter--;
                }
            }

            if (!entry.unmatchedIndices[i].second) {
                entry.indirectCounter++;
                entry.unmatchedIndices[i].second = true;
            }

            entry.unmatchedIndices.erase(entry.unmatchedIndices.begin(),
                                         entry.unmatchedIndices.begin() + i +
                                             1);

            const bool should_prefetch =
                entry.indirectCounter > prefetchThreshold;
            if (should_prefetch && !entry.prefetchOn) {
                entry.currPrefetchDistance = 1;
                DPRINTF(HWPrefetch,
                        "IndirectMemoryV2: prefetch enabled (hits=%u)\n",
                        (unsigned)entry.indirectCounter);
            } else if (!should_prefetch && entry.prefetchOn) {
                entry.currPrefetchDistance = 1;
                DPRINTF(HWPrefetch,
                        "IndirectMemoryV2: prefetch disabled (hits=%u)\n",
                        (unsigned)entry.indirectCounter);
            }
            entry.prefetchOn = should_prefetch;
            return true;
        }
    }
    return false;
}

bool
IndirectMemoryV2::enqueuePendingIndex(const PendingIndex &pending)
{
    const Addr pending_line = blockAddress(pending.indexaddr);
    auto range = pendingIndices.equal_range(pending_line);
    for (auto it = range.first; it != range.second; ++it) {
        const PendingIndex &existing = it->second;
        if (existing.indexaddr == pending.indexaddr &&
            existing.streamPc == pending.streamPc &&
            existing.baseAddr == pending.baseAddr &&
            existing.shift == pending.shift &&
            existing.secure == pending.secure) {
            return false;
        }
    }

    if (pendingIndices.size() >= static_cast<size_t>(maxPending)) {
        DPRINTF(HWPrefetch, "IndirectMemoryV2: pending queue full (%zu/%d)\n",
                pendingIndices.size(), maxPending);
        return false;
    }

    pendingIndices.emplace(pending_line, pending);
    return true;
}

bool
IndirectMemoryV2::enqueueReadyIndirectPrefetch(Addr target, bool secure)
{
    if (readyIndirectPrefetches.size() >=
        static_cast<size_t>(maxReadyIndices)) {
        DPRINTF(
            HWPrefetch,
            "IndirectMemoryV2: drop prefetch %#x; ready queue full (%zu/%d)\n",
            target, readyIndirectPrefetches.size(), maxReadyIndices);
        return false;
    }

    readyIndirectPrefetches.push_back(ReadyIndirectPrefetch{target, secure});
    return true;
}

void
IndirectMemoryV2::checkAccessMatchOnActiveEntries(const PrefetchInfo &pfi,
                                                  const CacheAccessor &cache)
{
    const Addr addr = pfi.getAddr();
    const bool is_secure = pfi.isSecure();
    for (auto &pt_entry : prefetchTable) {
        if (!pt_entry.enabled || !pt_entry.isValid()) {
            continue;
        }
        if (!isIndirectHit(pt_entry, addr)) {
            continue;
        }

        if (!use_multi_level || pt_entry.next_level != nullptr ||
            pt_entry.level_depth >= maxIndirectLevel) {
            continue;
        }
        unsigned a_size = pfi.getSize();
        if (a_size != sizeof(uint32_t) && a_size != sizeof(uint64_t)) {
            a_size = pt_entry.dataSize;
        }
        int64_t a_index = 0;
        if (!tryReadIndexValue(cache, addr, is_secure, a_size, a_index) ||
            a_index > static_cast<int64_t>(2ULL << 35)) {
            continue;
        }
        const IndirectPatternDetectorEntry::KeyType next_level_ipd_key{
            (Addr)&pt_entry, false};
        IndirectPatternDetectorEntry *next_level_ipd_entry =
            findIPDEntry(&pt_entry, 2);
        if (next_level_ipd_entry != nullptr) {
            ipd.accessEntry(next_level_ipd_entry);
            if (next_level_ipd_entry->idx1 != a_index) {
                if (next_level_ipd_entry->numIndices == 1) {
                    next_level_ipd_entry->idx2 = a_index;
                    next_level_ipd_entry->numIndices = 2;
                } else {
                    ipd.invalidate(next_level_ipd_entry);
                    pt_entry.currHit = 0;
                    if (pt_entry.numHitNextDetect == 0) {
                        pt_entry.numHitNextDetect = 1;
                    } else if (pt_entry.numHitNextDetect < 128) {
                        pt_entry.numHitNextDetect *= 2;
                    }
                }
            }
        } else if (pt_entry.currHit >= pt_entry.numHitNextDetect) {
            if (!allocateEmptyIPDEntry(ipd, next_level_ipd_entry)) {
                DPRINTF(HWPrefetch,
                        "IndirectMemoryV2: IPD table full for next-level\n");
            } else {
                ipd.insertEntry(next_level_ipd_key, next_level_ipd_entry);
                next_level_ipd_entry->detect_type = 2;
                next_level_ipd_entry->idx1 = a_index;
                next_level_ipd_entry->idx2 = 0;
                next_level_ipd_entry->numIndices = 1;
                next_level_ipd_entry->numMisses = 0;
                next_level_ipd_entry->pattern_entry = &pt_entry;
                pt_entry.currHit = 0;
            }
        } else {
            pt_entry.currHit++;
        }
    }
}

} // namespace prefetch
} // namespace gem5
