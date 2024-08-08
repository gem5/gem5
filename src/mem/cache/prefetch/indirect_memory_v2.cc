/*
 * Copyright (c) 2024 Samsung Electronics
 *
 * References:
 * X. Yu, C. J. Hughes, N. Satish and S. Devadas,
 * "IMP: Indirect memory prefetcher,"
 * 2015 48th Annual IEEE/ACM International Symposium on Microarchitecture
 * (MICRO), Waikiki, HI, USA, 2015, pp. 178-190, doi: 10.1145/2830772.2830807.
 */

/**
 * @file
 * Describes the indirect memory prefetcher
 *
 * Please refer to src/mem/cache/prefetch/indirect_memory_v2.hh for a short
 * description of the IMP algorithm.
 */

#include "mem/cache/prefetch/indirect_memory_v2.hh"

namespace gem5
{

    namespace prefetch
    {

        /*-------------------------STREAM DETECTOR---------------------------*/

        std::optional<Addr> IMPv2Internals::StreamDetector::addToStreamTable(
                const Addr IP, const Addr addr)
        {
            // Get the relevant entry
            const Addr stream_table_idx = IP % stream_table_size;
            StreamTableEntry& entry = stream_table[stream_table_idx];

            // This is the first access to this entry
            if (!entry.previous_address) {
                entry.previous_address = addr;
                return std::nullopt;
            }

            // This is the second access to this entry
            if (!entry.delta) {
                const Addr delta = addr - *entry.previous_address;

                // Don't set a delta of zero
                if (delta != 0) {
                    entry.delta = delta;
                    entry.confidence = 1;
                }
                entry.previous_address = addr;
                return std::nullopt;
            }

            // This is a repeated access
            if (entry.confidence == 0) {
                // Replace the entry if confidence falls to zero
                entry.delta.reset();
                entry.confidence = 0;
                entry.previous_address = addr;
            }
            else {
                /*
                 * Update the confidence based on whether the delta matches or
                 * not
                 */
                const Addr new_delta = addr - *entry.previous_address;
                if (*entry.delta == new_delta)
                    IMPv2Internals::saturating_increment(entry.confidence,
                            streaming_threshold);
                else
                    IMPv2Internals::saturating_decrement(entry.confidence);
                entry.previous_address = addr;
            }

            // Don't return the delta if the confidence is low
            if (entry.confidence < streaming_threshold)
                return std::nullopt;

            return entry.delta;
        }

        bool IMPv2Internals::StreamDetector::inStreamTable(const Addr IP,
                const Addr addr) const
        {
            // Get the relevant entry
            const Addr stream_table_idx = IP % stream_table_size;
            if (stream_table.find(stream_table_idx) == stream_table.end()) {
                return false;
            }
            const StreamTableEntry& entry = stream_table.at(stream_table_idx);

            // Return false if this entry wasn't completely allocated
            if (!entry.delta) {
                return false;
            }

            // Return false if stream detector is not confident enough
            if (entry.confidence < streaming_threshold) {
                return false;
            }

            const Addr predicted_addr = *entry.previous_address + *entry.delta;
            return (addr == predicted_addr);
        }

        /*------------------------INDIRECT TABLE-----------------------------*/

        void IMPv2Internals::IndirectTable::newIndex(const Addr IP,
                const int64_t index)
        {
            // Get the relevant entry
            const Addr indirect_table_idx = IP % indirect_table_size;
            if (indirect_table.find(indirect_table_idx) ==
                    indirect_table.end()) {
                return;
            }
            IndirectTableEntry& entry = indirect_table.at(indirect_table_idx);

            // Update the predicted_indirect_access
            if (entry.enabled) {
                const Addr offset = entry.shift > 0 ?
                    (index << entry.shift) :
                    (index >> (-entry.shift));
                entry.predicted_indirect_access = entry.base_address + offset;
            }

            /*
             * Reduce the confidence if we observed a new index before
             * observing a match
             */
            if (!entry.found_match) {
                saturating_decrement(entry.confidence);
            }
        }

        void IMPv2Internals::IndirectTable::updateConfidence(const Addr addr)
        {
            // Iterate over all entries and update the confidence
            for (auto& [ip, entry]: indirect_table) {
                if (*entry.predicted_indirect_access == addr) {
                    IMPv2Internals::saturating_increment(entry.confidence,
                            indirect_threshold);
                    entry.found_match = true;
                    saturating_increment(entry.hit_count,
                            max_hit_count);
                }
            }
        }

        void IMPv2Internals::IndirectTable::setBaseAddrAndShift(
                const Addr IP, const Addr base_addr, const int64_t shift)
        {
            // Get the relevant entry
            const Addr indirect_table_idx = IP % indirect_table_size;
            IndirectTableEntry& entry = indirect_table[indirect_table_idx];

            // Populate the fields
            entry.enabled = true;
            entry.base_address = base_addr;
            entry.shift = shift;
            entry.confidence = 1;
            entry.predicted_indirect_access.reset();
            entry.found_match = false;
        }

        std::optional<Addr> IMPv2Internals::IndirectTable::
            getPrefetchCandidate(const Addr IP, const int64_t index) const
        {
            // Get the relevant entry
            const Addr indirect_table_idx = IP % indirect_table_size;
            if (indirect_table.find(indirect_table_idx)
                    == indirect_table.end()) {
                return std::nullopt;
            }
            const IndirectTableEntry& entry =
                indirect_table.at(indirect_table_idx);

            /*
             * Don't generate any prefetch candidates if the entry is
             * disabled
             */
            if (!entry.enabled) {
                return std::nullopt;
            }

            /*
             * Don't generate any prefetch candidates if the confidence is
             * low
             */
            if (entry.confidence < indirect_threshold) {
                return std::nullopt;
            }

            return {entry.base_address + (index << entry.shift)};
        }

        std::optional<Addr> IMPv2Internals::IndirectTable::
            getIndirectPrefetchDistance(const Addr IP) const
        {
            // Get the relevant entry
            const Addr indirect_table_idx = IP % indirect_table_size;
            if (indirect_table.find(indirect_table_idx)
                    == indirect_table.end()) {
                return std::nullopt;
            }
            const IndirectTableEntry& entry =
                indirect_table.at(indirect_table_idx);

            /*
             * Refer to section 3.2.3, right side column, 3rd paragraph of the
             * paper for more details
             */
            return {entry.hit_count};
        }

        /*-----------------INDIRECT PATTERN DETECTOR-------------------------*/

        void IMPv2Internals::IndirectPatternDetector::setIndex(const Addr IP,
                const int64_t index)
        {
            // Update the active IP
            active_IP = IP;

            // Get the relevant entry
            const Addr IPD_table_idx = *active_IP % IPD_table_size;
            IPDEntry& entry = IPD_table[IPD_table_idx];

            // Reset the miss counter since this is a new index
            entry.recorded_misses = 0;

            if (!entry.index1) {
                entry.index1 = index;
                return;
            }
            if (!entry.index2) {
                /*
                 * If the base addresses were not generated using index1, then
                 * set index1 to the new index
                 */
                if (entry.base_address_candidates.size() == 0) {
                    entry.index1 = index;
                }
                else {
                    entry.index2 = index;
                }
                return;
            }

            // This is the third index from the indexing array, i.e. B
            if (!entry.found_params) {
                /*
                 * If IMP has still not detected an indirect pattern, free the
                 * entry
                 * Refer to the third page, left side column, second paragraph
                 * of the paper for more details
                 */
                entry.index1.reset();
                entry.index2.reset();
                entry.recorded_misses = 0;
                for (auto& v: entry.base_address_candidates) {
                    v.clear();
                }
                entry.base_address_candidates.clear();

                /*
                 * Now that the entry has been freed, we can record the new
                 * index as index1
                 */
                entry.index1 = index;
            }
        }

        void IMPv2Internals::IndirectPatternDetector::discardIndex()
        {
            // Do nothing if no IP is active
            if (!active_IP) {
                return;
            }

            // Get the relevant entry
            const Addr IPD_table_idx = *active_IP % IPD_table_size;
            if (IPD_table.find(IPD_table_idx) == IPD_table.end()) {
                    return;
            }
            IPDEntry& entry = IPD_table.at(IPD_table_idx);

            // Invalidate index2 if it exists
            if (entry.index2) {
                entry.index2.reset();
                return;
            }

            /*
             * Invalidate index1 if it exists. Note that we will only reach
             * point if index2 was not allocated yet
             */
            if (entry.index1) {
                entry.index1.reset();
                return;
            }
        }

        using IMPv2Internals::IndirectPatternDetector;
        std::optional<IndirectPatternDetector::IndirectParameters>
            IMPv2Internals::IndirectPatternDetector::recordMiss(
                    const Addr miss_addr)
        {
            // Do nothing if IMP is not tracking any index
            if (!active_IP) {
                return std::nullopt;
            }

            // Get the relevant entry
            const Addr IPD_table_idx = *active_IP % IPD_table_size;
            if (IPD_table.find(IPD_table_idx) == IPD_table.end()) {
                return std::nullopt;
            }
            IPDEntry& entry = IPD_table.at(IPD_table_idx);

            // Do nothing if there are no more misses left to record
            if (entry.recorded_misses >= num_miss_after_index) {
                return std::nullopt;
            }

            // Do nothing if the indirect pattern was already found
            if (entry.found_params) {
                return std::nullopt;
            }

            // Do nothing if index1 is not set yet
            if (!entry.index1) {
                return std::nullopt;
            }

            // Update the counter as IMP is recording a new miss
            entry.recorded_misses++;

            // Generate candidate base addresses if only index1 is set
            if (entry.index1 && !entry.index2) {
                /*
                 * Populate the base address set with empty vectors if this
                 * is the first time IMP is inserting into the base address
                 * set
                 */
                if (entry.base_address_candidates.size() == 0) {
                    for ([[maybe_unused]] const auto& shift: shift_values) {
                        entry.base_address_candidates.emplace_back();
                    }
                }

                // Compute base addresses for all shift values
                std::size_t i = 0;
                for (const auto& shift: shift_values) {
                    entry.base_address_candidates.emplace_back();
                    const Addr ba = shift > 0 ?
                        (miss_addr - (*entry.index1 << shift)) :
                        (miss_addr - (*entry.index1 >> (-shift)));

                    entry.base_address_candidates[i++].push_back(ba);
                }

                /*
                 * IMP cannot find the indirect parameters yet as it needs
                 * two index values
                 */
                return std::nullopt;
            }
            /*
             * Both index values are available. Now IMP can try to
             * calculate the base address
             */
            else {
                std::size_t i = 0;
                for (const auto& shift: shift_values) {
                    const Addr test_base_addr = shift > 0 ?
                        (miss_addr - (*entry.index2 << shift)) :
                        (miss_addr - (*entry.index2 >> (-shift)));

                    // Check if the base address matches
                    for (const auto& addr:
                            entry.base_address_candidates[i]) {
                        if (addr == test_base_addr) {
                            entry.found_params = true;
                            return {std::make_tuple(*active_IP, addr,
                                    shift)};
                        }
                    }

                    // Move to the next shift value
                    i++;
                }

                /*
                 * If we reach here, then IMP wasn't able to find any
                 * matches
                 */
                return std::nullopt;
            }
        }

        /*----------------INDIRECT MEMORY PREFETCHER-------------------------*/

        IMP::IMP(const IMPv2PrefetcherParams& p):  Queued(p),
        prefetch_table(p.prefetch_table_size, p.streaming_threshold,
                p.max_indirect_prefetch_distance, p.indirect_threshold),
        ipd(p.indirect_pattern_detector_size, p.num_miss_after_index,
                std::move(p.shift_values)),
        streaming_degree(p.streaming_degree),
        indirect_degree(p.indirect_degree),
        byteOrder(p.sys->getGuestByteOrder()),
        IMPv2Stats(this)
        {
            IMPv2Stats.stream_prefetches = 0;
            IMPv2Stats.indirect_prefetches = 0;
        }

        void IMP::updateConfidence(const Addr addr)
        {
            prefetch_table.getIndirectTable().updateConfidence(addr);
        }

        std::vector<Addr> IMP::updateStreamDetectorState(
                const PrefetchInfo& pfi)
        {
            std::vector<Addr> pf_candidates;

            /*
             * Index array accesses are always read accesses. Moreover, IMP
             * needs the IP to index into the stream detector table
             */
            if (!pfi.isWrite() && pfi.hasPC()) {
                /*
                 * Add this cache access to the stream table. If the
                 * confidence of the stream associated with this access is
                 * high enough, the stream table may return the stream
                 * delta
                 */
                std::optional<Addr> delta =
                    prefetch_table.getStreamDetector().addToStreamTable(
                            pfi.getPC(), pfi.getAddr());

                /*
                 * If IMP gets a delta from the stream table, it generates
                 * prefetches for the detected stream
                 */
                if (delta) {
                    // Get the address of the first streaming prefetch
                    Addr streaming_pf = pfi.getAddr();
                    std::optional<Addr> distance = prefetch_table.
                        getIndirectTable().
                        getIndirectPrefetchDistance(pfi.getPC());
                    if (distance) {
                        streaming_pf += *distance;
                    }

                    while (pf_candidates.size() < streaming_degree) {
                        /*
                         * Increase the prefetch candidate's address until we
                         * reach a new cache line
                         */
                        if (blockAddress(streaming_pf + *delta) ==
                                blockAddress(streaming_pf)) {
                            streaming_pf += *delta;
                        }
                        // Save the prefetch candidate
                        else {
                            streaming_pf += *delta;
                            pf_candidates.push_back(streaming_pf);

                            /*
                             * Store the necessary metadata to required to
                             * issue indirect prefetches using the streaming
                             * prefetch. The indirect prefetch candidates would
                             * be computed in notifyFill when IMP gets notified
                             * of the fills for the streaming prefetches
                             */
                            const Addr key = blockAddress(streaming_pf);
                            deferred_prefetch_metadata value = {
                                .IP = pfi.getPC(),
                                .size = pfi.getSize(),
                                .prefetch_address = streaming_pf,
                            };

                            // Only store the metadata once for each cache line
                            if (index_blkaddr_to_metadata.find(key) ==
                                    index_blkaddr_to_metadata.end())
                                index_blkaddr_to_metadata[key] = value;
                        }
                    }
                }
            }

            return pf_candidates;
        }

        void IMP::handleIndexAccess(const PrefetchInfo& pfi)
        {
            /*
             * If this access was to the indexing array B (IMP assumes this to
             * be the case if the stream detector says that this access was the
             * next address in the stream), read the index value and provide it
             * to the indirect pattern detector and the indirect table
             */

            // Check if the pfi has the data which was accessed in the cache
            if (pfi.hasData()) {
                /*
                 * Read the data corresponding to this cache access. IMP
                 * assumes that this data is the index value read from the
                 * indexing array B. At this point we don't know how much
                 * data was read (1 byte, 2 bytes, 4 bytes, or 8 bytes). So
                 * to match the interface of IMP we cast the data associated
                 * with this pfi to Addr
                 */
                std::optional<int64_t> index_value{};
                switch(pfi.getSize()) {
                    case sizeof(int8_t):
                        index_value = pfi.get<int8_t>(byteOrder);
                        break;
                    case sizeof(int16_t):
                        index_value = pfi.get<int16_t>(byteOrder);
                        break;
                    case sizeof(int32_t):
                        index_value = pfi.get<int32_t>(byteOrder);
                        break;
                    case sizeof(int64_t):
                        index_value = pfi.get<int64_t>(byteOrder);
                        break;
                }
                if (index_value) {
                    IMPv2Stats.ipd_updates++;

                    /*
                     * Update the IPD state by providing the index value
                     * which would be stored in the IPD_table
                     */
                    ipd.setIndex(pfi.getPC(), *index_value);

                    /*
                     * Update the indirect table state. The indirect table
                     * will use the IP of this access to update the
                     * confidence counters
                     */
                    prefetch_table.getIndirectTable().newIndex(pfi.getPC(),
                            *index_value);
                }
            }
            else {
                /*
                 * If IMP sees a new access to the indexing array, B, and
                 * it can't access the index value, then it should remove
                 * the last observed index from the IPD_table since the
                 * subsequent miss would not correspond to the stored index,
                 * rather it would correspond to the current index which was
                 * NOT read from the pfi
                 */
                ipd.discardIndex();
            }
        }

        void IMP::handleIndirectAccess(const PrefetchInfo& pfi)
        {
            // IMP only tracks misses, not hits
            if (pfi.isCacheMiss()) {
                IMPv2Stats.misses_recorded++;

                /*
                 * Record the miss in the IPD. This call may generate a
                 * (base_addr, shift pair)
                 */
                const auto indirect_parameters = ipd.recordMiss(
                        pfi.getAddr());

                /*
                 * Check if indirect parameters were generated
                 * If yes, store them in the indirect table
                 */
                if (indirect_parameters) {
                    const auto& [IP, base_addr, shift] =
                        *indirect_parameters;
                    prefetch_table.getIndirectTable().setBaseAddrAndShift(
                            IP, base_addr, shift);
                }
            }
        }

        std::vector<Addr> IMP::learnPattern(const PrefetchInfo& pfi)
        {
            /*
             * We need the PC to index the tables. If PC is not available,
             * them IMP cannot operate
             */
            if (!pfi.hasPC()) {
                return {};
            }

            /*
             * Check if this access is stream access before adding it to the
             * stream table. It is necessary to check for hits prior to
             * modifying the state of the stream detector because of how the
             * stream detector works. For an address to hit in the stream
             * detector, it must match with the next predicted address computed
             * by the stream detector. When we update the stream detector
             * state, we also implicitly move the predicted address ahead by
             * the stream delta. As a result if we perform the update first and
             * then check for matches, we will always miss
             */
            const bool stream_hit = prefetch_table.getStreamDetector().
                inStreamTable(pfi.getPC(), pfi.getAddr());

            if (stream_hit) {
                IMPv2Stats.stream_hits++;
            }

            /*
             * Update the stream detector and generate streaming prefetch
             * candidates
             */
            std::vector<Addr> pf_candidates = updateStreamDetectorState(pfi);

            // Learn prefetch patterns
            if (stream_hit) {
                handleIndexAccess(pfi);
            }
            else {
                handleIndirectAccess(pfi);
            }

            return pf_candidates;
        }

        std::optional<Addr> IMP::generateIndirectPrefetchCandidate(
                const PrefetchInfo& pfi)
        {
            /*
             * IMP necessarily needs the IP and the data read from the cache
             * (which it assumes is the index read from the array B) to
             * generate indirect prefetch candidates
             */
            if (!pfi.hasPC() || !pfi.hasData()) {
                return std::nullopt;
            }

            // Read the IP
            const Addr& IP = pfi.getPC();

            // Read the index value associated with this cache access
            std::optional<int64_t> index_value{};
            switch(pfi.getSize()) {
                case sizeof(int8_t):
                    index_value = pfi.get<int8_t>(byteOrder);
                    break;
                case sizeof(int16_t):
                    index_value = pfi.get<int16_t>(byteOrder);
                    break;
                case sizeof(int32_t):
                    index_value = pfi.get<int32_t>(byteOrder);
                    break;
                case sizeof(int64_t):
                    index_value = pfi.get<int64_t>(byteOrder);
                    break;
            }

            /*
             * Query the indirect table and return the prefetch candidate
             * generated by the pattern table
             */
            if (index_value) {
                return prefetch_table.getIndirectTable().
                    getPrefetchCandidate(IP, *index_value);
            }

            /*
             * If we reach this point, then it means that IMP was not able to
             * generate an indirect prefetch candidate
             */
            return std::nullopt;
        }

        void IMP::calculatePrefetch(const PrefetchInfo &pfi,
                std::vector<AddrPriority> &addresses,
                const CacheAccessor &cache)
        {
            addresses.clear();

            // Update the confidence counters
            updateConfidence(pfi.getAddr());

            // Learn the access pattern
            std::vector<Addr> index_pf_candidates = learnPattern(pfi);

            // Generate a prefetch candidate
            std::optional<Addr> indirect_pf_candidate =
                generateIndirectPrefetchCandidate(pfi);

            /*
             * Insert streaming prefetches with higher priority as indirect
             * prefetches have a dependency on the index value
             * These prefetches represent the loads from the indexing array, B
             */
            for (const auto& addr: index_pf_candidates) {
                addresses.push_back({addr, 1});
                IMPv2Stats.stream_prefetches++;
            }

            // Insert the indirect prefetch (if generated) with lower priority
            if (indirect_pf_candidate) {
                addresses.push_back({*indirect_pf_candidate, 0});
                IMPv2Stats.indirect_prefetches++;
            }

            // Insert the pending prefetches with lower priority
            for (const auto& addr: pending_prefetches) {
                addresses.push_back({addr, 0});
                IMPv2Stats.indirect_prefetches++;
                IMPv2Stats.deferred_indirect_prefetches++;
            }
            pending_prefetches.clear();

            return;
        }

        void IMP::notifyFill(const CacheAccessProbeArg &arg)
        {
            const PacketPtr& pkt = arg.pkt;

            /*
             * Read the address for the fill access. Note that pkt->getAddr()
             * almost always returns the block address. However, since
             * occasionally it may not return the block address, we need to
             * explicitly
             */
            const Addr request_blockaddr = blockAddress(pkt->getAddr());

            /*
             * Get the context metadata which triggered this prefetch. Also
             * erase the consumed metadata from the map as we no longer need it
             */
            if (index_blkaddr_to_metadata.find(request_blockaddr) ==
                    index_blkaddr_to_metadata.end()) {
                return;
            }
            const deferred_prefetch_metadata metadata =
                index_blkaddr_to_metadata.at(request_blockaddr);
            index_blkaddr_to_metadata.erase(request_blockaddr);

            /*
             * Check if the data, i.e the index value, is available. If it is,
             * then try to compute prefetch candidates
             */
            if (pkt->hasData() && pkt->getPtr<uint8_t>() != nullptr) {
                uint8_t* data_ptr = pkt->getPtr<uint8_t>();
                const int32_t block_offset = (metadata.prefetch_address -
                        request_blockaddr);

                /*
                 * Sanity check
                 * 1. Offset should be positive as we are reading words from
                 *    this cache line, and not the cache line behind this one
                 *    in the order of address
                 */
                if (block_offset < 0) {
                    return;
                }

                // 2. We should not read beyond the cache line
                if ((block_offset + metadata.size) > pkt->getSize()) {
                    return;
                }

                /*
                 * Move the data pointer forward to reach the word of which
                 * holds the index value
                 */
                data_ptr += block_offset;

                for (std::size_t i = 0; i < indirect_degree; i++) {
                    // Read the index value
                    std::optional<int64_t> index;
                    switch(metadata.size) {
                        case sizeof(int8_t):
                            index = *((int8_t*)data_ptr);
                            break;
                        case sizeof(int16_t):
                            index = *((int16_t*)data_ptr);
                            break;
                        case sizeof(int32_t):
                            index = *((int32_t*)data_ptr);
                            break;
                        case sizeof(int64_t):
                            index = *((int64_t*)data_ptr);
                            break;
                    }

                    if (index) {
                        /*
                         * Query the indirect table and issue the prefetch
                         * candidate generated by the pattern table
                         */
                        std::optional<Addr> pf_candidate =
                            prefetch_table.getIndirectTable().
                            getPrefetchCandidate(metadata.IP, *index);

                        /*
                         * Store the prefetch candidate in the
                         * pending_prefetches vector so that on the next call
                         * to calculatePrefetch, the indirect prefetch can be
                         * issued
                         */
                        if (pf_candidate)
                            pending_prefetches.push_back(*pf_candidate);
                    }

                    // Move the data pointer forward to the next index position
                    data_ptr += metadata.size;
                }
            }
        }

        // Statistics
        IMP::StatGroup::StatGroup(statistics::Group *parent):
            statistics::Group(parent),
            stream_prefetches(this, "stream_prefetches",
                    statistics::units::Count::get(),
                    "Number of stream prefetches issued"),
            indirect_prefetches(this, "indirect_prefetches",
                    statistics::units::Count::get(),
                    "Number of indirect prefetches issued"),
            stream_hits(this, "stream_hits",
                    statistics::units::Count::get(),
                    "Number of accesses which match with a predicted "
                    "stream address"),
            ipd_updates(this, "ipd_updates",
                    statistics::units::Count::get(),
                    "Number of index value updates made to the "
                    "Indirect Pattern Detector"),
            misses_recorded(this, "misses_recorded",
                    statistics::units::Count::get(),
                    "Number of accesses which match with a predicted "
                    "stream address"),
            deferred_indirect_prefetches(this, "deferred_indirect_prefetches",
                    statistics::units::Count::get(),
                    "Number of indirect prefetches issued after fill access")
        {
            stream_prefetches= 0;
            indirect_prefetches= 0;
            stream_hits= 0;
            ipd_updates= 0;
            misses_recorded= 0;
            deferred_indirect_prefetches = 0;
        }
    } // namespace prefetch
} // namespace gem5
