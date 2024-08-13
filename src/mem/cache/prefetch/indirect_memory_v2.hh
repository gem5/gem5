/**
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
 * This file describes the implementation of the Indirect Memory Prefetcher
 * (IMP).
 *
 * Working Principle:
 * The goal of IMP is to issue prefetches for memory accesses of type
 * A[B[i]] where A and B are arrays and i is the indexing variable. Array B's
 * accesses are strided, i.e. the addresses difference between consecutive
 * accesses to B is constant. Array A's accesses, on the other hand, need not
 * be strided. The access pattern for array A depends on the values stored in
 * the array B. Moreover, to compute the address for array A's accesses, IMP
 * needs to read the values of array B. The accesses to array A follow the
 * following equation
 *      address = base_address + (coefficient * value)
 * where
 *      address represents the access address of array A
 *      base_address represents the base address of array A, i.e. &A[0]
 *      coefficient represents the size of elements of A
 *      value is the index into array A, i.e. the value B[i]
 *
 * Since in most cases the coefficient is a power of 2, we can replace it with
 * a left arithmetic shift operation as follows:
 *      address = base_address + (value << shift)
 *
 * Therefore, to calculate the prefetch address for array A, i.e. address, we
 * need to calculate the base_address and shift. The IMP algorithm performs
 * this calculation by solving the above equation using two pairs of
 * (value, address). Thus, the problem now gets reduced to solving a system of
 * linear equations with two variables. The solution to this linear system is
 * the predicted pair (base_address, shift).
 *
 * Once the (base_address, shift) pair is known, then IMP can compute the
 * predicted access to array A, i.e. address, whenever it observes a load to
 * the indexing array B, i.e. value. Note that for this algorithm to work IMP
 * must figure out the value of index, i.e. the value loaded from array B.
 * Therefore, IMP must track access to array B and then uses these accesses,
 * along with access to array A, to compute the indirect access parameters
 * (base_address, shift). IMP uses a stream detector to track accesses to array
 * B. For computing the indirect access parameters, IMP uses a structure called
 * the IndirectPatternDetector (IPD). After IMP's IPD detects an indirect
 * pattern, i.e. the once it is able to compute the indirect parameters, IMP
 * stores the computed pair in its IndirectTable
 *
 * For more details about the IMP algorithm, please refer to the paper
 * mentioned in the references.
 *
 * What does this implementation DOESN'T do:
 * There are some differences between this implementation and the algorithm
 * described in the paper. Namely, this implementation does not support
 * indirect prefetching for multi-way and multi-level accesses as described in
 * section 3.3.2 of the paper
 */

#ifndef __MEM_CACHE_IMP_HH__
#define __MEM_CACHE_IMP_HH__

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IMPv2Prefetcher.hh"
#include "sim/system.hh"

namespace gem5
{
    namespace prefetch
    {
        namespace IMPv2Internals
        {
            template<typename T>
            void saturating_increment(T& old_value, const T& upper_bound)
            {
                if (old_value < upper_bound)
                    old_value += 1;
            }

            template<typename T>
            void saturating_decrement(T& old_value)
            {
                if (old_value > 0)
                    old_value -= 1;
            }

            /*
             * The StreamDetector performs two functions:
             *  1. It detects stream accesses to the indexing array, B
             *  2. If it is confident about a stream, it returns the predicted
             *     delta for that stream so that IMP may issue stream
             *     prefetches using the delta
             */
            class StreamDetector
            {
                private:
                    struct StreamTableEntry
                    {
                        std::optional<Addr> previous_address;
                        std::optional<Addr> delta;
                        uint64_t confidence;

                        StreamTableEntry()
                        {
                            previous_address = {};
                            delta = {};
                            confidence = 0;
                        }
                    };

                    /*
                     * The confidence threshold above which a stream is
                     * classified as detected
                     */
                    const uint64_t streaming_threshold;

                    /*
                     * The stream table is indexed using the IP of the load
                     * instruction
                     */
                    std::map<Addr, StreamTableEntry> stream_table;
                    const uint64_t stream_table_size;

                public:
                    StreamDetector(std::size_t size, uint64_t threshold):
                        streaming_threshold(threshold), stream_table_size(size)
                    {}

                    /*
                     * Add a load address, addr, to the stream table.
                     * The address was generated by a instruction which the IP
                     * points to. Return the address delta if the confidence
                     * is high enough for the stream associated with IP
                     */
                    std::optional<Addr> addToStreamTable(const Addr IP,
                            const Addr addr);

                    /*
                     * Check if an address hits in the stream table, i.e.
                     * whether the stored stream would predict this address
                     * next or not
                     */
                    bool inStreamTable(const Addr IP, const Addr addr) const;
            };

            /*
             * IndirectTable keeps track of indirect parameters, i.e.
             * (base_address, shift) pairs. This table is indexed using the IP
             * of the load instruction which accesses the indexing array B
             */
            class IndirectTable
            {
                private:
                    struct IndirectTableEntry
                    {
                        bool enabled;
                        // Supplied by IPD
                        Addr base_address;
                        // Supplied by IPD
                        int64_t shift;
                        uint64_t confidence;

                        /*
                         * The predicted indirect access is used to update the
                         * confidence of this entry
                         */
                        std::optional<Addr> predicted_indirect_access;

                        /*
                         * This flag tracks whether the predicted indirect
                         * access was observed or not. This flag is used to
                         * update the confidence
                         */
                        bool found_match;

                        /*
                         * This counter tracks the number of hits obtained on
                         * this indirect pattern. This counter is upper bounded
                         * by max_hit_count. This counter is then used by IMP
                         * to decide the prefetch distance for the stride
                         * component of IMP which is responsible for issuing
                         * prefetches for the index array, B
                         */
                        uint64_t hit_count;

                        IndirectTableEntry()
                        {
                            enabled = false;
                            base_address = 0;
                            shift = 0;
                            confidence = 0;
                            predicted_indirect_access = {};
                            found_match = false;
                            hit_count = 0;
                        }
                    };

                    /*
                     * The maximum permissible hit count value
                     */
                    const uint64_t max_hit_count;

                    /*
                     * The confidence threshold above which a
                     * IndirectTableEntry can be used to issue prefetches
                     */
                    const uint64_t indirect_threshold;

                    /*
                     * The indirect table is indexed using the IP of the load
                     * instruction responsible for reading the indexing array,
                     * i.e. B. Note that the stream table is also indexed using
                     * the IP
                     */
                    std::map<Addr, IndirectTableEntry> indirect_table;
                    const uint64_t indirect_table_size;

                public:
                    IndirectTable(const std::size_t size,
                            const uint64_t max_hit_cnt,
                            const uint64_t threshold):
                        max_hit_count(max_hit_cnt),
                        indirect_threshold(threshold),
                        indirect_table_size(size)
                    {}

                    /* This function is called whenever a new index value is
                     * observed, i.e. a read from array B. This function
                     * performs two jobs:
                     * 1. It updates the predicted indirect access value for
                     *    the entry associated with IP
                     * 2. If the found_match flag for the entry is set to
                     *    false when this function is called, the entry's
                     *    confidence is decremented.
                     * IP points
                     * to the load instruction which read the new index value
                     * and index is the index value was read from the cache.
                     */
                    void newIndex(const Addr IP, const int64_t index);

                    /*
                     * Update the confidence counter of all entries in the
                     * pattern table which would have predicted addr
                     */
                    void updateConfidence(const Addr addr);

                    /*
                     * Set the base address and shift values in the pattern
                     * table for a indexing array load instruction pointed to
                     * by IP
                     */
                    void setBaseAddrAndShift(const Addr IP,
                            const Addr base_addr, const int64_t shift);
                    /*
                     * Get the predicted indirect access address for the index
                     * value loaded by the instruction which the IP points to
                     */
                    std::optional<Addr> getPrefetchCandidate(const Addr IP,
                            const int64_t index) const;

                    /*
                     * Get the indirect prefetch distance corresponding to the
                     * pattern associated with IP
                     */
                    std::optional<Addr> getIndirectPrefetchDistance(
                            const Addr IP) const;
            };

            /*
             * PrefetchTable is a transparent wrapper which includes both the
             * StreamDetector and the IndirectTable
             */
            class PrefetchTable
            {
                private:
                    StreamDetector stream_detector;
                    IndirectTable indirect_table;

                public:
                    PrefetchTable(const std::size_t size,
                            const uint64_t streaming_threshold,
                            const uint64_t max_hit_count,
                            const uint64_t indirect_threshold):
                        stream_detector(size, streaming_threshold),
                        indirect_table(size, max_hit_count, indirect_threshold)
                    {}

                    constexpr StreamDetector& getStreamDetector()
                    {
                        return this->stream_detector;
                    }

                    constexpr IndirectTable& getIndirectTable()
                    {
                        return this->indirect_table;
                    }
            };

            /*
             * The IPD performs the computation to find the base address and
             * shift pairs
             */
            class IndirectPatternDetector
            {
                private:
                    struct IPDEntry
                    {
                        /*
                         * The observed index values (values from array B)
                         * which are used to compute the candidate base
                         * addresses
                         */
                        std::optional<int64_t> index1;
                        std::optional<int64_t> index2;

                        /*
                         * The number of misses for whom this enrty has base
                         * addresses
                         */
                        std::size_t recorded_misses;

                        /*
                         * This tracks whether this entry generated indirect
                         * parameters or not
                         */
                        bool found_params;

                        /* The list of base address candidates
                         * This structure is populate when index1 is set
                         * When index2 is set, IMP looks up this structure to
                         * search for matches
                         * The outer iterator is shift values and the inner
                         * iterator is the number of cache miss (which IMP
                         * assumes is due to an access to indirect array, A)
                         * observed after index1 was set, until index2 got set.
                         * The number of tracked cache misses, i.e. the range
                         * of the inner iterator is equal to
                         * num_miss_after_index
                         */
                        std::vector<std::vector<Addr>> base_address_candidates;

                        IPDEntry()
                        {
                            index1 = std::nullopt;
                            index2 = std::nullopt;
                            recorded_misses = 0;
                            found_params = false;
                            base_address_candidates.clear();
                        }

                        ~IPDEntry()
                        {
                            for (auto& vec: base_address_candidates)
                                vec.clear();
                            base_address_candidates.clear();
                        }
                    };

                    /*
                     * The IPD Table is indexed using the IP of the load
                     * instruction responsible for reading from the indexing
                     * array, i.e. B
                     */
                    std::map<Addr, IPDEntry> IPD_table;
                    const uint64_t IPD_table_size;

                    /*
                     * The IP of the most recent load instruction which read
                     * from the indexing array, B. IMP needs to keep a track of
                     * this since the IPD_table is updated on accesses to the
                     * indirect array, A. While updating the IPD_table, IMP
                     * uses the stored IP of the indexing load instruction as
                     * the index into the IPD_table
                     */
                    std::optional<Addr> active_IP;

                    /*
                     * List of supported shift values
                     */
                    const std::vector<int> shift_values;

                    /*
                     * Number of misses to track after an index access
                     */
                    const uint64_t num_miss_after_index;

                public:
                    /*
                     * This type represents the output of the IPD. The goal of
                     * the IPD is to compute the indirect access parameters for
                     * a given IP (which points to the load instruction
                     * responsible for accessing the indexing array B), i.e a
                     * (base address, shift) pair. Since these parameters need
                     * to be stored in the indirect table (which is indexed
                     * using the IP responsible for reading from the indexing
                     * array, B), IPD must also return the IP for which it
                     * computed the (base address, shift) pair
                     */
                    typedef std::tuple<Addr, Addr, uint64_t>
                        IndirectParameters;

                    IndirectPatternDetector(const std::size_t size,
                            const uint64_t num_miss,
                            const std::vector<int>&& shift_vals):
                        IPD_table_size(size), active_IP({}),
                        shift_values(std::move(shift_vals)),
                        num_miss_after_index(num_miss)
                        {}

                    /*
                     * Set the index1 and index2 fields for the entry
                     * corresponding to IP
                     */
                    void setIndex(const Addr IP, const int64_t index);

                    /*
                     * Discard the most recently observed index
                     */
                    void discardIndex();

                    /*
                     * Record a miss and compute the indirect access parameters
                     * miss_addr is the cache access address which missed and
                     * IMP assumes that the miss_addr corresponds to an access
                     * to the indirect array, A
                     */
                    std::optional<IndirectParameters> recordMiss(
                            const Addr miss_addr);
            };
        } // namespace IMPv2Internals

        class IMP : public Queued
        {
            private:
                IMPv2Internals::PrefetchTable prefetch_table;
                IMPv2Internals::IndirectPatternDetector ipd;

                /*
                 * The degree for streaming and indirect prefetches
                 */
                const uint64_t streaming_degree;
                const uint64_t indirect_degree;

                /*
                 * Endianness of the micro-architecture
                 */
                const ByteOrder byteOrder;

                /*
                 * Responsible for updating the confidence counts.
                 * addr is a cache access address.
                 */
                void updateConfidence(const Addr addr);

                /*
                 * Responsible for updating the stream detector state
                 *
                 * pfi holds the information regarding a cache access
                 */
                std::vector<Addr> updateStreamDetectorState(
                        const PrefetchInfo& pfi);

                /*
                 * Responsible for handling index accesses, i.e. accesses
                 * to the indexing array, B
                 *
                 * pfi holds the information regarding a cache access
                 */
                void handleIndexAccess(const PrefetchInfo& pfi);

                /*
                 * Responsible for handling indirect accesses, i.e accesses to
                 * array A
                 *
                 * pfi holds the information regarding a cache access
                 */
                void handleIndirectAccess(const PrefetchInfo& pfi);

                /*
                 * Responsible for learning indirect patterns and issuing
                 * streaming prefetches.
                 *
                 * pfi holds the information regarding a cache access
                 */
                std::vector<Addr> learnPattern(const PrefetchInfo& pfi);

                /*
                 * Responsible for generating the indirect prefetch candidate
                 */
                std::optional<Addr> generateIndirectPrefetchCandidate(
                        const PrefetchInfo& pfi);

                /*
                 * This structure holds the necessary metadata which must be
                 * tracked to issue deferred prefetches. A deferred prefetch
                 * is an indirect prefetch associated with an index value I
                 * such that at the time of prefetch calculation, I is not
                 * available. For example, suppose IMP has classified a load
                 * load IP as a stream access IP, i.e. it has classified this
                 * IP as an IP which accesses the indexing array B. Moreover,
                 * let's also assume that IMP has computed the base address of
                 * the indirect array A which uses values from B to perform
                 * indirect loads. Since all the prefetching structures are
                 * trained, IMP can now issue indirect prefetches for the array
                 * A. Now lets suppose that IMP sees an access to B[i]. At this
                 * point IMP will also receive the value of B[i] in the
                 * prefetch information (pfi) structure (in the
                 * calculatePrefetch function). Using the value B[i], and the
                 * pre-computed base address for A, IMP can now issue a single
                 * indirect prefetch for A[B[i]]. However, IMP cannot issue any
                 * prefetches for A[B[i+delta]] for any delta > 0. This is
                 * because although IMP can calculate the address of
                 * B[i+delta], it cannot calculate the value of B[i+delta].
                 * Therefore, all that IMP can do at this point is issue a
                 * prefetch for B[i+delta], i.e. a streaming prefetch, and wait
                 * to receive a notification from the cache for the cache
                 * fill of B[i+delta]. Once IMP receives this notification, it
                 * can read the value of B[i+delta] using the value stored in
                 * the associated pfi. The processing of this notification
                 * happens in the notifyFill function. However, at the
                 * notifyFill function, IMP does not have the necessary context
                 * which resulted in the prefetch of B[i+delta]. Therefore,
                 * even though IMP has all the information needed for indirect
                 * prefetches, it still cannot issue them. To solve this issue,
                 * we store all the necessary context, which would be required
                 * to issue indirect prefetches, in this structure. Note that
                 * one instance of the structure corresponds to only stream
                 * prefetch. However, since there can be multiple index values
                 * in a single cache line, at notifyFill, we can issue multiple
                 * indirect prefetches using a single instance of this
                 * structure
                 *
                 * NOTE: This structure is present in IMP only for
                 * implementation purposes. This structure does NOT have any
                 * equivalent counterpart at the hardware level
                 */
                struct deferred_prefetch_metadata
                {
                    /*
                     * The IP responsible for the creation of this structure,
                     * i.e. the load IP responsible for reading the index array
                     * B. This IP is used to index the indirect table to
                     * compute the indirect prefetch candidate
                     */
                    Addr IP;

                    /*
                     * The size of the index value in bytes. This variable
                     * serves two purposes:
                     *  1. It allows IMP to read the correct number of bytes
                     *     which form the index value in a cache line
                     *  2. It allows IMP to issue multiple indirect prefetches
                     *     from the same cache line by striding across the
                     *     cache line with stride delta of size
                     */
                    std::size_t size;

                    /*
                     * The actual address for which the prefetch was issued.
                     * Note that it is necessary to track this value since at
                     * notifyFill IMP receives cache lines. The actual word
                     * address which triggered the fill is erased. Therefore,
                     * to figure out the where in the cache line the index
                     * value lies, we need the word address of the streaming
                     * prefetch responsible for the fill
                     */
                    Addr prefetch_address;
                };

                /*
                 * This map keeps a track of which prefetches have been issued
                 * for stream accesses. The key is the prefetch block address,
                 * i.e the address B[i] rounded down to the cache line address
                 * which holds the value B[i], and the value is an instance of
                 * the deferred_prefetch_metadata structure. The reason why we
                 * use the cache line address as the key is because in the
                 * notifyFill function, where this map would be accessed, the
                 * pfi that IMP receives usually contains the cache line
                 * address and not the word address
                 *
                 * NOTE: This map is present in IMP only for implementation
                 * purposes. This structure does NOT have any equivalent
                 * counterpart at the hardware level
                 */
                std::map<Addr, deferred_prefetch_metadata>
                    index_blkaddr_to_metadata;

                /*
                 * This vector is filled in notifyFill, and emptied in
                 * calculatePrefetch. notifyFill inserts prefetch candidates
                 * into this vector and on the next call to calculatePrefetch,
                 * prefetches are issued from this vector. This vector is
                 * necessary since notifyFill does not have access to the
                 * prefetch queue (i.e. the addresses parameter of
                 * calculatePrefetch).
                 *
                 * NOTE: This structure is present in IMP only for
                 * implementation purposes. This structure does NOT have any
                 * equivalent counterpart at the hardware level
                 */
                std::vector<Addr> pending_prefetches;

                /*
                 * Statistics
                 */
                struct StatGroup: public statistics::Group
                {
                    StatGroup(statistics::Group *parent);
                    statistics::Scalar stream_prefetches;
                    statistics::Scalar indirect_prefetches;
                    statistics::Scalar stream_hits;
                    statistics::Scalar ipd_updates;
                    statistics::Scalar misses_recorded;
                    statistics::Scalar deferred_indirect_prefetches;
                } IMPv2Stats;

            public:
                IMP(const IMPv2PrefetcherParams& p);
                ~IMP() = default;

                /*
                 * IMP performs three different tasks for every access:
                 * 1. It learns the prefetch pattern
                 * 2. It updates the confidence of indirect patterns
                 * 3. It generates prefetches
                 */
                void calculatePrefetch(const PrefetchInfo &pfi,
                        std::vector<AddrPriority> &addresses,
                        const CacheAccessor &cache) override;

                /*
                 * Whenever IMP issues a streaming prefetch (for the index
                 * array, B), it tracks the associated context in the map
                 * index_blkaddr_to_metadata. When IMP receives a fill
                 * notification, it queries the map to see if any metadata was
                 * associated with the prefetch request which resulted in the
                 * fill. If such metadata exists, then IMP uses it, along with
                 * the data in the fill access (which serves as the index
                 * value), to add indirect prefetch candidates to the
                 * pending_prefetches vector. The pending_prefetches vector
                 * would be accessed by calculatePrefetch to issue the prefetch
                 */
                void notifyFill(const CacheAccessProbeArg &arg) override;
        };

    } // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_IMP_HH__
