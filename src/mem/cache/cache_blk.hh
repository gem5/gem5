/*
 * Copyright (c) 2012-2018, 2023-2024 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2020 Inria
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Definitions of a simple cache block class.
 */

#ifndef __MEM_CACHE_CACHE_BLK_HH__
#define __MEM_CACHE_CACHE_BLK_HH__

#include <cassert>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <list>
#include <string>

#include "base/printable.hh"
#include "base/types.hh"
#include "mem/cache/tags/tagged_entry.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

/**
 * A Basic Cache block.
 * Contains information regarding its coherence, prefetching status, as
 * well as a pointer to its data.
 */
class CacheBlk : public TaggedEntry
{
  public:
    /**
     * Cache block's enum listing the supported coherence bits. The valid
     * bit is not defined here because it is part of a TaggedEntry.
     */
    enum CoherenceBits : unsigned
    {
        /** write permission */
        WritableBit =       0x02,
        /**
         * Read permission. Note that a block can be valid but not readable
         * if there is an outstanding write upgrade miss.
         */
        ReadableBit =       0x04,
        /** dirty (modified) */
        DirtyBit =          0x08,

        /**
         * Helper enum value that includes all other bits. Whenever a new
         * bits is added, this should be updated.
         */
        AllBits  =          0x0E,
    };

    /**
     * Contains a copy of the data in this block for easy access. This is used
     * for efficient execution when the data could be actually stored in
     * another format (COW, compressed, sub-blocked, etc). In all cases the
     * data stored here should be kept consistant with the actual data
     * referenced by this block.
     */
    uint8_t *data = nullptr;

    /**
     * Which curTick() will this block be accessible. Its value is only
     * meaningful if the block is valid.
     */
    Tick whenReady = 0;

  protected:
    /**
     * Represents that the indicated thread context has a "lock" on
     * the block, in the LL/SC sense.
     */
    class Lock
    {
      public:
        ContextID contextId;     // locking context
        Addr lowAddr;      // low address of lock range
        Addr highAddr;     // high address of lock range

        // check for matching execution context, and an address that
        // is within the lock
        bool matches(const RequestPtr &req) const
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() -1;
            return (contextId == req->contextId()) &&
                   (req_low >= lowAddr) && (req_high <= highAddr);
        }

        // check if a request is intersecting and thus invalidating the lock
        bool intersects(const RequestPtr &req) const
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() - 1;

            return (req_low <= highAddr) && (req_high >= lowAddr);
        }

        Lock(const RequestPtr &req)
            : contextId(req->contextId()),
              lowAddr(req->getPaddr()),
              highAddr(lowAddr + req->getSize() - 1)
        {
        }
    };

    /** List of thread contexts that have performed a load-locked (LL)
     * on the block since the last store. */
    std::list<Lock> lockList;

  public:
    CacheBlk() : TaggedEntry(nullptr)
    {
        invalidate();
    }

    CacheBlk(const CacheBlk&) = delete;
    CacheBlk& operator=(const CacheBlk&) = delete;
    CacheBlk(const CacheBlk&&) = delete;
    /**
     * Move assignment operator.
     * This should only be used to move an existing valid entry into an
     * invalid one, not to create a new entry. In the end the valid entry
     * will become invalid, and the invalid, valid. All location related
     * variables will remain the same, that is, an entry cannot move its
     * data, just its metadata contents.
     */
    virtual CacheBlk&
    operator=(CacheBlk&& other)
    {
        // Copying an entry into a valid one would imply in skipping all
        // replacement steps, so it cannot be allowed
        assert(!isValid());
        assert(other.isValid());

        insert({other.getTag(), other.isSecure()});

        if (other.wasPrefetched()) {
            setPrefetched();
        }
        setCoherenceBits(other.coherence);
        setTaskId(other.getTaskId());
        setPartitionId(other.getPartitionId());
        setWhenReady(curTick());
        setRefCount(other.getRefCount());
        setSrcRequestorId(other.getSrcRequestorId());
        std::swap(lockList, other.lockList);

        other.invalidate();

        return *this;
    }
    virtual ~CacheBlk() {};

    /**
     * Invalidate the block and clear all state.
     */
    virtual void invalidate() override
    {
        TaggedEntry::invalidate();

        clearPrefetched();
        clearCoherenceBits(AllBits);

        setTaskId(context_switch_task_id::Unknown);
        setPartitionId(std::numeric_limits<uint64_t>::max());
        setWhenReady(MaxTick);
        setRefCount(0);
        setSrcRequestorId(Request::invldRequestorId);
        lockList.clear();
    }

    /**
     * Sets the corresponding coherence bits.
     *
     * @param bits The coherence bits to be set.
     */
    void
    setCoherenceBits(unsigned bits)
    {
        assert(isValid());
        coherence |= bits;
    }

    /**
     * Clear the corresponding coherence bits.
     *
     * @param bits The coherence bits to be cleared.
     */
    void clearCoherenceBits(unsigned bits) { coherence &= ~bits; }

    /**
     * Checks the given coherence bits are set.
     *
     * @return True if the block is readable.
     */
    bool
    isSet(unsigned bits) const
    {
        return isValid() && (coherence & bits);
    }

    /**
     * Check if this block was the result of a hardware prefetch, yet to
     * be touched.
     * @return True if the block was a hardware prefetch, unaccesed.
     */
    bool wasPrefetched() const { return _prefetched; }

    /**
     * Clear the prefetching bit. Either because it was recently used, or due
     * to the block being invalidated.
     */
    void clearPrefetched() { _prefetched = false; }

    /** Marks this blocks as a recently prefetched block. */
    void setPrefetched() { _prefetched = true; }

    /**
     * Get tick at which block's data will be available for access.
     *
     * @return Data ready tick.
     */
    Tick getWhenReady() const
    {
        assert(whenReady != MaxTick);
        return whenReady;
    }

    /**
     * Set tick at which block's data will be available for access. The new
     * tick must be chronologically sequential with respect to previous
     * accesses.
     *
     * @param tick New data ready tick.
     */
    void setWhenReady(const Tick tick)
    {
        assert(tick >= _tickInserted);
        whenReady = tick;
    }

    /** Get the task id associated to this block. */
    uint32_t getTaskId() const { return _taskId; }

    /** Get the requestor id associated to this block. */
    uint32_t getSrcRequestorId() const { return _srcRequestorId; }

    /** Getter for _partitionId */
    uint64_t getPartitionId() const { return _partitionId; }

    /** Get the number of references to this block since insertion. */
    unsigned getRefCount() const { return _refCount; }

    /** Get the number of references to this block since insertion. */
    void increaseRefCount() { _refCount++; }

    /**
     * Get the block's age, that is, the number of ticks since its insertion.
     *
     * @return The block's age.
     */
    Tick
    getAge() const
    {
        assert(_tickInserted <= curTick());
        return curTick() - _tickInserted;
    }

    /**
     * Set member variables when a block insertion occurs. Resets reference
     * count to 1 (the insertion counts as a reference), and touch block if
     * it hadn't been touched previously. Sets the insertion tick to the
     * current tick. Marks the block valid.
     *
     * @param tag Block address tag.
     * @param is_secure Whether the block is in secure space or not.
     * @param src_requestor_ID The source requestor ID.
     * @param task_ID The new task ID.
     * @param partition_id The source partition ID.
     */
    void insert(const KeyType &tag,
        const int src_requestor_ID, const uint32_t task_ID,
        const uint64_t partition_id);
    using TaggedEntry::insert;

    /**
     * Track the fact that a local locked was issued to the
     * block. Invalidate any previous LL to the same address.
     */
    void trackLoadLocked(PacketPtr pkt)
    {
        assert(pkt->isLLSC());
        auto l = lockList.begin();
        while (l != lockList.end()) {
            if (l->intersects(pkt->req))
                l = lockList.erase(l);
            else
                ++l;
        }

        lockList.emplace_front(pkt->req);
    }

    /**
     * Clear the any load lock that intersect the request, and is from
     * a different context.
     */
    void clearLoadLocks(const RequestPtr &req)
    {
        auto l = lockList.begin();
        while (l != lockList.end()) {
            if (l->intersects(req) && l->contextId != req->contextId()) {
                l = lockList.erase(l);
            } else {
                ++l;
            }
        }
    }

    /**
     * Pretty-print tag, set and way, and interpret state bits to readable form
     * including mapping to a MOESI state.
     *
     * @return string with basic state information
     */
    std::string
    print() const override
    {
        /**
         *  state       M   O   E   S   I
         *  writable    1   0   1   0   0
         *  dirty       1   1   0   0   0
         *  valid       1   1   1   1   0
         *
         *  state   writable    dirty   valid
         *  M       1           1       1
         *  O       0           1       1
         *  E       1           0       1
         *  S       0           0       1
         *  I       0           0       0
         *
         * Note that only one cache ever has a block in Modified or
         * Owned state, i.e., only one cache owns the block, or
         * equivalently has the DirtyBit bit set. However, multiple
         * caches on the same path to memory can have a block in the
         * Exclusive state (despite the name). Exclusive means this
         * cache has the only copy at this level of the hierarchy,
         * i.e., there may be copies in caches above this cache (in
         * various states), but there are no peers that have copies on
         * this branch of the hierarchy, and no caches at or above
         * this level on any other branch have copies either.
         **/
        unsigned state =
            isSet(WritableBit) << 2 | isSet(DirtyBit) << 1 | isValid();
        char s = '?';
        switch (state) {
          case 0b111: s = 'M'; break;
          case 0b011: s = 'O'; break;
          case 0b101: s = 'E'; break;
          case 0b001: s = 'S'; break;
          case 0b000: s = 'I'; break;
          default:    s = 'T'; break; // @TODO add other types
        }
        return csprintf("state: %x (%c) writable: %d readable: %d "
            "dirty: %d prefetched: %d | %s", coherence, s,
            isSet(WritableBit), isSet(ReadableBit), isSet(DirtyBit),
            wasPrefetched(), TaggedEntry::print());
    }

    /**
     * Handle interaction of load-locked operations and stores.
     * @return True if write should proceed, false otherwise.  Returns
     * false only in the case of a failed store conditional.
     */
    bool checkWrite(PacketPtr pkt)
    {
        assert(pkt->isWrite());

        // common case
        if (!pkt->isLLSC() && lockList.empty())
            return true;

        const RequestPtr &req = pkt->req;

        if (pkt->isLLSC()) {
            // it's a store conditional... have to check for matching
            // load locked.
            bool success = false;

            auto l = lockList.begin();
            while (!success && l != lockList.end()) {
                if (l->matches(pkt->req)) {
                    // it's a store conditional, and as far as the
                    // memory system can tell, the requesting
                    // context's lock is still valid.
                    success = true;
                    lockList.erase(l);
                } else {
                    ++l;
                }
            }

            req->setExtraData(success ? 1 : 0);
            // clear any intersected locks from other contexts (our LL
            // should already have cleared them)
            clearLoadLocks(req);
            return success;
        } else {
            // a normal write, if there is any lock not from this
            // context we clear the list, thus for a private cache we
            // never clear locks on normal writes
            clearLoadLocks(req);
            return true;
        }
    }

  protected:
    /** The current coherence status of this block. @sa CoherenceBits */
    unsigned coherence;

    // The following setters have been marked as protected because their
    // respective variables should only be modified at 2 moments:
    // invalidation and insertion. Because of that, they shall only be
    // called by the functions that perform those actions.

    /** Set the task id value. */
    void setTaskId(const uint32_t task_id) { _taskId = task_id; }

    /** Set the source requestor id. */
    void setSrcRequestorId(const uint32_t id) { _srcRequestorId = id; }

    /** Setter for _partitionId */
    void
    setPartitionId(const uint64_t partitionId) { _partitionId = partitionId; }

    /** Set the number of references to this block since insertion. */
    void setRefCount(const unsigned count) { _refCount = count; }

    /** Set the current tick as this block's insertion tick. */
    void setTickInserted() { _tickInserted = curTick(); }

  private:
    /** Task Id associated with this block */
    uint32_t _taskId = 0;

    /** holds the source requestor ID for this block. */
    int _srcRequestorId = 0;

    /** Partition ID of the activity that allocated this block */
    /* This ID is used to enforce resource partitioning policies */
    uint64_t _partitionId;

    /** Number of references to this block since it was brought in. */
    unsigned _refCount = 0;

    /**
     * Tick on which the block was inserted in the cache. Its value is only
     * meaningful if the block is valid.
     */
    Tick _tickInserted = 0;

    /** Whether this block is an unaccessed hardware prefetch. */
    bool _prefetched = 0;
};

/**
 * Special instance of CacheBlk for use with tempBlk that deals with its
 * block address regeneration.
 * @sa Cache
 */
class TempCacheBlk final : public CacheBlk
{
  private:
    /**
     * Copy of the block's address, used to regenerate tempBlock's address.
     */
    Addr _addr;

  public:
    /**
     * Creates a temporary cache block, with its own storage.
     * @param size The size (in bytes) of this cache block.
     */
    TempCacheBlk(unsigned size, TaggedIndexingPolicy *ip) : CacheBlk()
    {
        data = new uint8_t[size];

        setIndexingPolicy(ip);
    }
    TempCacheBlk(const TempCacheBlk&) = delete;
    using CacheBlk::operator=;
    TempCacheBlk& operator=(const TempCacheBlk&) = delete;
    ~TempCacheBlk() { delete [] data; };

    /**
     * Invalidate the block and clear all state.
     */
    void invalidate() override {
        CacheBlk::invalidate();

        _addr = MaxAddr;
    }

    void
    insert(const KeyType &tag) override
    {
        CacheBlk::insert(tag);
        _addr = tag.address;
    }

    /**
     * Get block's address.
     *
     * @return addr Address value.
     */
    Addr getAddr() const
    {
        return _addr;
    }
};

/**
 * Simple class to provide virtual print() method on cache blocks
 * without allocating a vtable pointer for every single cache block.
 * Just wrap the CacheBlk object in an instance of this before passing
 * to a function that requires a Printable object.
 */
class CacheBlkPrintWrapper : public Printable
{
    CacheBlk *blk;
  public:
    CacheBlkPrintWrapper(CacheBlk *_blk) : blk(_blk) {}
    virtual ~CacheBlkPrintWrapper() {}
    void print(std::ostream &o, int verbosity = 0,
               const std::string &prefix = "") const;
};

} // namespace gem5

#endif //__MEM_CACHE_CACHE_BLK_HH__
