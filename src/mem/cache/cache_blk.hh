/*
 * Copyright (c) 2012-2018 ARM Limited
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
#include <list>
#include <string>

#include "base/printable.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

/**
 * Cache block status bit assignments
 */
enum CacheBlkStatusBits : unsigned {
    /** valid, readable */
    BlkValid =          0x01,
    /** write permission */
    BlkWritable =       0x02,
    /** read permission (yes, block can be valid but not readable) */
    BlkReadable =       0x04,
    /** dirty (modified) */
    BlkDirty =          0x08,
    /** block was a hardware prefetch yet unaccessed*/
    BlkHWPrefetched =   0x20,
    /** block holds data from the secure memory space */
    BlkSecure =         0x40,
    /** block holds compressed data */
    BlkCompressed =     0x80
};

/**
 * A Basic Cache block.
 * Contains the tag, status, and a pointer to data.
 */
class CacheBlk : public ReplaceableEntry
{
  public:
    /** Task Id associated with this block */
    uint32_t task_id;

    /** Data block tag value. */
    Addr tag;
    /**
     * Contains a copy of the data in this block for easy access. This is used
     * for efficient execution when the data could be actually stored in
     * another format (COW, compressed, sub-blocked, etc). In all cases the
     * data stored here should be kept consistant with the actual data
     * referenced by this block.
     */
    uint8_t *data;

    /** block state: OR of CacheBlkStatusBit */
    typedef unsigned State;

    /** The current status of this block. @sa CacheBlockStatusBits */
    State status;

    /**
     * Which curTick() will this block be accessible. Its value is only
     * meaningful if the block is valid.
     */
    Tick whenReady;

    /** Number of references to this block since it was brought in. */
    unsigned refCount;

    /** holds the source requestor ID for this block. */
    int srcRequestorId;

    /**
     * Tick on which the block was inserted in the cache. Its value is only
     * meaningful if the block is valid.
     */
    Tick tickInserted;

  protected:
    /**
     * Represents that the indicated thread context has a "lock" on
     * the block, in the LL/SC sense.
     */
    class Lock {
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
    CacheBlk() : data(nullptr), tickInserted(0)
    {
        invalidate();
    }

    CacheBlk(const CacheBlk&) = delete;
    CacheBlk& operator=(const CacheBlk&) = delete;
    virtual ~CacheBlk() {};

    /**
     * Checks the write permissions of this block.
     * @return True if the block is writable.
     */
    bool isWritable() const
    {
        const State needed_bits = BlkWritable | BlkValid;
        return (status & needed_bits) == needed_bits;
    }

    /**
     * Checks the read permissions of this block.  Note that a block
     * can be valid but not readable if there is an outstanding write
     * upgrade miss.
     * @return True if the block is readable.
     */
    bool isReadable() const
    {
        const State needed_bits = BlkReadable | BlkValid;
        return (status & needed_bits) == needed_bits;
    }

    /**
     * Checks that a block is valid.
     * @return True if the block is valid.
     */
    bool isValid() const
    {
        return (status & BlkValid) != 0;
    }

    /**
     * Invalidate the block and clear all state.
     */
    virtual void invalidate()
    {
        tag = MaxAddr;
        task_id = ContextSwitchTaskId::Unknown;
        status = 0;
        whenReady = MaxTick;
        refCount = 0;
        srcRequestorId = Request::invldRequestorId;
        lockList.clear();
    }

    /**
     * Check to see if a block has been written.
     * @return True if the block is dirty.
     */
    bool isDirty() const
    {
        return (status & BlkDirty) != 0;
    }

    /**
     * Check if this block was the result of a hardware prefetch, yet to
     * be touched.
     * @return True if the block was a hardware prefetch, unaccesed.
     */
    bool wasPrefetched() const
    {
        return (status & BlkHWPrefetched) != 0;
    }

    /**
     * Check if this block holds data from the secure memory space.
     * @return True if the block holds data from the secure memory space.
     */
    bool isSecure() const
    {
        return (status & BlkSecure) != 0;
    }

    /**
     * Set valid bit.
     */
    virtual void setValid()
    {
        assert(!isValid());
        status |= BlkValid;
    }

    /**
     * Set secure bit.
     */
    virtual void setSecure()
    {
        status |= BlkSecure;
    }

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
        assert(tick >= tickInserted);
        whenReady = tick;
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
     */
    virtual void insert(const Addr tag, const bool is_secure,
                        const int src_requestor_ID, const uint32_t task_ID);

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
         * equivalently has the BlkDirty bit set. However, multiple
         * caches on the same path to memory can have a block in the
         * Exclusive state (despite the name). Exclusive means this
         * cache has the only copy at this level of the hierarchy,
         * i.e., there may be copies in caches above this cache (in
         * various states), but there are no peers that have copies on
         * this branch of the hierarchy, and no caches at or above
         * this level on any other branch have copies either.
         **/
        unsigned state = isWritable() << 2 | isDirty() << 1 | isValid();
        char s = '?';
        switch (state) {
          case 0b111: s = 'M'; break;
          case 0b011: s = 'O'; break;
          case 0b101: s = 'E'; break;
          case 0b001: s = 'S'; break;
          case 0b000: s = 'I'; break;
          default:    s = 'T'; break; // @TODO add other types
        }
        return csprintf("state: %x (%c) valid: %d writable: %d readable: %d "
                        "dirty: %d | tag: %#x %s", status, s,
                        isValid(), isWritable(), isReadable(), isDirty(), tag,
                        ReplaceableEntry::print());
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
    TempCacheBlk(unsigned size) : CacheBlk()
    {
        data = new uint8_t[size];
    }
    TempCacheBlk(const TempCacheBlk&) = delete;
    TempCacheBlk& operator=(const TempCacheBlk&) = delete;
    ~TempCacheBlk() { delete [] data; };

    /**
     * Invalidate the block and clear all state.
     */
    void invalidate() override {
        CacheBlk::invalidate();

        _addr = MaxAddr;
    }

    void insert(const Addr addr, const bool is_secure,
                const int src_requestor_ID=0, const uint32_t task_ID=0)
                override
    {
        // Make sure that the block has been properly invalidated
        assert(status == 0);

        // Set block address
        _addr = addr;

        // Set secure state
        if (is_secure) {
            setSecure();
        }

        // Validate block
        setValid();
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

#endif //__MEM_CACHE_CACHE_BLK_HH__
