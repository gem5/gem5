/*
 * Copyright (c) 2012-2013 ARM Limited
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
 *
 * Authors: Erik Hallnor
 *          Andreas Sandberg
 */

/** @file
 * Definitions of a simple cache block class.
 */

#ifndef __CACHE_BLK_HH__
#define __CACHE_BLK_HH__

#include <list>

#include "base/printable.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/core.hh"          // for Tick

/**
 * Cache block status bit assignments
 */
enum CacheBlkStatusBits {
    /** valid, readable */
    BlkValid =          0x01,
    /** write permission */
    BlkWritable =       0x02,
    /** read permission (yes, block can be valid but not readable) */
    BlkReadable =       0x04,
    /** dirty (modified) */
    BlkDirty =          0x08,
    /** block was referenced */
    BlkReferenced =     0x10,
    /** block was a hardware prefetch yet unaccessed*/
    BlkHWPrefetched =   0x20,
    /** block holds data from the secure memory space */
    BlkSecure =         0x40
};

/**
 * A Basic Cache block.
 * Contains the tag, status, and a pointer to data.
 */
class CacheBlk
{
  public:
    /** Task Id associated with this block */
    uint32_t task_id;

    /** The address space ID of this block. */
    int asid;
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
    /** the number of bytes stored in this block. */
    int size;

    /** block state: OR of CacheBlkStatusBit */
    typedef unsigned State;

    /** The current status of this block. @sa CacheBlockStatusBits */
    State status;

    /** Which curTick() will this block be accessable */
    Tick whenReady;

    /**
     * The set this block belongs to.
     * @todo Move this into subclasses when we fix CacheTags to use them.
     */
    int set;

    /** whether this block has been touched */
    bool isTouched;

    /** Number of references to this block since it was brought in. */
    int refCount;

    /** holds the source requestor ID for this block. */
    int srcMasterId;

    Tick tickInserted;

  protected:
    /**
     * Represents that the indicated thread context has a "lock" on
     * the block, in the LL/SC sense.
     */
    class Lock {
      public:
        int contextId;     // locking context
        Addr lowAddr;      // low address of lock range
        Addr highAddr;     // high address of lock range

        // check for matching execution context
        bool matchesContext(Request *req)
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() -1;
            return (contextId == req->contextId()) &&
                   (req_low >= lowAddr) && (req_high <= highAddr);
        }

        bool overlapping(Request *req)
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() - 1;

            return (req_low <= highAddr) && (req_high >= lowAddr);
        }

        Lock(Request *req)
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

    CacheBlk()
        : task_id(ContextSwitchTaskId::Unknown),
          asid(-1), tag(0), data(0) ,size(0), status(0), whenReady(0),
          set(-1), isTouched(false), refCount(0),
          srcMasterId(Request::invldMasterId),
          tickInserted(0)
    {}

    /**
     * Copy the state of the given block into this one.
     * @param rhs The block to copy.
     * @return a const reference to this block.
     */
    const CacheBlk& operator=(const CacheBlk& rhs)
    {
        asid = rhs.asid;
        tag = rhs.tag;
        data = rhs.data;
        size = rhs.size;
        status = rhs.status;
        whenReady = rhs.whenReady;
        set = rhs.set;
        refCount = rhs.refCount;
        task_id = rhs.task_id;
        return *this;
    }

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
    void invalidate()
    {
        status = 0;
        isTouched = false;
        clearLoadLocks();
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
     * Check if this block has been referenced.
     * @return True if the block has been referenced.
     */
    bool isReferenced() const
    {
        return (status & BlkReferenced) != 0;
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
     * Track the fact that a local locked was issued to the block.  If
     * multiple LLs get issued from the same context we could have
     * redundant records on the list, but that's OK, as they'll all
     * get blown away at the next store.
     */
    void trackLoadLocked(PacketPtr pkt)
    {
        assert(pkt->isLLSC());
        lockList.push_front(Lock(pkt->req));
    }

    /**
     * Clear the list of valid load locks.  Should be called whenever
     * block is written to or invalidated.
     */
    void clearLoadLocks(Request *req = NULL)
    {
        if (!req) {
            // No request, invaldate all locks to this line
            lockList.clear();
        } else {
            // Only invalidate locks that overlap with this request
            std::list<Lock>::iterator lock_itr = lockList.begin();
            while (lock_itr != lockList.end()) {
                if (lock_itr->overlapping(req)) {
                    lock_itr = lockList.erase(lock_itr);
                } else {
                    ++lock_itr;
                }
            }
        }
    }

    /**
     * Pretty-print a tag, and interpret state bits to readable form
     * including mapping to a MOESI stat.
     *
     * @return string with basic state information
     */
    std::string print() const
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
                        "dirty: %d tag: %x data: %x", status, s, isValid(),
                        isWritable(), isReadable(), isDirty(), tag, *data);
    }

    /**
     * Handle interaction of load-locked operations and stores.
     * @return True if write should proceed, false otherwise.  Returns
     * false only in the case of a failed store conditional.
     */
    bool checkWrite(PacketPtr pkt)
    {
        Request *req = pkt->req;
        if (pkt->isLLSC()) {
            // it's a store conditional... have to check for matching
            // load locked.
            bool success = false;

            for (std::list<Lock>::iterator i = lockList.begin();
                 i != lockList.end(); ++i)
            {
                if (i->matchesContext(req)) {
                    // it's a store conditional, and as far as the memory
                    // system can tell, the requesting context's lock is
                    // still valid.
                    success = true;
                    break;
                }
            }

            req->setExtraData(success ? 1 : 0);
            clearLoadLocks(req);
            return success;
        } else {
            // for *all* stores (conditional or otherwise) we have to
            // clear the list of load-locks as they're all invalid now.
            clearLoadLocks(req);
            return true;
        }
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

/**
 * Wrap a method and present it as a cache block visitor.
 *
 * For example the forEachBlk method in the tag arrays expects a
 * callable object/function as their parameter. This class wraps a
 * method in an object and presents  callable object that adheres to
 * the cache block visitor protocol.
 */
template <typename T, typename BlkType>
class CacheBlkVisitorWrapper
{
  public:
    typedef bool (T::*visitorPtr)(BlkType &blk);

    CacheBlkVisitorWrapper(T &_obj, visitorPtr _visitor)
        : obj(_obj), visitor(_visitor) {}

    bool operator()(BlkType &blk) {
        return (obj.*visitor)(blk);
    }

  private:
    T &obj;
    visitorPtr visitor;
};

/**
 * Cache block visitor that determines if there are dirty blocks in a
 * cache.
 *
 * Use with the forEachBlk method in the tag array to determine if the
 * array contains dirty blocks.
 */
template <typename BlkType>
class CacheBlkIsDirtyVisitor
{
  public:
    CacheBlkIsDirtyVisitor()
        : _isDirty(false) {}

    bool operator()(BlkType &blk) {
        if (blk.isDirty()) {
            _isDirty = true;
            return false;
        } else {
            return true;
        }
    }

    /**
     * Does the array contain a dirty line?
     *
     * \return true if yes, false otherwise.
     */
    bool isDirty() const { return _isDirty; };

  private:
    bool _isDirty;
};

#endif //__CACHE_BLK_HH__
