/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 *          Steve Reinhardt
 *          Ali Saidi
 */

/**
 * @file
 * Declaration of a request, the overall memory request consisting of
 the parts of the request that are persistent throughout the transaction.
 */

#ifndef __MEM_REQUEST_HH__
#define __MEM_REQUEST_HH__

#include <cassert>

#include "base/fast_alloc.hh"
#include "base/flags.hh"
#include "base/misc.hh"
#include "sim/host.hh"
#include "sim/core.hh"

class Request;

typedef Request* RequestPtr;

class Request : public FastAlloc
{
    friend class Packet;

  public:
    typedef uint32_t FlagsType;
    typedef ::Flags<FlagsType> Flags;

    /** ASI information for this request if it exists. */
    static const FlagsType ASI_BITS                    = 0x000000FF;
    /** The request is a Load locked/store conditional. */
    static const FlagsType LLSC                        = 0x00000100;
    /** The virtual address is also the physical address. */
    static const FlagsType PHYSICAL                    = 0x00000200;
    /** The request is an ALPHA VPTE pal access (hw_ld). */
    static const FlagsType VPTE                        = 0x00000400;
    /** Use the alternate mode bits in ALPHA. */
    static const FlagsType ALTMODE                     = 0x00000800;
    /** The request is to an uncacheable address. */
    static const FlagsType UNCACHEABLE                 = 0x00001000;
    /** The request should not cause a page fault. */
    static const FlagsType NO_FAULT                    = 0x00002000;
    /** The request should not cause a memory access. */
    static const FlagsType NO_ACCESS                   = 0x00004000;
    /** This request will lock or unlock the accessed memory. When used with
     * a load, the access locks the particular chunk of memory. When used
     * with a store, it unlocks. The rule is that locked accesses have to be
     * made up of a locked load, some operation on the data, and then a locked
     * store.
     */
    static const FlagsType LOCKED                      = 0x00008000;
    /** The request should be prefetched into the exclusive state. */
    static const FlagsType PF_EXCLUSIVE                = 0x00010000;
    /** The request should be marked as LRU. */
    static const FlagsType EVICT_NEXT                  = 0x00020000;
    /** The request should ignore unaligned access faults */
    static const FlagsType NO_ALIGN_FAULT              = 0x00040000;
    /** The request was an instruction read. */
    static const FlagsType INST_READ                   = 0x00080000;
    /** This request is for a memory swap. */
    static const FlagsType MEM_SWAP                    = 0x00100000;
    static const FlagsType MEM_SWAP_COND               = 0x00200000;
    /** The request should ignore unaligned access faults */
    static const FlagsType NO_HALF_WORD_ALIGN_FAULT    = 0x00400000;
    /** This request is to a memory mapped register. */
    static const FlagsType MMAPED_IPR                  = 0x00800000;

  private:
    static const FlagsType PUBLIC_FLAGS                = 0x00FFFFFF;
    static const FlagsType PRIVATE_FLAGS               = 0xFF000000;

    /** Whether or not the size is valid. */
    static const FlagsType VALID_SIZE                  = 0x01000000;
    /** Whether or not paddr is valid (has been written yet). */
    static const FlagsType VALID_PADDR                 = 0x02000000;
    /** Whether or not the vaddr & asid are valid. */
    static const FlagsType VALID_VADDR                 = 0x04000000;
    /** Whether or not the pc is valid. */
    static const FlagsType VALID_PC                    = 0x10000000;
    /** Whether or not the context ID is valid. */
    static const FlagsType VALID_CONTEXT_ID            = 0x20000000;
    static const FlagsType VALID_THREAD_ID             = 0x40000000;
    /** Whether or not the sc result is valid. */
    static const FlagsType VALID_EXTRA_DATA            = 0x80000000;

  private:
    /**
     * The physical address of the request. Valid only if validPaddr
     * is set.
     */
    Addr paddr;

    /**
     * The size of the request. This field must be set when vaddr or
     * paddr is written via setVirt() or setPhys(), so it is always
     * valid as long as one of the address fields is valid.
     */
    int size;

    /** Flag structure for the request. */
    Flags flags;

    /**
     * The time this request was started. Used to calculate
     * latencies. This field is set to curTick any time paddr or vaddr
     * is written.
     */
    Tick time;

    /** The address space ID. */
    int asid;

    /** The virtual address of the request. */
    Addr vaddr;

    /**
     * Extra data for the request, such as the return value of
     * store conditional or the compare value for a CAS. */
    uint64_t extraData;

    /** The context ID (for statistics, typically). */
    int _contextId;
    /** The thread ID (id within this CPU) */
    int _threadId;

    /** program counter of initiating access; for tracing/debugging */
    Addr pc;

  public:
    /** Minimal constructor.  No fields are initialized. */
    Request()
    {}

    /**
     * Constructor for physical (e.g. device) requests.  Initializes
     * just physical address, size, flags, and timestamp (to curTick).
     * These fields are adequate to perform a request. 
     */
    Request(Addr paddr, int size, Flags flags)
    {
        setPhys(paddr, size, flags);
    }

    Request(int asid, Addr vaddr, int size, Flags flags, Addr pc,
            int cid, int tid)
    {
        setThreadContext(cid, tid);
        setVirt(asid, vaddr, size, flags, pc);
    }

    ~Request() {}  // for FastAlloc

    /**
     * Set up CPU and thread numbers.
     */
    void
    setThreadContext(int context_id, int thread_id)
    {
        _contextId = context_id;
        _threadId = thread_id;
        flags.set(VALID_CONTEXT_ID|VALID_THREAD_ID);
    }

    /**
     * Set up a physical (e.g. device) request in a previously
     * allocated Request object.
     */
    void
    setPhys(Addr _paddr, int _size, Flags _flags)
    {
        assert(_size >= 0);
        paddr = _paddr;
        size = _size;
        time = curTick;

        flags.set(VALID_PADDR|VALID_SIZE);
        flags.clear(VALID_VADDR|VALID_PC|VALID_EXTRA_DATA|MMAPED_IPR);
        flags.update(_flags, PUBLIC_FLAGS);
    }

    /**
     * Set up a virtual (e.g., CPU) request in a previously
     * allocated Request object.
     */
    void
    setVirt(int _asid, Addr _vaddr, int _size, Flags _flags, Addr _pc)
    {
        assert(_size >= 0);
        asid = _asid;
        vaddr = _vaddr;
        size = _size;
        pc = _pc;
        time = curTick;

        flags.set(VALID_VADDR|VALID_SIZE|VALID_PC);
        flags.clear(VALID_PADDR|VALID_EXTRA_DATA|MMAPED_IPR);
        flags.update(_flags, PUBLIC_FLAGS);
    }

    /**
     * Set just the physical address.  This should only be used to
     * record the result of a translation, and thus the vaddr must be
     * valid before this method is called.  Otherwise, use setPhys()
     * to guarantee that the size and flags are also set.
     */
    void
    setPaddr(Addr _paddr)
    {
        assert(flags.isSet(VALID_VADDR));
        paddr = _paddr;
        flags.set(VALID_PADDR);
    }

    /**
     * Generate two requests as if this request had been split into two
     * pieces. The original request can't have been translated already.
     */
    void splitOnVaddr(Addr split_addr, RequestPtr &req1, RequestPtr &req2)
    {
        assert(flags.isSet(VALID_VADDR));
        assert(flags.noneSet(VALID_PADDR));
        assert(split_addr > vaddr && split_addr < vaddr + size);
        req1 = new Request;
        *req1 = *this;
        req2 = new Request;
        *req2 = *this;
        req1->size = split_addr - vaddr;
        req2->vaddr = split_addr;
        req2->size = size - req1->size;
    }

    /**
     * Accessor for paddr.
     */
    Addr
    getPaddr()
    {
        assert(flags.isSet(VALID_PADDR));
        return paddr;
    }

    /**
     *  Accessor for size.
     */
    int
    getSize()
    {
        assert(flags.isSet(VALID_SIZE));
        return size;
    }

    /** Accessor for time. */
    Tick
    getTime()
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        return time;
    }

    void
    setTime(Tick when)
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        time = when;
    }

    /** Accessor for flags. */
    Flags
    getFlags()
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        return flags & PUBLIC_FLAGS;
    }

    Flags
    anyFlags(Flags _flags)
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        assert(_flags.noneSet(~PUBLIC_FLAGS));
        return flags.isSet(_flags);
    }

    Flags
    allFlags(Flags _flags)
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        assert(_flags.noneSet(~PUBLIC_FLAGS));
        return flags.allSet(_flags);
    }

    /** Accessor for flags. */
    void
    setFlags(Flags _flags)
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        assert(_flags.noneSet(~PUBLIC_FLAGS));
        flags.set(_flags);
    }

    void
    clearFlags(Flags _flags)
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        assert(_flags.noneSet(~PUBLIC_FLAGS));
        flags.clear(_flags);
    }

    void
    clearFlags()
    {
        assert(flags.isSet(VALID_PADDR|VALID_VADDR));
        flags.clear(PUBLIC_FLAGS);
    }

    /** Accessor function for vaddr.*/
    Addr
    getVaddr()
    {
        assert(flags.isSet(VALID_VADDR));
        return vaddr;
    }

    /** Accessor function for asid.*/
    int
    getAsid()
    {
        assert(flags.isSet(VALID_VADDR));
        return asid;
    }

    /** Accessor function for asi.*/
    uint8_t
    getAsi()
    {
        assert(flags.isSet(VALID_VADDR));
        return flags & ASI_BITS;
    }

    /** Accessor function for asi.*/
    void
    setAsi(uint8_t a)
    {
        assert(flags.isSet(VALID_VADDR));
        flags.update(a, ASI_BITS);
    }

    /** Accessor function for asi.*/
    bool
    isMmapedIpr()
    {
        assert(flags.isSet(VALID_PADDR));
        return flags.isSet(MMAPED_IPR);
    }

    /** Accessor function for asi.*/
    void
    setMmapedIpr(bool r)
    {
        assert(VALID_VADDR);
        flags.set(MMAPED_IPR);
    }

    /** Accessor function to check if sc result is valid. */
    bool
    extraDataValid()
    {
        return flags.isSet(VALID_EXTRA_DATA);
    }

    /** Accessor function for store conditional return value.*/
    uint64_t
    getExtraData() const
    {
        assert(flags.isSet(VALID_EXTRA_DATA));
        return extraData;
    }

    /** Accessor function for store conditional return value.*/
    void
    setExtraData(uint64_t _extraData)
    {
        extraData = _extraData;
        flags.set(VALID_EXTRA_DATA);
    }

    bool
    hasContextId() const
    {
        return flags.isSet(VALID_CONTEXT_ID);
    }

    /** Accessor function for context ID.*/
    int
    contextId() const
    {
        assert(flags.isSet(VALID_CONTEXT_ID));
        return _contextId;
    }

    /** Accessor function for thread ID. */
    int
    threadId() const
    {
        assert(flags.isSet(VALID_THREAD_ID));
        return _threadId;
    }

    bool
    hasPC() const
    {
        return flags.isSet(VALID_PC);
    }

    /** Accessor function for pc.*/
    Addr
    getPC() const
    {
        assert(flags.isSet(VALID_PC));
        return pc;
    }

    /** Accessor Function to Check Cacheability. */
    bool isUncacheable() const { return flags.isSet(UNCACHEABLE); }
    bool isInstRead() const { return flags.isSet(INST_READ); }
    bool isLLSC() const { return flags.isSet(LLSC); }
    bool isLocked() const { return flags.isSet(LOCKED); }
    bool isSwap() const { return flags.isSet(MEM_SWAP|MEM_SWAP_COND); }
    bool isCondSwap() const { return flags.isSet(MEM_SWAP_COND); }

    bool
    isMisaligned() const
    {
        if (flags.isSet(NO_ALIGN_FAULT))
            return false;

        if ((vaddr & 0x1))
            return true;

        if (flags.isSet(NO_HALF_WORD_ALIGN_FAULT))
            return false;

        if ((vaddr & 0x2))
            return true;

        return false;
    }
};

#endif // __MEM_REQUEST_HH__
