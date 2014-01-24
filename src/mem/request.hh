/*
 * Copyright (c) 2012-2013 ARM Limited
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
#include <climits>

#include "base/flags.hh"
#include "base/misc.hh"
#include "base/types.hh"
#include "sim/core.hh"

/**
 * Special TaskIds that are used for per-context-switch stats dumps
 * and Cache Occupancy. Having too many tasks seems to be a problem
 * with vector stats. 1024 seems to be a reasonable number that
 * doesn't cause a problem with stats and is large enough to realistic
 * benchmarks (Linux/Android boot, BBench, etc.)
 */

namespace ContextSwitchTaskId {
    enum TaskId {
        MaxNormalTaskId = 1021, /* Maximum number of normal tasks */
        Prefetcher = 1022, /* For cache lines brought in by prefetcher */
        DMA = 1023, /* Mostly Table Walker */
        Unknown = 1024,
        NumTaskId
    };
}

class Request;

typedef Request* RequestPtr;
typedef uint16_t MasterID;

class Request
{
  public:
    typedef uint32_t FlagsType;
    typedef uint8_t ArchFlagsType;
    typedef ::Flags<FlagsType> Flags;

    /**
     * Architecture specific flags.
     *
     * These bits int the flag field are reserved for
     * architecture-specific code. For example, SPARC uses them to
     * represent ASIs.
     */
    static const FlagsType ARCH_BITS                   = 0x000000FF;
    /** The request was an instruction fetch. */
    static const FlagsType INST_FETCH                  = 0x00000100;
    /** The virtual address is also the physical address. */
    static const FlagsType PHYSICAL                    = 0x00000200;
    /** The request is an ALPHA VPTE pal access (hw_ld). */
    static const FlagsType VPTE                        = 0x00000400;
    /** Use the alternate mode bits in ALPHA. */
    static const FlagsType ALTMODE                     = 0x00000800;
    /** The request is to an uncacheable address. */
    static const FlagsType UNCACHEABLE                 = 0x00001000;
    /** This request is to a memory mapped register. */
    static const FlagsType MMAPPED_IPR                  = 0x00002000;
    /** This request is a clear exclusive. */
    static const FlagsType CLEAR_LL                    = 0x00004000;
    /** This request is made in privileged mode. */
    static const FlagsType PRIVILEGED                   = 0x00008000;

    /** The request should not cause a memory access. */
    static const FlagsType NO_ACCESS                   = 0x00080000;
    /** This request will lock or unlock the accessed memory. When used with
     * a load, the access locks the particular chunk of memory. When used
     * with a store, it unlocks. The rule is that locked accesses have to be
     * made up of a locked load, some operation on the data, and then a locked
     * store.
     */
    static const FlagsType LOCKED                      = 0x00100000;
    /** The request is a Load locked/store conditional. */
    static const FlagsType LLSC                        = 0x00200000;
    /** This request is for a memory swap. */
    static const FlagsType MEM_SWAP                    = 0x00400000;
    static const FlagsType MEM_SWAP_COND               = 0x00800000;

    /** The request is a prefetch. */
    static const FlagsType PREFETCH                    = 0x01000000;
    /** The request should be prefetched into the exclusive state. */
    static const FlagsType PF_EXCLUSIVE                = 0x02000000;
    /** The request should be marked as LRU. */
    static const FlagsType EVICT_NEXT                  = 0x04000000;

    /** The request should be handled by the generic IPR code (only
     * valid together with MMAPPED_IPR) */
    static const FlagsType GENERIC_IPR                 = 0x08000000;

    /** The request targets the secure memory space. */
    static const FlagsType SECURE                      = 0x10000000;
    /** The request is a page table walk */
    static const FlagsType PT_WALK                     = 0x20000000;

    /** These flags are *not* cleared when a Request object is reused
       (assigned a new address). */
    static const FlagsType STICKY_FLAGS = INST_FETCH;

    /** Request Ids that are statically allocated
     * @{*/
    /** This request id is used for writeback requests by the caches */
    static const MasterID wbMasterId = 0;
    /** This request id is used for functional requests that don't come from a
     * particular device
     */
    static const MasterID funcMasterId = 1;
    /** This request id is used for message signaled interrupts */
    static const MasterID intMasterId = 2;
    /** Invalid request id for assertion checking only. It is invalid behavior
     * to ever send this id as part of a request.
     * @todo C++1x replace with numeric_limits when constexpr is added  */
    static const MasterID invldMasterId = USHRT_MAX;
    /** @} */

    /** Invalid or unknown Pid. Possible when operating system is not present
     *  or has not assigned a pid yet */
    static const uint32_t invldPid = UINT_MAX;

  private:
    typedef uint8_t PrivateFlagsType;
    typedef ::Flags<PrivateFlagsType> PrivateFlags;

    /** Whether or not the size is valid. */
    static const PrivateFlagsType VALID_SIZE           = 0x00000001;
    /** Whether or not paddr is valid (has been written yet). */
    static const PrivateFlagsType VALID_PADDR          = 0x00000002;
    /** Whether or not the vaddr & asid are valid. */
    static const PrivateFlagsType VALID_VADDR          = 0x00000004;
    /** Whether or not the pc is valid. */
    static const PrivateFlagsType VALID_PC             = 0x00000010;
    /** Whether or not the context ID is valid. */
    static const PrivateFlagsType VALID_CONTEXT_ID     = 0x00000020;
    static const PrivateFlagsType VALID_THREAD_ID      = 0x00000040;
    /** Whether or not the sc result is valid. */
    static const PrivateFlagsType VALID_EXTRA_DATA     = 0x00000080;

    /** These flags are *not* cleared when a Request object is reused
       (assigned a new address). */
    static const PrivateFlagsType STICKY_PRIVATE_FLAGS =
        VALID_CONTEXT_ID | VALID_THREAD_ID;

  private:
    /**
     * The physical address of the request. Valid only if validPaddr
     * is set.
     */
    Addr _paddr;

    /**
     * The size of the request. This field must be set when vaddr or
     * paddr is written via setVirt() or setPhys(), so it is always
     * valid as long as one of the address fields is valid.
     */
    int _size;

    /** The requestor ID which is unique in the system for all ports
     * that are capable of issuing a transaction
     */
    MasterID _masterId;

    /** Flag structure for the request. */
    Flags _flags;

    /** Private flags for field validity checking. */
    PrivateFlags privateFlags;

    /**
     * The time this request was started. Used to calculate
     * latencies. This field is set to curTick() any time paddr or vaddr
     * is written.
     */
    Tick _time;

    /**
     * The task id associated with this request
     */
    uint32_t _taskId;

    /** The address space ID. */
    int _asid;

    /** The virtual address of the request. */
    Addr _vaddr;

    /**
     * Extra data for the request, such as the return value of
     * store conditional or the compare value for a CAS. */
    uint64_t _extraData;

    /** The context ID (for statistics, typically). */
    int _contextId;
    /** The thread ID (id within this CPU) */
    int _threadId;

    /** program counter of initiating access; for tracing/debugging */
    Addr _pc;

  public:
    /** Minimal constructor.  No fields are initialized. 
     *  (Note that _flags and privateFlags are cleared by Flags
     *  default constructor.)
     */
    Request()
        : _taskId(ContextSwitchTaskId::Unknown),
        translateDelta(0), accessDelta(0), depth(0)
    {}

    /**
     * Constructor for physical (e.g. device) requests.  Initializes
     * just physical address, size, flags, and timestamp (to curTick()).
     * These fields are adequate to perform a request. 
     */
    Request(Addr paddr, int size, Flags flags, MasterID mid)
        : _taskId(ContextSwitchTaskId::Unknown)
    {
        setPhys(paddr, size, flags, mid);
    }

    Request(Addr paddr, int size, Flags flags, MasterID mid, Tick time)
        : _taskId(ContextSwitchTaskId::Unknown)
    {
        setPhys(paddr, size, flags, mid, time);
    }

    Request(Addr paddr, int size, Flags flags, MasterID mid, Tick time, Addr pc)
        : _taskId(ContextSwitchTaskId::Unknown)
    {
        setPhys(paddr, size, flags, mid, time);
        privateFlags.set(VALID_PC);
        _pc = pc;
    }

    Request(int asid, Addr vaddr, int size, Flags flags, MasterID mid, Addr pc,
            int cid, ThreadID tid)
        : _taskId(ContextSwitchTaskId::Unknown)
    {
        setVirt(asid, vaddr, size, flags, mid, pc);
        setThreadContext(cid, tid);
    }

    ~Request() {}

    /**
     * Set up CPU and thread numbers.
     */
    void
    setThreadContext(int context_id, ThreadID tid)
    {
        _contextId = context_id;
        _threadId = tid;
        privateFlags.set(VALID_CONTEXT_ID|VALID_THREAD_ID);
    }

    /**
     * Set up a physical (e.g. device) request in a previously
     * allocated Request object.
     */
    void
    setPhys(Addr paddr, int size, Flags flags, MasterID mid, Tick time)
    {
        assert(size >= 0);
        _paddr = paddr;
        _size = size;
        _time = time;
        _masterId = mid;
        _flags.clear(~STICKY_FLAGS);
        _flags.set(flags);
        privateFlags.clear(~STICKY_PRIVATE_FLAGS);
        privateFlags.set(VALID_PADDR|VALID_SIZE);
        depth = 0;
        accessDelta = 0;
        //translateDelta = 0;
    }

    void
    setPhys(Addr paddr, int size, Flags flags, MasterID mid)
    {
        setPhys(paddr, size, flags, mid, curTick());
    }

    /**
     * Set up a virtual (e.g., CPU) request in a previously
     * allocated Request object.
     */
    void
    setVirt(int asid, Addr vaddr, int size, Flags flags, MasterID mid, Addr pc)
    {
        assert(size >= 0);
        _asid = asid;
        _vaddr = vaddr;
        _size = size;
        _masterId = mid;
        _pc = pc;
        _time = curTick();

        _flags.clear(~STICKY_FLAGS);
        _flags.set(flags);
        privateFlags.clear(~STICKY_PRIVATE_FLAGS);
        privateFlags.set(VALID_VADDR|VALID_SIZE|VALID_PC);
        depth = 0;
        accessDelta = 0;
        translateDelta = 0;
    }

    /**
     * Set just the physical address.  This usually used to record the
     * result of a translation. However, when using virtualized CPUs
     * setPhys() is sometimes called to finalize a physical address
     * without a virtual address, so we can't check if the virtual
     * address is valid.
     */
    void
    setPaddr(Addr paddr)
    {
        _paddr = paddr;
        privateFlags.set(VALID_PADDR);
    }

    /**
     * Generate two requests as if this request had been split into two
     * pieces. The original request can't have been translated already.
     */
    void splitOnVaddr(Addr split_addr, RequestPtr &req1, RequestPtr &req2)
    {
        assert(privateFlags.isSet(VALID_VADDR));
        assert(privateFlags.noneSet(VALID_PADDR));
        assert(split_addr > _vaddr && split_addr < _vaddr + _size);
        req1 = new Request;
        *req1 = *this;
        req2 = new Request;
        *req2 = *this;
        req1->_size = split_addr - _vaddr;
        req2->_vaddr = split_addr;
        req2->_size = _size - req1->_size;
    }

    /**
     * Accessor for paddr.
     */
    bool
    hasPaddr()
    {
        return privateFlags.isSet(VALID_PADDR);
    }

    Addr
    getPaddr()
    {
        assert(privateFlags.isSet(VALID_PADDR));
        return _paddr;
    }

    /**
     * Time for the TLB/table walker to successfully translate this request.
     */
    Tick translateDelta;

    /**
     * Access latency to complete this memory transaction not including
     * translation time.
     */
    Tick accessDelta;

    /**
     * Level of the cache hierachy where this request was responded to
     * (e.g. 0 = L1; 1 = L2).
     */
    int depth;

    /**
     *  Accessor for size.
     */
    bool
    hasSize()
    {
        return privateFlags.isSet(VALID_SIZE);
    }

    int
    getSize()
    {
        assert(privateFlags.isSet(VALID_SIZE));
        return _size;
    }

    /** Accessor for time. */
    Tick
    time() const
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        return _time;
    }

    void
    time(Tick time)
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        _time = time;
    }

    /** Accessor for flags. */
    Flags
    getFlags()
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        return _flags;
    }

    /** Note that unlike other accessors, this function sets *specific
       flags* (ORs them in); it does not assign its argument to the
       _flags field.  Thus this method should rightly be called
       setFlags() and not just flags(). */
    void
    setFlags(Flags flags)
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        _flags.set(flags);
    }

    void
    setArchFlags(Flags flags)
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        _flags.set(flags & ARCH_BITS);
    }

    /** Accessor function for vaddr.*/
    Addr
    getVaddr()
    {
        assert(privateFlags.isSet(VALID_VADDR));
        return _vaddr;
    }

    /** Accesssor for the requestor id. */
    MasterID
    masterId()
    {
        return _masterId;
    }

    uint32_t
    taskId() const
    {
        return _taskId;
    }

    void
    taskId(uint32_t id) {
        _taskId = id;
    }

    /** Accessor function for asid.*/
    int
    getAsid()
    {
        assert(privateFlags.isSet(VALID_VADDR));
        return _asid;
    }

    /** Accessor function for asid.*/
    void
    setAsid(int asid)
    {
        _asid = asid;
    }

    /** Accessor function for architecture-specific flags.*/
    ArchFlagsType
    getArchFlags()
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        return _flags & ARCH_BITS;
    }

    /** Accessor function to check if sc result is valid. */
    bool
    extraDataValid()
    {
        return privateFlags.isSet(VALID_EXTRA_DATA);
    }

    /** Accessor function for store conditional return value.*/
    uint64_t
    getExtraData() const
    {
        assert(privateFlags.isSet(VALID_EXTRA_DATA));
        return _extraData;
    }

    /** Accessor function for store conditional return value.*/
    void
    setExtraData(uint64_t extraData)
    {
        _extraData = extraData;
        privateFlags.set(VALID_EXTRA_DATA);
    }

    bool
    hasContextId() const
    {
        return privateFlags.isSet(VALID_CONTEXT_ID);
    }

    /** Accessor function for context ID.*/
    int
    contextId() const
    {
        assert(privateFlags.isSet(VALID_CONTEXT_ID));
        return _contextId;
    }

    /** Accessor function for thread ID. */
    int
    threadId() const
    {
        assert(privateFlags.isSet(VALID_THREAD_ID));
        return _threadId;
    }

    bool
    hasPC() const
    {
        return privateFlags.isSet(VALID_PC);
    }

    /** Accessor function for pc.*/
    Addr
    getPC() const
    {
        assert(privateFlags.isSet(VALID_PC));
        return _pc;
    }

    /**
     * Increment/Get the depth at which this request is responded to.
     * This currently happens when the request misses in any cache level.
     */
    void incAccessDepth() { depth++; }
    int getAccessDepth() const { return depth; }

    /**
     * Set/Get the time taken for this request to be successfully translated.
     */
    void setTranslateLatency() { translateDelta = curTick() - _time; }
    Tick getTranslateLatency() const { return translateDelta; }

    /**
     * Set/Get the time taken to complete this request's access, not including
     *  the time to successfully translate the request.
     */
    void setAccessLatency() { accessDelta = curTick() - _time - translateDelta; }
    Tick getAccessLatency() const { return accessDelta; }

    /** Accessor functions for flags.  Note that these are for testing
       only; setting flags should be done via setFlags(). */
    bool isUncacheable() const { return _flags.isSet(UNCACHEABLE); }
    bool isInstFetch() const { return _flags.isSet(INST_FETCH); }
    bool isPrefetch() const { return _flags.isSet(PREFETCH); }
    bool isLLSC() const { return _flags.isSet(LLSC); }
    bool isPriv() const { return _flags.isSet(PRIVILEGED); }
    bool isLocked() const { return _flags.isSet(LOCKED); }
    bool isSwap() const { return _flags.isSet(MEM_SWAP|MEM_SWAP_COND); }
    bool isCondSwap() const { return _flags.isSet(MEM_SWAP_COND); }
    bool isMmappedIpr() const { return _flags.isSet(MMAPPED_IPR); }
    bool isClearLL() const { return _flags.isSet(CLEAR_LL); }
    bool isSecure() const { return _flags.isSet(SECURE); }
    bool isPTWalk() const { return _flags.isSet(PT_WALK); }
};

#endif // __MEM_REQUEST_HH__
