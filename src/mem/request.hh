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
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
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
#include "cpu/inst_seq.hh"
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

    enum : FlagsType {
        /**
         * Architecture specific flags.
         *
         * These bits int the flag field are reserved for
         * architecture-specific code. For example, SPARC uses them to
         * represent ASIs.
         */
        ARCH_BITS                   = 0x000000FF,
        /** The request was an instruction fetch. */
        INST_FETCH                  = 0x00000100,
        /** The virtual address is also the physical address. */
        PHYSICAL                    = 0x00000200,
        /**
         * The request is to an uncacheable address.
         *
         * @note Uncacheable accesses may be reordered by CPU models. The
         * STRICT_ORDER flag should be set if such reordering is
         * undesirable.
         */
        UNCACHEABLE                = 0x00000400,
        /**
         * The request is required to be strictly ordered by <i>CPU
         * models</i> and is non-speculative.
         *
         * A strictly ordered request is guaranteed to never be
         * re-ordered or executed speculatively by a CPU model. The
         * memory system may still reorder requests in caches unless
         * the UNCACHEABLE flag is set as well.
         */
        STRICT_ORDER                = 0x00000800,
        /** This request is to a memory mapped register. */
        MMAPPED_IPR                 = 0x00002000,
        /** This request is made in privileged mode. */
        PRIVILEGED                  = 0x00008000,

        /**
         * This is a write that is targeted and zeroing an entire
         * cache block.  There is no need for a read/modify/write
         */
        CACHE_BLOCK_ZERO            = 0x00010000,

        /** The request should not cause a memory access. */
        NO_ACCESS                   = 0x00080000,
        /**
         * This request will lock or unlock the accessed memory. When
         * used with a load, the access locks the particular chunk of
         * memory. When used with a store, it unlocks. The rule is
         * that locked accesses have to be made up of a locked load,
         * some operation on the data, and then a locked store.
         */
        LOCKED_RMW                  = 0x00100000,
        /** The request is a Load locked/store conditional. */
        LLSC                        = 0x00200000,
        /** This request is for a memory swap. */
        MEM_SWAP                    = 0x00400000,
        MEM_SWAP_COND               = 0x00800000,

        /** The request is a prefetch. */
        PREFETCH                    = 0x01000000,
        /** The request should be prefetched into the exclusive state. */
        PF_EXCLUSIVE                = 0x02000000,
        /** The request should be marked as LRU. */
        EVICT_NEXT                  = 0x04000000,
        /** The request should be marked with ACQUIRE. */
        ACQUIRE                     = 0x00020000,
        /** The request should be marked with RELEASE. */
        RELEASE                     = 0x00040000,

        /** The request is an atomic that returns data. */
        ATOMIC_RETURN_OP            = 0x40000000,
        /** The request is an atomic that does not return data. */
        ATOMIC_NO_RETURN_OP         = 0x80000000,

        /** The request should be marked with KERNEL.
          * Used to indicate the synchronization associated with a GPU kernel
          * launch or completion.
          */
        KERNEL                      = 0x00001000,

        /**
         * The request should be handled by the generic IPR code (only
         * valid together with MMAPPED_IPR)
         */
        GENERIC_IPR                 = 0x08000000,

        /** The request targets the secure memory space. */
        SECURE                      = 0x10000000,
        /** The request is a page table walk */
        PT_WALK                     = 0x20000000,

        /**
         * These flags are *not* cleared when a Request object is
         * reused (assigned a new address).
         */
        STICKY_FLAGS = INST_FETCH
    };

    /** Master Ids that are statically allocated
     * @{*/
    enum : MasterID {
        /** This master id is used for writeback requests by the caches */
        wbMasterId = 0,
        /**
         * This master id is used for functional requests that
         * don't come from a particular device
         */
        funcMasterId = 1,
        /** This master id is used for message signaled interrupts */
        intMasterId = 2,
        /**
         * Invalid master id for assertion checking only. It is
         * invalid behavior to ever send this id as part of a request.
         */
        invldMasterId = std::numeric_limits<MasterID>::max()
    };
    /** @} */

    typedef uint32_t MemSpaceConfigFlagsType;
    typedef ::Flags<MemSpaceConfigFlagsType> MemSpaceConfigFlags;

    enum : MemSpaceConfigFlagsType {
        /** Has a synchronization scope been set? */
        SCOPE_VALID            = 0x00000001,
        /** Access has Wavefront scope visibility */
        WAVEFRONT_SCOPE        = 0x00000002,
        /** Access has Workgroup scope visibility */
        WORKGROUP_SCOPE        = 0x00000004,
        /** Access has Device (e.g., GPU) scope visibility */
        DEVICE_SCOPE           = 0x00000008,
        /** Access has System (e.g., CPU + GPU) scope visibility */
        SYSTEM_SCOPE           = 0x00000010,

        /** Global Segment */
        GLOBAL_SEGMENT         = 0x00000020,
        /** Group Segment */
        GROUP_SEGMENT          = 0x00000040,
        /** Private Segment */
        PRIVATE_SEGMENT        = 0x00000080,
        /** Kergarg Segment */
        KERNARG_SEGMENT        = 0x00000100,
        /** Readonly Segment */
        READONLY_SEGMENT       = 0x00000200,
        /** Spill Segment */
        SPILL_SEGMENT          = 0x00000400,
        /** Arg Segment */
        ARG_SEGMENT            = 0x00000800,
    };

  private:
    typedef uint8_t PrivateFlagsType;
    typedef ::Flags<PrivateFlagsType> PrivateFlags;

    enum : PrivateFlagsType {
        /** Whether or not the size is valid. */
        VALID_SIZE           = 0x00000001,
        /** Whether or not paddr is valid (has been written yet). */
        VALID_PADDR          = 0x00000002,
        /** Whether or not the vaddr & asid are valid. */
        VALID_VADDR          = 0x00000004,
        /** Whether or not the instruction sequence number is valid. */
        VALID_INST_SEQ_NUM   = 0x00000008,
        /** Whether or not the pc is valid. */
        VALID_PC             = 0x00000010,
        /** Whether or not the context ID is valid. */
        VALID_CONTEXT_ID     = 0x00000020,
        /** Whether or not the sc result is valid. */
        VALID_EXTRA_DATA     = 0x00000080,
        /**
         * These flags are *not* cleared when a Request object is reused
         * (assigned a new address).
         */
        STICKY_PRIVATE_FLAGS = VALID_CONTEXT_ID
    };

  private:

    /**
     * Set up a physical (e.g. device) request in a previously
     * allocated Request object.
     */
    void
    setPhys(Addr paddr, unsigned size, Flags flags, MasterID mid, Tick time)
    {
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
    unsigned _size;

    /** The requestor ID which is unique in the system for all ports
     * that are capable of issuing a transaction
     */
    MasterID _masterId;

    /** Flag structure for the request. */
    Flags _flags;

    /** Memory space configuraiton flag structure for the request. */
    MemSpaceConfigFlags _memSpaceConfigFlags;

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

    /** The context ID (for statistics, locks, and wakeups). */
    ContextID _contextId;

    /** program counter of initiating access; for tracing/debugging */
    Addr _pc;

    /** Sequence number of the instruction that creates the request */
    InstSeqNum _reqInstSeqNum;

    /** A pointer to an atomic operation */
    AtomicOpFunctor *atomicOpFunctor;

  public:

    /**
     * Minimal constructor. No fields are initialized. (Note that
     *  _flags and privateFlags are cleared by Flags default
     *  constructor.)
     */
    Request()
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(0),
          _reqInstSeqNum(0), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {}

    Request(Addr paddr, unsigned size, Flags flags, MasterID mid,
            InstSeqNum seq_num, ContextID cid)
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(0),
          _reqInstSeqNum(seq_num), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {
        setPhys(paddr, size, flags, mid, curTick());
        setContext(cid);
        privateFlags.set(VALID_INST_SEQ_NUM);
    }

    /**
     * Constructor for physical (e.g. device) requests.  Initializes
     * just physical address, size, flags, and timestamp (to curTick()).
     * These fields are adequate to perform a request.
     */
    Request(Addr paddr, unsigned size, Flags flags, MasterID mid)
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(0),
          _reqInstSeqNum(0), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {
        setPhys(paddr, size, flags, mid, curTick());
    }

    Request(Addr paddr, unsigned size, Flags flags, MasterID mid, Tick time)
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(0),
          _reqInstSeqNum(0), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {
        setPhys(paddr, size, flags, mid, time);
    }

    Request(Addr paddr, unsigned size, Flags flags, MasterID mid, Tick time,
            Addr pc)
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(pc),
          _reqInstSeqNum(0), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {
        setPhys(paddr, size, flags, mid, time);
        privateFlags.set(VALID_PC);
    }

    Request(int asid, Addr vaddr, unsigned size, Flags flags, MasterID mid,
            Addr pc, ContextID cid)
        : _paddr(0), _size(0), _masterId(invldMasterId), _time(0),
          _taskId(ContextSwitchTaskId::Unknown), _asid(0), _vaddr(0),
          _extraData(0), _contextId(0), _pc(0),
          _reqInstSeqNum(0), atomicOpFunctor(nullptr), translateDelta(0),
          accessDelta(0), depth(0)
    {
        setVirt(asid, vaddr, size, flags, mid, pc);
        setContext(cid);
    }

    Request(int asid, Addr vaddr, unsigned size, Flags flags, MasterID mid,
            Addr pc, ContextID cid, AtomicOpFunctor *atomic_op)
        : atomicOpFunctor(atomic_op)
    {
        setVirt(asid, vaddr, size, flags, mid, pc);
        setContext(cid);
    }

    ~Request()
    {
        if (hasAtomicOpFunctor()) {
            delete atomicOpFunctor;
        }
    }

    /**
     * Set up Context numbers.
     */
    void
    setContext(ContextID context_id)
    {
        _contextId = context_id;
        privateFlags.set(VALID_CONTEXT_ID);
    }

    /**
     * Set up a virtual (e.g., CPU) request in a previously
     * allocated Request object.
     */
    void
    setVirt(int asid, Addr vaddr, unsigned size, Flags flags, MasterID mid,
            Addr pc)
    {
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
        req1 = new Request(*this);
        req2 = new Request(*this);
        req1->_size = split_addr - _vaddr;
        req2->_vaddr = split_addr;
        req2->_size = _size - req1->_size;
    }

    /**
     * Accessor for paddr.
     */
    bool
    hasPaddr() const
    {
        return privateFlags.isSet(VALID_PADDR);
    }

    Addr
    getPaddr() const
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
    mutable int depth;

    /**
     *  Accessor for size.
     */
    bool
    hasSize() const
    {
        return privateFlags.isSet(VALID_SIZE);
    }

    unsigned
    getSize() const
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

    /**
     * Accessor for atomic-op functor.
     */
    bool
    hasAtomicOpFunctor()
    {
        return atomicOpFunctor != NULL;
    }

    AtomicOpFunctor *
    getAtomicOpFunctor()
    {
        assert(atomicOpFunctor != NULL);
        return atomicOpFunctor;
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
    setMemSpaceConfigFlags(MemSpaceConfigFlags extraFlags)
    {
        assert(privateFlags.isSet(VALID_PADDR | VALID_VADDR));
        _memSpaceConfigFlags.set(extraFlags);
    }

    /** Accessor function for vaddr.*/
    bool
    hasVaddr() const
    {
        return privateFlags.isSet(VALID_VADDR);
    }

    Addr
    getVaddr() const
    {
        assert(privateFlags.isSet(VALID_VADDR));
        return _vaddr;
    }

    /** Accesssor for the requestor id. */
    MasterID
    masterId() const
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
    getAsid() const
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
    getArchFlags() const
    {
        assert(privateFlags.isSet(VALID_PADDR|VALID_VADDR));
        return _flags & ARCH_BITS;
    }

    /** Accessor function to check if sc result is valid. */
    bool
    extraDataValid() const
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
    ContextID
    contextId() const
    {
        assert(privateFlags.isSet(VALID_CONTEXT_ID));
        return _contextId;
    }

    void
    setPC(Addr pc)
    {
        privateFlags.set(VALID_PC);
        _pc = pc;
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
    void incAccessDepth() const { depth++; }
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

    /**
     * Accessor for the sequence number of instruction that creates the
     * request.
     */
    bool
    hasInstSeqNum() const
    {
        return privateFlags.isSet(VALID_INST_SEQ_NUM);
    }

    InstSeqNum
    getReqInstSeqNum() const
    {
        assert(privateFlags.isSet(VALID_INST_SEQ_NUM));
        return _reqInstSeqNum;
    }

    void
    setReqInstSeqNum(const InstSeqNum seq_num)
    {
        privateFlags.set(VALID_INST_SEQ_NUM);
        _reqInstSeqNum = seq_num;
    }

    /** Accessor functions for flags. Note that these are for testing
        only; setting flags should be done via setFlags(). */
    bool isUncacheable() const { return _flags.isSet(UNCACHEABLE); }
    bool isStrictlyOrdered() const { return _flags.isSet(STRICT_ORDER); }
    bool isInstFetch() const { return _flags.isSet(INST_FETCH); }
    bool isPrefetch() const { return _flags.isSet(PREFETCH); }
    bool isLLSC() const { return _flags.isSet(LLSC); }
    bool isPriv() const { return _flags.isSet(PRIVILEGED); }
    bool isLockedRMW() const { return _flags.isSet(LOCKED_RMW); }
    bool isSwap() const { return _flags.isSet(MEM_SWAP|MEM_SWAP_COND); }
    bool isCondSwap() const { return _flags.isSet(MEM_SWAP_COND); }
    bool isMmappedIpr() const { return _flags.isSet(MMAPPED_IPR); }
    bool isSecure() const { return _flags.isSet(SECURE); }
    bool isPTWalk() const { return _flags.isSet(PT_WALK); }
    bool isAcquire() const { return _flags.isSet(ACQUIRE); }
    bool isRelease() const { return _flags.isSet(RELEASE); }
    bool isKernel() const { return _flags.isSet(KERNEL); }
    bool isAtomicReturn() const { return _flags.isSet(ATOMIC_RETURN_OP); }
    bool isAtomicNoReturn() const { return _flags.isSet(ATOMIC_NO_RETURN_OP); }

    bool
    isAtomic() const
    {
        return _flags.isSet(ATOMIC_RETURN_OP) ||
               _flags.isSet(ATOMIC_NO_RETURN_OP);
    }

    /**
     * Accessor functions for the memory space configuration flags and used by
     * GPU ISAs such as the Heterogeneous System Architecture (HSA). Note that
     * these are for testing only; setting extraFlags should be done via
     * setMemSpaceConfigFlags().
     */
    bool isScoped() const { return _memSpaceConfigFlags.isSet(SCOPE_VALID); }

    bool
    isWavefrontScope() const
    {
        assert(isScoped());
        return _memSpaceConfigFlags.isSet(WAVEFRONT_SCOPE);
    }

    bool
    isWorkgroupScope() const
    {
        assert(isScoped());
        return _memSpaceConfigFlags.isSet(WORKGROUP_SCOPE);
    }

    bool
    isDeviceScope() const
    {
        assert(isScoped());
        return _memSpaceConfigFlags.isSet(DEVICE_SCOPE);
    }

    bool
    isSystemScope() const
    {
        assert(isScoped());
        return _memSpaceConfigFlags.isSet(SYSTEM_SCOPE);
    }

    bool
    isGlobalSegment() const
    {
        return _memSpaceConfigFlags.isSet(GLOBAL_SEGMENT) ||
               (!isGroupSegment() && !isPrivateSegment() &&
                !isKernargSegment() && !isReadonlySegment() &&
                !isSpillSegment() && !isArgSegment());
    }

    bool
    isGroupSegment() const
    {
        return _memSpaceConfigFlags.isSet(GROUP_SEGMENT);
    }

    bool
    isPrivateSegment() const
    {
        return _memSpaceConfigFlags.isSet(PRIVATE_SEGMENT);
    }

    bool
    isKernargSegment() const
    {
        return _memSpaceConfigFlags.isSet(KERNARG_SEGMENT);
    }

    bool
    isReadonlySegment() const
    {
        return _memSpaceConfigFlags.isSet(READONLY_SEGMENT);
    }

    bool
    isSpillSegment() const
    {
        return _memSpaceConfigFlags.isSet(SPILL_SEGMENT);
    }

    bool
    isArgSegment() const
    {
        return _memSpaceConfigFlags.isSet(ARG_SEGMENT);
    }
};

#endif // __MEM_REQUEST_HH__
