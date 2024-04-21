/*
 * Copyright (c) 2012-2013,2017-2022 Arm Limited
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
 */

/**
 * @file
 * Declaration of a request, the overall memory request consisting of
 the parts of the request that are persistent throughout the transaction.
 */

#ifndef __MEM_REQUEST_HH__
#define __MEM_REQUEST_HH__

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "base/amo.hh"
#include "base/compiler.hh"
#include "base/extensible.hh"
#include "base/flags.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "mem/htm.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

/**
 * Special TaskIds that are used for per-context-switch stats dumps
 * and Cache Occupancy. Having too many tasks seems to be a problem
 * with vector stats. 1024 seems to be a reasonable number that
 * doesn't cause a problem with stats and is large enough to realistic
 * benchmarks (Linux/Android boot, BBench, etc.)
 */
namespace context_switch_task_id
{
    enum TaskId
    {
        MaxNormalTaskId = 1021, /* Maximum number of normal tasks */
        Prefetcher = 1022, /* For cache lines brought in by prefetcher */
        DMA = 1023, /* Mostly Table Walker */
        Unknown = 1024,
        NumTaskId
    };
}

class Packet;
class Request;
class ThreadContext;

typedef std::shared_ptr<Request> RequestPtr;
typedef uint16_t RequestorID;

class Request : public Extensible<Request>
{
  public:
    typedef uint64_t FlagsType;
    typedef uint8_t ArchFlagsType;
    typedef gem5::Flags<FlagsType> Flags;

    enum : FlagsType
    {
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
        UNCACHEABLE                 = 0x00000400,
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
        /** This request is a read which will be followed by a write. */
        READ_MODIFY_WRITE           = 0x0020000000000000,

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

        /** The request targets the secure memory space. */
        SECURE                      = 0x10000000,
        /** The request is a page table walk */
        PT_WALK                     = 0x20000000,

        /** The request invalidates a memory location */
        INVALIDATE                  = 0x0000000100000000,
        /** The request cleans a memory location */
        CLEAN                       = 0x0000000200000000,

        /** The request targets the point of unification */
        DST_POU                     = 0x0000001000000000,

        /** The request targets the point of coherence */
        DST_POC                     = 0x0000002000000000,

        /** Bits to define the destination of a request */
        DST_BITS                    = 0x0000003000000000,

        /** hardware transactional memory **/

        /** The request starts a HTM transaction */
        HTM_START                   = 0x0000010000000000,

        /** The request commits a HTM transaction */
        HTM_COMMIT                  = 0x0000020000000000,

        /** The request cancels a HTM transaction */
        HTM_CANCEL                  = 0x0000040000000000,

        /** The request aborts a HTM transaction */
        HTM_ABORT                   = 0x0000080000000000,

        // What is the different between HTM cancel and abort?
        //
        // HTM_CANCEL will originate from a user instruction, e.g.
        // Arm's TCANCEL or x86's XABORT. This is an explicit request
        // to end a transaction and restore from the last checkpoint.
        //
        // HTM_ABORT is an internally generated request used to synchronize
        // a transaction's failure between the core and memory subsystem.
        // If a transaction fails in the core, e.g. because an instruction
        // within the transaction generates an exception, the core will prepare
        // itself to stop fetching/executing more instructions and send an
        // HTM_ABORT to the memory subsystem before restoring the checkpoint.
        // Similarly, the transaction could fail in the memory subsystem and
        // this will be communicated to the core via the Packet object.
        // Once the core notices, it will do the same as the above and send
        // a HTM_ABORT to the memory subsystem.
        // A HTM_CANCEL sent to the memory subsystem will ultimately return
        // to the core which in turn will send a HTM_ABORT.
        //
        // This separation is necessary to ensure the disjoint components
        // of the system work correctly together.

        /** The Request is a TLB shootdown */
        TLBI                        = 0x0000100000000000,

        /** The Request is a TLB shootdown sync */
        TLBI_SYNC                   = 0x0000200000000000,

        /** The Request tells the CPU model that a
            remote TLB Sync has been requested */
        TLBI_EXT_SYNC               = 0x0000400000000000,

        /** The Request tells the interconnect that a
            remote TLB Sync request has completed */
        TLBI_EXT_SYNC_COMP          = 0x0000800000000000,

        /**
         * These flags are *not* cleared when a Request object is
         * reused (assigned a new address).
         */
        STICKY_FLAGS = INST_FETCH,
        MEMTIME                     = 0x0001000000000000,
    };
    static const FlagsType STORE_NO_DATA = CACHE_BLOCK_ZERO |
        CLEAN | INVALIDATE;

    static const FlagsType HTM_CMD = HTM_START | HTM_COMMIT |
        HTM_CANCEL | HTM_ABORT;

    static const FlagsType TLBI_CMD = TLBI | TLBI_SYNC |
        TLBI_EXT_SYNC | TLBI_EXT_SYNC_COMP;

    /** Requestor Ids that are statically allocated
     * @{*/
    enum : RequestorID
    {
        /** This requestor id is used for writeback requests by the caches */
        wbRequestorId = 0,
        /**
         * This requestor id is used for functional requests that
         * don't come from a particular device
         */
        funcRequestorId = 1,
        /** This requestor id is used for message signaled interrupts */
        intRequestorId = 2,
        /**
         * Invalid requestor id for assertion checking only. It is
         * invalid behavior to ever send this id as part of a request.
         */
        invldRequestorId = std::numeric_limits<RequestorID>::max()
    };
    /** @} */

    typedef uint64_t CacheCoherenceFlagsType;
    typedef gem5::Flags<CacheCoherenceFlagsType> CacheCoherenceFlags;

    /**
     * These bits are used to set the coherence policy for the GPU and are
     * encoded in the Vega instructions. The Vega ISA defines two cache levels
     * See the AMD Vega ISA Architecture Manual for more details.
     *
     * INV_L1: L1 cache invalidation
     * FLUSH_L2: L2 cache flush
     *
     * Invalidation means to simply discard all cache contents. This can be
     * done in the L1 since it is implemented as a write-through cache and
     * there are other copies elsewhere in the hierarchy.
     *
     * For flush the contents of the cache need to be written back to memory
     * when dirty and can be discarded otherwise. This operation is more
     * involved than invalidation and therefore we do not flush caches with
     * redundant copies of data.
     *
     * SLC: System Level Coherent. Accesses are forced to miss in the L2 cache
     *      and are coherent with system memory.
     *
     * GLC: Globally Coherent. Controls how reads and writes are handled by
     *      the L1 cache. Global here referes to the data being visible
     *      globally on the GPU (i.e., visible to all WGs).
     *
     * For atomics, the GLC bit is used to distinguish between between atomic
     * return/no-return operations. These flags are used by GPUDynInst.
     */
    enum : CacheCoherenceFlagsType
    {
        /** mem_sync_op flags */
        I_CACHE_INV             = 0x00000001,
        INV_L1                  = I_CACHE_INV,
        V_CACHE_INV             = 0x00000002,
        K_CACHE_INV             = 0x00000004,
        GL1_CACHE_INV           = 0x00000008,
        K_CACHE_WB              = 0x00000010,
        FLUSH_L2                = 0x00000020,
        GL2_CACHE_INV           = 0x00000040,
        /** user-policy flags */
        SLC_BIT                 = 0x00000080,
        DLC_BIT                 = 0x00000100,
        GLC_BIT                 = 0x00000200,
        /** mtype flags */
        CACHED                  = 0x00000400,
        READ_WRITE              = 0x00000800,
        SHARED                  = 0x00001000,

    };

    using LocalAccessor =
        std::function<Cycles(ThreadContext *tc, Packet *pkt)>;

  private:
    typedef uint16_t PrivateFlagsType;
    typedef gem5::Flags<PrivateFlagsType> PrivateFlags;

    enum : PrivateFlagsType
    {
        /** Whether or not the size is valid. */
        VALID_SIZE           = 0x00000001,
        /** Whether or not paddr is valid (has been written yet). */
        VALID_PADDR          = 0x00000002,
        /** Whether or not the vaddr is valid. */
        VALID_VADDR          = 0x00000004,
        /** Whether or not the instruction sequence number is valid. */
        VALID_INST_SEQ_NUM   = 0x00000008,
        /** Whether or not the pc is valid. */
        VALID_PC             = 0x00000010,
        /** Whether or not the context ID is valid. */
        VALID_CONTEXT_ID     = 0x00000020,
        /** Whether or not the sc result is valid. */
        VALID_EXTRA_DATA     = 0x00000080,
        /** Whether or not the stream ID and substream ID is valid. */
        VALID_STREAM_ID      = 0x00000100,
        VALID_SUBSTREAM_ID   = 0x00000200,
        // hardware transactional memory
        /** Whether or not the abort cause is valid. */
        VALID_HTM_ABORT_CAUSE = 0x00000400,
        /** Whether or not the instruction count is valid. */
        VALID_INST_COUNT      = 0x00000800,
        /**
         * These flags are *not* cleared when a Request object is reused
         * (assigned a new address).
         */
        STICKY_PRIVATE_FLAGS = VALID_CONTEXT_ID
    };

  private:

    /**
     * The physical address of the request. Valid only if validPaddr
     * is set.
     */
    Addr _paddr = 0;

    /**
     * The size of the request. This field must be set when vaddr or
     * paddr is written via setVirt() or a phys basec constructor, so it is
     * always valid as long as one of the address fields is valid.
     */
    unsigned _size = 0;

    /** Byte-enable mask for writes. */
    std::vector<bool> _byteEnable;

    /** The requestor ID which is unique in the system for all ports
     * that are capable of issuing a transaction
     */
    RequestorID _requestorId = invldRequestorId;

    /** Flag structure for the request. */
    Flags _flags;

    /** Flags that control how downstream cache system maintains coherence*/
    CacheCoherenceFlags _cacheCoherenceFlags;

    /** Private flags for field validity checking. */
    PrivateFlags privateFlags;

    /**
     * The time this request was started. Used to calculate
     * latencies. This field is set to curTick() any time paddr or vaddr
     * is written.
     */
    Tick _time = MaxTick;

    /**
     * The task id associated with this request
     */
    uint32_t _taskId = context_switch_task_id::Unknown;

    /**
     * The stream ID uniquely identifies a device behind the
     * SMMU/IOMMU Each transaction arriving at the SMMU/IOMMU is
     * associated with exactly one stream ID.
     */
    uint32_t _streamId = 0;

    /**
     * The substream ID identifies an "execution context" within a
     * device behind an SMMU/IOMMU. It's intended to map 1-to-1 to
     * PCIe PASID (Process Address Space ID). The presence of a
     * substream ID is optional.
     */
    uint32_t _substreamId = 0;

    /**
     * For fullsystem GPU simulation, this determines if a requests
     * destination is system (host) memory or dGPU (device) memory.
     */
    bool _systemReq = 0;

    /** The virtual address of the request. */
    Addr _vaddr = MaxAddr;

    /**
     * Extra data for the request, such as the return value of
     * store conditional or the compare value for a CAS. */
    uint64_t _extraData = 0;

    /** The context ID (for statistics, locks, and wakeups). */
    ContextID _contextId = InvalidContextID;

    /** program counter of initiating access; for tracing/debugging */
    Addr _pc = MaxAddr;

    /** Sequence number of the instruction that creates the request */
    InstSeqNum _reqInstSeqNum = 0;

    /** A pointer to an atomic operation */
    AtomicOpFunctorPtr atomicOpFunctor = nullptr;

    LocalAccessor _localAccessor;

    /** The instruction count at the time this request is created */
    Counter _instCount = 0;

    /** The cause for HTM transaction abort */
    HtmFailureFaultCause _htmAbortCause = HtmFailureFaultCause::INVALID;

  public:

    /**
     * Minimal constructor. No fields are initialized. (Note that
     *  _flags and privateFlags are cleared by Flags default
     *  constructor.)
     */
    Request() {}

    /**
     * Constructor for physical (e.g. device) requests.  Initializes
     * just physical address, size, flags, and timestamp (to curTick()).
     * These fields are adequate to perform a request.
     */
    Request(Addr paddr, unsigned size, Flags flags, RequestorID id) :
        _paddr(paddr), _size(size), _requestorId(id), _time(curTick())
    {
        _flags.set(flags);
        privateFlags.set(VALID_PADDR|VALID_SIZE);
        _byteEnable = std::vector<bool>(size, true);
    }

    Request(Addr vaddr, unsigned size, Flags flags,
            RequestorID id, Addr pc, ContextID cid,
            AtomicOpFunctorPtr atomic_op=nullptr)
    {
        setVirt(vaddr, size, flags, id, pc, std::move(atomic_op));
        setContext(cid);
        _byteEnable = std::vector<bool>(size, true);
    }

    Request(const Request& other)
        : Extensible<Request>(other),
          _paddr(other._paddr), _size(other._size),
          _byteEnable(other._byteEnable),
          _requestorId(other._requestorId),
          _flags(other._flags),
          _cacheCoherenceFlags(other._cacheCoherenceFlags),
          privateFlags(other.privateFlags),
          _time(other._time),
          _taskId(other._taskId), _vaddr(other._vaddr),
          _extraData(other._extraData), _contextId(other._contextId),
          _pc(other._pc), _reqInstSeqNum(other._reqInstSeqNum),
          _localAccessor(other._localAccessor),
          translateDelta(other.translateDelta),
          accessDelta(other.accessDelta), depth(other.depth)
    {
        atomicOpFunctor.reset(other.atomicOpFunctor ?
                                other.atomicOpFunctor->clone() : nullptr);
    }

    ~Request() {}

    /**
     * Factory method for creating memory management requests, with
     * unspecified addr and size.
     */
    static RequestPtr
    createMemManagement(Flags flags, RequestorID id)
    {
        auto mgmt_req = std::make_shared<Request>();
        mgmt_req->_flags.set(flags);
        mgmt_req->_requestorId = id;
        mgmt_req->_time = curTick();

        assert(mgmt_req->isMemMgmt());
        return mgmt_req;
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

    void
    setStreamId(uint32_t sid)
    {
        _streamId = sid;
        privateFlags.set(VALID_STREAM_ID);
    }

    void
    setSubstreamId(uint32_t ssid)
    {
        assert(hasStreamId());
        _substreamId = ssid;
        privateFlags.set(VALID_SUBSTREAM_ID);
    }

    /**
     * Set up a virtual (e.g., CPU) request in a previously
     * allocated Request object.
     */
    void
    setVirt(Addr vaddr, unsigned size, Flags flags, RequestorID id, Addr pc,
            AtomicOpFunctorPtr amo_op=nullptr)
    {
        _vaddr = vaddr;
        _size = size;
        _requestorId = id;
        _pc = pc;
        _time = curTick();

        _flags.clear(~STICKY_FLAGS);
        _flags.set(flags);
        privateFlags.clear(~STICKY_PRIVATE_FLAGS);
        privateFlags.set(VALID_VADDR|VALID_SIZE|VALID_PC);
        depth = 0;
        accessDelta = 0;
        translateDelta = 0;
        atomicOpFunctor = std::move(amo_op);
        _localAccessor = nullptr;
    }

    /**
     * Set just the physical address. This usually used to record the
     * result of a translation.
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
    // TODO: this function is still required by TimingSimpleCPU - should be
    // removed once TimingSimpleCPU will support arbitrarily long multi-line
    // mem. accesses
    void splitOnVaddr(Addr split_addr, RequestPtr &req1, RequestPtr &req2)
    {
        assert(hasVaddr());
        assert(!hasPaddr());
        assert(split_addr > _vaddr && split_addr < _vaddr + _size);
        req1 = std::make_shared<Request>(*this);
        req2 = std::make_shared<Request>(*this);
        req1->_size = split_addr - _vaddr;
        req2->_vaddr = split_addr;
        req2->_size = _size - req1->_size;
        req1->_byteEnable = std::vector<bool>(
            _byteEnable.begin(),
            _byteEnable.begin() + req1->_size);
        req2->_byteEnable = std::vector<bool>(
            _byteEnable.begin() + req1->_size,
            _byteEnable.end());
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
        assert(hasPaddr());
        return _paddr;
    }

    /**
     * Accessor for instruction count.
     */
    bool
    hasInstCount() const
    {
      return privateFlags.isSet(VALID_INST_COUNT);
    }

    Counter getInstCount() const
    {
        assert(hasInstCount());
        return _instCount;
    }

    void setInstCount(Counter val)
    {
        privateFlags.set(VALID_INST_COUNT);
        _instCount = val;
    }

    /**
     * Time for the TLB/table walker to successfully translate this request.
     */
    Tick translateDelta = 0;

    /**
     * Access latency to complete this memory transaction not including
     * translation time.
     */
    Tick accessDelta = 0;

    /**
     * Level of the cache hierachy where this request was responded to
     * (e.g. 0 = L1; 1 = L2).
     */
    mutable int depth = 0;

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
        assert(hasSize());
        return _size;
    }

    const std::vector<bool>&
    getByteEnable() const
    {
        return _byteEnable;
    }

    void
    setByteEnable(const std::vector<bool>& be)
    {
        assert(be.size() == _size);
        _byteEnable = be;
    }

    /**
     * Returns true if the memory request is masked, which means
     * there is at least one byteEnable element which is false
     * (byte is masked)
     */
    bool
    isMasked() const
    {
        return std::find(
            _byteEnable.begin(),
            _byteEnable.end(),
            false) != _byteEnable.end();
    }

    /** Accessor for time. */
    Tick
    time() const
    {
        assert(hasPaddr() || hasVaddr());
        return _time;
    }

    /** Is this request for a local memory mapped resource/register? */
    bool isLocalAccess() { return (bool)_localAccessor; }
    /** Set the function which will enact that access. */
    void setLocalAccessor(LocalAccessor acc) { _localAccessor = acc; }
    /** Perform the installed local access. */
    Cycles
    localAccessor(ThreadContext *tc, Packet *pkt)
    {
        return _localAccessor(tc, pkt);
    }

    /**
     * Accessor for atomic-op functor.
     */
    bool
    hasAtomicOpFunctor()
    {
        return (bool)atomicOpFunctor;
    }

    AtomicOpFunctor *
    getAtomicOpFunctor()
    {
        assert(atomicOpFunctor);
        return atomicOpFunctor.get();
    }

    void
    setAtomicOpFunctor(AtomicOpFunctorPtr amo_op)
    {
        atomicOpFunctor = std::move(amo_op);
    }


    /**
     * Accessor for hardware transactional memory abort cause.
     */
    bool
    hasHtmAbortCause() const
    {
      return privateFlags.isSet(VALID_HTM_ABORT_CAUSE);
    }

    HtmFailureFaultCause
    getHtmAbortCause() const
    {
        assert(hasHtmAbortCause());
        return _htmAbortCause;
    }

    void
    setHtmAbortCause(HtmFailureFaultCause val)
    {
        assert(isHTMAbort());
        privateFlags.set(VALID_HTM_ABORT_CAUSE);
        _htmAbortCause = val;
    }

    /** Accessor for flags. */
    Flags
    getFlags()
    {
        assert(hasPaddr() || hasVaddr());
        return _flags;
    }

    /** Note that unlike other accessors, this function sets *specific
        flags* (ORs them in); it does not assign its argument to the
        _flags field.  Thus this method should rightly be called
        setFlags() and not just flags(). */
    void
    setFlags(Flags flags)
    {
        assert(hasPaddr() || hasVaddr());
        _flags.set(flags);
    }

    void
    clearFlags(Flags flags)
    {
        assert(hasPaddr() || hasVaddr());
        _flags.clear(flags);
    }

    void
    setCacheCoherenceFlags(CacheCoherenceFlags extraFlags)
    {
        // TODO: do mem_sync_op requests have valid paddr/vaddr?
        assert(hasPaddr() || hasVaddr());
        _cacheCoherenceFlags.set(extraFlags);
    }

    void
    clearCacheCoherenceFlags(CacheCoherenceFlags extraFlags)
    {
        // TODO: do mem_sync_op requests have valid paddr/vaddr?
        assert(hasPaddr() || hasVaddr());
        _cacheCoherenceFlags.clear(extraFlags);
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
    RequestorID
    requestorId() const
    {
        return _requestorId;
    }

    void
    requestorId(RequestorID rid)
    {
        _requestorId = rid;
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

    /** Accessor function for architecture-specific flags.*/
    ArchFlagsType
    getArchFlags() const
    {
        assert(hasPaddr() || hasVaddr());
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
        assert(extraDataValid());
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
        assert(hasContextId());
        return _contextId;
    }

    /* For GPU fullsystem mark this request is not to device memory. */
    void setSystemReq(bool sysReq) { _systemReq = sysReq; }
    bool systemReq() const { return _systemReq; }

    bool
    hasStreamId() const
    {
      return privateFlags.isSet(VALID_STREAM_ID);
    }

    uint32_t
    streamId() const
    {
        assert(hasStreamId());
        return _streamId;
    }

    bool
    hasSubstreamId() const
    {
        return privateFlags.isSet(VALID_SUBSTREAM_ID);
    }

    uint32_t
    substreamId() const
    {
        assert(hasSubstreamId());
        return _substreamId;
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
        assert(hasPC());
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
        assert(hasInstSeqNum());
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
    bool isMemtime() const { return _flags.isSet(MEMTIME); }
    bool
    isPrefetch() const
    {
        return (_flags.isSet(PREFETCH | PF_EXCLUSIVE));
    }
    bool isPrefetchEx() const { return _flags.isSet(PF_EXCLUSIVE); }
    bool isLLSC() const { return _flags.isSet(LLSC); }
    bool isPriv() const { return _flags.isSet(PRIVILEGED); }
    bool isLockedRMW() const { return _flags.isSet(LOCKED_RMW); }
    bool isSwap() const { return _flags.isSet(MEM_SWAP | MEM_SWAP_COND); }
    bool isCondSwap() const { return _flags.isSet(MEM_SWAP_COND); }
    bool
    isReadModifyWrite() const
    {
        return _flags.isSet(LOCKED_RMW | READ_MODIFY_WRITE);
    }
    bool isSecure() const { return _flags.isSet(SECURE); }
    bool isPTWalk() const { return _flags.isSet(PT_WALK); }
    bool isRelease() const { return _flags.isSet(RELEASE); }
    bool isKernel() const { return _flags.isSet(KERNEL); }
    bool isAtomicReturn() const { return _flags.isSet(ATOMIC_RETURN_OP); }
    bool isAtomicNoReturn() const { return _flags.isSet(ATOMIC_NO_RETURN_OP); }
    // hardware transactional memory
    bool isHTMStart() const { return _flags.isSet(HTM_START); }
    bool isHTMCommit() const { return _flags.isSet(HTM_COMMIT); }
    bool isHTMCancel() const { return _flags.isSet(HTM_CANCEL); }
    bool isHTMAbort() const { return _flags.isSet(HTM_ABORT); }
    bool
    isHTMCmd() const
    {
        return (isHTMStart() || isHTMCommit() ||
                isHTMCancel() || isHTMAbort());
    }

    bool isTlbi() const { return _flags.isSet(TLBI); }
    bool isTlbiSync() const { return _flags.isSet(TLBI_SYNC); }
    bool isTlbiExtSync() const { return _flags.isSet(TLBI_EXT_SYNC); }
    bool isTlbiExtSyncComp() const { return _flags.isSet(TLBI_EXT_SYNC_COMP); }
    bool
    isTlbiCmd() const
    {
        return (isTlbi() || isTlbiSync() ||
                isTlbiExtSync() || isTlbiExtSyncComp());
    }
    bool isMemMgmt() const { return isTlbiCmd() || isHTMCmd(); }

    bool
    isAtomic() const
    {
        return _flags.isSet(ATOMIC_RETURN_OP) ||
               _flags.isSet(ATOMIC_NO_RETURN_OP);
    }

    /**
     * Accessor functions for the destination of a memory request. The
     * destination flag can specify a point of reference for the
     * operation (e.g. a cache block clean to the the point of
     * unification). At the moment the destination is only used by the
     * cache maintenance operations.
     */
    bool isToPOU() const { return _flags.isSet(DST_POU); }
    bool isToPOC() const { return _flags.isSet(DST_POC); }
    Flags getDest() const { return _flags & DST_BITS; }

    bool isAcquire() const { return _cacheCoherenceFlags.isSet(ACQUIRE); }


    /**
     * Accessor functions for the cache bypass flags. The cache bypass
     * can specify which levels in the hierarchy to bypass. If GLC_BIT
     * is set, the requests are globally coherent and bypass TCP.
     * If SLC_BIT is set, then the requests are system level coherent
     * and bypass both TCP and TCC.
     */
    bool isGLCSet() const {return _cacheCoherenceFlags.isSet(GLC_BIT); }
    bool isSLCSet() const {return _cacheCoherenceFlags.isSet(SLC_BIT); }

    /**
     * Accessor functions for the memory space configuration flags and used by
     * GPU ISAs such as the Heterogeneous System Architecture (HSA). Note that
     * setting extraFlags should be done via setCacheCoherenceFlags().
     */
    bool isInvL1() const { return _cacheCoherenceFlags.isSet(INV_L1); }
    bool isInvL2() const { return _cacheCoherenceFlags.isSet(GL2_CACHE_INV); }

    bool
    isGL2CacheFlush() const
    {
        return _cacheCoherenceFlags.isSet(FLUSH_L2);
    }

    /**
     * Accessor functions to determine whether this request is part of
     * a cache maintenance operation. At the moment three operations
     * are supported:

     * 1) A cache clean operation updates all copies of a memory
     * location to the point of reference,
     * 2) A cache invalidate operation invalidates all copies of the
     * specified block in the memory above the point of reference,
     * 3) A clean and invalidate operation is a combination of the two
     * operations.
     * @{ */
    bool isCacheClean() const { return _flags.isSet(CLEAN); }
    bool isCacheInvalidate() const { return _flags.isSet(INVALIDATE); }
    bool isCacheMaintenance() const { return _flags.isSet(CLEAN|INVALIDATE); }
    /** @} */
};

} // namespace gem5

#endif // __MEM_REQUEST_HH__
