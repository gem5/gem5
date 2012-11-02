/*
 * Copyright (c) 2010-2012 ARM Limited
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
 * Authors: Ali Saidi
 */

#ifndef __ARCH_ARM_TABLE_WALKER_HH__
#define __ARCH_ARM_TABLE_WALKER_HH__

#include <list>

#include "arch/arm/miscregs.hh"
#include "arch/arm/tlb.hh"
#include "dev/dma_device.hh"
#include "mem/mem_object.hh"
#include "mem/request.hh"
#include "params/ArmTableWalker.hh"
#include "sim/eventq.hh"
#include "sim/fault_fwd.hh"

class ThreadContext;

namespace ArmISA {
class Translation;
class TLB;

class TableWalker : public MemObject
{
  public:
    struct L1Descriptor {
        /** Type of page table entry ARM DDI 0406B: B3-8*/
        enum EntryType {
            Ignore,
            PageTable,
            Section,
            Reserved
        };

        /** The raw bits of the entry */
        uint32_t data;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        EntryType type() const
        {
            return (EntryType)(data & 0x3);
        }

        /** Is the page a Supersection (16MB)?*/
        bool supersection() const
        {
            return bits(data, 18);
        }

        /** Return the physcal address of the entry, bits in position*/
        Addr paddr() const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20);
        }
        /** Return the physcal address of the entry, bits in position*/
        Addr paddr(Addr va) const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20) | mbits(va, 19, 0);
        }


        /** Return the physical frame, bits shifted right */
        Addr pfn() const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return bits(data, 31, 20);
        }

        /** Is the translation global (no asid used)? */
        bool global() const
        {
            return bits(data, 17);
        }

        /** Is the translation not allow execution? */
        bool xn() const
        {
            return bits(data, 4);
        }

        /** Three bit access protection flags */
        uint8_t ap() const
        {
            return (bits(data, 15) << 2) | bits(data, 11, 10);
        }

        /** Domain Client/Manager: ARM DDI 0406B: B3-31 */
        uint8_t domain() const
        {
            return bits(data, 8, 5);
        }

        /** Address of L2 descriptor if it exists */
        Addr l2Addr() const
        {
            return mbits(data, 31, 10);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32.
         * These bits are largly ignored by M5 and only used to
         * provide the illusion that the memory system cares about
         * anything but cachable vs. uncachable.
         */
        uint8_t texcb() const
        {
            return bits(data, 2) | bits(data, 3) << 1 | bits(data, 14, 12) << 2;
        }

        /** If the section is shareable. See texcb() comment. */
        bool shareable() const
        {
            return bits(data, 16);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void setAp0()
        {
            data |= 1 << 10;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool dirty() const
        {
            return _dirty;
        }
    };

    /** Level 2 page table descriptor */
    struct L2Descriptor {

        /** The raw bits of the entry. */
        uint32_t data;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        /** Is the entry invalid */
        bool invalid() const
        {
            return bits(data, 1, 0) == 0;
        }

        /** What is the size of the mapping? */
        bool large() const
        {
            return bits(data, 1) == 0;
        }

        /** Is execution allowed on this mapping? */
        bool xn() const
        {
            return large() ? bits(data, 15) : bits(data, 0);
        }

        /** Is the translation global (no asid used)? */
        bool global() const
        {
            return !bits(data, 11);
        }

        /** Three bit access protection flags */
        uint8_t ap() const
        {
           return bits(data, 5, 4) | (bits(data, 9) << 2);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32 */
        uint8_t texcb() const
        {
            return large() ?
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 14, 12) << 2)) :
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 8, 6) << 2));
        }

        /** Return the physical frame, bits shifted right */
        Addr pfn() const
        {
            return large() ? bits(data, 31, 16) : bits(data, 31, 12);
        }

        /** Return complete physical address given a VA */
        Addr paddr(Addr va) const
        {
            if (large())
                return mbits(data, 31, 16) | mbits(va, 15, 0);
            else
                return mbits(data, 31, 12) | mbits(va, 11, 0);
        }

        /** If the section is shareable. See texcb() comment. */
        bool shareable() const
        {
            return bits(data, 10);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void setAp0()
        {
            data |= 1 << 4;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool dirty() const
        {
            return _dirty;
        }

    };

  protected:

    /**
     * A snooping DMA port that currently does nothing besides
     * extending the DMA port to accept snoops without complaining.
     */
    class SnoopingDmaPort : public DmaPort
    {

      protected:

        virtual void recvTimingSnoopReq(PacketPtr pkt)
        { }

        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        { return 0; }

        virtual void recvFunctionalSnoop(PacketPtr pkt)
        { }

        virtual bool isSnooping() const { return true; }

      public:

        /**
         * A snooping DMA port merely calls the construtor of the DMA
         * port.
         */
        SnoopingDmaPort(MemObject *dev, System *s) :
            DmaPort(dev, s)
        { }
    };

    struct WalkerState //: public SimObject
    {
        /** Thread context that we're doing the walk for */
        ThreadContext *tc;

        /** Request that is currently being serviced */
        RequestPtr req;

        /** Context ID that we're servicing the request under */
        uint8_t contextId;

        /** Translation state for delayed requests */
        TLB::Translation *transState;

        /** The fault that we are going to return */
        Fault fault;

        /** The virtual address that is being translated */
        Addr vaddr;

        /** Cached copy of the sctlr as it existed when translation began */
        SCTLR sctlr;

        /** Width of the base address held in TTRB0 */
        uint32_t N;

        /** If the access is a write */
        bool isWrite;

        /** If the access is a fetch (for execution, and no-exec) must be checked?*/
        bool isFetch;

        /** If the mode is timing or atomic */
        bool timing;

        /** If the atomic mode should be functional */
        bool functional;

        /** Save mode for use in delayed response */
        BaseTLB::Mode mode;

        L1Descriptor l1Desc;
        L2Descriptor l2Desc;

        /** Whether L1/L2 descriptor response is delayed in timing mode */
        bool delayed;

        TableWalker *tableWalker;

        void doL1Descriptor();
        void doL2Descriptor();

        std::string name() const {return tableWalker->name();}
    };


    /** Queue of requests that need processing first level translation */
    std::list<WalkerState *> stateQueueL1;

    /** Queue of requests that have passed first level translation and
     * require an additional level. */
    std::list<WalkerState *> stateQueueL2;

    /** Queue of requests that have passed are waiting because the walker is
     * currently busy. */
    std::list<WalkerState *> pendingQueue;


    /** Port to issue translation requests from */
    SnoopingDmaPort port;

    /** If we're draining keep the drain event around until we're drained */
    DrainManager *drainManager;

    /** TLB that is initiating these table walks */
    TLB *tlb;

    /** Cached copy of the sctlr as it existed when translation began */
    SCTLR sctlr;

    WalkerState *currState;

    /** If a timing translation is currently in progress */
    bool pending;

    /** Request id for requests generated by this walker */
    MasterID masterId;

    /** The number of walks belonging to squashed instructions that can be
     * removed from the pendingQueue per cycle. */
    unsigned numSquashable;

  public:
    typedef ArmTableWalkerParams Params;
    TableWalker(const Params *p);
    virtual ~TableWalker();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /** Checks if all state is cleared and if so, completes drain */
    void completeDrain();
    unsigned int drain(DrainManager *dm);
    void drainResume();
    virtual BaseMasterPort& getMasterPort(const std::string &if_name,
                                          PortID idx = InvalidPortID);

    Fault walk(RequestPtr req, ThreadContext *tc, uint8_t cid, TLB::Mode mode,
            TLB::Translation *_trans, bool timing, bool functional = false);

    void setTlb(TLB *_tlb) { tlb = _tlb; }
    void memAttrs(ThreadContext *tc, TlbEntry &te, SCTLR sctlr,
                  uint8_t texcb, bool s);

  private:

    void doL1Descriptor();
    void doL1DescriptorWrapper();
    EventWrapper<TableWalker, &TableWalker::doL1DescriptorWrapper> doL1DescEvent;

    void doL2Descriptor();
    void doL2DescriptorWrapper();
    EventWrapper<TableWalker, &TableWalker::doL2DescriptorWrapper> doL2DescEvent;

    Fault processWalk();
    void processWalkWrapper();
    EventWrapper<TableWalker, &TableWalker::processWalkWrapper> doProcessEvent;

    void nextWalk(ThreadContext *tc);
};


} // namespace ArmISA

#endif //__ARCH_ARM_TABLE_WALKER_HH__

